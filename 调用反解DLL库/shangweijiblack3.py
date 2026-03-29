import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
from ctypes import CDLL, c_double, pointer, POINTER, c_int
import time
import math
import numpy as np
import threading
import queue
import cv2
import sys
import os

# 添加当前目录到Python路径，以便导入视觉检测模块
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 直接导入您提供的视觉检测模块中的类
try:
    from visonblack import RealSenseCamera, BlackRectangleDetector, BlackRectangleDetectionSystem
except ImportError:
    # 如果直接导入失败，尝试动态导入
    print("请将您的视觉检测代码文件命名为 'your_vision_file.py' 并放在同一目录下")
    sys.exit(1)


class VisionDetectionThread:
    """视觉检测线程管理类"""

    def __init__(self):
        self.detection_system = None
        self.detection_thread = None
        self.running = False
        self.detection_queue = queue.Queue(maxsize=10)
        self.latest_detection = None

    def start_detection(self):
        """启动视觉检测"""
        if self.running:
            return

        self.running = True
        self.detection_thread = threading.Thread(target=self._detection_loop)
        self.detection_thread.daemon = True
        self.detection_thread.start()

    def stop_detection(self):
        """停止视觉检测"""
        self.running = False
        if self.detection_system:
            self.detection_system.cleanup()

    def _detection_loop(self):
        """检测循环"""
        try:
            # 创建检测系统
            self.detection_system = BlackRectangleDetectionSystem(width=640, height=480, fps=30)
            detector = self.detection_system.detector

            # 修改原检测器的运行逻辑，使其适合线程运行
            while self.running:
                # 获取帧数据
                depth_image, color_image = self.detection_system.camera.get_frames()
                if depth_image is None or color_image is None:
                    continue

                # 更新参数
                detector._update_params()
                detector._calculate_fps()

                # 检测矩形
                detected_rectangles, result_image, black_mask, depth_mask = detector.detect_rectangles(
                    color_image, depth_image)

                # 更新检测结果
                if detected_rectangles:
                    self.latest_detection = detected_rectangles[0]  # 取第一个检测到的矩形
                    try:
                        self.detection_queue.put_nowait(self.latest_detection)
                    except queue.Full:
                        pass  # 如果队列满了，跳过最新的检测结果

                # 显示结果（可选）
                cv2.imshow('Vision Detection', result_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except Exception as e:
            print(f"视觉检测错误: {e}")
        finally:
            cv2.destroyAllWindows()

    def get_latest_detection(self):
        """获取最新检测结果"""
        try:
            # 先尝试从队列获取最新结果
            while not self.detection_queue.empty():
                self.latest_detection = self.detection_queue.get_nowait()
            return self.latest_detection
        except queue.Empty:
            return self.latest_detection


class RSS6RobotController:
    def __init__(self, root):
        self.root = root
        self.root.title("6RSS并联机器人控制器 - 视觉引导正方形轨迹")
        self.root.geometry("800x600")
        self.root.resizable(False, False)

        # 加载DLL求解器
        self.dll = None
        self.load_solver()

        # 串口相关变量
        self.serial_port = None
        self.is_connected = False

        # 轨迹控制变量
        self.trajectory_running = False
        self.current_step = 0
        self.total_steps = 0
        self.square_side = 0  # 通过视觉检测获取
        self.center_x = 0
        self.center_y = 0
        self.center_z = 0
        self.speed = 100  # 运动速度(ms/点)

        # 视觉检测相关
        self.vision_detector = VisionDetectionThread()
        self.vision_enabled = False

        # 正方形轨迹参数
        self.square_path = []
        self.current_position = (0, 0, 0)
        self.trajectory_completed = False

        # 创建UI
        self.create_widgets()

    def load_solver(self):
        """加载逆解求解器DLL"""
        try:
            self.dll = CDLL('./RSS6RBT_InverseDLL.dll')
            self.dll.InK6RSS.argtypes = [
                c_double, c_double, c_double,
                c_double, c_double, c_double,
                POINTER(c_double * 6)
            ]
            self.dll.InK6RSS.restype = c_int
            print("求解器加载成功")
        except Exception as e:
            messagebox.showerror("错误", f"加载求解器失败: {str(e)}")
            self.dll = None

    def generate_square_path(self):
        """生成正方形轨迹点"""
        self.square_path = []

        if self.square_side <= 0:
            messagebox.showwarning("警告", "请先通过视觉检测获取正方形边长")
            return False

        # 正方形四个顶点（相对于中心）
        half_side = self.square_side / 2


        self.square_path.append((0, 0, 0))
        for i in range(100):
            self.square_path.append((0, -half_side, (-half_side + (i * half_side / 50))))
        for i in range(100):
            self.square_path.append((0, (-half_side + (i * half_side / 50)), half_side))
        for i in range(100):
            self.square_path.append((0, half_side, (half_side - (i * half_side / 50))))
        for i in range(100):
            self.square_path.append((0, (half_side - (i * half_side / 50)), -half_side))
        # 生成四条边的轨迹

        # 从最后一个顶点返回中心
        self.square_path.append((0, 0, 0))

        self.total_steps = len(self.square_path)
        return True

    def create_widgets(self):
        """创建界面组件"""
        # 创建主框架
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 视觉检测区域
        vision_frame = ttk.LabelFrame(main_frame, text="视觉检测控制", padding="10")
        vision_frame.pack(fill=tk.X, pady=(0, 10))

        self.start_vision_btn = ttk.Button(vision_frame, text="启动视觉检测",
                                           command=self.toggle_vision)
        self.start_vision_btn.pack(side=tk.LEFT, padx=5)

        self.detect_btn = ttk.Button(vision_frame, text="获取正方形参数",
                                     command=self.get_square_parameters, state=tk.DISABLED)
        self.detect_btn.pack(side=tk.LEFT, padx=5)

        # 检测结果显示
        self.detection_var = tk.StringVar(value="等待检测...")
        ttk.Label(vision_frame, textvariable=self.detection_var, width=50).pack(side=tk.LEFT, padx=20)

        # 轨迹参数区域
        traj_frame = ttk.LabelFrame(main_frame, text="检测到的正方形参数", padding="10")
        traj_frame.pack(fill=tk.X, pady=(0, 10))

        # 创建参数显示框架
        param_frame = ttk.Frame(traj_frame)
        param_frame.pack(fill=tk.X)

        ttk.Label(param_frame, text="边长 (mm):").grid(row=0, column=0, padx=10, pady=5, sticky=tk.W)
        self.side_var = tk.StringVar(value="0.0")
        side_label = ttk.Label(param_frame, textvariable=self.side_var, width=10, relief="sunken")
        side_label.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(param_frame, text="深度 (mm):").grid(row=0, column=2, padx=10, pady=5, sticky=tk.W)
        self.depth_var = tk.StringVar(value="0.0")
        depth_label = ttk.Label(param_frame, textvariable=self.depth_var, width=10, relief="sunken")
        depth_label.grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(param_frame, text="X偏移 (mm):").grid(row=0, column=4, padx=10, pady=5, sticky=tk.W)
        self.dx_var = tk.StringVar(value="0.0")
        dx_label = ttk.Label(param_frame, textvariable=self.dx_var, width=10, relief="sunken")
        dx_label.grid(row=0, column=5, padx=5, pady=5)

        ttk.Label(param_frame, text="Y偏移 (mm):").grid(row=0, column=6, padx=10, pady=5, sticky=tk.W)
        self.dy_var = tk.StringVar(value="0.0")
        dy_label = ttk.Label(param_frame, textvariable=self.dy_var, width=10, relief="sunken")
        dy_label.grid(row=0, column=7, padx=5, pady=5)

        # 运动控制区域
        control_frame = ttk.LabelFrame(main_frame, text="运动控制", padding="10")
        control_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(control_frame, text="运动速度 (ms/点):").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.speed_entry = ttk.Entry(control_frame, width=10)
        self.speed_entry.grid(row=0, column=1, padx=5, pady=5)
        self.speed_entry.insert(0, str(self.speed))

        self.start_btn = ttk.Button(control_frame, text="开始正方形轨迹",
                                    command=self.start_trajectory, state=tk.DISABLED)
        self.start_btn.grid(row=0, column=2, padx=10, pady=5)

        self.stop_btn = ttk.Button(control_frame, text="停止运动",
                                   command=self.stop_trajectory, state=tk.DISABLED)
        self.stop_btn.grid(row=0, column=3, padx=10, pady=5)

        self.reset_btn = ttk.Button(control_frame, text="重置",
                                    command=self.reset_trajectory)
        self.reset_btn.grid(row=0, column=4, padx=10, pady=5)

        # 当前位置显示区域
        pos_frame = ttk.LabelFrame(main_frame, text="机器人当前位置", padding="10")
        pos_frame.pack(fill=tk.X, pady=(0, 10))

        self.pos_labels = {}
        coords = ["X (mm):", "Y (mm):", "Z (mm):", "运动进度:"]
        for i, coord in enumerate(coords):
            ttk.Label(pos_frame, text=coord).grid(row=0, column=i * 2, padx=15, pady=5, sticky=tk.W)
            self.pos_labels[coord] = ttk.Label(pos_frame, text="0.00", width=12, relief="sunken")
            self.pos_labels[coord].grid(row=0, column=i * 2 + 1, padx=5, pady=5)

        # 舵机角度显示区域
        servo_frame = ttk.LabelFrame(main_frame, text="舵机角度", padding="10")
        servo_frame.pack(fill=tk.X, pady=(0, 10))

        self.servo_labels = []
        for i in range(6):
            frame = ttk.Frame(servo_frame)
            frame.grid(row=i // 3, column=i % 3, padx=15, pady=5, sticky=tk.W)

            ttk.Label(frame, text=f"舵机 {i + 1} (°):").pack(side=tk.LEFT, padx=5)
            val_label = ttk.Label(frame, text="--", width=8, relief="sunken")
            val_label.pack(side=tk.LEFT)
            self.servo_labels.append(val_label)

        # 串口设置区域
        serial_frame = ttk.LabelFrame(main_frame, text="串口设置", padding="10")
        serial_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(serial_frame, text="串口号:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.port_var = tk.StringVar()

        # 保存引用
        self.port_combo = ttk.Combobox(serial_frame, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(serial_frame, text="波特率:").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.baud_var = tk.StringVar(value="115200")

        self.baud_combo = ttk.Combobox(serial_frame, textvariable=self.baud_var, width=12,
                                       values=["9600", "19200", "38400", "57600", "115200"], state="readonly")
        self.baud_combo.grid(row=0, column=3, padx=5, pady=5)

        refresh_btn = ttk.Button(serial_frame, text="刷新串口", command=self.refresh_ports)
        refresh_btn.grid(row=0, column=4, padx=5, pady=5)

        self.connect_btn = ttk.Button(serial_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=5, pady=5)

        # 状态区域
        self.status_var = tk.StringVar(value="系统就绪，请启动视觉检测")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

        self.refresh_ports()

        self.last_angles = None

    def toggle_vision(self):
        """切换视觉检测状态"""
        if self.vision_enabled:
            self.vision_detector.stop_detection()
            self.vision_enabled = False
            self.start_vision_btn.config(text="启动视觉检测")
            self.detect_btn.config(state=tk.DISABLED)
            self.status_var.set("视觉检测已停止")
            cv2.destroyAllWindows()
        else:
            self.vision_detector.start_detection()
            self.vision_enabled = True
            self.start_vision_btn.config(text="停止视觉检测")
            self.detect_btn.config(state=tk.NORMAL)
            self.status_var.set("视觉检测已启动，请将正方形放置在相机视野内")

    def get_square_parameters(self):
        """通过视觉检测获取正方形参数"""
        if not self.vision_enabled:
            messagebox.showwarning("警告", "请先启动视觉检测")
            return

        detection = self.vision_detector.get_latest_detection()
        if detection:
            # 直接从视觉检测结果中获取参数
            self.square_side = detection['physical_size']  # 正方形边长
            self.center_x = detection['depth']  # 深度
            dx_mm = detection['dx_mm']  # X偏移
            dy_mm = detection['dy_mm']  # Y偏移

            # 更新显示
            self.side_var.set(f"{self.square_side:.1f}")
            self.depth_var.set(f"{self.center_x:.1f}")
            self.dx_var.set(f"{dx_mm:.1f}")
            self.dy_var.set(f"{dy_mm:.1f}")

            self.detection_var.set(f"检测成功: 边长{self.square_side:.1f}mm, 深度{self.center_x:.1f}mm")
            self.status_var.set("正方形参数获取成功，可以开始轨迹运动")

            # 启用轨迹开始按钮
            self.start_btn.config(state=tk.NORMAL)
        else:
            self.status_var.set("未检测到正方形，请调整相机位置或检测参数")

    def refresh_ports(self):
        """刷新可用串口号列表"""
        try:
            ports = serial.tools.list_ports.comports()
            port_list = [port.device for port in ports]

            self.port_combo['values'] = port_list
            if ports and not self.port_var.get():
                self.port_combo.set(ports[0].device)

            self.status_var.set(f"找到 {len(ports)} 个串口设备")

        except Exception as e:
            messagebox.showerror("错误", f"刷新串口失败: {str(e)}")
    def toggle_connection(self):
        """切换串口连接"""
        if self.is_connected:
            try:
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.close()
                self.is_connected = False
                self.connect_btn.config(text="连接")
                self.status_var.set("已断开连接")
            except Exception as e:
                messagebox.showerror("错误", f"断开连接失败: {str(e)}")
        else:
            try:
                port = self.port_var.get()
                baudrate = int(self.baud_var.get())

                if not port:
                    messagebox.showwarning("警告", "请选择串口号")
                    return

                self.serial_port = serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    timeout=1
                )

                if self.serial_port.is_open:
                    self.is_connected = True
                    self.connect_btn.config(text="断开")
                    self.status_var.set(f"已连接到 {port}")
            except Exception as e:
                messagebox.showerror("错误", f"连接失败: {str(e)}")

    def reset_trajectory(self):
        """重置轨迹"""
        self.current_step = 0
        self.trajectory_completed = False
        self.current_position = (0, 0, self.center_z)
        self.update_position_display()
        self.status_var.set("轨迹已重置")

    def calculate_angles(self, x, y, z):
        """计算舵机角度"""
        if not self.dll:
            return False

        try:
            yaw = np.arctan(-y / self.center_x) * 180 / np.pi
            pitch = np.arctan(z / self.center_x) * 180 / np.pi
            roll = 0.0
            angles = (c_double * 6)()
            ret = self.dll.InK6RSS(
                c_double(0), c_double(0), c_double(188),
                c_double(yaw), c_double(pitch), c_double(roll),
                pointer(angles)
            )

            # 更新位置显示
            self.pos_labels["X (mm):"].config(text=f"{x:.2f}")
            self.pos_labels["Y (mm):"].config(text=f"{y:.2f}")
            self.pos_labels["Z (mm):"].config(text=f"{z:.2f}")

            if ret == 0 or any(angle == -360 for angle in angles):
                self.status_var.set("角度计算失败，可能超出工作空间")
                self.last_angles = None
                for label in self.servo_labels:
                    label.config(text="--")
                return False

            self.last_angles = [round(angle, 2) for angle in angles]
            for i, label in enumerate(self.servo_labels):
                label.config(text=str(self.last_angles[i]))

            return True
        except Exception as e:
            self.status_var.set(f"角度计算失败: {str(e)}")
            return False

    def send_angles(self):
        """发送角度到单片机"""
        if not self.is_connected:
            self.status_var.set("未连接串口")
            return False

        if not self.last_angles:
            self.status_var.set("无角度数据")
            return False

        try:
            data_str = f"S,{','.join(map(str, self.last_angles))},E\n"
            self.serial_port.write(data_str.encode('utf-8'))
            time.sleep(0.05)
            self.status_var.set(f"运动中: {self.current_step}/{self.total_steps}")
            return True
        except Exception as e:
            self.status_var.set(f"发送失败: {str(e)}")
            self.toggle_connection()  # 发送失败时断开连接
            return False

    def update_position_display(self):
        """更新位置显示"""
        progress = (self.current_step / self.total_steps * 100) if self.total_steps > 0 else 0
        self.pos_labels["运动进度:"].config(text=f"{progress:.1f}%")

    def start_trajectory(self):
        """开始轨迹运动"""
        if self.trajectory_running:
            return

        if not self.dll:
            messagebox.showwarning("警告", "求解器未加载")
            return

        if not self.is_connected:
            messagebox.showwarning("警告", "请先连接串口")
            return

        # 更新速度参数
        try:
            self.speed = float(self.speed_entry.get())
            if self.speed <= 0:
                raise ValueError("速度必须大于0")
        except ValueError:
            messagebox.showerror("错误", "速度参数无效")
            return

        # 生成轨迹
        if not self.generate_square_path():
            return

        self.trajectory_running = True
        self.trajectory_completed = False
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.status_var.set("开始正方形轨迹运动...")

        self.run_trajectory()

    def stop_trajectory(self):
        """停止轨迹运动"""
        self.trajectory_running = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.status_var.set("轨迹运动已停止")

    def run_trajectory(self):
        """运行轨迹循环"""
        if not self.trajectory_running or self.trajectory_completed:
            return

        if self.current_step >= self.total_steps:
            self.trajectory_completed = True
            self.stop_trajectory()
            self.status_var.set("正方形轨迹运动完成!")
            return

        # 获取当前轨迹点
        x, y, z = self.square_path[self.current_step]
        self.current_position = (x, y, z)

        # 更新显示并发送角度
        self.update_position_display()
        if self.calculate_angles(x, y, z):
            self.send_angles()

        self.current_step += 1
        self.root.after(int(self.speed), self.run_trajectory)

    def cleanup(self):
        """清理资源"""
        if self.vision_enabled:
            self.vision_detector.stop_detection()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    root = tk.Tk()
    app = RSS6RobotController(root)


    def on_closing():
        app.cleanup()
        root.destroy()


    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()