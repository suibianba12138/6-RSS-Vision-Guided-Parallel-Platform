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
from PIL import Image, ImageTk


class RSS6RobotController:
    def __init__(self, root):
        self.root = root
        self.root.title("6RSS并联机器人控制器 - 视觉伺服控制")
        self.root.geometry("1000x600")
        self.root.resizable(False, False)

        # 加载DLL求解器
        self.dll = None
        self.load_solver()

        # 串口相关变量
        self.serial_port = None
        self.is_connected = False

        # 控制变量
        self.control_running = False
        self.target_x = 0
        self.target_y = 0
        self.target_z = 188
        self.current_x = 0
        self.current_y = 0
        self.current_z = 188
        self.deviation_x = 0
        self.deviation_y = 0
        self.deviation_z = 0
        self.kp = 0.1

        # 图像检测相关
        self.detector = None
        self.detection_running = False
        self.deviation_queue = queue.Queue()
        self.last_frame = None

        # 摄像头图像显示相关
        self.camera_frame = None
        self.camera_label = None
        self.camera_image = None

        # 创建UI
        self.create_widgets()

        # 主循环更新
        self.update_interval = 50  # 毫秒
        self.root.after(self.update_interval, self.update)

    def load_solver(self):
        """加载逆解求解器DLL"""
        try:
            self.dll = CDLL('./RSS6RBT_InverseDLL.dll')
            # 设置函数参数和返回值类型
            self.dll.InK6RSS.argtypes = [
                c_double, c_double, c_double,  # XP, YP, ZP
                c_double, c_double, c_double,  # YAW, PITCH, ROLL
                POINTER(c_double * 6)  # 输出角度数组
            ]
            self.dll.InK6RSS.restype = c_int
            print("求解器加载成功")
        except Exception as e:
            messagebox.showerror("错误", f"加载求解器失败: {str(e)}")
            self.dll = None

    def create_widgets(self):
        """创建界面组件"""
        # 创建主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 创建左右分栏
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(10, 0))

        # 控制参数设置区域
        control_frame = ttk.LabelFrame(left_frame, text="控制参数", padding="10")
        control_frame.pack(fill=tk.X, pady=(0, 10))

        # 比例系数
        ttk.Label(control_frame, text="比例系数 Kp:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.kp_entry = ttk.Entry(control_frame, width=10)
        self.kp_entry.grid(row=0, column=1, padx=5, pady=5)
        self.kp_entry.insert(0, str(self.kp))

        # 目标位置显示
        ttk.Label(control_frame, text="目标位置 (mm):").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.target_pos_label = ttk.Label(control_frame, text=f"({self.target_x}, {self.target_y}, {self.target_z})")
        self.target_pos_label.grid(row=0, column=3, padx=5, pady=5)

        # 当前位置显示区域
        pos_frame = ttk.LabelFrame(left_frame, text="当前位置和偏差", padding="10")
        pos_frame.pack(fill=tk.X, pady=(0, 10))

        # 当前位置
        ttk.Label(pos_frame, text="当前 X (mm):").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.current_x_label = ttk.Label(pos_frame, text="0.00", width=10)
        self.current_x_label.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(pos_frame, text="当前 Y (mm):").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.current_y_label = ttk.Label(pos_frame, text="0.00", width=10)
        self.current_y_label.grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(pos_frame, text="当前 Z (mm):").grid(row=0, column=4, padx=5, pady=5, sticky=tk.W)
        self.current_z_label = ttk.Label(pos_frame, text="188.00", width=10)
        self.current_z_label.grid(row=0, column=5, padx=5, pady=5)

        # 偏差显示
        ttk.Label(pos_frame, text="X偏差 (mm):").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.deviation_x_label = ttk.Label(pos_frame, text="0.00", width=10)
        self.deviation_x_label.grid(row=1, column=1, padx=5, pady=5)

        ttk.Label(pos_frame, text="Y偏差 (mm):").grid(row=1, column=2, padx=5, pady=5, sticky=tk.W)
        self.deviation_y_label = ttk.Label(pos_frame, text="0.00", width=10)
        self.deviation_y_label.grid(row=1, column=3, padx=5, pady=5)

        ttk.Label(pos_frame, text="Z偏差 (mm):").grid(row=1, column=4, padx=5, pady=5, sticky=tk.W)
        self.deviation_z_label = ttk.Label(pos_frame, text="0.00", width=10)
        self.deviation_z_label.grid(row=1, column=5, padx=5, pady=5)

        # 检测状态显示
        ttk.Label(pos_frame, text="检测状态:").grid(row=2, column=0, padx=5, pady=5, sticky=tk.W)
        self.detection_status_label = ttk.Label(pos_frame, text="未启动", foreground="red")
        self.detection_status_label.grid(row=2, column=1, padx=5, pady=5)

        ttk.Label(pos_frame, text="检测到圆:").grid(row=2, column=2, padx=5, pady=5, sticky=tk.W)
        self.detected_circles_label = ttk.Label(pos_frame, text="0", width=10)
        self.detected_circles_label.grid(row=2, column=3, padx=5, pady=5)

        # 姿态角度显示区域
        angle_frame = ttk.LabelFrame(left_frame, text="姿态角度", padding="10")
        angle_frame.pack(fill=tk.X, pady=(0, 10))

        self.angle_labels = {}
        for i, coord in enumerate(["YAW角度 (°):", "PITCH角度 (°):", "ROLL角度 (°):"]):
            ttk.Label(angle_frame, text=coord).grid(row=0, column=i * 2, padx=15, pady=5, sticky=tk.W)
            self.angle_labels[coord] = ttk.Label(angle_frame, text="0.00", width=10)
            self.angle_labels[coord].grid(row=0, column=i * 2 + 1, padx=5, pady=5)

        # 舵机角度显示区域
        servo_frame = ttk.LabelFrame(left_frame, text="舵机角度", padding="10")
        servo_frame.pack(fill=tk.X, pady=(0, 10))

        self.servo_labels = []
        for i in range(6):
            frame = ttk.Frame(servo_frame)
            frame.grid(row=i // 3, column=i % 3, padx=10, pady=5, sticky=tk.W)

            ttk.Label(frame, text=f"舵机 {i + 1} (°):").pack(side=tk.LEFT, padx=5)
            val_label = ttk.Label(frame, text="--", width=10)
            val_label.pack(side=tk.LEFT)
            self.servo_labels.append(val_label)

        # 控制按钮区域
        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(fill=tk.X, pady=(0, 10))

        self.start_detection_btn = ttk.Button(btn_frame, text="启动检测", command=self.start_detection)
        self.start_detection_btn.pack(side=tk.LEFT, padx=5)

        self.stop_detection_btn = ttk.Button(btn_frame, text="停止检测", command=self.stop_detection, state=tk.DISABLED)
        self.stop_detection_btn.pack(side=tk.LEFT, padx=5)

        self.start_control_btn = ttk.Button(btn_frame, text="开始控制", command=self.start_control)
        self.start_control_btn.pack(side=tk.LEFT, padx=5)

        self.stop_control_btn = ttk.Button(btn_frame, text="停止控制", command=self.stop_control, state=tk.DISABLED)
        self.stop_control_btn.pack(side=tk.LEFT, padx=5)

        self.update_params_btn = ttk.Button(btn_frame, text="更新参数", command=self.update_parameters)
        self.update_params_btn.pack(side=tk.LEFT, padx=5)

        # 串口设置区域
        serial_frame = ttk.LabelFrame(left_frame, text="串口设置", padding="10")
        serial_frame.pack(fill=tk.X, pady=(0, 10))

        # 串口选择
        ttk.Label(serial_frame, text="串口号:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(serial_frame, textvariable=self.port_var, width=10)  # 保存为成员变量
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)

        # 波特率选择
        ttk.Label(serial_frame, text="波特率:").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.baud_var = tk.StringVar(value="115200")
        baud_combo = ttk.Combobox(serial_frame, textvariable=self.baud_var, width=10,
                                  values=["9600", "19200", "38400", "57600", "115200"])
        baud_combo.grid(row=0, column=3, padx=5, pady=5)

        # 刷新串口按钮
        refresh_btn = ttk.Button(serial_frame, text="刷新", command=self.refresh_ports)
        refresh_btn.grid(row=0, column=4, padx=5, pady=5)

        # 连接/断开按钮
        self.connect_btn = ttk.Button(serial_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=5, pady=5)

        # 摄像头图像显示区域（右侧）
        camera_display_frame = ttk.LabelFrame(right_frame, text="摄像头图像", padding="10")
        camera_display_frame.pack(fill=tk.BOTH, expand=True)

        # 创建图像显示标签
        self.camera_label = ttk.Label(camera_display_frame)
        self.camera_label.pack(fill=tk.BOTH, expand=True)

        # 状态区域
        self.status_var = tk.StringVar(value="就绪")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

        # 初始刷新串口列表
        self.refresh_ports()

        # 保存计算结果
        self.last_angles = None

    def refresh_ports(self):
        """刷新可用串口号列表"""
        try:
            ports = serial.tools.list_ports.comports()
            port_list = [port.device for port in ports]
            self.port_combo['values'] = port_list  # 直接操作保存的组合框

            if port_list:
                self.port_var.set(port_list[0])  # 默认选择第一个
                self.status_var.set(f"发现 {len(port_list)} 个串口")
            else:
                self.status_var.set("未检测到可用串口")
        except Exception as e:
            messagebox.showerror("错误", f"刷新串口失败: {str(e)}")
            self.status_var.set("刷新串口失败")

    def toggle_connection(self):
        """切换串口连接状态"""
        if self.is_connected:
            # 断开连接
            try:
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.close()
                self.is_connected = False
                self.connect_btn.config(text="连接")
                self.status_var.set("已断开连接")
            except Exception as e:
                messagebox.showerror("错误", f"断开连接失败: {str(e)}")
        else:
            # 建立连接
            try:
                port = self.port_var.get()
                baudrate = int(self.baud_var.get())

                if not port:
                    messagebox.showwarning("警告", "请选择串口号")
                    return

                self.serial_port = serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    timeout=1,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS
                )

                if self.serial_port.is_open:
                    self.is_connected = True
                    self.connect_btn.config(text="断开")
                    self.status_var.set(f"已连接到 {port} ({baudrate} bps)")
            except Exception as e:
                messagebox.showerror("错误", f"连接失败: {str(e)}")

    def update_parameters(self):
        """更新控制参数"""
        try:
            self.kp = float(self.kp_entry.get())
            if self.kp <= 0 or self.kp > 1:
                raise ValueError("比例系数应在0-1之间")

            self.status_var.set("控制参数已更新")
        except ValueError as e:
            messagebox.showerror("错误", f"参数无效: {str(e)}")

    def update(self):
        """主循环更新函数"""
        if self.detection_running and self.detector:
            self.process_detection()

        if self.control_running:
            self.run_control_step()

        self.root.after(self.update_interval, self.update)

    def process_detection(self):
        """处理图像检测"""
        try:
            # 处理一帧图像
            circles, success = self.detector.process_frame()

            if success:
                # 更新检测状态
                self.detected_circles_label.config(text=str(len(circles)))

                if circles:
                    # 取第一个检测到的圆
                    circle = circles[0]
                    dx_mm = circle['dx_mm']
                    dy_mm = circle['dy_mm']
                    depth_mm = circle['depth']
                    dz_mm = depth_mm

                    # 将偏差放入队列
                    self.deviation_queue.put((dx_mm, dy_mm, dz_mm))

                # 更新摄像头图像显示
                if hasattr(self.detector, 'color_image'):
                    self.update_camera_display(self.detector.color_image)

        except Exception as e:
            print(f"检测错误: {e}")

    def update_camera_display(self, cv_image):
        """更新摄像头图像显示"""
        try:
            # 调整图像大小
            h, w = cv_image.shape[:2]
            max_size = 400
            scale = min(max_size / w, max_size / h)
            resized = cv2.resize(cv_image, (int(w * scale), int(h * scale)))

            # 转换颜色空间
            rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(rgb_image)

            # 更新显示
            self.camera_image = ImageTk.PhotoImage(pil_image)
            self.camera_label.config(image=self.camera_image)
            self.camera_label.image = self.camera_image  # 保持引用

        except Exception as e:
            print(f"更新摄像头显示错误: {e}")

    def run_control_step(self):
        """运行单步控制"""
        # 计算新位置
        x, y, z = self.calculate_new_position()

        # 计算并发送角度
        if self.calculate_angles(x, y, z):
            self.send_angles()

    def start_detection(self):
        """启动图像检测"""
        try:
            from vision import RedCircleDetector
            self.detector = RedCircleDetector(width=640, height=480, fps=30)
            self.detection_running = True

            self.start_detection_btn.config(state=tk.DISABLED)
            self.stop_detection_btn.config(state=tk.NORMAL)
            self.detection_status_label.config(text="运行中", foreground="green")
            self.status_var.set("图像检测已启动")

        except Exception as e:
            messagebox.showerror("错误", f"启动检测失败: {str(e)}")

    def stop_detection(self):
        """停止图像检测"""
        self.detection_running = False
        if self.detector:
            self.detector.cleanup()
            self.detector = None

        self.start_detection_btn.config(state=tk.NORMAL)
        self.stop_detection_btn.config(state=tk.DISABLED)
        self.detection_status_label.config(text="未启动", foreground="red")
        self.status_var.set("图像检测已停止")

    def set_deviation(self, dx, dy, dz):
        """设置当前偏差值"""
        self.deviation_x = dx
        self.deviation_y = dy
        self.deviation_z = dz

        # 更新显示
        self.deviation_x_label.config(text=f"{dx:.2f}")
        self.deviation_y_label.config(text=f"{dy:.2f}")
        self.deviation_z_label.config(text=f"{dz:.2f}")

    def calculate_new_position(self):
        """根据偏差计算新位置"""
        # 检查是否有新的偏差数据
        try:
            if not self.deviation_queue.empty():
                dy, dz, dx = self.deviation_queue.get_nowait()
                self.set_deviation(dx, dy, dz)
        except queue.Empty:
            pass

        # 使用比例控制计算新位置
        new_x = self.deviation_x
        new_y = self.current_y + self.deviation_y * self.kp
        new_z = self.current_z + self.deviation_z * self.kp

        # 更新当前位置
        self.current_x = new_x
        self.current_y = new_y
        self.current_z = new_z

        # 更新显示
        self.current_x_label.config(text=f"{new_x:.2f}")
        self.current_y_label.config(text=f"{new_y:.2f}")
        self.current_z_label.config(text=f"{new_z:.2f}")

        return new_x, new_y, new_z

    def calculate_angles(self, x, y, z):
        """计算舵机角度"""
        if not self.dll:
            messagebox.showerror("错误", "求解器未加载成功")
            return False

        try:
            # 计算姿态角度 (YAW和PITCH)
            yaw = np.arctan(-y / x) * 180 / np.pi

            pitch = np.arctan(-z / x) * 180 / np.pi
            roll = 0.0  # 保持水平

            # 调用求解器
            angles = (c_double * 6)()
            ret = self.dll.InK6RSS(
                c_double(0), c_double(0), c_double(188),
                c_double(yaw), c_double(pitch), c_double(roll),
                pointer(angles)
            )

            # 更新角度显示
            self.angle_labels["YAW角度 (°):"].config(text=f"{yaw:.2f}")
            self.angle_labels["PITCH角度 (°):"].config(text=f"{pitch:.2f}")
            self.angle_labels["ROLL角度 (°):"].config(text=f"{roll:.2f}")

            # 检查结果
            if ret == 0 or any(angle == -360 for angle in angles):
                self.status_var.set("无法计算有效的舵机角度，可能超出工作范围")
                self.last_angles = None
                for label in self.servo_labels:
                    label.config(text="--")
                return False

            # 显示结果
            self.last_angles = [round(angle, 2) for angle in angles]
            for i, label in enumerate(self.servo_labels):
                label.config(text=str(self.last_angles[i]))

            return True

        except Exception as e:
            self.status_var.set(f"计算失败: {str(e)}")
            return False

    def send_angles(self):
        """发送角度到单片机"""
        if not self.is_connected or not self.serial_port or not self.serial_port.is_open:
            self.status_var.set("未建立串口连接")
            return False

        if not self.last_angles:
            self.status_var.set("无可用角度数据")
            return False

        try:
            # 格式化数据，格式: "S,angle1,angle2,angle3,angle4,angle5,angle6,E\n"
            data_str = f"S,{','.join(map(str, self.last_angles))},E\n"
            self.serial_port.write(data_str.encode('utf-8'))

            # 等待响应
            time.sleep(0.05)
            if self.serial_port.in_waiting > 0:
                response = self.serial_port.readline().decode('utf-8').strip()
                self.status_var.set(f"发送成功: {response}")
            else:
                self.status_var.set(f"已发送角度: {self.last_angles[0]}...")

            return True
        except Exception as e:
            self.status_var.set(f"发送失败: {str(e)}")
            self.toggle_connection()  # 发送失败时断开连接
            return False

    def start_control(self):
        """开始偏差控制"""
        if self.control_running:
            return

        # 检查必要条件
        if not self.dll:
            messagebox.showwarning("警告", "求解器未加载，无法开始控制")
            return

        if not self.is_connected or not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("警告", "请先建立串口连接")
            return

        # 更新参数
        self.update_parameters()

        # 初始化位置
        self.current_x = 0
        self.current_y = 0
        self.current_z = 0

        # 开始控制
        self.control_running = True
        self.start_control_btn.config(state=tk.DISABLED)
        self.stop_control_btn.config(state=tk.NORMAL)
        self.update_params_btn.config(state=tk.DISABLED)
        self.status_var.set("开始偏差控制...")

        # 启动控制循环
        self.run_control()

    def stop_control(self):
        """停止偏差控制"""
        self.control_running = False
        self.start_control_btn.config(state=tk.NORMAL)
        self.stop_control_btn.config(state=tk.DISABLED)
        self.update_params_btn.config(state=tk.NORMAL)
        self.status_var.set("已停止控制")

    def run_control(self):
        """运行控制循环"""
        if not self.control_running:
            return

        # 计算新位置
        x, y, z = self.calculate_new_position()

        # 计算并发送角度
        if self.calculate_angles(x, y, z):
            self.send_angles()
        


        # 继续控制循环
        self.root.after(100, self.run_control)  # 每100ms更新一次


if __name__ == "__main__":
    root = tk.Tk()
    app = RSS6RobotController(root)
    root.mainloop()