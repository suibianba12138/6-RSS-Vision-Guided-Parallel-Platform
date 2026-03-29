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
import pyrealsense2 as rs
from typing import List, Dict, Tuple, Optional


class RealSenseCamera:
    """RealSense相机封装类"""

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        self.width = width
        self.height = height
        self.fps = fps

        # 初始化管道和配置
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # 配置流
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        # 启动管道
        self.profile = self.pipeline.start(self.config)

        # 创建对齐对象
        self.align = rs.align(rs.stream.color)

        # 获取相机内参
        depth_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth))
        self.depth_intrinsics = depth_profile.get_intrinsics()

        # 设置深度传感器参数
        depth_sensor = self.profile.get_device().first_depth_sensor()
        if depth_sensor.supports(rs.option.visual_preset):
            depth_sensor.set_option(rs.option.visual_preset, 3)  # 高清预设

    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """获取对齐后的深度和彩色帧"""
        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                return None, None

            # 转换为numpy数组
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            return depth_image, color_image

        except Exception as e:
            print(f"获取帧数据错误: {e}")
            return None, None

    def stop(self):
        """停止相机"""
        self.pipeline.stop()


class DepthMaskProcessor:
    """深度掩码处理器"""

    def __init__(self):
        self.params = {
            'min_depth': 800,
            'max_depth': 940,
            'morph_size': 5,
            'blur_size': 5,
        }

    def create_mask(self, depth_image: np.ndarray) -> np.ndarray:
        """创建深度掩码"""
        # 创建深度掩码
        depth_mask = np.logical_and(
            depth_image >= self.params['min_depth'],
            depth_image <= self.params['max_depth']
        )
        depth_mask_8bit = np.uint8(depth_mask * 255)

        # 模糊处理
        blur_size = max(1, self.params['blur_size'] * 2 + 1)
        if blur_size > 1:
            depth_mask_8bit = cv2.GaussianBlur(depth_mask_8bit, (blur_size, blur_size), 0)

        # 形态学操作
        morph_size = max(1, self.params['morph_size'])
        if morph_size > 0:
            kernel = np.ones((morph_size, morph_size), np.uint8)
            depth_mask_8bit = cv2.morphologyEx(depth_mask_8bit, cv2.MORPH_CLOSE, kernel)
            depth_mask_8bit = cv2.morphologyEx(depth_mask_8bit, cv2.MORPH_OPEN, kernel)

        return depth_mask_8bit


class BlackRectangleDetector:
    """黑色矩形检测器"""

    def __init__(self, camera: RealSenseCamera):
        self.camera = camera
        self.depth_processor = DepthMaskProcessor()
        self.detection_enabled = False
        self.latest_detection = None
        self.detection_queue = queue.Queue()

        # 检测参数
        self.detection_params = {
            'h_low': 0, 'h_high': 180,
            's_low': 0, 's_high': 50,
            'v_low': 0, 'v_high': 50,
            'min_area': 100,
            'min_radius_mm': 150,
            'max_radius_mm': 200,
        }

        # 状态变量
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0
        self.detected_rectangles = []

    def enable_detection(self):
        """启用检测"""
        self.detection_enabled = True

    def disable_detection(self):
        """禁用检测"""
        self.detection_enabled = False

    def get_latest_detection(self):
        """获取最新检测结果"""
        try:
            return self.detection_queue.get_nowait()
        except queue.Empty:
            return self.latest_detection

    def detect_rectangles(self, color_image: np.ndarray, depth_image: np.ndarray) -> Tuple[
        List[Dict], np.ndarray, np.ndarray, np.ndarray]:
        """检测黑色矩形"""
        # 创建深度掩码
        depth_mask = self.depth_processor.create_mask(depth_image)

        # 应用深度掩码
        masked_color = cv2.bitwise_and(color_image, color_image, mask=depth_mask)

        # 转换为HSV并检测黑色
        hsv = cv2.cvtColor(masked_color, cv2.COLOR_BGR2HSV)
        lower_black = np.array([
            self.detection_params['h_low'],
            self.detection_params['s_low'],
            self.detection_params['v_low']
        ])
        upper_black = np.array([
            self.detection_params['h_high'],
            self.detection_params['s_high'],
            self.detection_params['v_high']
        ])
        black_mask = cv2.inRange(hsv, lower_black, upper_black)

        # 形态学操作
        morph_size = max(1, self.depth_processor.params['morph_size'])
        if morph_size > 0:
            kernel = np.ones((morph_size, morph_size), np.uint8)
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)

        # 查找轮廓并检测矩形
        contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected_rectangles = []
        result_image = color_image.copy()

        for contour in contours:
            rectangle = self._process_contour(contour, depth_image)
            if rectangle:
                detected_rectangles.append(rectangle)
                self._draw_rectangle_info(result_image, rectangle)

        return detected_rectangles, result_image, black_mask, depth_mask

    def _process_contour(self, contour: np.ndarray, depth_image: np.ndarray) -> Optional[Dict]:
        """处理单个轮廓，检测是否为矩形"""
        # 计算轮廓面积
        area = cv2.contourArea(contour)
        if area < self.detection_params['min_area']:
            return None

        # 轮廓近似
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # 检查是否为四边形
        if len(approx) != 4:
            return None

        # 计算最小外接矩形
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        # 计算中心点
        center = np.mean(box, axis=0).astype(int)
        center = (center[0], center[1])

        # 获取深度值
        if not (0 <= center[1] < depth_image.shape[0] and 0 <= center[0] < depth_image.shape[1]):
            return None

        depth_value = depth_image[center[1], center[0]]
        if depth_value == 0:
            return None

        # 计算物理尺寸
        width_px = np.linalg.norm(box[0] - box[1])
        height_px = np.linalg.norm(box[1] - box[2])
        rect_size_px = max(width_px, height_px)
        physical_size = rect_size_px * depth_value / self.camera.depth_intrinsics.fx

        # 检查尺寸范围
        if not (self.detection_params['min_radius_mm'] <= physical_size <= self.detection_params['max_radius_mm']):
            return None

        # 计算与相机中心的距离
        camera_center = (self.camera.width // 2, self.camera.height // 2)
        dx = center[0] - camera_center[0]
        dy = center[1] - camera_center[1]
        dx_mm = (dx * depth_value) / self.camera.depth_intrinsics.fx
        dy_mm = (dy * depth_value) / self.camera.depth_intrinsics.fy

        return {
            'center': center,
            'corners': box,
            'physical_size': physical_size,
            'depth': depth_value,
            'dx_mm': dx_mm,
            'dy_mm': dy_mm
        }

    def _draw_rectangle_info(self, image: np.ndarray, rectangle: Dict):
        """在图像上绘制矩形信息"""
        center = rectangle['center']
        corners = rectangle['corners']
        depth_value = rectangle['depth']
        physical_size = rectangle['physical_size']

        # 绘制矩形
        cv2.drawContours(image, [corners], 0, (0, 255, 0), 2)
        cv2.circle(image, center, 3, (0, 0, 255), -1)

        # 显示信息
        info_text = f"Size: {physical_size / 10:.1f}cm, Depth: {depth_value / 10:.1f}cm"
        cv2.putText(image, info_text, (center[0] - 50, center[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    def run_detection_loop(self):
        """运行检测循环"""
        try:
            while self.detection_enabled:
                # 获取帧数据
                depth_image, color_image = self.camera.get_frames()
                if depth_image is None or color_image is None:
                    continue

                # 检测矩形
                detected_rectangles, result_image, black_mask, depth_mask = self.detect_rectangles(
                    color_image, depth_image)

                # 更新检测结果
                if detected_rectangles:
                    self.latest_detection = detected_rectangles[0]  # 取第一个检测到的矩形
                    try:
                        self.detection_queue.put_nowait(self.latest_detection)
                    except queue.Full:
                        pass  # 如果队列满了，跳过

                # 显示结果（可选）
                cv2.imshow('Detection Result', result_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except Exception as e:
            print(f"检测循环错误: {e}")

    def start_detection_thread(self):
        """启动检测线程"""
        self.detection_enabled = True
        detection_thread = threading.Thread(target=self.run_detection_loop)
        detection_thread.daemon = True
        detection_thread.start()

    def stop_detection(self):
        """停止检测"""
        self.detection_enabled = False
        cv2.destroyAllWindows()


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
        self.camera = RealSenseCamera()
        self.detector = BlackRectangleDetector(self.camera)
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
        vertices = [
            (-half_side, -half_side),  # 左下
            (-half_side, half_side),  # 左上
            (half_side, half_side),  # 右上
            (half_side, -half_side)  # 右下
        ]

        # 生成轨迹点（每边分成若干段）
        segments_per_side = 10

        # 从中心移动到第一个顶点
        self.square_path.append((0, 0,0))
        for i in range(100):
            self.square_path.append((0, -half_side, (-half_side+(i*half_side/50))))
        for i in range(100):
            self.square_path.append((0, (-half_side+(i*half_side/50)), half_side))
        for i in range(100):
            self.square_path.append((0,half_side,(half_side-(i*half_side/50))))
        for i in range(100):
            self.square_path.append((0,(half_side-(i*half_side/50)),-half_side))
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
        vision_frame = ttk.LabelFrame(main_frame, text="视觉检测", padding="10")
        vision_frame.pack(fill=tk.X, pady=(0, 10))

        self.start_vision_btn = ttk.Button(vision_frame, text="开始视觉检测",
                                           command=self.toggle_vision)
        self.start_vision_btn.pack(side=tk.LEFT, padx=5)

        self.detect_btn = ttk.Button(vision_frame, text="获取正方形参数",
                                     command=self.get_square_parameters, state=tk.DISABLED)
        self.detect_btn.pack(side=tk.LEFT, padx=5)

        # 检测结果显示
        self.detection_var = tk.StringVar(value="未检测")
        ttk.Label(vision_frame, textvariable=self.detection_var).pack(side=tk.LEFT, padx=20)

        # 轨迹参数区域
        traj_frame = ttk.LabelFrame(main_frame, text="正方形轨迹参数", padding="10")
        traj_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(traj_frame, text="边长 (mm):").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.side_var = tk.StringVar(value="0.0")
        side_label = ttk.Label(traj_frame, textvariable=self.side_var, width=10)
        side_label.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(traj_frame, text="深度 (mm):").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.depth_var = tk.StringVar(value="0.0")
        depth_label = ttk.Label(traj_frame, textvariable=self.depth_var, width=10)
        depth_label.grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(traj_frame, text="速度 (ms/点):").grid(row=0, column=4, padx=5, pady=5, sticky=tk.W)
        self.speed_entry = ttk.Entry(traj_frame, width=10)
        self.speed_entry.grid(row=0, column=5, padx=5, pady=5)
        self.speed_entry.insert(0, str(self.speed))

        # 当前位置显示区域
        pos_frame = ttk.LabelFrame(main_frame, text="当前位置", padding="10")
        pos_frame.pack(fill=tk.X, pady=(0, 10))

        self.pos_labels = {}
        coords = ["X (mm):", "Y (mm):", "Z (mm):", "进度:"]
        for i, coord in enumerate(coords):
            ttk.Label(pos_frame, text=coord).grid(row=0, column=i * 2, padx=15, pady=5, sticky=tk.W)
            self.pos_labels[coord] = ttk.Label(pos_frame, text="0.00", width=10)
            self.pos_labels[coord].grid(row=0, column=i * 2 + 1, padx=5, pady=5)

        # 舵机角度显示区域
        servo_frame = ttk.LabelFrame(main_frame, text="舵机角度", padding="10")
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
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=(0, 10))

        self.start_btn = ttk.Button(control_frame, text="开始轨迹", command=self.start_trajectory, state=tk.DISABLED)
        self.start_btn.pack(side=tk.LEFT, padx=5)

        self.stop_btn = ttk.Button(control_frame, text="停止轨迹", command=self.stop_trajectory, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=5)

        self.reset_btn = ttk.Button(control_frame, text="重置轨迹", command=self.reset_trajectory)
        self.reset_btn.pack(side=tk.LEFT, padx=5)

        # 串口设置区域
        serial_frame = ttk.LabelFrame(main_frame, text="串口设置", padding="10")
        serial_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(serial_frame, text="串口号:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.port_var = tk.StringVar()
        port_combo = ttk.Combobox(serial_frame, textvariable=self.port_var, width=10)
        port_combo.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(serial_frame, text="波特率:").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.baud_var = tk.StringVar(value="115200")
        baud_combo = ttk.Combobox(serial_frame, textvariable=self.baud_var, width=10,
                                  values=["9600", "19200", "38400", "57600", "115200"])
        baud_combo.grid(row=0, column=3, padx=5, pady=5)

        refresh_btn = ttk.Button(serial_frame, text="刷新", command=self.refresh_ports)
        refresh_btn.grid(row=0, column=4, padx=5, pady=5)

        self.connect_btn = ttk.Button(serial_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=5, pady=5)

        # 状态区域
        self.status_var = tk.StringVar(value="就绪")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

        self.refresh_ports()

        self.last_angles = None

    def toggle_vision(self):
        """切换视觉检测状态"""
        if self.vision_enabled:
            self.detector.stop_detection()
            self.vision_enabled = False
            self.start_vision_btn.config(text="开始视觉检测")
            self.detect_btn.config(state=tk.DISABLED)
            self.status_var.set("视觉检测已停止")
        else:
            self.detector.start_detection_thread()
            self.vision_enabled = True
            self.start_vision_btn.config(text="停止视觉检测")
            self.detect_btn.config(state=tk.NORMAL)
            self.status_var.set("视觉检测已启动")

    def get_square_parameters(self):
        """通过视觉检测获取正方形参数"""
        if not self.vision_enabled:
            messagebox.showwarning("警告", "请先启动视觉检测")
            return

        detection = self.detector.get_latest_detection()
        if detection:
            self.square_side = detection['physical_size']  # 获取边长
            self.center_x = detection['depth']  # 获取深度

            self.side_var.set(f"{self.square_side:.1f}")
            self.depth_var.set(f"{self.center_z:.1f}")

            self.detection_var.set(f"边长: {self.square_side:.1f}mm, 深度: {self.center_z:.1f}mm")
            self.status_var.set("正方形参数获取成功")

            # 启用轨迹开始按钮
            self.start_btn.config(state=tk.NORMAL)
        else:
            self.status_var.set("未检测到正方形，请调整相机位置")

    def refresh_ports(self):
        """刷新串口列表"""
        try:
            ports = serial.tools.list_ports.comports()
            port_combo = self.root.nametowidget(".!frame.!labelframe4.!combobox")
            port_combo['values'] = [port.device for port in ports]
            if ports:
                self.port_var.set(ports[0].device)
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
            yaw = np.arctan(-y /self.center_x) * 180 / np.pi
            pitch = np.arctan(z /self.center_x) * 180 / np.pi
            roll = 0.0
            angles = (c_double * 6)()
            ret = self.dll.InK6RSS(
                c_double(0), c_double(0), c_double(188),
                c_double(yaw), c_double(pitch), c_double(roll),
                pointer(angles)
            )

            self.pos_labels["X (mm):"].config(text=f"{x:.2f}")
            self.pos_labels["Y (mm):"].config(text=f"{y:.2f}")
            self.pos_labels["Z (mm):"].config(text=f"{z:.2f}")

            if ret == 0 or any(angle == -360 for angle in angles):
                self.status_var.set("角度计算失败")
                self.last_angles = None
                for label in self.servo_labels:
                    label.config(text="--")
                return False

            self.last_angles = [round(angle, 2) for angle in angles]
            for i, label in enumerate(self.servo_labels):
                label.config(text=str(self.last_angles[i]))

            return True
        except Exception as e:
            self.status_var.set(f"计算失败: {str(e)}")
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
            self.status_var.set(f"已发送: 步进 {self.current_step}/{self.total_steps}")
            return True
        except Exception as e:
            self.status_var.set(f"发送失败: {str(e)}")
            return False

    def update_position_display(self):
        """更新位置显示"""
        progress = (self.current_step / self.total_steps * 100) if self.total_steps > 0 else 0
        self.pos_labels["进度:"].config(text=f"{progress:.1f}%")

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
            self.status_var.set("正方形轨迹完成!")
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
            self.detector.stop_detection()
        if self.camera:
            self.camera.stop()
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