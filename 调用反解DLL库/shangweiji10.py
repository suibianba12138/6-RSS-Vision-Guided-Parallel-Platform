import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
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
import re


class FunctionParser:
    """数学函数解析器"""

    def __init__(self):
        self.supported_functions = {
            'linear', 'quadratic', 'sin', 'cos', 'circle', 'ellipse', 'spiral'
        }

    def parse_function(self, function_str, x_range=(-50, 50), resolution=100):
        """解析数学函数并生成坐标点"""
        try:
            function_str = function_str.strip().lower()

            if function_str.startswith('y='):
                return self._parse_explicit_function(function_str[2:], x_range, resolution)
            elif function_str.startswith('x='):
                return self._parse_parametric_x(function_str[2:], x_range, resolution)
            elif 'x^2' in function_str or 'y^2' in function_str:
                return self._parse_implicit_function(function_str, x_range, resolution)
            else:
                return self._parse_named_function(function_str, x_range, resolution)

        except Exception as e:
            raise ValueError(f"函数解析错误: {str(e)}")

    def _parse_explicit_function(self, expr, x_range, resolution):
        """解析显式函数 y = f(x)"""
        points = []
        x_min, x_max = x_range
        step = (x_max - x_min) / resolution

        for i in range(resolution + 1):
            x = x_min + i * step
            try:
                y = self._safe_eval(expr, x)
                if not (math.isnan(y) or math.isinf(y)):
                    points.append((x, y))
            except:
                continue

        return points

    def _parse_parametric_x(self, expr, y_range, resolution):
        """解析参数方程 x = f(y)"""
        points = []
        y_min, y_max = y_range
        step = (y_max - y_min) / resolution

        for i in range(resolution + 1):
            y = y_min + i * step
            try:
                x = self._safe_eval(expr, y)
                if not (math.isnan(x) or math.isinf(x)):
                    points.append((x, y))
            except:
                continue

        return points

    def _parse_implicit_function(self, expr, x_range, resolution):
        """解析隐函数"""
        points = []
        x_min, x_max = x_range

        step = (x_max - x_min) / resolution

        for i in range(resolution + 1):
            x = x_min + i * step
            try:
                if 'x^2 + y^2' in expr:
                    match = re.search(r'x\^2\s*\+\s*y\^2\s*=\s*(\d+)', expr)
                    if match:
                        r = math.sqrt(float(match.group(1)))
                        y1 = math.sqrt(max(0, r ** 2 - x ** 2))
                        y2 = -math.sqrt(max(0, r ** 2 - x ** 2))
                        if not math.isnan(y1):
                            points.append((x, y1))
                        if not math.isnan(y2):
                            points.append((x, y2))
            except:
                continue

        return points

    def _parse_named_function(self, func_name, x_range, resolution):
        """解析命名函数"""
        func_name = func_name.lower()
        points = []
        x_min, x_max = x_range
        step = (x_max - x_min) / resolution

        for i in range(resolution + 1):
            x = x_min + i * step

            if func_name == 'sin':
                y = math.sin(x * 0.1) * 10  # 调整幅度
            elif func_name == 'cos':
                y = math.cos(x * 0.1) * 10
            elif func_name == 'quadratic':
                y = (x * 0.1)  ** 2
            elif func_name == 'linear':
                y = x * 0.2
            elif func_name == 'circle':
                if -5 <= x <= 5:
                    y1 = math.sqrt(25 - x ** 2)
                    y2 = -math.sqrt(25 - x ** 2)
                    points.append((x, y1))
                    points.append((x, y2))
            else:
                y = x * 0.2
                points.append((x, y))

            if func_name != 'circle':
                points.append((x, y))

        return points

    def _safe_eval(self, expr, x):
        """安全地计算数学表达式"""
        expr = expr.replace('^', '**')
        expr = expr.replace('sin', 'math.sin')
        expr = expr.replace('cos', 'math.cos')
        expr = expr.replace('tan', 'math.tan')
        expr = expr.replace('sqrt', 'math.sqrt')
        expr = expr.replace('log', 'math.log')
        expr = expr.replace('exp', 'math.exp')

        safe_dict = {
            'math': math,
            'x': x,
            'sin': math.sin,
            'cos': math.cos,
            'tan': math.tan,
            'sqrt': math.sqrt,
            'log': math.log,
            'exp': math.exp,
            '__builtins__': {}
        }

        return eval(expr, safe_dict)


class TrajectoryPlanner:
    """轨迹规划器"""

    def __init__(self):
        self.trajectory_points = []
        self.current_index = 0
        self.origin_offset = (0, 0)

    def set_origin_offset(self, dx, dy):
        """设置原点偏移"""
        self.origin_offset = (dx, dy)

    def plan_trajectory(self, function_points, scale=1.0):
        """规划机器人运动轨迹"""
        if not function_points:
            return []

        scaled_points = []
        for x, y in function_points:
            scaled_x = (x * scale) + self.origin_offset[0]
            scaled_y = (y * scale) + self.origin_offset[1]
            scaled_points.append((scaled_x, scaled_y))

        smoothed_points = self._smooth_trajectory(scaled_points)
        self.trajectory_points = smoothed_points
        self.current_index = 0
        return smoothed_points

    def _smooth_trajectory(self, points, window_size=3):
        """轨迹平滑处理"""
        if len(points) <= window_size:
            return points

        smoothed = []
        for i in range(len(points)):
            start = max(0, i - window_size // 2)
            end = min(len(points), i + window_size // 2 + 1)

            window_points = points[start:end]
            avg_x = sum(p[0] for p in window_points) / len(window_points)
            avg_y = sum(p[1] for p in window_points) / len(window_points)

            smoothed.append((avg_x, avg_y))

        return smoothed

    def get_next_point(self):
        """获取下一个轨迹点"""
        if self.current_index < len(self.trajectory_points):
            point = self.trajectory_points[self.current_index]
            self.current_index += 1
            return point
        return None

    def get_progress(self):
        """获取绘制进度"""
        if not self.trajectory_points:
            return 0
        return self.current_index / len(self.trajectory_points) * 100


class AdvancedFunctionPlotter:
    """高级函数绘图系统主控制器"""

    def __init__(self, root):
        self.root = root
        self.root.title("6RSS并联机器人 - 高级函数绘图系统")
        self.root.geometry("1200x800")

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

        # 摄像头图像显示相关
        self.camera_frame = None
        self.camera_label = None
        self.camera_image = None

        # 函数绘图相关
        self.function_parser = FunctionParser()
        self.trajectory_planner = TrajectoryPlanner()
        self.plotting_active = False
        self.origin_calibrated = False
        self.origin_offset = (0, 0)
        self.current_trajectory = []

        # 创建UI
        self.create_widgets()

        # 主循环更新
        self.update_interval = 50
        self.root.after(self.update_interval, self.update)

        # 启动绘图监控线程
        self.plot_monitor_thread = threading.Thread(target=self._plot_monitor, daemon=True)
        self.plot_monitor_thread.start()

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

        # === 左侧：基础控制区域 ===
        # 控制参数设置区域
        control_frame = ttk.LabelFrame(left_frame, text="控制参数", padding="10")
        control_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(control_frame, text="比例系数 Kp:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.kp_entry = ttk.Entry(control_frame, width=10)
        self.kp_entry.grid(row=0, column=1, padx=5, pady=5)
        self.kp_entry.insert(0, str(self.kp))

        # 当前位置显示区域
        pos_frame = ttk.LabelFrame(left_frame, text="当前位置和偏差", padding="10")
        pos_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(pos_frame, text="当前 X (mm):").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.current_x_label = ttk.Label(pos_frame, text="0.00", width=10)
        self.current_x_label.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(pos_frame, text="当前 Y (mm):").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.current_y_label = ttk.Label(pos_frame, text="0.00", width=10)
        self.current_y_label.grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(pos_frame, text="当前 Z (mm):").grid(row=0, column=4, padx=5, pady=5, sticky=tk.W)
        self.current_z_label = ttk.Label(pos_frame, text="188.00", width=10)
        self.current_z_label.grid(row=0, column=5, padx=5, pady=5)

        # 检测状态显示
        ttk.Label(pos_frame, text="检测状态:").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.detection_status_label = ttk.Label(pos_frame, text="未启动", foreground="red")
        self.detection_status_label.grid(row=1, column=1, padx=5, pady=5)

        ttk.Label(pos_frame, text="检测到圆:").grid(row=1, column=2, padx=5, pady=5, sticky=tk.W)
        self.detected_circles_label = ttk.Label(pos_frame, text="0", width=10)
        self.detected_circles_label.grid(row=1, column=3, padx=5, pady=5)

        # 姿态角度显示区域
        angle_frame = ttk.LabelFrame(left_frame, text="姿态角度", padding="10")
        angle_frame.pack(fill=tk.X, pady=(0, 10))

        self.angle_labels = {}
        angle_names = ["YAW角度 (°):", "PITCH角度 (°):", "ROLL角度 (°):"]
        for i, name in enumerate(angle_names):
            ttk.Label(angle_frame, text=name).grid(row=0, column=i * 2, padx=15, pady=5, sticky=tk.W)
            self.angle_labels[name] = ttk.Label(angle_frame, text="0.00", width=10)
            self.angle_labels[name].grid(row=0, column=i * 2 + 1, padx=5, pady=5)

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

        # 基础控制按钮区域
        base_btn_frame = ttk.Frame(left_frame)
        base_btn_frame.pack(fill=tk.X, pady=(0, 10))

        self.start_detection_btn = ttk.Button(base_btn_frame, text="启动检测", command=self.start_detection)
        self.start_detection_btn.pack(side=tk.LEFT, padx=5)

        self.stop_detection_btn = ttk.Button(base_btn_frame, text="停止检测", command=self.stop_detection,
                                             state=tk.DISABLED)
        self.stop_detection_btn.pack(side=tk.LEFT, padx=5)

        self.update_params_btn = ttk.Button(base_btn_frame, text="更新参数", command=self.update_parameters)
        self.update_params_btn.pack(side=tk.LEFT, padx=5)

        # 串口设置区域
        serial_frame = ttk.LabelFrame(left_frame, text="串口设置", padding="10")
        serial_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(serial_frame, text="串口号:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(serial_frame, textvariable=self.port_var, width=10)
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(serial_frame, text="波特率:").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.baud_var = tk.StringVar(value="115200")
        baud_combo = ttk.Combobox(serial_frame, textvariable=self.baud_var, width=10,
                                  values=["9600", "19200", "38400", "57600", "115200"])
        baud_combo.grid(row=0, column=3, padx=5, pady=5)

        refresh_btn = ttk.Button(serial_frame, text="刷新", command=self.refresh_ports)
        refresh_btn.grid(row=0, column=4, padx=5, pady=5)

        self.connect_btn = ttk.Button(serial_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=5, pady=5)

        # === 右侧：高级函数绘图区域 ===
        advanced_frame = ttk.LabelFrame(right_frame, text="高级函数绘图", padding="10")
        advanced_frame.pack(fill=tk.BOTH, expand=True)

        # 函数输入区域
        func_frame = ttk.Frame(advanced_frame)
        func_frame.pack(fill=tk.X, pady=5)

        ttk.Label(func_frame, text="数学函数:").pack(side=tk.LEFT, padx=5)
        self.function_entry = ttk.Entry(func_frame, width=30)
        self.function_entry.pack(side=tk.LEFT, padx=5)
        self.function_entry.insert(0, "y=sin(x)")

        # 参数设置区域
        param_frame = ttk.Frame(advanced_frame)
        param_frame.pack(fill=tk.X, pady=5)

        ttk.Label(param_frame, text="X范围:").pack(side=tk.LEFT, padx=5)
        self.x_min_entry = ttk.Entry(param_frame, width=8)
        self.x_min_entry.pack(side=tk.LEFT, padx=2)
        self.x_min_entry.insert(0, "-30")

        ttk.Label(param_frame, text="到").pack(side=tk.LEFT, padx=2)
        self.x_max_entry = ttk.Entry(param_frame, width=8)
        self.x_max_entry.pack(side=tk.LEFT, padx=2)
        self.x_max_entry.insert(0, "30")

        ttk.Label(param_frame, text="缩放:").pack(side=tk.LEFT, padx=(20, 5))
        self.scale_entry = ttk.Entry(param_frame, width=8)
        self.scale_entry.pack(side=tk.LEFT, padx=2)
        self.scale_entry.insert(0, "1.0")

        ttk.Label(param_frame, text="分辨率:").pack(side=tk.LEFT, padx=(20, 5))
        self.resolution_entry = ttk.Entry(param_frame, width=8)
        self.resolution_entry.pack(side=tk.LEFT, padx=2)
        self.resolution_entry.insert(0, "100")

        # 原点校准区域
        origin_frame = ttk.Frame(advanced_frame)
        origin_frame.pack(fill=tk.X, pady=5)

        self.calibrate_origin_btn = ttk.Button(origin_frame, text="校准原点", command=self.calibrate_origin)
        self.calibrate_origin_btn.pack(side=tk.LEFT, padx=5)

        self.origin_status_label = ttk.Label(origin_frame, text="未校准", foreground="red")
        self.origin_status_label.pack(side=tk.LEFT, padx=5)

        ttk.Label(origin_frame, text="原点偏移:").pack(side=tk.LEFT, padx=(20, 5))
        self.origin_offset_label = ttk.Label(origin_frame, text="(0.0, 0.0)")
        self.origin_offset_label.pack(side=tk.LEFT, padx=5)

        # 绘图控制区域
        plot_frame = ttk.Frame(advanced_frame)
        plot_frame.pack(fill=tk.X, pady=5)

        self.preview_btn = ttk.Button(plot_frame, text="预览轨迹", command=self.preview_trajectory)
        self.preview_btn.pack(side=tk.LEFT, padx=5)

        self.start_plot_btn = ttk.Button(plot_frame, text="开始绘图", command=self.start_plotting)
        self.start_plot_btn.pack(side=tk.LEFT, padx=5)

        self.stop_plot_btn = ttk.Button(plot_frame, text="停止绘图", command=self.stop_plotting, state=tk.DISABLED)
        self.stop_plot_btn.pack(side=tk.LEFT, padx=5)

        self.progress_label = ttk.Label(plot_frame, text="进度: 0%")
        self.progress_label.pack(side=tk.LEFT, padx=20)

        # 轨迹显示区域
        trajectory_frame = ttk.LabelFrame(advanced_frame, text="轨迹预览", padding="5")
        trajectory_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.trajectory_text = scrolledtext.ScrolledText(trajectory_frame, height=8, width=50)
        self.trajectory_text.pack(fill=tk.BOTH, expand=True)

        # 摄像头图像显示区域
        camera_frame = ttk.LabelFrame(right_frame, text="摄像头图像", padding="10")
        camera_frame.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        self.camera_label = ttk.Label(camera_frame)
        self.camera_label.pack(fill=tk.BOTH, expand=True)

        # 状态栏
        self.status_var = tk.StringVar(value="就绪")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

        # 初始刷新串口列表
        self.refresh_ports()

        self.last_angles = None

    def refresh_ports(self):
        """刷新可用串口号列表"""
        try:
            ports = serial.tools.list_ports.comports()
            port_list = [port.device for port in ports]
            self.port_combo['values'] = port_list

            if port_list:
                self.port_var.set(port_list[0])
                self.status_var.set(f"发现 {len(port_list)} 个串口")
            else:
                self.status_var.set("未检测到可用串口")
        except Exception as e:
            messagebox.showerror("错误", f"刷新串口失败: {str(e)}")

    def toggle_connection(self):
        """切换串口连接状态"""
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
        self.root.after(self.update_interval, self.update)

    def process_detection(self):
        """处理图像检测"""
        try:
            circles, success = self.detector.process_frame()

            if success:
                self.detected_circles_label.config(text=str(len(circles)))

                if circles:
                    circle = circles[0]
                    dx_mm = circle['dx_mm']
                    dy_mm = circle['dy_mm']
                    depth_mm = circle['depth']

                    self.deviation_queue.put((dx_mm, dy_mm, depth_mm))

                if hasattr(self.detector, 'color_image'):
                    self.update_camera_display(self.detector.color_image)

        except Exception as e:
            print(f"检测错误: {e}")

    def update_camera_display(self, cv_image):
        """更新摄像头图像显示"""
        try:
            h, w = cv_image.shape[:2]
            max_size = 400
            scale = min(max_size / w, max_size / h)
            resized = cv2.resize(cv_image, (int(w * scale), int(h * scale)))

            rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(rgb_image)

            self.camera_image = ImageTk.PhotoImage(pil_image)
            self.camera_label.config(image=self.camera_image)
            self.camera_label.image = self.camera_image

        except Exception as e:
            print(f"更新摄像头显示错误: {e}")

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

    def calculate_angles(self, x, y, z):
            """计算舵机角度"""
            if not self.dll:
                return False

            try:
                # 计算姿态角度
                yaw = math.atan2(-y, x) * 180 / math.pi if x != 0 else 0
                pitch = math.atan2(-z, math.sqrt(x ** 2 + y ** 2)) * 180 / math.pi if (x != 0 or y != 0) else 0
                roll = 0.0

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

                if ret == 0 or any(angle == -360 for angle in angles):
                    self.status_var.set("角度计算失败，可能超出工作空间")
                    self.last_angles = None
                    return False

                # 保存角度结果
                self.last_angles = [round(angle, 2) for angle in angles]
                for i, label in enumerate(self.servo_labels):
                    label.config(text=str(self.last_angles[i]))

                return True

            except Exception as e:
                self.status_var.set(f"角度计算错误: {str(e)}")
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
                # 格式化数据
                data_str = f"S,{','.join(map(str, self.last_angles))},E\n"
                self.serial_port.write(data_str.encode('utf-8'))

                time.sleep(0.05)
                if self.serial_port.in_waiting > 0:
                    response = self.serial_port.readline().decode('utf-8').strip()
                    self.status_var.set(f"发送成功: {response}")
                else:
                    self.status_var.set(f"已发送角度")

                return True
            except Exception as e:
                self.status_var.set(f"发送失败: {str(e)}")
                self.toggle_connection()
                return False

        # === 高级函数绘图方法 ===
    def calibrate_origin(self):
            """校准红色原点位置"""
            if not self.detection_running or not self.detector:
                messagebox.showwarning("警告", "请先启动视觉检测")
                return

            try:
                # 获取当前检测到的圆点
                circles, success = self.detector.process_frame()

                if success and circles:
                    # 取第一个检测到的圆作为原点
                    circle = circles[0]
                    dx_mm = circle['dx_mm']
                    dy_mm = circle['dy_mm']
                    self.current_x=circle['depth']

                    self.origin_offset = (dx_mm, dy_mm)
                    self.trajectory_planner.set_origin_offset(dx_mm, dy_mm)

                    self.origin_calibrated = True
                    self.origin_status_label.config(text="已校准", foreground="green")
                    self.origin_offset_label.config(text=f"({dx_mm:.1f}, {dy_mm:.1f})")

                    self.status_var.set(f"原点校准完成: 偏移({dx_mm:.1f}, {dy_mm:.1f})mm")
                else:
                    messagebox.showwarning("警告", "未检测到红色原点，请确保红色原点在视野内")

            except Exception as e:
                messagebox.showerror("错误", f"原点校准失败: {str(e)}")

    def preview_trajectory(self):
            """预览生成的轨迹"""
            if not self.origin_calibrated:
                messagebox.showwarning("警告", "请先校准原点")
                return

            try:
                # 获取输入参数
                function_str = self.function_entry.get()
                x_min = float(self.x_min_entry.get())
                x_max = float(self.x_max_entry.get())
                scale = float(self.scale_entry.get())
                resolution = int(self.resolution_entry.get())

                # 生成轨迹点
                points = self.function_parser.parse_function(
                    function_str, (x_min, x_max), resolution=resolution
                )

                if not points:
                    messagebox.showwarning("警告", "无法生成轨迹点，请检查函数表达式")
                    return

                # 规划轨迹
                trajectory = self.trajectory_planner.plan_trajectory(points, scale)
                self.current_trajectory = trajectory

                # 显示轨迹信息
                self.trajectory_text.delete(1.0, tk.END)
                self.trajectory_text.insert(tk.END, f"函数: {function_str}\n")
                self.trajectory_text.insert(tk.END, f"轨迹点数: {len(trajectory)}\n")
                self.trajectory_text.insert(tk.END, f"范围: X[{x_min}, {x_max}], 缩放: {scale}\n\n")
                self.trajectory_text.insert(tk.END, "前10个点:\n")

                for i, (x, y) in enumerate(trajectory[:10]):
                    self.trajectory_text.insert(tk.END, f"点{i + 1}: ({x:.1f}, {y:.1f})\n")

                self.status_var.set(f"轨迹生成完成: {len(trajectory)}个点")

            except Exception as e:
                messagebox.showerror("错误", f"轨迹预览失败: {str(e)}")

    def start_plotting(self):
            """开始绘图"""
            if not self.origin_calibrated:
                messagebox.showwarning("警告", "请先校准原点")
                return

            if not self.current_trajectory:
                messagebox.showwarning("警告", "请先生成轨迹预览")
                return

            if not self.is_connected:
                messagebox.showwarning("警告", "请先连接串口")
                return

            try:
                self.plotting_active = True
                self.start_plot_btn.config(state=tk.DISABLED)
                self.stop_plot_btn.config(state=tk.NORMAL)
                self.preview_btn.config(state=tk.DISABLED)

                # 重置轨迹索引
                self.trajectory_planner.current_index = 0

                self.status_var.set("开始绘图...")

            except Exception as e:
                messagebox.showerror("错误", f"开始绘图失败: {str(e)}")

    def stop_plotting(self):
            """停止绘图"""
            self.plotting_active = False
            self.start_plot_btn.config(state=tk.NORMAL)
            self.stop_plot_btn.config(state=tk.DISABLED)
            self.preview_btn.config(state=tk.NORMAL)
            self.status_var.set("绘图已停止")

    def _plot_monitor(self):
            """绘图监控线程"""
            while True:
                if self.plotting_active and self.origin_calibrated:
                    try:
                        # 获取下一个轨迹点
                        point = self.trajectory_planner.get_next_point()

                        if point is None:
                            # 绘图完成
                            self.stop_plotting()
                            self.status_var.set("绘图完成")
                            continue

                        y, z = point
                        x =self.current_x

                        # 更新当前位置显示
                        self.current_x = x
                        self.current_y = y
                        self.current_z = z
                        self.current_x_label.config(text=f"{x:.2f}")
                        self.current_y_label.config(text=f"{y:.2f}")
                        self.current_z_label.config(text=f"{z:.2f}")

                        # 计算机器人角度并发送
                        if self.calculate_angles(x, y, z):
                            self.send_angles()

                        # 更新进度显示
                        progress = self.trajectory_planner.get_progress()
                        self.progress_label.config(text=f"进度: {progress:.1f}%")

                        # 控制绘图速度
                        time.sleep(0.2)  # 200ms间隔，保证稳定性

                    except Exception as e:
                        print(f"绘图错误: {e}")
                        self.stop_plotting()
                        self.status_var.set(f"绘图错误: {e}")

                time.sleep(0.05)  # 线程休眠



    def run_control(self):
            """运行控制循环"""
            if not self.control_running:
                return

            self.run_control_step()
            self.root.after(100, self.run_control)  # 每100ms更新一次

    def update(self):
            """主循环更新函数"""
            if self.detection_running and self.detector:
                self.process_detection()

            if self.control_running and not self.plotting_active:
                self.run_control_step()

            self.root.after(self.update_interval, self.update)

    def on_closing(self):
            """程序关闭时的清理工作"""
            self.plotting_active = False
            self.control_running = False
            self.detection_running = False

            if self.detector:
                self.detector.cleanup()

            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()

            self.root.destroy()



if __name__ == "__main__":
    root = tk.Tk()
    root.title("6RSS并联机器人 - 高级函数绘图系统")
    root.geometry("1200x800")

    app = AdvancedFunctionPlotter(root)

    # 设置关闭事件
    root.protocol("WM_DELETE_WINDOW", app.on_closing)

    root.mainloop()