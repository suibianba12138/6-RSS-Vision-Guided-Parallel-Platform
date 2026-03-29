import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
from ctypes import CDLL, c_double, pointer, POINTER, c_int
import time
import math
import numpy as np


class RSS6RobotController:
    def __init__(self, root):
        self.root = root
        self.root.title("6RSS并联机器人控制器 - 正方形轨迹")
        self.root.geometry("700x550")
        self.root.resizable(False, False)

        # 加载DLL求解器
        self.dll = None
        self.load_solver()

        # 串口相关变量
        self.serial_port = None
        self.is_connected = False

        # 轨迹控制变量
        self.trajectory_running = False
        self.current_step = 0  # 当前步骤
        self.total_steps = 0  # 总步骤数
        self.square_side = 40  # 正方形边长
        self.center_x = 0  # 中心X坐标
        self.center_y = 0  # 中心Y坐标
        self.center_z = 188  # 中心Z坐标
        self.speed = 5  # 运动速度(度/步)

        # 正方形轨迹参数
        self.square_path = []  # 存储正方形轨迹点
        self.current_position = (0, 0, 188)  # 当前位置
        self.trajectory_completed = False  # 轨迹是否完成

        # 创建UI
        self.create_widgets()
        self.generate_square_path()  # 生成正方形轨迹

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

    def generate_square_path(self):
        """生成正方形轨迹点"""
        self.square_path = []

        # 正方形四个顶点（相对于中心）
        half_side = self.square_side / 2
        vertices = [
            (-half_side, -half_side),  # 左下
            (-half_side, half_side),  # 左上
            (half_side, half_side),  # 右上
            (half_side, -half_side)  # 右下
        ]

        # 生成轨迹点（每边分成若干段）
        segments_per_side = 20  # 每边分段数

        # 从中心移动到第一个顶点
        self.square_path.append((0, 0, self.center_z))

        # 生成四条边的轨迹
        for i in range(4):
            start_x, start_y = vertices[i]
            end_x, end_y = vertices[(i + 1) % 4]

            for j in range(segments_per_side + 1):
                t = j / segments_per_side
                x = start_x + (end_x - start_x) * t
                y = start_y + (end_y - start_y) * t
                self.square_path.append((x, y, self.center_z))

        # 从最后一个顶点返回中心
        self.square_path.append((0, 0, self.center_z))

        self.total_steps = len(self.square_path)

    def create_widgets(self):
        """创建界面组件"""
        # 创建主框架
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 轨迹参数设置区域
        traj_frame = ttk.LabelFrame(main_frame, text="正方形轨迹参数", padding="10")
        traj_frame.pack(fill=tk.X, pady=(0, 15))

        # 轨迹参数控件
        ttk.Label(traj_frame, text="中心 X (mm):").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.center_x_entry = ttk.Entry(traj_frame, width=10)
        self.center_x_entry.grid(row=0, column=1, padx=5, pady=5)
        self.center_x_entry.insert(0, str(self.center_x))

        ttk.Label(traj_frame, text="中心 Y (mm):").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.center_y_entry = ttk.Entry(traj_frame, width=10)
        self.center_y_entry.grid(row=0, column=3, padx=5, pady=5)
        self.center_y_entry.insert(0, str(self.center_y))

        ttk.Label(traj_frame, text="中心 Z (mm):").grid(row=0, column=4, padx=5, pady=5, sticky=tk.W)
        self.center_z_entry = ttk.Entry(traj_frame, width=10)
        self.center_z_entry.grid(row=0, column=5, padx=5, pady=5)
        self.center_z_entry.insert(0, str(self.center_z))

        ttk.Label(traj_frame, text="边长 (mm):").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.side_entry = ttk.Entry(traj_frame, width=10)
        self.side_entry.grid(row=1, column=1, padx=5, pady=5)
        self.side_entry.insert(0, str(self.square_side))

        ttk.Label(traj_frame, text="速度 (ms/点):").grid(row=1, column=2, padx=5, pady=5, sticky=tk.W)
        self.speed_entry = ttk.Entry(traj_frame, width=10)
        self.speed_entry.grid(row=1, column=3, padx=5, pady=5)
        self.speed_entry.insert(0, str(self.speed))

        # 姿态参数设置
        ttk.Label(traj_frame, text="YAW (°):").grid(row=1, column=4, padx=5, pady=5, sticky=tk.W)
        self.yaw_entry = ttk.Entry(traj_frame, width=10)
        self.yaw_entry.grid(row=1, column=5, padx=5, pady=5)
        self.yaw_entry.insert(0, "0")

        # 当前位置显示区域
        pos_frame = ttk.LabelFrame(main_frame, text="当前位置", padding="10")
        pos_frame.pack(fill=tk.X, pady=(0, 15))

        self.pos_labels = {}
        coords = ["X (mm):", "Y (mm):", "Z (mm):", "进度:"]
        for i, coord in enumerate(coords):
            ttk.Label(pos_frame, text=coord).grid(row=0, column=i * 2, padx=15, pady=5, sticky=tk.W)
            self.pos_labels[coord] = ttk.Label(pos_frame, text="0.00", width=10)
            self.pos_labels[coord].grid(row=0, column=i * 2 + 1, padx=5, pady=5)

        # 舵机角度显示区域
        servo_frame = ttk.LabelFrame(main_frame, text="舵机角度", padding="10")
        servo_frame.pack(fill=tk.X, pady=(0, 15))

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
        control_frame.pack(fill=tk.X, pady=(0, 15))

        self.start_btn = ttk.Button(control_frame, text="开始轨迹", command=self.start_trajectory)
        self.start_btn.pack(side=tk.LEFT, padx=5)

        self.stop_btn = ttk.Button(control_frame, text="停止轨迹", command=self.stop_trajectory, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=5)

        self.reset_btn = ttk.Button(control_frame, text="重置轨迹", command=self.reset_trajectory)
        self.reset_btn.pack(side=tk.LEFT, padx=5)

        self.update_params_btn = ttk.Button(control_frame, text="更新参数", command=self.update_parameters)
        self.update_params_btn.pack(side=tk.LEFT, padx=5)

        # 串口设置区域
        serial_frame = ttk.LabelFrame(main_frame, text="串口设置", padding="10")
        serial_frame.pack(fill=tk.X, pady=(0, 15))

        # 串口选择
        ttk.Label(serial_frame, text="串口号:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.port_var = tk.StringVar()
        port_combo = ttk.Combobox(serial_frame, textvariable=self.port_var, width=10)
        port_combo.grid(row=0, column=1, padx=5, pady=5)

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
            port_combo = self.root.nametowidget(".!frame.!labelframe4.!combobox")
            port_combo['values'] = [port.device for port in ports]
            if ports:
                self.port_var.set(ports[0].device)
        except Exception as e:
            messagebox.showerror("错误", f"刷新串口失败: {str(e)}")

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
        """更新轨迹参数"""
        try:
            self.center_x = float(self.center_x_entry.get())
            self.center_y = float(self.center_y_entry.get())
            self.center_z = float(self.center_z_entry.get())
            self.square_side = float(self.side_entry.get())
            self.speed = float(self.speed_entry.get())

            # 确保参数合理
            if self.square_side <= 0:
                raise ValueError("边长必须为正数")
            if self.speed <= 0:
                raise ValueError("速度必须为正数")

            # 重新生成轨迹
            self.generate_square_path()
            self.reset_trajectory()

            self.status_var.set("轨迹参数已更新")
        except ValueError as e:
            messagebox.showerror("错误", f"参数无效: {str(e)}")

    def reset_trajectory(self):
        """重置轨迹到起始位置"""
        self.current_step = 0
        self.trajectory_completed = False
        self.current_position = (0, 0, self.center_z)
        self.update_position_display()
        self.status_var.set("轨迹已重置")

    def calculate_angles(self, x, y, z):
        """计算舵机角度"""
        if not self.dll:
            messagebox.showerror("错误", "求解器未加载成功")
            return False

        try:
            # 计算姿态角度（保持末端执行器朝向合适方向）
            yaw = 0.0
            pitch = 0.0
            roll = 0.0

            # 调用求解器
            angles = (c_double * 6)()
            ret = self.dll.InK6RSS(
                c_double(x), c_double(y), c_double(z),
                c_double(yaw), c_double(pitch), c_double(roll),
                pointer(angles)
            )

            # 显示姿态角度
            self.pos_labels["X (mm):"].config(text=f"{x:.2f}")
            self.pos_labels["Y (mm):"].config(text=f"{y:.2f}")
            self.pos_labels["Z (mm):"].config(text=f"{z:.2f}")

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
                self.status_var.set(f"已发送角度到步进 {self.current_step}/{self.total_steps}")

            return True
        except Exception as e:
            self.status_var.set(f"发送失败: {str(e)}")
            self.toggle_connection()  # 发送失败时断开连接
            return False

    def update_position_display(self):
        """更新位置显示"""
        progress = (self.current_step / self.total_steps * 100) if self.total_steps > 0 else 0
        self.pos_labels["进度:"].config(text=f"{progress:.1f}%")

    def start_trajectory(self):
        """开始正方形轨迹运动"""
        if self.trajectory_running:
            return

        # 检查必要条件
        if not self.dll:
            messagebox.showwarning("警告", "求解器未加载，无法开始轨迹")
            return

        if not self.is_connected or not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("警告", "请先建立串口连接")
            return

        # 更新参数
        self.update_parameters()

        # 开始轨迹
        self.trajectory_running = True
        self.trajectory_completed = False
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.reset_btn.config(state=tk.DISABLED)
        self.update_params_btn.config(state=tk.DISABLED)
        self.status_var.set("开始正方形轨迹运动...")

        # 启动轨迹循环
        self.run_trajectory()

    def stop_trajectory(self):
        """停止正方形轨迹运动"""
        self.trajectory_running = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.reset_btn.config(state=tk.NORMAL)
        self.update_params_btn.config(state=tk.NORMAL)
        self.status_var.set("已停止轨迹运动")

    def run_trajectory(self):
        """运行轨迹循环"""
        if not self.trajectory_running or self.trajectory_completed:
            return

        # 检查是否完成轨迹
        if self.current_step >= self.total_steps:
            self.trajectory_completed = True
            self.stop_trajectory()
            self.status_var.set("正方形轨迹完成!")
            return

        # 获取当前轨迹点
        x, y, z = self.square_path[self.current_step]
        self.current_position = (x, y, z)

        # 更新显示
        self.update_position_display()

        # 计算并发送角度
        if self.calculate_angles(x, y, z):
            self.send_angles()

        # 移动到下一步
        self.current_step += 1

        # 继续轨迹循环
        self.root.after(int(self.speed), self.run_trajectory)


if __name__ == "__main__":
    root = tk.Tk()
    app = RSS6RobotController(root)
    root.mainloop()