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
        self.root.title("6RSS并联机器人控制器 - 圆形轨迹")
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
        self.current_angle = 0  # 圆形轨迹当前角度(弧度)
        self.radius = 20  # 圆形轨迹半径
        self.center_x = 0  # 圆心X坐标
        self.center_y = 0  # 圆心Y坐标
        self.center_z = 188  # 圆心Z坐标
        self.speed = 5  # 运动速度(度/步)

        # 创建UI
        self.create_widgets()

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
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 轨迹参数设置区域
        traj_frame = ttk.LabelFrame(main_frame, text="圆形轨迹参数", padding="10")
        traj_frame.pack(fill=tk.X, pady=(0, 15))

        # 轨迹参数控件
        ttk.Label(traj_frame, text="圆心 X (mm):").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.center_x_entry = ttk.Entry(traj_frame, width=10)
        self.center_x_entry.grid(row=0, column=1, padx=5, pady=5)
        self.center_x_entry.insert(0, str(self.center_x))

        ttk.Label(traj_frame, text="圆心 Y (mm):").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.center_y_entry = ttk.Entry(traj_frame, width=10)
        self.center_y_entry.grid(row=0, column=3, padx=5, pady=5)
        self.center_y_entry.insert(0, str(self.center_y))

        ttk.Label(traj_frame, text="圆心 Z (mm):").grid(row=0, column=4, padx=5, pady=5, sticky=tk.W)
        self.center_z_entry = ttk.Entry(traj_frame, width=10)
        self.center_z_entry.grid(row=0, column=5, padx=5, pady=5)
        self.center_z_entry.insert(0, str(self.center_z))

        ttk.Label(traj_frame, text="半径 (mm):").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.radius_entry = ttk.Entry(traj_frame, width=10)
        self.radius_entry.grid(row=1, column=1, padx=5, pady=5)
        self.radius_entry.insert(0, str(self.radius))

        ttk.Label(traj_frame, text="速度 (度/步):").grid(row=1, column=2, padx=5, pady=5, sticky=tk.W)
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
        for i, coord in enumerate(["YAW角度 (°):","PITCH角度 (°):", "ROLL角度 (°):"]):
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
            self.port_var.set("")
            port_combo = self.root.nametowidget(".!frame.!labelframe4.!combobox")
            port_combo['values'] = [port.device for port in ports]
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
            self.radius = float(self.radius_entry.get())
            self.speed = float(self.speed_entry.get())

            # 确保参数合理
            if self.radius <= 0:
                raise ValueError("半径必须为正数")
            if self.speed <= 0 or self.speed > 30:
                raise ValueError("速度应在0-30之间")

            self.status_var.set("轨迹参数已更新")
        except ValueError as e:
            messagebox.showerror("错误", f"参数无效: {str(e)}")

    def calculate_circular_position(self):
        """计算圆形轨迹上的当前位置"""
        # 将角度从度转换为弧度
        rad = math.radians(self.current_angle)

        # 计算X和Y坐标（在YOZ平面内画圆）
        x = 0
        y = self.center_y+self.radius * math.cos(rad)
        z = self.center_z + self.radius * math.sin(rad)

        # 更新角度（循环0-360度）
        self.current_angle = (self.current_angle + self.speed) % 360

        return x, y, z

    def calculate_angles(self, x, y, z):
        """计算舵机角度"""
        if not self.dll:
            messagebox.showerror("错误", "求解器未加载成功")
            return False

        try:
            # 获取姿态角度

            roll = 0.0  # 保持水平
            yaw = np.arctan(-y / self.center_x) * 180 / np.pi
            pitch = np.arctan((z - 188) / self.center_x) * 180 / np.pi
            # 调用求解器

            angles = (c_double * 6)()
            ret = self.dll.InK6RSS(
                c_double(0), c_double(0), c_double(188),
                c_double(yaw), c_double(pitch), c_double(roll),
                pointer(angles)
            )
            self.pos_labels["YAW角度 (°):"].config(text=f"{yaw :.2f}")
            self.pos_labels["PITCH角度 (°):"].config(text=f"{pitch:.2f}")
            self.pos_labels["ROLL角度 (°):"].config(text=f"{roll:.2f}")
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

        except ValueError:
            self.status_var.set("姿态角度输入无效")
            return False
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

    def start_trajectory(self):
        """开始圆形轨迹运动"""
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
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.update_params_btn.config(state=tk.DISABLED)
        self.status_var.set("开始圆形轨迹运动...")

        # 启动轨迹循环
        self.run_trajectory()

    def stop_trajectory(self):
        """停止圆形轨迹运动"""
        self.trajectory_running = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.update_params_btn.config(state=tk.NORMAL)
        self.status_var.set("已停止轨迹运动")

    def run_trajectory(self):
        """运行轨迹循环"""
        if not self.trajectory_running:
            return

        # 计算当前位置
        x, y, z = self.calculate_circular_position()


        # 显示当前位置
        #self.pos_labels["X (mm):"].config(text=f"{x:.2f}")
        #self.pos_labels["Y (mm):"].config(text=f"{y:.2f}")
        #self.pos_labels["Z (mm):"].config(text=f"{z:.2f}")

        # 计算并发送角度
        if self.calculate_angles(x, y, z):
            self.send_angles()

        # 继续轨迹循环（控制运动速度）
        self.root.after(100, self.run_trajectory)  # 每100ms更新一次位置


if __name__ == "__main__":
    root = tk.Tk()
    app = RSS6RobotController(root)
    root.mainloop()
