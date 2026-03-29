import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
from ctypes import CDLL, c_double, pointer, POINTER, c_int
import time

class RSS6RobotController:
    def __init__(self, root):
        self.root = root
        self.root.title("6RSS并联机器人控制器")
        self.root.geometry("650x500")
        self.root.resizable(False, False)
        
        # 加载DLL求解器
        self.dll = None
        self.load_solver()
        
        # 串口相关变量
        self.serial_port = None
        self.is_connected = False
        
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
                POINTER(c_double * 6)          # 输出角度数组
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
        
        # 坐标输入区域
        coord_frame = ttk.LabelFrame(main_frame, text="位置与姿态", padding="10")
        coord_frame.pack(fill=tk.X, pady=(0, 15))
        
        # 坐标输入控件
        labels = ["X坐标 (mm):", "Y坐标 (mm):", "Z坐标 (mm):", 
                 "YAW角度 (°):", "PITCH角度 (°):", "ROLL角度 (°):"]
        self.entries = {}
        
        for i, label in enumerate(labels):
            ttk.Label(coord_frame, text=label).grid(row=i//3, column=(i%3)*2, padx=5, pady=5, sticky=tk.W)
            entry = ttk.Entry(coord_frame, width=15)
            entry.grid(row=i//3, column=(i%3)*2 + 1, padx=5, pady=5)
            self.entries[label] = entry
        
        # 设置默认值
        self.entries["X坐标 (mm):"].insert(0, "0")
        self.entries["Y坐标 (mm):"].insert(0, "0")
        self.entries["Z坐标 (mm):"].insert(0, "188")  # 168+20
        self.entries["YAW角度 (°):"].insert(0, "0")
        self.entries["PITCH角度 (°):"].insert(0, "0")
        self.entries["ROLL角度 (°):"].insert(0, "0")
        
        # 计算按钮
        calc_btn = ttk.Button(main_frame, text="计算舵机角度", command=self.calculate_angles)
        calc_btn.pack(pady=(0, 15))
        
        # 舵机角度显示区域
        servo_frame = ttk.LabelFrame(main_frame, text="舵机角度", padding="10")
        servo_frame.pack(fill=tk.X, pady=(0, 15))
        
        self.servo_labels = []
        for i in range(6):
            frame = ttk.Frame(servo_frame)
            frame.grid(row=i//3, column=i%3, padx=10, pady=5, sticky=tk.W)
            
            ttk.Label(frame, text=f"舵机 {i+1} (°):").pack(side=tk.LEFT, padx=5)
            val_label = ttk.Label(frame, text="--", width=10)
            val_label.pack(side=tk.LEFT)
            self.servo_labels.append(val_label)
        
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
        
        # 发送按钮
        self.send_btn = ttk.Button(main_frame, text="发送角度到单片机", command=self.send_angles, state=tk.DISABLED)
        self.send_btn.pack(pady=(0, 15))
        
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
        ports = serial.tools.list_ports.comports()
        self.port_var.set("")
        port_combo = self.root.nametowidget(".!frame.!labelframe3.!combobox")
        port_combo['values'] = [port.device for port in ports]
    
    def toggle_connection(self):
        """切换串口连接状态"""
        if self.is_connected:
            # 断开连接
            try:
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.close()
                self.is_connected = False
                self.connect_btn.config(text="连接")
                self.send_btn.config(state=tk.DISABLED)
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
                    self.send_btn.config(state=tk.NORMAL if self.last_angles else tk.DISABLED)
                    self.status_var.set(f"已连接到 {port} ({baudrate} bps)")
            except Exception as e:
                messagebox.showerror("错误", f"连接失败: {str(e)}")
    
    def calculate_angles(self):
        """计算舵机角度"""
        if not self.dll:
            messagebox.showerror("错误", "求解器未加载成功")
            return
        
        try:
            # 获取输入值
            xp = float(self.entries["X坐标 (mm):"].get())
            yp = float(self.entries["Y坐标 (mm):"].get())
            zp = float(self.entries["Z坐标 (mm):"].get())
            yaw = float(self.entries["YAW角度 (°):"].get())
            pitch = float(self.entries["PITCH角度 (°):"].get())
            roll = float(self.entries["ROLL角度 (°):"].get())
            
            # 调用求解器
            angles = (c_double * 6)()
            ret = self.dll.InK6RSS(
                c_double(xp), c_double(yp), c_double(zp),
                c_double(yaw), c_double(pitch), c_double(roll),
                pointer(angles)
            )
            
            # 检查结果
            if ret == 0 or any(angle == -360 for angle in angles):
                messagebox.showwarning("警告", "无法计算有效的舵机角度，可能超出工作范围")
                self.last_angles = None
                for label in self.servo_labels:
                    label.config(text="--")
                self.send_btn.config(state=tk.DISABLED)
                return
            
            # 显示结果
            self.last_angles = [round(angle, 2) for angle in angles]
            for i, label in enumerate(self.servo_labels):
                label.config(text=str(self.last_angles[i]))
            
            # 更新发送按钮状态
            if self.is_connected:
                self.send_btn.config(state=tk.NORMAL)
            
            self.status_var.set("角度计算成功")
            
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数值")
        except Exception as e:
            messagebox.showerror("错误", f"计算失败: {str(e)}")
    
    def send_angles(self):
        """发送角度到单片机"""
        if not self.is_connected or not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("警告", "请先建立串口连接")
            return
        
        if not self.last_angles:
            messagebox.showwarning("警告", "请先计算舵机角度")
            return
        
        try:
            # 格式化数据，格式: "S,angle1,angle2,angle3,angle4,angle5,angle6,E\n"
            data_str = f"S,{','.join(map(str, self.last_angles))},E\n"
            self.serial_port.write(data_str.encode('utf-8'))
            
            # 等待响应
            time.sleep(0.1)
            if self.serial_port.in_waiting > 0:
                response = self.serial_port.readline().decode('utf-8').strip()
                self.status_var.set(f"发送成功，收到响应: {response}")
            else:
                self.status_var.set("角度已发送，未收到响应")
                
        except Exception as e:
            messagebox.showerror("错误", f"发送失败: {str(e)}")
            self.toggle_connection()  # 发送失败时断开连接

if __name__ == "__main__":
    root = tk.Tk()
    app = RSS6RobotController(root)
    root.mainloop()