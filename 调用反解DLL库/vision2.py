import cv2
import numpy as np
import pyrealsense2 as rs
import json


class AdjustableLaserDetector:
    def __init__(self):
        # 原始初始化逻辑修正
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # 可调节参数 - 红色检测
        self.lower_red1 = np.array([0, 120, 70])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])

        # 可调节参数 - 白色检测
        self.lower_white = np.array([0, 0, 200])
        self.upper_white = np.array([180, 60, 255])

        # 可调节参数 - 形态学操作
        self.blur_kernel_size = 5
        self.morph_open_kernel = 2
        self.morph_close_kernel = 3

        # 可调节参数 - 面积过滤
        self.min_contour_area = 30
        self.max_contour_area = 500
        self.min_white_area = 10

        # 可调节参数 - 距离阈值
        self.max_center_distance = 20

        # 图像中心坐标
        self.image_center_x = 320  # 640/2
        self.image_center_y = 240  # 480/2

        # 相机状态标记
        self.is_camera_running = False
        self.depth_scale = None

        # 创建滑动条窗口
        self.setup_trackbars()

    # 原始方法完全保留
    def setup_trackbars(self):
        cv2.namedWindow('Parameters')

        # 红色检测参数
        cv2.createTrackbar('Red1_H_min', 'Parameters', 162, 180, self.nothing)
        cv2.createTrackbar('Red1_H_max', 'Parameters', 180, 180, self.nothing)
        cv2.createTrackbar('Red1_S_min', 'Parameters', 30, 255, self.nothing)
        cv2.createTrackbar('Red1_V_min', 'Parameters', 170, 255, self.nothing)

        cv2.createTrackbar('Red2_H_min', 'Parameters', 170, 180, self.nothing)
        cv2.createTrackbar('Red2_H_max', 'Parameters', 180, 180, self.nothing)
        cv2.createTrackbar('Red2_S_min', 'Parameters', 125, 255, self.nothing)
        cv2.createTrackbar('Red2_V_min', 'Parameters', 70, 255, self.nothing)

        # 白色检测参数
        cv2.createTrackbar('White_H_min', 'Parameters', 150, 180, self.nothing)
        cv2.createTrackbar('White_H_max', 'Parameters', 180, 180, self.nothing)
        cv2.createTrackbar('White_S_max', 'Parameters', 255, 255, self.nothing)
        cv2.createTrackbar('White_V_min', 'Parameters', 195, 255, self.nothing)

        # 处理参数
        cv2.createTrackbar('Blur_Size', 'Parameters', 1, 15, self.nothing)
        cv2.createTrackbar('Open_Kernel', 'Parameters', 2, 10, self.nothing)
        cv2.createTrackbar('Close_Kernel', 'Parameters', 3, 10, self.nothing)

        # 面积参数
        cv2.createTrackbar('Min_Area', 'Parameters', 30, 200, self.nothing)
        cv2.createTrackbar('Max_Area', 'Parameters', 600, 1000, self.nothing)
        cv2.createTrackbar('Min_White_Area', 'Parameters', 10, 100, self.nothing)
        cv2.createTrackbar('Max_Distance', 'Parameters', 20, 100, self.nothing)

    # 原始方法完全保留
    def nothing(self, x):
        pass

    # 原始方法完全保留
    def update_parameters(self):
        # 更新红色检测参数
        self.lower_red1 = np.array([
            cv2.getTrackbarPos('Red1_H_min', 'Parameters'),
            cv2.getTrackbarPos('Red1_S_min', 'Parameters'),
            cv2.getTrackbarPos('Red1_V_min', 'Parameters')
        ])
        self.upper_red1 = np.array([
            cv2.getTrackbarPos('Red1_H_max', 'Parameters'),
            255, 255
        ])

        self.lower_red2 = np.array([
            cv2.getTrackbarPos('Red2_H_min', 'Parameters'),
            cv2.getTrackbarPos('Red2_S_min', 'Parameters'),
            cv2.getTrackbarPos('Red2_V_min', 'Parameters')
        ])
        self.upper_red2 = np.array([
            cv2.getTrackbarPos('Red2_H_max', 'Parameters'),
            255, 255
        ])

        # 更新白色检测参数
        self.lower_white = np.array([
            cv2.getTrackbarPos('White_H_min', 'Parameters'),
            0,
            cv2.getTrackbarPos('White_V_min', 'Parameters')
        ])
        self.upper_white = np.array([
            cv2.getTrackbarPos('White_H_max', 'Parameters'),
            cv2.getTrackbarPos('White_S_max', 'Parameters'),
            255
        ])

        # 更新处理参数（确保为奇数）
        self.blur_kernel_size = max(1, cv2.getTrackbarPos('Blur_Size', 'Parameters') * 2 + 1)
        self.morph_open_kernel = cv2.getTrackbarPos('Open_Kernel', 'Parameters')
        self.morph_close_kernel = cv2.getTrackbarPos('Close_Kernel', 'Parameters')

        # 更新面积参数
        self.min_contour_area = cv2.getTrackbarPos('Min_Area', 'Parameters')
        self.max_contour_area = cv2.getTrackbarPos('Max_Area', 'Parameters')
        self.min_white_area = cv2.getTrackbarPos('Min_White_Area', 'Parameters')
        self.max_center_distance = cv2.getTrackbarPos('Max_Distance', 'Parameters')

    # 原始方法完全保留
    def setup_camera(self):
        try:
            profile = self.pipeline.start(self.config)
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            self.is_camera_running = True
            print(f"D455相机初始化成功，深度比例尺: {self.depth_scale}")
            return True
        except Exception as e:
            print(f"相机初始化失败: {e}")
            self.is_camera_running = False
            return False

    # 原始方法完全保留
    def preprocess_image(self, frame):
        blurred = cv2.GaussianBlur(frame, (self.blur_kernel_size, self.blur_kernel_size), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        return hsv

    # 原始方法完全保留
    def detect_red_regions(self, hsv):
        mask_red1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_red2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        red_mask = cv2.bitwise_or(mask_red1, mask_red2)

        if self.morph_open_kernel > 0:
            kernel_open = np.ones((self.morph_open_kernel, self.morph_open_kernel), np.uint8)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel_open)

        if self.morph_close_kernel > 0:
            kernel_close = np.ones((self.morph_close_kernel, self.morph_close_kernel), np.uint8)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel_close)

        return red_mask

    # 原始方法完全保留
    def detect_white_center(self, hsv):
        white_mask = cv2.inRange(hsv, self.lower_white, self.upper_white)

        if self.morph_open_kernel > 0:
            kernel_open = np.ones((self.morph_open_kernel, self.morph_open_kernel), np.uint8)
            white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel_open)

        return white_mask

    # 原始方法完全保留
    def get_depth_distance(self, depth_frame, x, y):
        try:
            depth_value = depth_frame.get_distance(x, y)
            if depth_value > 0:
                return depth_value
            else:
                return None
        except Exception as e:
            return None

    # 原始方法完全保留
    def detect_laser(self, frame, depth_frame):
        self.update_parameters()
        hsv = self.preprocess_image(frame)
        red_mask = self.detect_red_regions(hsv)
        white_mask = self.detect_white_center(hsv)

        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        white_contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        laser_points = []

        for red_contour in red_contours:
            red_area = cv2.contourArea(red_contour)
            if red_area < self.min_contour_area or red_area > self.max_contour_area:
                continue

            for white_contour in white_contours:
                white_area = cv2.contourArea(white_contour)
                if white_area < self.min_white_area:
                    continue

                M_red = cv2.moments(red_contour)
                M_white = cv2.moments(white_contour)

                if M_red['m00'] == 0 or M_white['m00'] == 0:
                    continue

                cx_red = int(M_red['m10'] / M_red['m00'])
                cy_red = int(M_red['m01'] / M_red['m00'])
                cx_white = int(M_white['m10'] / M_white['m00'])
                cy_white = int(M_white['m01'] / M_white['m00'])

                distance = np.sqrt((cx_red - cx_white) ** 2 + (cy_red - cy_white) ** 2)

                if distance < self.max_center_distance:
                    depth_distance = self.get_depth_distance(depth_frame, cx_white, cy_white)
                    horizontal_distance = cx_white - self.image_center_x
                    vertical_distance = cy_white - self.image_center_y

                    if depth_distance is not None:
                        horizontal_real = (horizontal_distance / self.image_center_x) * depth_distance * 0.5
                        vertical_real = (vertical_distance / self.image_center_y) * depth_distance * 0.5
                    else:
                        horizontal_real = None
                        vertical_real = None

                    laser_info = {
                        'position': (cx_white, cy_white),
                        'depth': depth_distance,
                        'pixel_offset': (horizontal_distance, vertical_distance),
                        'real_offset': (horizontal_real, vertical_real)
                    }
                    laser_points.append(laser_info)
                    break

        return laser_points, red_mask, white_mask

    # 原始方法完全保留
    def display_parameters(self, frame):
        param_text = [
            f"Red1: {self.lower_red1} - {self.upper_red1}",
            f"Red2: {self.lower_red2} - {self.upper_red2}",
            f"White: {self.lower_white} - {self.upper_white}",
            f"Blur: {self.blur_kernel_size}, Open: {self.morph_open_kernel}, Close: {self.morph_close_kernel}",
            f"MinArea: {self.min_contour_area}, MaxArea: {self.max_contour_area}",
            f"MaxDist: {self.max_center_distance}"
        ]

        for i, text in enumerate(param_text):
            cv2.putText(frame, text, (10, 30 + i * 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    # 新增：获取激光数据的接口方法
    def get_laser_data(self):
        """
        获取激光点的深度、水平距离和竖直距离

        返回:
            tuple: (depth, horizontal, vertical)
                  若未检测到则为 (None, None, None)
        """
        if not self.is_camera_running:
            return (None, None, None)

        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                return (None, None, None)

            frame = np.asanyarray(color_frame.get_data())
            laser_points, _, _ = self.detect_laser(frame, depth_frame)

            if laser_points:
                first_point = laser_points[0]
                return (
                    first_point['depth'],
                    first_point['real_offset'][0],
                    first_point['real_offset'][1]
                )
            return (None, None, None)

        except Exception as e:
            print(f"获取激光数据出错: {e}")
            return (None, None, None)

    # 原始运行方法保留，增加状态判断
    def run_adjustable_detection(self):
        if not self.setup_camera():
            return

        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()

                if not color_frame or not depth_frame:
                    continue

                frame = np.asanyarray(color_frame.get_data())
                laser_points, red_mask, white_mask = self.detect_laser(frame, depth_frame)

                result_frame = frame.copy()
                cv2.line(result_frame, (self.image_center_x, 0), (self.image_center_x, 480), (255, 255, 0), 1)
                cv2.line(result_frame, (0, self.image_center_y), (640, self.image_center_y), (255, 255, 0), 1)
                cv2.circle(result_frame, (self.image_center_x, self.image_center_y), 3, (255, 255, 0), -1)

                for laser_info in laser_points:
                    point = laser_info['position']
                    depth = laser_info['depth']
                    h_offset, v_offset = laser_info['pixel_offset']
                    h_real, v_real = laser_info['real_offset']

                    cv2.circle(result_frame, point, 5, (0, 255, 0), -1)
                    cv2.circle(result_frame, point, 15, (0, 255, 255), 2)

                    info_texts = [
                        f"Position: {point}",
                        f"Depth: {depth:.3f}m" if depth is not None else "Depth: N/A",
                        f"Offset: ({h_offset:+d}, {v_offset:+d}) px"
                    ]

                    if h_real is not None and v_real is not None:
                        info_texts.append(f"Real Offset: ({h_real:+.3f}, {v_real:+.3f}) m")

                    cv2.line(result_frame, (self.image_center_x, self.image_center_y), point, (0, 255, 255), 1)

                    for i, text in enumerate(info_texts):
                        y_offset = point[1] - 60 + i * 15
                        cv2.putText(result_frame, text, (point[0] + 20, y_offset),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

                self.display_parameters(result_frame)

                cv2.imshow('Laser Detection', result_frame)
                cv2.imshow('Red Mask', red_mask)
                cv2.imshow('White Mask', white_mask)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    self.save_parameters()

        finally:
            self.stop()

    # 原始方法完全保留
    def save_parameters(self):
        params = {
            'red1_lower': self.lower_red1.tolist(),
            'red1_upper': self.upper_red1.tolist(),
            'red2_lower': self.lower_red2.tolist(),
            'red2_upper': self.upper_red2.tolist(),
            'white_lower': self.lower_white.tolist(),
            'white_upper': self.upper_white.tolist(),
            'blur_kernel': self.blur_kernel_size,
            'morph_open': self.morph_open_kernel,
            'morph_close': self.morph_close_kernel,
            'min_area': self.min_contour_area,
            'max_area': self.max_contour_area,
            'min_white_area': self.min_white_area,
            'max_distance': self.max_center_distance
        }

        with open('laser_params.json', 'w') as f:
            json.dump(params, f, indent=4)
        print("参数已保存到 laser_params.json")

    # 新增：加载参数方法
    def load_parameters(self, file_path='laser_params.json'):
        try:
            with open(file_path, 'r') as f:
                params = json.load(f)

            self.lower_red1 = np.array(params['red1_lower'])
            self.upper_red1 = np.array(params['red1_upper'])
            self.lower_red2 = np.array(params['red2_lower'])
            self.upper_red2 = np.array(params['red2_upper'])
            self.lower_white = np.array(params['white_lower'])
            self.upper_white = np.array(params['white_upper'])
            self.blur_kernel_size = params['blur_kernel']
            self.morph_open_kernel = params['morph_open']
            self.morph_close_kernel = params['morph_close']
            self.min_contour_area = params['min_area']
            self.max_contour_area = params['max_area']
            self.min_white_area = params['min_white_area']
            self.max_center_distance = params['max_distance']

            print(f"已从 {file_path} 加载参数")
            return True
        except Exception as e:
            print(f"加载参数失败: {e}")
            return False

    # 新增：停止相机方法
    def stop(self):
        if self.is_camera_running:
            self.pipeline.stop()
            self.is_camera_running = False
        cv2.destroyAllWindows()
        print("检测已停止，资源已释放")


# 使用示例
if __name__ == "__main__":
    # 方式1：运行带界面的可调式检测（原始功能）
    detector = AdjustableLaserDetector()
    detector.run_adjustable_detection()

    # 方式2：仅获取数据（无界面，适合集成）
    # detector = AdjustableLaserDetector()
    # if detector.setup_camera():
    #     try:
    #         while True:
    #             depth, horizontal, vertical = detector.get_laser_data()
    #             if depth is not None:
    #                 print(f"深度: {depth:.3f}m, 水平距离: {horizontal:.3f}m, 竖直距离: {vertical:.3f}m")
    #             else:
    #                 print("未检测到激光点")
    #             if cv2.waitKey(1) & 0xFF == ord('q'):
    #                 break
    #     finally:
    #         detector.stop()