import pyrealsense2 as rs
import numpy as np
import cv2
import math
import time


class RedCircleDetector:
    def __init__(self, width=640, height=480, fps=30):
        """
        初始化红色圆形检测器

        Args:
            width: 图像宽度
            height: 图像高度
            fps: 帧率
        """
        self.width = width
        self.height = height
        self.fps = fps

        # 初始化相机管道
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # 配置深度和彩色流
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        # 启动流
        self.profile = self.pipeline.start(self.config)

        # 获取深度传感器并设置参数
        depth_sensor = self.profile.get_device().first_depth_sensor()
        if depth_sensor.supports(rs.option.visual_preset):
            depth_sensor.set_option(rs.option.visual_preset, 3)  # 设置高清预设

        # 对齐深度帧到彩色帧
        self.align = rs.align(rs.stream.color)

        # 获取相机内参
        depth_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth))
        self.depth_intrinsics = depth_profile.get_intrinsics()

        # 初始化参数
        self.params = {
            'min_depth': 860,
            'max_depth': 940,
            'h_low1': 0,
            'h_high1': 9,
            's_low': 45,
            'v_low': 110,
            'h_low2': 162,
            'h_high2': 180,
            'kernel_size': 5,
            'morph_close': 1,
            'morph_open': 1,
            'min_area': 80,
            'min_radius_mm': 15,
            'max_radius_mm': 20,
            'circularity': 70,
        }

        # 帧率计算
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0

        # 检测结果
        self.detected_circles = []

        # 创建参数调节窗口
        cv2.namedWindow('Parameters', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Parameters', 600, 400)
        self._create_trackbars()

    def _create_trackbars(self):
        """创建参数调节滑动条"""
        cv2.createTrackbar('Min Depth (mm)', 'Parameters', self.params['min_depth'], 3000, lambda x: None)
        cv2.createTrackbar('Max Depth (mm)', 'Parameters', self.params['max_depth'], 3000, lambda x: None)

        cv2.createTrackbar('H Low1', 'Parameters', self.params['h_low1'], 180, lambda x: None)
        cv2.createTrackbar('H High1', 'Parameters', self.params['h_high1'], 180, lambda x: None)
        cv2.createTrackbar('S Low', 'Parameters', self.params['s_low'], 255, lambda x: None)
        cv2.createTrackbar('V Low', 'Parameters', self.params['v_low'], 255, lambda x: None)
        cv2.createTrackbar('H Low2', 'Parameters', self.params['h_low2'], 180, lambda x: None)
        cv2.createTrackbar('H High2', 'Parameters', self.params['h_high2'], 180, lambda x: None)

        cv2.createTrackbar('Kernel Size', 'Parameters', self.params['kernel_size'], 20, lambda x: None)
        cv2.createTrackbar('Morph Close', 'Parameters', self.params['morph_close'], 1, lambda x: None)
        cv2.createTrackbar('Morph Open', 'Parameters', self.params['morph_open'], 1, lambda x: None)

        cv2.createTrackbar('Min Area', 'Parameters', self.params['min_area'], 1000, lambda x: None)
        cv2.createTrackbar('Min Radius (mm)', 'Parameters', self.params['min_radius_mm'], 50, lambda x: None)
        cv2.createTrackbar('Max Radius (mm)', 'Parameters', self.params['max_radius_mm'], 50, lambda x: None)
        cv2.createTrackbar('Circularity (%)', 'Parameters', self.params['circularity'], 100, lambda x: None)

    def _update_params(self):
        """从滑动条更新参数值"""
        self.params['min_depth'] = cv2.getTrackbarPos('Min Depth (mm)', 'Parameters')
        self.params['max_depth'] = cv2.getTrackbarPos('Max Depth (mm)', 'Parameters')

        self.params['h_low1'] = cv2.getTrackbarPos('H Low1', 'Parameters')
        self.params['h_high1'] = cv2.getTrackbarPos('H High1', 'Parameters')
        self.params['s_low'] = cv2.getTrackbarPos('S Low', 'Parameters')
        self.params['v_low'] = cv2.getTrackbarPos('V Low', 'Parameters')
        self.params['h_low2'] = cv2.getTrackbarPos('H Low2', 'Parameters')
        self.params['h_high2'] = cv2.getTrackbarPos('H High2', 'Parameters')

        self.params['kernel_size'] = max(1, cv2.getTrackbarPos('Kernel Size', 'Parameters'))
        self.params['morph_close'] = cv2.getTrackbarPos('Morph Close', 'Parameters')
        self.params['morph_open'] = cv2.getTrackbarPos('Morph Open', 'Parameters')

        self.params['min_area'] = cv2.getTrackbarPos('Min Area', 'Parameters')
        self.params['min_radius_mm'] = cv2.getTrackbarPos('Min Radius (mm)', 'Parameters')
        self.params['max_radius_mm'] = cv2.getTrackbarPos('Max Radius (mm)', 'Parameters')
        self.params['circularity'] = cv2.getTrackbarPos('Circularity (%)', 'Parameters') / 100.0

    def _calculate_fps(self):
        """计算帧率"""
        self.frame_count += 1
        if self.frame_count >= 30:
            end_time = time.time()
            self.fps = self.frame_count / (end_time - self.start_time)
            self.frame_count = 0
            self.start_time = time.time()

    def _detect_red_circles(self, color_image, depth_image):
        """检测红色圆形

        Args:
            color_image: 彩色图像
            depth_image: 深度图像

        Returns:
            detected_circles: 检测到的圆形列表
            result_image: 标记了检测结果的图像
            red_mask: 红色掩码图像
            depth_vis: 深度可视化图像
        """
        # 获取图像分辨率
        height, width = color_image.shape[:2]
        camera_center = (width // 2, height // 2)

        # 创建深度掩码
        mask = np.logical_and(depth_image >= self.params['min_depth'],
                              depth_image <= self.params['max_depth'])
        depth_mask = mask.astype(np.uint8) * 255

        # 形态学操作优化边缘
        kernel = np.ones((self.params['kernel_size'], self.params['kernel_size']), np.uint8)

        if self.params['morph_close']:
            depth_mask = cv2.morphologyEx(depth_mask, cv2.MORPH_CLOSE, kernel)
        if self.params['morph_open']:
            depth_mask = cv2.morphologyEx(depth_mask, cv2.MORPH_OPEN, kernel)

        # 应用深度掩码到彩色图像
        masked_color = cv2.bitwise_and(color_image, color_image, mask=depth_mask)

        # 转换为HSV颜色空间
        hsv = cv2.cvtColor(masked_color, cv2.COLOR_BGR2HSV)

        # 检测红色区域
        lower_red1 = np.array([self.params['h_low1'], self.params['s_low'], self.params['v_low']])
        upper_red1 = np.array([self.params['h_high1'], 255, 255])
        lower_red2 = np.array([self.params['h_low2'], self.params['s_low'], self.params['v_low']])
        upper_red2 = np.array([self.params['h_high2'], 255, 255])

        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask_red1, mask_red2)

        # 形态学操作优化红色区域
        if self.params['morph_close']:
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        if self.params['morph_open']:
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 检测圆形物体
        detected_circles = []
        result_image = color_image.copy()

        for contour in contours:
            # 计算轮廓面积
            area = cv2.contourArea(contour)
            if area < self.params['min_area']:
                continue

            # 计算最小外接圆
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)

            # 获取中心点深度值
            if 0 <= center[1] < depth_image.shape[0] and 0 <= center[0] < depth_image.shape[1]:
                # 修改 _detect_red_circles 方法中的这部分代码
                depth_value = float(depth_image[center[1], center[0]])

                # 跳过无效深度点
                if depth_value <= 0:
                    continue

                # 计算实际物理尺寸（毫米）
                physical_radius = radius * depth_value / self.depth_intrinsics.fx

                # 检查是否在目标半径范围内
                if self.params['min_radius_mm'] <= physical_radius <= self.params['max_radius_mm']:
                    # 计算圆度（圆形度）
                    perimeter = cv2.arcLength(contour, True)
                    circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0

                    # 过滤非圆形物体
                    if circularity > self.params['circularity']:
                        # 计算圆心与相机中心点的距离（毫米）
                        dx = center[0] - camera_center[0]
                        dy = center[1] - camera_center[1]
                        dx_mm = (float(dx) * depth_value) / float(self.depth_intrinsics.fx)
                        dy_mm = (float(dy) * depth_value) / float(self.depth_intrinsics.fy)

                        circle_info = {
                            'center': center,
                            'radius': radius,
                            'depth': depth_value,
                            'physical_radius': physical_radius,
                            'dx_mm': dx_mm,
                            'dy_mm': dy_mm
                        }
                        detected_circles.append(circle_info)

        # 在原图上标记检测结果
        for i, circle in enumerate(detected_circles):
            center = circle['center']
            radius = circle['radius']
            depth_value = circle['depth']
            physical_radius = circle['physical_radius']
            dx_mm = circle['dx_mm']
            dy_mm = circle['dy_mm']

            # 绘制圆形和中心点
            cv2.circle(result_image, center, radius, (0, 255, 0), 2)
            cv2.circle(result_image, center, 2, (0, 0, 255), 3)

            # 显示深度和尺寸信息
            info_text = f"Depth: {depth_value / 10:.1f}cm, Radius: {physical_radius / 10:.1f}cm"
            cv2.putText(result_image, info_text, (center[0] - 50, center[1] - radius - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # 显示圆心与相机中心的距离（毫米）
            dist_text = f"X: {dx_mm:.1f}mm, Y: {dy_mm:.1f}mm"
            cv2.putText(result_image, dist_text, (center[0] - 40, center[1] + radius + 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            # 显示圆心坐标（红色字体）
            coord_text = f"({center[0]}, {center[1]})"
            cv2.putText(result_image, coord_text, (center[0] - 20, center[1] + radius + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # 创建深度图显示
        depth_vis = masked_color.copy()
        contours, _ = cv2.findContours(depth_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(depth_vis, contours, -1, (0, 255, 0), 2)

        # 添加深度范围信息
        depth_info = f"Depth: {self.params['min_depth']}-{self.params['max_depth']}mm"
        cv2.putText(depth_vis, depth_info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return detected_circles, result_image, red_mask, depth_vis

    def _update_display(self, color_image, depth_vis, red_mask, result_image):
        """更新显示窗口"""
        # 显示帧率和分辨率
        fps_text = f"FPS: {self.fps:.1f}"
        res_text = f"Res: {self.width}x{self.height}"
        cv2.putText(result_image, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(result_image, res_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 在所有图像上标记相机中心点
        camera_center = (self.width // 2, self.height // 2)
        for img in [color_image, depth_vis, result_image]:
            cv2.circle(img, camera_center, 5, (255, 0, 0), -1)
            cross_size = 15
            cv2.line(img, (camera_center[0] - cross_size, camera_center[1]),
                     (camera_center[0] + cross_size, camera_center[1]), (255, 0, 0), 2)
            cv2.line(img, (camera_center[0], camera_center[1] - cross_size),
                     (camera_center[0], camera_center[1] + cross_size), (255, 0, 0), 2)

        # 显示所有图像
        cv2.imshow('Original', color_image)
        cv2.imshow('Depth Masked Color', depth_vis)
        cv2.imshow('Red Mask', red_mask)
        cv2.imshow('Result', result_image)

        # 更新参数窗口
        self._update_parameter_display()

    def _update_parameter_display(self):
        """更新参数显示窗口"""
        param_display = np.zeros((400, 600, 3), dtype=np.uint8)
        cv2.putText(param_display, f"Depth Range: {self.params['min_depth']}-{self.params['max_depth']}mm", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1)
        cv2.putText(param_display,
                    f"Red Range1: H{self.params['h_low1']}-{self.params['h_high1']}, S{self.params['s_low']}+, V{self.params['v_low']}+",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
        cv2.putText(param_display, f"Red Range2: H{self.params['h_low2']}-{self.params['h_high2']}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
        cv2.putText(param_display,
                    f"Kernel Size: {self.params['kernel_size']}, Close: {'On' if self.params['morph_close'] else 'Off'}, Open: {'On' if self.params['morph_open'] else 'Off'}",
                    (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 1)
        cv2.putText(param_display,
                    f"Min Area: {self.params['min_area']}, Radius: {self.params['min_radius_mm']}-{self.params['max_radius_mm']}mm",
                    (10, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 1)
        cv2.putText(param_display, f"Circularity: {self.params['circularity'] * 100:.0f}%", (10, 180),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        cv2.putText(param_display, f"Detected Circles: {len(self.detected_circles)}", (10, 210),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 1)
        cv2.putText(param_display, f"FPS: {self.fps:.1f}, Resolution: {self.width}x{self.height}", (10, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

        # 显示检测到的圆的坐标和距离信息
        y_offset = 270
        for i, circle in enumerate(self.detected_circles):
            center = circle['center']
            depth_value = circle['depth']
            dx_mm = circle['dx_mm']
            dy_mm = circle['dy_mm']

            coord_text = f"Circle {i + 1}: ({center[0]}, {center[1]})"
            dist_text = f"  Offset: X={dx_mm:.1f}mm, Y={dy_mm:.1f}mm, Depth: {depth_value / 10:.1f}cm"
            cv2.putText(param_display, coord_text, (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
            cv2.putText(param_display, dist_text, (10, y_offset + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
            y_offset += 40

        cv2.imshow('Parameters', param_display)

    def process_frame(self):
        """处理一帧图像并返回检测结果

        Returns:
            detected_circles: 检测到的圆形列表，每个圆形包含中心点、深度、水平垂直距离等信息
            success: 是否成功处理帧
        """
        # 等待一组帧
        frames = self.pipeline.wait_for_frames()

        # 对齐深度帧到彩色帧
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            return [], False

        # 转换为numpy数组
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # 更新参数和计算帧率
        self._update_params()
        self._calculate_fps()

        # 检测红色圆形
        self.detected_circles, result_image, red_mask, depth_vis = self._detect_red_circles(color_image, depth_image)

        # 更新显示
        self._update_display(color_image, depth_vis, red_mask, result_image)

        return self.detected_circles, True

    def run(self):
        """运行主循环"""
        try:
            while True:
                circles, success = self.process_frame()

                # 输出检测结果
                if success and circles:
                    for i, circle in enumerate(circles):
                        print(
                            f"红色圆 {i + 1}: 水平距离={circle['dx_mm']:.1f}mm, 垂直距离={circle['dy_mm']:.1f}mm, 深度={circle['depth'] / 10:.1f}cm")

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        self.pipeline.stop()
        cv2.destroyAllWindows()


# 主函数
def main():
    # 创建红色圆形检测器
    detector = RedCircleDetector(width=640, height=480, fps=30)

    # 运行检测
    detector.run()


if __name__ == "__main__":
    main()