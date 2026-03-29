import pyrealsense2 as rs
import numpy as np
import cv2
import time
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

        # 初始化UI
        self._init_ui()

    def _init_ui(self):
        """初始化用户界面"""
        cv2.namedWindow('Control', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Control', 600, 400)
        self._create_trackbars()

    def _create_trackbars(self):
        """创建控制滑动条"""
        # 深度控制
        cv2.createTrackbar('Min Depth (mm)', 'Control',
                           self.depth_processor.params['min_depth'], 3000, lambda x: None)
        cv2.createTrackbar('Max Depth (mm)', 'Control',
                           self.depth_processor.params['max_depth'], 3000, lambda x: None)
        cv2.createTrackbar('Morph Size', 'Control',
                           self.depth_processor.params['morph_size'], 15, lambda x: None)
        cv2.createTrackbar('Blur Size', 'Control',
                           self.depth_processor.params['blur_size'], 15, lambda x: None)

        # 颜色控制
        cv2.createTrackbar('H Low', 'Control', self.detection_params['h_low'], 180, lambda x: None)
        cv2.createTrackbar('H High', 'Control', self.detection_params['h_high'], 180, lambda x: None)
        cv2.createTrackbar('S Low', 'Control', self.detection_params['s_low'], 255, lambda x: None)
        cv2.createTrackbar('S High', 'Control', self.detection_params['s_high'], 255, lambda x: None)
        cv2.createTrackbar('V Low', 'Control', self.detection_params['v_low'], 255, lambda x: None)
        cv2.createTrackbar('V High', 'Control', self.detection_params['v_high'], 255, lambda x: None)

        # 形状控制
        cv2.createTrackbar('Min Area', 'Control', self.detection_params['min_area'], 1000, lambda x: None)
        cv2.createTrackbar('Min Radius (mm)', 'Control',
                           self.detection_params['min_radius_mm'], 300, lambda x: None)
        cv2.createTrackbar('Max Radius (mm)', 'Control',
                           self.detection_params['max_radius_mm'], 300, lambda x: None)

    def _update_params(self):
        """从滑动条更新参数"""
        # 深度处理参数
        self.depth_processor.params['min_depth'] = cv2.getTrackbarPos('Min Depth (mm)', 'Control')
        self.depth_processor.params['max_depth'] = cv2.getTrackbarPos('Max Depth (mm)', 'Control')
        self.depth_processor.params['morph_size'] = cv2.getTrackbarPos('Morph Size', 'Control')
        self.depth_processor.params['blur_size'] = cv2.getTrackbarPos('Blur Size', 'Control')

        # 检测参数
        self.detection_params['h_low'] = cv2.getTrackbarPos('H Low', 'Control')
        self.detection_params['h_high'] = cv2.getTrackbarPos('H High', 'Control')
        self.detection_params['s_low'] = cv2.getTrackbarPos('S Low', 'Control')
        self.detection_params['s_high'] = cv2.getTrackbarPos('S High', 'Control')
        self.detection_params['v_low'] = cv2.getTrackbarPos('V Low', 'Control')
        self.detection_params['v_high'] = cv2.getTrackbarPos('V High', 'Control')
        self.detection_params['min_area'] = cv2.getTrackbarPos('Min Area', 'Control')
        self.detection_params['min_radius_mm'] = cv2.getTrackbarPos('Min Radius (mm)', 'Control')
        self.detection_params['max_radius_mm'] = cv2.getTrackbarPos('Max Radius (mm)', 'Control')

    def _calculate_fps(self):
        """计算帧率"""
        self.frame_count += 1
        if self.frame_count >= 30:
            end_time = time.time()
            self.fps = self.frame_count / (end_time - self.start_time)
            self.frame_count = 0
            self.start_time = time.time()

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
        box = np.int32(box)

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

    def _create_visualization(self, color_image: np.ndarray, depth_image: np.ndarray,
                              black_mask: np.ndarray, depth_mask: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """创建可视化图像"""
        # 深度图彩色化
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET
        )

        # 添加文本信息
        depth_range_text = f"Depth: {self.depth_processor.params['min_depth']}-{self.depth_processor.params['max_depth']}mm"
        fps_text = f"FPS: {self.fps:.1f}"

        for img, pos in [(color_image, (10, 30)), (depth_colormap, (10, 30)),
                         (depth_mask, (10, 30))]:
            cv2.putText(img, depth_range_text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(img, fps_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return depth_colormap, self._create_info_display()

    def _create_info_display(self) -> np.ndarray:
        """创建信息显示面板"""
        info_display = np.zeros((200, 600, 3), dtype=np.uint8)
        cv2.putText(info_display, f"Detected Rectangles: {len(self.detected_rectangles)}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        y_offset = 60
        for i, rect in enumerate(self.detected_rectangles):
            text = f"Rect {i + 1}: Size={rect['physical_size'] / 10:.1f}cm, Depth={rect['depth'] / 10:.1f}cm"
            cv2.putText(info_display, text, (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            y_offset += 25

        return info_display

    def process_frame(self) -> Tuple[List[Dict], bool]:
        """处理单帧图像"""
        # 获取帧数据
        depth_image, color_image = self.camera.get_frames()
        if depth_image is None or color_image is None:
            return [], False

        # 更新参数和计算帧率
        self._update_params()
        self._calculate_fps()

        # 检测矩形
        self.detected_rectangles, result_image, black_mask, depth_mask = self.detect_rectangles(color_image,
                                                                                                depth_image)

        # 创建可视化
        depth_colormap, info_display = self._create_visualization(color_image, depth_image, black_mask, depth_mask)

        # 显示结果
        self._display_results(color_image, depth_colormap, depth_mask, black_mask, result_image, info_display)

        return self.detected_rectangles, True

    def _display_results(self, color_image: np.ndarray, depth_colormap: np.ndarray,
                         depth_mask: np.ndarray, black_mask: np.ndarray,
                         result_image: np.ndarray, info_display: np.ndarray):
        """显示所有结果窗口"""
        cv2.imshow('Original', color_image)
        cv2.imshow('Depth', depth_colormap)
        cv2.imshow('Depth Mask', depth_mask)
        cv2.imshow('Black Mask', black_mask)
        cv2.imshow('Result', result_image)
        cv2.imshow('Detection Info', info_display)

    def run(self):
        """运行主循环"""
        try:
            while True:
                rectangles, success = self.process_frame()

                # 输出检测结果
                if success and rectangles:
                    for i, rect in enumerate(rectangles):
                        print(f"矩形 {i + 1}: 尺寸={rect['physical_size'] / 10:.1f}cm, "
                              f"深度={rect['depth'] / 10:.1f}cm, "
                              f"偏移=({rect['dx_mm']:.1f}mm, {rect['dy_mm']:.1f}mm)")

                # 检查退出条件
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except KeyboardInterrupt:
            print("程序被用户中断")
        except Exception as e:
            print(f"运行时错误: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        self.camera.stop()
        cv2.destroyAllWindows()


class BlackRectangleDetectionSystem:
    """黑色矩形检测系统（高级封装）"""

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        self.camera = RealSenseCamera(width, height, fps)
        self.detector = BlackRectangleDetector(self.camera)

    def run(self):
        """运行检测系统"""
        self.detector.run()

    def cleanup(self):
        """清理资源"""
        self.detector.cleanup()


# 使用示例
def main():
    """主函数"""
    # 创建检测系统
    detection_system = BlackRectangleDetectionSystem(width=640, height=480, fps=30)

    # 运行检测
    try:
        detection_system.run()
    finally:
        detection_system.cleanup()


# 简化使用方式
def simple_demo():
    """简化版演示"""
    system = BlackRectangleDetectionSystem()
    system.run()


if __name__ == "__main__":
    main()