import pyrealsense2 as rs
import numpy as np
import cv2
import math
import time

# 初始化相机管道
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 启动流
profile = pipeline.start(config)

# 获取深度传感器并设置参数
depth_sensor = profile.get_device().first_depth_sensor()
if depth_sensor.supports(rs.option.visual_preset):
    depth_sensor.set_option(rs.option.visual_preset, 3)  # 设置高清预设

# 对齐深度帧到彩色帧
align = rs.align(rs.stream.color)

# 获取相机内参
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()

# 创建参数调节窗口
cv2.namedWindow('Parameters', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Parameters', 600, 400)

# 初始化参数（使用图片中的数值）
params = {
    # 深度范围参数
    'min_depth': 860,
    'max_depth': 940,
    
    # 红色检测参数
    'h_low1': 0,
    'h_high1': 9,
    's_low': 45,
    'v_low': 110,
    'h_low2': 162,
    'h_high2': 180,
    
    # 形态学操作参数
    'kernel_size': 5,
    'morph_close': 1,
    'morph_open': 1,
    
    # 圆形检测参数
    'min_area': 80,
    'min_radius_mm': 15,
    'max_radius_mm': 20,
    'circularity': 70,
}

# 创建滑动条
cv2.createTrackbar('Min Depth (mm)', 'Parameters', params['min_depth'], 3000, lambda x: None)
cv2.createTrackbar('Max Depth (mm)', 'Parameters', params['max_depth'], 3000, lambda x: None)

cv2.createTrackbar('H Low1', 'Parameters', params['h_low1'], 180, lambda x: None)
cv2.createTrackbar('H High1', 'Parameters', params['h_high1'], 180, lambda x: None)
cv2.createTrackbar('S Low', 'Parameters', params['s_low'], 255, lambda x: None)
cv2.createTrackbar('V Low', 'Parameters', params['v_low'], 255, lambda x: None)
cv2.createTrackbar('H Low2', 'Parameters', params['h_low2'], 180, lambda x: None)
cv2.createTrackbar('H High2', 'Parameters', params['h_high2'], 180, lambda x: None)

cv2.createTrackbar('Kernel Size', 'Parameters', params['kernel_size'], 20, lambda x: None)
cv2.createTrackbar('Morph Close', 'Parameters', params['morph_close'], 1, lambda x: None)
cv2.createTrackbar('Morph Open', 'Parameters', params['morph_open'], 1, lambda x: None)

cv2.createTrackbar('Min Area', 'Parameters', params['min_area'], 1000, lambda x: None)
cv2.createTrackbar('Min Radius (mm)', 'Parameters', params['min_radius_mm'], 50, lambda x: None)
cv2.createTrackbar('Max Radius (mm)', 'Parameters', params['max_radius_mm'], 50, lambda x: None)
cv2.createTrackbar('Circularity (%)', 'Parameters', params['circularity'], 100, lambda x: None)

# 帧率计算
frame_count = 0
start_time = time.time()
fps = 0

try:
    while True:
        # 从滑动条获取当前参数值
        params['min_depth'] = cv2.getTrackbarPos('Min Depth (mm)', 'Parameters')
        params['max_depth'] = cv2.getTrackbarPos('Max Depth (mm)', 'Parameters')
        
        params['h_low1'] = cv2.getTrackbarPos('H Low1', 'Parameters')
        params['h_high1'] = cv2.getTrackbarPos('H High1', 'Parameters')
        params['s_low'] = cv2.getTrackbarPos('S Low', 'Parameters')
        params['v_low'] = cv2.getTrackbarPos('V Low', 'Parameters')
        params['h_low2'] = cv2.getTrackbarPos('H Low2', 'Parameters')
        params['h_high2'] = cv2.getTrackbarPos('H High2', 'Parameters')
        
        params['kernel_size'] = max(1, cv2.getTrackbarPos('Kernel Size', 'Parameters'))
        params['morph_close'] = cv2.getTrackbarPos('Morph Close', 'Parameters')
        params['morph_open'] = cv2.getTrackbarPos('Morph Open', 'Parameters')
        
        params['min_area'] = cv2.getTrackbarPos('Min Area', 'Parameters')
        params['min_radius_mm'] = cv2.getTrackbarPos('Min Radius (mm)', 'Parameters')
        params['max_radius_mm'] = cv2.getTrackbarPos('Max Radius (mm)', 'Parameters')
        params['circularity'] = cv2.getTrackbarPos('Circularity (%)', 'Parameters') / 100.0
        
        # 等待一组帧
        frames = pipeline.wait_for_frames()

        # 对齐深度帧到彩色帧
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # 转换为numpy数组
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # 计算帧率
        frame_count += 1
        if frame_count >= 30:
            end_time = time.time()
            fps = frame_count / (end_time - start_time)
            frame_count = 0
            start_time = time.time()
        
        # 获取图像分辨率
        height, width = color_image.shape[:2]
        resolution = f"{width}x{height}"
        
        # 计算相机中心点
        center_x = width // 2
        center_y = height // 2
        camera_center = (center_x, center_y)
        
        # 创建深度掩码
        mask = np.logical_and(depth_image >= params['min_depth'], depth_image <= params['max_depth'])
        depth_mask = mask.astype(np.uint8) * 255

        # 形态学操作优化边缘
        kernel = np.ones((params['kernel_size'], params['kernel_size']), np.uint8)
        
        if params['morph_close']:
            depth_mask = cv2.morphologyEx(depth_mask, cv2.MORPH_CLOSE, kernel)
        if params['morph_open']:
            depth_mask = cv2.morphologyEx(depth_mask, cv2.MORPH_OPEN, kernel)
        
        # 应用深度掩码到彩色图像 - 这是相机原视角抠图
        masked_color = cv2.bitwise_and(color_image, color_image, mask=depth_mask)
        
        # 转换为HSV颜色空间
        hsv = cv2.cvtColor(masked_color, cv2.COLOR_BGR2HSV)
        
        # 检测红色区域
        lower_red1 = np.array([params['h_low1'], params['s_low'], params['v_low']])
        upper_red1 = np.array([params['h_high1'], 255, 255])
        lower_red2 = np.array([params['h_low2'], params['s_low'], params['v_low']])
        upper_red2 = np.array([params['h_high2'], 255, 255])
        
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask_red1, mask_red2)
        
        # 形态学操作优化红色区域
        if params['morph_close']:
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        if params['morph_open']:
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 检测圆形物体
        detected_circles = []
        result_image = color_image.copy()
        
        for contour in contours:
            # 计算轮廓面积
            area = cv2.contourArea(contour)
            if area < params['min_area']:  # 过滤小噪点
                continue
                
            # 计算最小外接圆
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            
            # 获取中心点深度值
            if 0 <= center[1] < depth_image.shape[0] and 0 <= center[0] < depth_image.shape[1]:
                depth_value = depth_image[center[1], center[0]]
                
                # 跳过无效深度点
                if depth_value == 0:
                    continue
                    
                # 计算实际物理尺寸（毫米）
                physical_radius = radius * depth_value / depth_intrinsics.fx
                
                # 检查是否在目标半径范围内
                if params['min_radius_mm'] <= physical_radius <= params['max_radius_mm']:
                    # 计算圆度（圆形度）
                    perimeter = cv2.arcLength(contour, True)
                    circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
                    
                    # 过滤非圆形物体
                    if circularity > params['circularity']:
                        detected_circles.append((center, radius, depth_value, physical_radius))
        
        # 在原图上标记检测结果
        for i, (center, radius, depth_value, physical_radius) in enumerate(detected_circles):
            # 绘制圆形和中心点
            cv2.circle(result_image, center, radius, (0, 255, 0), 2)
            cv2.circle(result_image, center, 2, (0, 0, 255), 3)
            
            # 计算圆心与相机中心点的距离（像素）
            dx = center[0] - camera_center[0]  # 水平距离（像素）
            dy = center[1] - camera_center[1]  # 垂直距离（像素）
            
            # 将像素距离转换为实际距离（毫米）
            # 使用深度值和相机内参进行转换
            fx = depth_intrinsics.fx  # 水平焦距
            fy = depth_intrinsics.fy  # 垂直焦距
            dx_mm = (dx * depth_value) / fx  # 水平实际距离（毫米）
            dy_mm = (dy * depth_value) / fy  # 垂直实际距离（毫米）
            
            # 显示深度和尺寸信息
            info_text = f"Depth: {depth_value/10:.1f}cm, Radius: {physical_radius/10:.1f}cm"
            cv2.putText(result_image, info_text, (center[0] - 50, center[1] - radius - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            # 显示圆心与相机中心的距离（毫米）
            dist_text = f"X: {dx_mm:.1f}mm, Y: {dy_mm:.1f}mm"
            cv2.putText(result_image, dist_text, (center[0] - 40, center[1] + radius + 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)  # 青色
            
            # 显示圆心坐标（红色字体）
            coord_text = f"({center[0]}, {center[1]})"
            cv2.putText(result_image, coord_text, (center[0] - 20, center[1] + radius + 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)  # 红色字体
        
        # 显示帧率和分辨率
        fps_text = f"FPS: {fps:.1f}"
        res_text = f"Res: {resolution}"
        cv2.putText(result_image, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(result_image, res_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 创建深度图显示 - 相机原视角抠图
        # 这是深度范围内的原始彩色图像，其他区域为黑色
        depth_vis = masked_color.copy()
        
        # 在深度图上标记深度范围边界
        contours, _ = cv2.findContours(depth_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(depth_vis, contours, -1, (0, 255, 0), 2)  # 用绿色轮廓标记深度范围边界
        
        # 添加深度范围信息
        depth_info = f"Depth: {params['min_depth']}-{params['max_depth']}mm"
        cv2.putText(depth_vis, depth_info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 在所有图像上标记相机中心点（去掉Center标签）
        for img in [color_image, depth_vis, result_image]:
            # 绘制相机中心点（蓝色大点）
            cv2.circle(img, camera_center, 5, (255, 0, 0), -1)  # 蓝色实心圆
            # 绘制十字准星
            cross_size = 15
            cv2.line(img, (center_x - cross_size, center_y), (center_x + cross_size, center_y), (255, 0, 0), 2)
            cv2.line(img, (center_x, center_y - cross_size), (center_x, center_y + cross_size), (255, 0, 0), 2)
        
        # 显示所有图像
        cv2.imshow('Original', color_image)
        cv2.imshow('Depth Masked Color', depth_vis)  # 显示相机原视角抠图
        cv2.imshow('Red Mask', red_mask)
        cv2.imshow('Result', result_image)
        
        # 添加参数信息到参数窗口
        param_display = np.zeros((400, 600, 3), dtype=np.uint8)
        cv2.putText(param_display, f"Depth Range: {params['min_depth']}-{params['max_depth']}mm", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1)
        cv2.putText(param_display, f"Red Range1: H{params['h_low1']}-{params['h_high1']}, S{params['s_low']}+, V{params['v_low']}+", (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
        cv2.putText(param_display, f"Red Range2: H{params['h_low2']}-{params['h_high2']}", (10, 90), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
        cv2.putText(param_display, f"Kernel Size: {params['kernel_size']}, Close: {'On' if params['morph_close'] else 'Off'}, Open: {'On' if params['morph_open'] else 'Off'}", (10, 120), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 1)
        cv2.putText(param_display, f"Min Area: {params['min_area']}, Radius: {params['min_radius_mm']}-{params['max_radius_mm']}mm", (10, 150), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 1)
        cv2.putText(param_display, f"Circularity: {params['circularity']*100:.0f}%", (10, 180), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        cv2.putText(param_display, f"Detected Circles: {len(detected_circles)}", (10, 210), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 1)
        cv2.putText(param_display, f"FPS: {fps:.1f}, Resolution: {resolution}", (10, 240), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        
        # 显示检测到的圆的坐标和距离信息
        y_offset = 270
        for i, (center, radius, depth_value, physical_radius) in enumerate(detected_circles):
            # 计算距离
            dx = center[0] - camera_center[0]
            dy = center[1] - camera_center[1]
            fx = depth_intrinsics.fx
            fy = depth_intrinsics.fy
            dx_mm = (dx * depth_value) / fx
            dy_mm = (dy * depth_value) / fy
            
            coord_text = f"Circle {i+1}: ({center[0]}, {center[1]})"
            dist_text = f"  Offset: X={dx_mm:.1f}mm, Y={dy_mm:.1f}mm"
            cv2.putText(param_display, coord_text, (10, y_offset), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
            cv2.putText(param_display, dist_text, (10, y_offset + 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
            y_offset += 40
        
        cv2.imshow('Parameters', param_display)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()