import pyrealsense2 as rs
import cv2
import os
import numpy as np

# 创建文件夹保存图像
os.makedirs("camera1_images", exist_ok=True)
os.makedirs("camera2_images", exist_ok=True)

# 初始化相机管道
pipeline1 = rs.pipeline()
pipeline2 = rs.pipeline()

# 配置两个相机
config1 = rs.config()
config2 = rs.config()

# 设置相机序列号（使用 Realsense Viewer 获取序列号）
camera1_serial = "342522073176"  # 替换为相机1的序列号
camera2_serial = "327122077711"  # 替换为相机2的序列号

config1.enable_device(camera1_serial)
config2.enable_device(camera2_serial)

config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 启动相机
pipeline1.start(config1)
pipeline2.start(config2)

try:
    print("按 's' 键捕获图像，按 'q' 键退出。")
    index = 0
    while True:
        # 获取两台相机的帧
        frames1 = pipeline1.wait_for_frames()
        frames2 = pipeline2.wait_for_frames()

        color_frame1 = frames1.get_color_frame()
        color_frame2 = frames2.get_color_frame()

        # 转换为 numpy 数组
        img1 = np.asanyarray(color_frame1.get_data())
        img2 = np.asanyarray(color_frame2.get_data())

        # 显示图像
        cv2.imshow("Camera 1", img1)
        cv2.imshow("Camera 2", img2)

        key = cv2.waitKey(1)
        if key == ord('s'):  # 保存图像
            cv2.imwrite(f"camera1_images/img_{index}.png", img1)
            cv2.imwrite(f"camera2_images/img_{index}.png", img2)
            print(f"保存了第 {index} 组图像")
            index += 1
        elif key == ord('q'):  # 退出
            break
finally:
    pipeline1.stop()
    pipeline2.stop()
    cv2.destroyAllWindows()
