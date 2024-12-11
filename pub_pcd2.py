import threading
import rospy
import pyrealsense2 as rs
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
from sensor_msgs import point_cloud2
import time 
def depth_to_pointcloud_optimized(depth_frame, color_frame, intrinsics):
    """
    使用矢量化操作将对齐后的深度图像和RGB图像转化为点云。
    """
    # 获取深度图像和RGB图像数据
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # 获取图像的尺寸
    height, width = depth_image.shape

    # 转换深度单位为米
    depth_scale = 0.001  # 深度值转换为米
    depth_image = depth_image * depth_scale

    # 创建像素坐标网格
    u, v = np.meshgrid(np.arange(width), np.arange(height))

    # 根据相机内参计算点云的 x, y, z 坐标
    x = (u - intrinsics.ppx) * depth_image / intrinsics.fx
    y = (v - intrinsics.ppy) * depth_image / intrinsics.fy
    z = depth_image

    # 将无效深度点过滤（例如 0 或超出范围的点）
    valid = (z > 0.3) & (z <= 1.0)  # 设置深度范围
    x, y, z = x[valid], y[valid], z[valid]
    colors = color_image[valid]

    # 将点云数据合并
    points = np.column_stack((x, y, z, colors))

    return points

def process_frames(pipeline, align, intrinsics, pub):
    while not rospy.is_shutdown():
        # 获取帧并对齐
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        # 获取RGB和深度帧
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        # 如果帧为空，跳过
        if not depth_frame or not color_frame:
            continue

        # 生成点云数据
        cur_time = time.time()  
        pc_data = depth_to_pointcloud_optimized(depth_frame, color_frame, intrinsics)
        print(f"Point cloud generated in {time.time() - cur_time:.5f} seconds")
        # 发布点云数据
        # header = std_msgs.msg.Header()
        # header.stamp = rospy.Time.now()
        # header.frame_id = "camera_link"
        # cloud_msg = point_cloud2.create_cloud_xyz32(header, pc_data[:, :3])
        # pub.publish(cloud_msg)

def main():
    rospy.init_node('realsense_pointcloud', anonymous=True)

    # 创建Realsense管道
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    # 创建对齐对象
    align = rs.align(rs.stream.color)

    # 获取相机内参
    profile = pipeline.get_active_profile()
    intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

    # 创建点云发布者
    pub = rospy.Publisher('/pcd', PointCloud2, queue_size=1)

    # 启动帧处理线程
    threading.Thread(target=process_frames, args=(pipeline, align, intrinsics, pub), daemon=True).start()

    rospy.spin()
    pipeline.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
