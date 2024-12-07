import rospy
import pyrealsense2 as rs
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
from sensor_msgs import point_cloud2

def depth_to_pointcloud(depth_frame, color_frame, intrinsics):
    """
    将对齐后的深度图像和RGB图像转化为点云
    """
    # 获取深度图像数据
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    
    # 获取图像的尺寸
    height, width = depth_image.shape
    
    # 获取相机内参
    depth_intrinsics = intrinsics
    depth_scale = 0.001  # 深度图像的单位通常是毫米，我们将其转换为米
    
    # 创建空的点云数组
    points = []
    
    # 生成点云数据
    for v in range(height):
        for u in range(width):
            # 获取深度值
            depth = depth_image[v, u] * depth_scale
            if depth == 0:
                continue  # 跳过无效的点
            
            # 获取颜色值
            color = color_image[v, u]
            
            # 计算世界坐标 (x, y, z)
            x = (u - depth_intrinsics.ppx) * depth / depth_intrinsics.fx
            y = (v - depth_intrinsics.ppy) * depth / depth_intrinsics.fy
            z = depth
            
            # 添加点云数据
            points.append([x, y, z, color[0], color[1], color[2]])
    
    # 转换为NumPy数组
    points = np.array(points, dtype=np.float32)
    
    # 将点云数据转换为PointCloud2消息
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "camera_link"
    
    pc_data = point_cloud2.create_cloud_xyz32(header, points[:, :3])
    
    # 使用Open3D进行可视化
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])  # 设置3D点坐标
    pcd.colors = o3d.utility.Vector3dVector(points[:, 3:] / 255.0)  # 设置颜色（归一化为0-1）
    
    # 可视化点云
    o3d.visualization.draw_geometries([pcd])
    
    return pc_data

def main():
    rospy.init_node('realsense_pointcloud', anonymous=True)
    
    # 创建Realsense管道
    pipeline = rs.pipeline()
    
    # 配置流
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    # 启动管道
    pipeline.start(config)
    
    # 创建对齐对象，深度图像和RGB图像对齐
    align_to = rs.stream.color
    align = rs.align(align_to)
    
    # 获取相机内参
    profile = pipeline.get_active_profile()
    intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    
    # 创建点云发布者
    pub = rospy.Publisher('/pcd', PointCloud2, queue_size=1)
    
    rate = rospy.Rate(10)  # 发布频率 10Hz
    
    while not rospy.is_shutdown():
        # 获取帧
        frames = pipeline.wait_for_frames()
        
        # 对齐帧
        aligned_frames = align.process(frames)
        
        # 获取RGB和对齐后的深度帧
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        # 获取点云
        pc_data = depth_to_pointcloud(depth_frame, color_frame, intrinsics)
        
        # 发布点云数据
        pub.publish(pc_data)
        
        rate.sleep()
    
    # 停止管道
    pipeline.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
