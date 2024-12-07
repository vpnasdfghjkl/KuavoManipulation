import rospy
import pyrealsense2 as rs
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from sensor_msgs import point_cloud2
import std_msgs.msg
from cv_bridge import CvBridge
import message_filters

class RealsensePointCloud:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('realsense_pointcloud', anonymous=True)
        
        # 创建cv_bridge实例，用于将ROS图像消息转换为OpenCV格式
        self.bridge = CvBridge()

        # 订阅RGB和深度图像
        self.depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        self.color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        
        # 订阅相机内参信息
        self.info_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo)
        
        # 使用时间同步来订阅多个话题
        self.ts = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.color_sub, self.info_sub], 10, 0.1)
        self.ts.registerCallback(self.callback)

        # 发布点云消息
        self.pc_pub = rospy.Publisher('/pcd', PointCloud2, queue_size=1)

        # 初始化Open3D点云
        self.pcd = o3d.geometry.PointCloud()

    def callback(self, depth_msg, color_msg, camera_info_msg):
        """
        处理接收到的图像数据，转换为点云并发布。
        """
        # 使用cv_bridge将ROS图像消息转换为OpenCV格式
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        
        # 获取相机内参
        depth_intrinsics = camera_info_msg.K
        
        # 转换为内参格式
        fx = depth_intrinsics[0]  # 焦距
        fy = depth_intrinsics[4]  # 焦距
        ppx = depth_intrinsics[2] # 光心X
        ppy = depth_intrinsics[5] # 光心Y
        
        # 获取图像尺寸
        height, width = depth_image.shape

        # 获取深度缩放系数
        depth_scale = 0.001  # 深度图像的单位通常是毫米，我们将其转换为米

        # 创建空的点云数据
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
                x = (u - ppx) * depth / fx
                y = (v - ppy) * depth / fy
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
        self.pcd.points = o3d.utility.Vector3dVector(points[:, :3])  # 设置3D点坐标
        self.pcd.colors = o3d.utility.Vector3dVector(points[:, 3:] / 255.0)  # 设置颜色（归一化为0-1）
        
        # 可视化点云
        o3d.visualization.draw_geometries([self.pcd])

        # 发布点云数据
        self.pc_pub.publish(pc_data)

    def run(self):
        rospy.spin()  # 保持ROS节点运行

if __name__ == '__main__':
    try:
        realsense_pc = RealsensePointCloud()
        realsense_pc.run()
    except rospy.ROSInterruptException:
        pass
