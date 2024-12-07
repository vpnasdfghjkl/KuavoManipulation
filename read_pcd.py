import rosbag
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import struct
def extract_pointcloud_from_rosbag(bag_file, topic_name, output_pcd_file):
    """
    从 rosbag 文件中提取点云数据并保存为 PCD 文件。
    
    参数:
        bag_file: str, rosbag 文件路径
        topic_name: str, 点云数据的 topic 名称
        output_pcd_file: str, 保存的 PCD 文件路径
    """
    # 打开 rosbag 文件
    bag = rosbag.Bag(bag_file, 'r')
    
    # 初始化用于存储点云的列表
    points = []

    # 遍历 rosbag 中的指定 topic
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        if topic == topic_name:
            print(f"读取消息类型: {type(msg)}")  # 打印消息类型，以确保正确解析
    
            # 将 PointCloud2 数据转换为点云点的生成器
            pc_generator = pc2.read_points(
                msg, field_names=("x", "y", "z", "rgb"), skip_nans=True
            )
            
            # 将点云点从生成器转换为列表
            for point in pc_generator:
                # 提取 x, y, z, rgb 数据
                x, y, z, rgb = point[:4]
                print(type(rgb))
                

                points.append([x, y, z, rgb])

        break

    # 关闭 rosbag 文件
    bag.close()

    # 创建 Open3D 的点云对象
    point_cloud = o3d.geometry.PointCloud()

    # 转换为 numpy 数组
    points_np = np.array(points, dtype=np.float32)

    # 设置点云的坐标
    point_cloud.points = o3d.utility.Vector3dVector(points_np[:, :3])

    # 处理 RGB 数据
    colors = []
    for rgb in points_np[:, 3]:
        # 假设 rgb 是一个 32 位整数，且存储为 0xRRGGBB 格式
        # 使用 struct.pack() 将 float 转换为字节
        byte_rep = struct.pack('f', rgb)  # 'f' 表示 float 类
        # 打印字节表示
        print(byte_rep)
        # 以十六进制格式打印字节表示
        print("Hex format:", byte_rep.hex())
        rgb_int = struct.unpack('I', struct.pack('f', rgb))[0]
        print(rgb_int)
        # 提取 RGB 分量
        b = (rgb_int & 0xFF)  # 提取红色通道
        g = ((rgb_int >> 8) & 0xFF)  # 提取绿色通道
        r = ((rgb_int >> 16) & 0xFF)  # 提取蓝色通道
        
        # 将 RGB 归一化到 [0, 1] 之间
        colors.append([r / 255.0, g / 255.0, b / 255.0])

    # 设置点云的颜色
    point_cloud.colors = o3d.utility.Vector3dVector(np.array(colors))

    # 保存点云为 PCD 文件
    o3d.io.write_point_cloud(output_pcd_file, point_cloud)
    print(f"点云已保存到 {output_pcd_file}")

if __name__ == "__main__":
    # 配置 rosbag 文件和点云 topic 名称
    bag_file_path = "/home/lab/2024-12-07-17-08-36.bag"  # 替换为您的 rosbag 文件路径
    topic_name = "/cam_2/depth/color/points"  # 替换为您的点云 topic
    output_pcd_file = "/home/lab/pcd.pcd" # 保存点云的路径

    # 调用函数提取并保存点云
    extract_pointcloud_from_rosbag(bag_file_path, topic_name, output_pcd_file)
    
    # 读取和可视化点云
    pcd = o3d.io.read_point_cloud(output_pcd_file)
    o3d.visualization.draw_geometries([pcd])




'''
[
  132,
  86,
  52,
  191,
  190,
  17,
  13,
  191,
  209,
  34,
  171,
  63,
  0,
  0,
  0,
  0,
  196,
  200,
  199,
  0,

  82,
  197,
  51,
  191,
  190,
  17,
  13,
  191,
  209,
  34,
  171,
  63,
  0,
  0,
  0,
  0,
  194,
  198,
  197,
  0,

]
'''