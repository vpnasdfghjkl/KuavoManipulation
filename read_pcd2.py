import open3d as o3d
import numpy as np
# 读取 PCD 文件
pcd = o3d.io.read_point_cloud("/home/lab/hanxiao/dataset/kuavo/task_pcd_test/pcd/1733549244.628916264.pcd")
import open3d as o3d
import numpy as np

def load_pcd_and_visualize(pcd_file):
    """
    从 PCD 文件加载点云数据并用 NumPy 数组可视化。
    
    参数:
        pcd_file: str, PCD 文件的路径
    """
    # 从 PCD 文件加载点云
    pcd = o3d.io.read_point_cloud(pcd_file)

    # 提取点云坐标数据（转换为 NumPy 数组）
    points_np = np.asarray(pcd.points)

    # 提取点云颜色数据（转换为 NumPy 数组）
    colors_np = np.asarray(pcd.colors)

    # 可选：你可以对 points_np 和 colors_np 做进一步处理或转换

    # 创建新的点云对象
    new_pcd = o3d.geometry.PointCloud()

    # 设置新的点云坐标
    new_pcd.points = o3d.utility.Vector3dVector(points_np)

    # 设置新的点云颜色
    new_pcd.colors = o3d.utility.Vector3dVector(colors_np)

    # 可视化点云
    o3d.visualization.draw_geometries([new_pcd])

# 示例用法
pcd_file_path = "/home/lab/hanxiao/dataset/kuavo/task_pcd_test/pcd/1733549244.628916264.pcd"  # 替换为你的 PCD 文件路径
load_pcd_and_visualize(pcd_file_path)
