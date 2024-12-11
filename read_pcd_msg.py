#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import time
from utils.pcd_util import save_to_pcd, display_point_cloud, optimized_extract_xyzrgb_from_pointcloud2,display_point_cloud_with_cursor_info
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf2_ros
import geometry_msgs.msg

def transform_point_cloud(xyzrgb, transform_matrix):
    """
    将点云从一个坐标系转换到另一个坐标系。
    """
    xyz = xyzrgb[:, :3]
    ones = np.ones((xyz.shape[0], 1))
    homogenous_xyz = np.hstack([xyz, ones])
    transformed_xyz = homogenous_xyz @ transform_matrix.T
    xyzrgb[:, :3] = transformed_xyz[:, :3]
    return xyzrgb


def merge_point_clouds(clouds):
    """
    合并多个点云。
    """
    return np.vstack(clouds)


class PointCloudMerger:
    def __init__(self, output_files, merge_file, transform_matrices):
        self.output_files = output_files
        self.merge_file = merge_file
        self.transform_matrices = transform_matrices  # 自定义变换矩阵
        self.point_clouds = [None] * len(output_files)

    def callback(self, msg1, msg2):
        try:
            # 从两个话题提取点云
            xyzrgb1 = optimized_extract_xyzrgb_from_pointcloud2(msg1)
            xyzrgb2 = optimized_extract_xyzrgb_from_pointcloud2(msg2)

            # 将点云变换到世界坐标系
            xyzrgb1 = transform_point_cloud(xyzrgb1, self.transform_matrices[0])
            xyzrgb2 = transform_point_cloud(xyzrgb2, self.transform_matrices[1])

            # # 保存单独的点云
            # save_to_pcd(xyzrgb1, self.output_files[0])
            # save_to_pcd(xyzrgb2, self.output_files[1])
            # rospy.loginfo(f"Saved transformed point clouds to {self.output_files}")

            # # 合并点云
            merged_cloud = merge_point_clouds([xyzrgb1, xyzrgb2])
            # save_to_pcd(merged_cloud, self.merge_file)
            display_point_cloud(merged_cloud)
            # rospy.loginfo(f"Saved merged point cloud to {self.merge_file}")

        except Exception as e:
            rospy.logerr(f"Error processing point clouds: {e}")


def main():
    rospy.init_node("pointcloud_merger", anonymous=True)
    output_files = ["cam1.pcd", "cam2.pcd"]
    merge_file = "merge.pcd"

    # 自定义变换矩阵 (根据实际配置修改)
    transform_matrices = [
    # First transformation matrix
    np.array([
        [ 0.79964245,  0.47627515, -0.3656965 ,  0.43955772 ],
        [ 0.0440602,   0.56083086,  0.82675719, -0.47576271],
        [ 0.59885778, -0.6772228,   0.42747939,  0.77819415],
        [ 0,           0,           0,           1          ]
    ]),

    # Identity matrix (no transformation)
    np.eye(4)  # Equivalent to np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    ]


    merger = PointCloudMerger(output_files, merge_file, transform_matrices)

    # 使用 ApproximateTimeSynchronizer 确保点云同步
    topic1 = "/cam_1/depth/color/points"
    topic2 = "/cam_2/depth/color/points"

    sub1 = Subscriber(topic1, PointCloud2)
    sub2 = Subscriber(topic2, PointCloud2)

    ats = ApproximateTimeSynchronizer([sub1, sub2], queue_size=10, slop=0.1)
    ats.registerCallback(merger.callback)

    rospy.loginfo("Listening to synchronized point cloud topics...")
    rospy.spin()


if __name__ == "__main__":
    main()
