import rospy
import pyrealsense2 as rs
import numpy as np
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
from sensor_msgs import point_cloud2
import time
import threading

class PointCloudPublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('realsense_pointcloud', anonymous=True)

        # Create Realsense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        # Create alignment object
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # Get camera intrinsics
        profile = self.pipeline.get_active_profile()
        self.intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

        # Create point cloud publisher
        self.pub = rospy.Publisher('/pcd', PointCloud2, queue_size=1)

        # Create a thread for publishing
        self.pub_thread = threading.Thread(target=self.publish_point_cloud)
        self.pub_thread.daemon = True
        self.pub_thread.start()

    def depth_to_pointcloud(self, depth_frame, color_frame):
        """
        Convert depth and color frames to a point cloud.
        """
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Convert depth units to meters
        depth_scale = 0.001
        depth_image = depth_image * depth_scale

        height, width = depth_image.shape
        u, v = np.meshgrid(np.arange(width), np.arange(height))

        # Compute 3D coordinates
        x = (u - self.intrinsics.ppx) * depth_image / self.intrinsics.fx
        y = (v - self.intrinsics.ppy) * depth_image / self.intrinsics.fy
        z = depth_image

        # Filter invalid points
        valid = (z > 0.3) & (z <= 1.0)
        x, y, z = x[valid], y[valid], z[valid]
        colors = color_image[valid]

        points = np.column_stack((x, y, z, colors))

        # Create PointCloud2 message
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_link"
        pc_data = point_cloud2.create_cloud_xyz32(header, points[:, :3])

        return pc_data

    def publish_point_cloud(self):
        """
        Continuously capture frames and publish point cloud data.
        """
        rate = rospy.Rate(30)  # Publish at 30Hz

        while not rospy.is_shutdown():
            start_time = time.time()
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            # Process and publish the point cloud
            pc_data = self.depth_to_pointcloud(depth_frame, color_frame)
            self.pub.publish(pc_data)

            print(f"Point cloud published in {time.time() - start_time:.5f} seconds")
            rate.sleep()

    def stop(self):
        self.pipeline.stop()

if __name__ == '__main__':
    try:
        point_cloud_publisher = PointCloudPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
