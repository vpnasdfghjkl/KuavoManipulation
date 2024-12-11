import rospy
import tf2_ros
import geometry_msgs.msg
import numpy as np
def publish_transform():
    
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    transform_cam2 = geometry_msgs.msg.TransformStamped()
    current_time = rospy.Time.now()
    transform_cam2.header.stamp = current_time
    transform_cam2.header.frame_id = "world"
    transform_cam2.child_frame_id = "cam_2_link"

    transform_cam2.transform.translation.x = 0.0
    transform_cam2.transform.translation.y = 0.0
    transform_cam2.transform.translation.z = 0.0
    transform_cam2.transform.rotation.x = 0.0
    transform_cam2.transform.rotation.y = 0.0
    transform_cam2.transform.rotation.z = 0.0
    transform_cam2.transform.rotation.w = 1.0



    tf_broadcaster.sendTransform(transform_cam2)
    transform_cam1 = geometry_msgs.msg.TransformStamped()
    transform_cam1.header.stamp = current_time
    transform_cam1.header.frame_id = "world" 
    transform_cam1.child_frame_id = "cam_1_link"

    transform_cam1.transform.translation.x = 0.43955772  
    transform_cam1.transform.translation.y = -0.47576271
    transform_cam1.transform.translation.z = 0.77819415
    
    
    def normalize_quaternion(x, y, z, w):
        norm = np.sqrt(x**2 + y**2 + z**2 + w**2)
        return x / norm, y / norm, z / norm, w / norm
    
    transform_matrix = np.array([
        [ 0.79964245,  0.47627515, -0.3656965 ,  0.43955772 ],
        [ 0.0440602,   0.56083086,  0.82675719, -0.47576271],
        [ 0.59885778, -0.6772228,   0.42747939,  0.77819415],
        [ 0,           0,           0,           1          ]
    ])
    rotation_matrix = np.array([
    [ 0.79964245,  0.47627515, -0.3656965 ],
    [ 0.0440602,   0.56083086,  0.82675719],
    [ 0.59885778, -0.6772228,   0.42747939]
])
    from scipy.spatial.transform import Rotation as R
    # 交换 X 和 Z 轴
    rotation_matrix_swapped = rotation_matrix.copy()
    # rotation_matrix_swapped[:, [0, 2]] = rotation_matrix[:, [2, 0]]

    # 使用 scipy 的 Rotation 类将旋转矩阵转换为四元数
    rotation = R.from_matrix(rotation_matrix_swapped)
    qx, qy, qz, qw = rotation.as_quat()
    qx, qy, qz, qw = normalize_quaternion(qx, qy, qz, qw)
   
    transform_cam1.transform.rotation.x = qx
    transform_cam1.transform.rotation.y = qy
    transform_cam1.transform.rotation.z = qz
    transform_cam1.transform.rotation.w = qw
    

    tf_broadcaster.sendTransform(transform_cam1)

if __name__ == "__main__":
    rospy.init_node("coordinate_transform_publisher")
    rospy.loginfo("Publishing coordinate transforms...")
    
    rate = rospy.Rate(10)  # 设置发布频率 (10Hz)
    while not rospy.is_shutdown():
        publish_transform()
        rate.sleep()