import rosbag
from scipy.spatial.transform import Rotation as R
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, JointState
import time
bag_name = '/home/lab/hanxiao/dataset/kuavo/task_toy/toy_1/task_toy_2024-11-27-15-24-17.bag'

bag = rosbag.Bag(bag_name, 'r')
rospy.init_node('check_ik', anonymous=True)
target_pub = rospy.Publisher(
            "/drake_ik/target_LHandEef",
            Float32MultiArray, 
            queue_size=10
        )

joint_pub = rospy.Publisher(
            "/kuavo_arm_traj",
            JointState,
            queue_size=10
        )

traj_eef_a = []
for topic, msg, t in bag.read_messages(topics=[ '/kuavo_arm_traj',\
                                                    '/robot_arm_q_v_tau',\
                                                    '/drake_ik/cmd_arm_hand_pose',\
                                                    '/drake_ik/real_arm_hand_pose', \
                                                    '/robot_hand_eff',\
                                                    '/robot_hand_position',\
                                                    '/camera1/color/image_raw/compressed',\
                                                    '/camera2/color/image_raw/compressed',
                                                  ]):
    # if topic == '/drake_ik/cmd_arm_hand_pose':
    #     left_right_eef = [
    #         np.concatenate((np.array(pose.pos_xyz), R.from_quat(pose.quat_xyzw).as_euler('xyz')))
    #         for pose in [msg.left_pose, msg.right_pose]
    #     ]
    #     print(left_right_eef) 
    #     traj_eef_a.append(np.concatenate(left_right_eef))        

    #     target_pub.publish(Float32MultiArray(data=left_right_eef[0]))
    #     rospy.loginfo(f"Published message at time {t.to_sec()}: {msg}")
    #     time.sleep(0.01)

    if topic == '/kuavo_arm_traj':
        joint = msg.position[:7]
        # joint = np.rad2deg(joint)
        arm_min = [-180, -20, -135, -100, -135, -45, -45, -180, -180, -180, -180, -180, -45, -45]
        arm_max = [30,   135,  135,  100,  135,  45,  45,  180,  180,  180,  180,  180,  45,  45]
        joint_state = JointState()
        positions  = [0 for _ in range(14)]
        velocities = [0 for _ in range(14)]
        for i in range(len(joint)):
            if joint[i] < arm_min[i]:
                joint[i] = arm_min[i]
            elif joint[i] > arm_max[i]:
                joint[i] = arm_max[i]
        positions[0:7] = joint
        print("send_angle:",[round(x,1) for x in joint])
        velocities[0:7] = [0]*7
        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.header.stamp = rospy.Time.now()
        joint_pub.publish(joint_state)
        rospy.loginfo(f"Published message at time {t.to_sec()}: {msg}")
        time.sleep(0.01)

from matplotlib import pyplot as plt
traj_eef_a = np.array(traj_eef_a)
print(traj_eef_a.shape)
for i in range(7):
    plt.plot(traj_eef_a[:,i])
plt.show()
        