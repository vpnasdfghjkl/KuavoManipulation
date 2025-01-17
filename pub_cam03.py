#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def publish_image():
    # 初始化ROS节点
    rospy.init_node('camera_publisher', anonymous=True)

    # 创建一个Publisher，发布到/image_raw话题
    image_pub = rospy.Publisher('/cam_3/image_raw', Image, queue_size=10)

    # 使用cv_bridge将OpenCV图像转换为ROS图像消息
    bridge = CvBridge()
    ok_i = 0
    # 检查前 10 个索引
    for i in range(20):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ok_i = i
            print(f"摄像头索引 {i} 可用")
            cap.release()
        else:
            print(f"摄像头索引 {i} 不可用")
    # 打开第三个摄像头（索引为2）
    cap = cv2.VideoCapture(ok_i)  # 第三个相机的索引是2

    if not cap.isOpened():
        rospy.logerr("无法打开第三个摄像头！")
        return

    # 设置循环频率
    rate = rospy.Rate(30)  # 10Hz

    while not rospy.is_shutdown():
        # 读取一帧图像
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("无法读取第三个摄像头的图像")
            break

        # 将OpenCV图像转换为ROS图像消息
        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
        time_stamp = rospy.Time.now()
        ros_image.header.stamp = time_stamp
        # 发布图像消息
        image_pub.publish(ros_image)

        # 按照循环频率延时
        rate.sleep()

    # 释放摄像头
    cap.release()

if __name__ == '__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass