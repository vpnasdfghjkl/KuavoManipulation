import cv2
import numpy as np
import pyrealsense2 as rs
import glob
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 标定板参数
pattern_size = (8, 6)  # 格点数量 (列, 行)
square_size = 0.025  # 每个方块的边长（单位：米）

# 查找标定板角点
def find_corners(images, pattern_size):
    obj_points = []
    img_points = []
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    for img_path in images:
        img = cv2.imread(img_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if ret:
            obj_points.append(objp)
            img_points.append(corners)

            # 可视化角点
            cv2.drawChessboardCorners(img, pattern_size, corners, ret)
            # cv2.imshow("Chessboard Corners", img)
            cv2.waitKey(500)  # 等待 500ms
        else:
            print(f"Warning: Failed to find corners in {img_path}")

    cv2.destroyAllWindows()
    return obj_points, img_points, gray.shape[::-1] if img_points else (0, 0)

# 获取相机内参
def get_camera_intrinsics(serial):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 启动相机流
    pipeline.start(config)
    profile = pipeline.get_active_profile()
    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    pipeline.stop()

    # 构造内参矩阵和畸变系数
    camera_matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                               [0, intrinsics.fy, intrinsics.ppy],
                               [0, 0, 1]])
    dist_coeffs = np.array(intrinsics.coeffs)

    return camera_matrix, dist_coeffs

# 可视化相机姿态
def visualize_camera_pose(R, T):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 绘制原点和坐标系
    ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='X')
    ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='Y')
    ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='Z')

    # 绘制第二个相机的坐标系
    cam2_origin = T.ravel()
    cam2_axes = R @ np.eye(3)
    ax.quiver(cam2_origin[0], cam2_origin[1], cam2_origin[2], cam2_axes[0, 0], cam2_axes[1, 0], cam2_axes[2, 0], color='r', linestyle='dashed')
    ax.quiver(cam2_origin[0], cam2_origin[1], cam2_origin[2], cam2_axes[0, 1], cam2_axes[1, 1], cam2_axes[2, 1], color='g', linestyle='dashed')
    ax.quiver(cam2_origin[0], cam2_origin[1], cam2_origin[2], cam2_axes[0, 2], cam2_axes[1, 2], cam2_axes[2, 2], color='b', linestyle='dashed')

    # 设置图形属性
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.set_title('Camera Pose Visualization')
    ax.legend()
    plt.show()

# 主程序
serial1 = "342522073176"  # 替换为实际序列号
serial2 = "327122077711"  # 替换为实际序列号

# 自动获取相机内参
camera_matrix1, dist_coeffs1 = get_camera_intrinsics(serial1)
camera_matrix2, dist_coeffs2 = get_camera_intrinsics(serial2)

# 加载图像
images_cam1 = sorted(glob.glob("/home/lab/hanxiao/KuavoManipulation/camera1_images/*.png"))
images_cam2 = sorted(glob.glob("/home/lab/hanxiao/KuavoManipulation/camera2_images/*.png"))

# 查找标定板角点
obj_points, img_points_cam1, img_size = find_corners(images_cam1, pattern_size)
_, img_points_cam2, _ = find_corners(images_cam2, pattern_size)

# 立体标定
RET = cv2.stereoCalibrate(
    obj_points, img_points_cam1, img_points_cam2,
    camera_matrix1, dist_coeffs1,
    camera_matrix2, dist_coeffs2,
    img_size, flags=cv2.CALIB_FIX_INTRINSIC
)

print("立体标定结果:")
print("旋转矩阵 R:\n", RET[5])
print("平移向量 T:\n", RET[6])

# 可视化相机姿态
visualize_camera_pose(RET[5], RET[6])

'''
立体标定结果:
旋转矩阵 R:
 [[ 0.74125169  0.51794476 -0.42694164]
 [ 0.04629505  0.59509685  0.80231945]
 [ 0.66962878 -0.61448593  0.41713827]]
平移向量 T:
 [[ 0.56455036]
 [-0.47033225]
 [ 0.19663884]]
 '''
 
'''
旋转矩阵 R:
 [[ 0.73297107  0.52502894 -0.43254828]
 [ 0.04750707  0.59479756  0.80247052]
 [ 0.67859892 -0.60873678  0.41102682]]
平移向量 T:
 [[ 0.5666239 ]
 [-0.46957261]
 [ 0.20388565]]
 
 旋转矩阵 R:
 [[ 0.79232755  0.48457278 -0.37068351]
 [ 0.04370907  0.56093533  0.82670495]
 [ 0.6085282  -0.67122334  0.4232643 ]]
平移向量 T:
 [[ 0.43915629]
 [-0.47573846]
 [ 0.77638518]]
 
 旋转矩阵 R:
 [[ 0.79964245  0.47627515 -0.3656965 ]
 [ 0.0440602   0.56083086  0.82675719]
 [ 0.59885778 -0.6772228   0.42747939]]
平移向量 T:
 [[ 0.43955772]
 [-0.47576271]
 [ 0.77819415]]
 '''