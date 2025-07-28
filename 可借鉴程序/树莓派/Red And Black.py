import numpy as np
import cv2
# from Unet import *
# import multiprocessing as mp
# from cv_bridge import CvBridge,CvBridgeError #ROS和cv通道
# from ackermann_msgs.msg import AckermannDrive
# from std_msgs.msg import Bool
# import signal
# from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped

def nothing(x):
    pass
#
# def apply_color_filter(image, lower_color, upper_color):
#     hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#     mask = cv2.inRange(hsv, lower_color, upper_color)
#     result = cv2.bitwise_and(image, image, mask=mask)
#     return result

# 摄像头每帧图像处理函数
def process_frame(frame):
    # 将图像转换为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 使用Canny边缘检测算法获取图像边缘
    edges = cv2.Canny(gray, 50, 150)

    # 利用霍夫变换检测直线
    lines = cv2.HoughLinesP(edges, 1, 3.14 / 180, 100, minLineLength=50, maxLineGap=10)

    # 绘制检测到的直线
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

    return frame


def main():
    cap = cv2.VideoCapture(0)  # 打开摄像头
    cv2.namedWindow('Result')

    # 设置红色的颜色范围
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    # 设置黑色的颜色范围
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([179, 255, 30])

    cv2.createTrackbar('H_min', 'Result', 0, 255, nothing)
    cv2.createTrackbar('S_min', 'Result', 0, 255, nothing)
    cv2.createTrackbar('V_min', 'Result', 0, 255, nothing)
    cv2.createTrackbar('H_max', 'Result', 0, 255, nothing)
    cv2.createTrackbar('S_max', 'Result', 0, 255, nothing)
    cv2.createTrackbar('V_max', 'Result', 0, 255, nothing)

    cv2.setTrackbarPos('H_min', 'Result', 113)
    cv2.setTrackbarPos('S_min', 'Result', 24)
    cv2.setTrackbarPos('V_min', 'Result', 210)
    cv2.setTrackbarPos('H_max', 'Result', 216)
    cv2.setTrackbarPos('S_max', 'Result', 39)
    cv2.setTrackbarPos('V_max', 'Result', 246)

    while True:
        ret, frame = cap.read()  # 读取视频帧

        if not ret:
            break
        # 处理当前帧图像
        processed_frame = process_frame(frame)

        # 显示处理后的图像
        cv2.imshow('Line Detection', processed_frame)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


        h_min = cv2.getTrackbarPos('H_min', 'Result')
        s_min = cv2.getTrackbarPos('S_min', 'Result')
        v_min = cv2.getTrackbarPos('V_min', 'Result')
        h_max = cv2.getTrackbarPos('H_max', 'Result')
        s_max = cv2.getTrackbarPos('S_max', 'Result')
        v_max = cv2.getTrackbarPos('V_max', 'Result')

        lower_color = np.array([h_min, s_min, v_min])
        upper_color = np.array([h_max, s_max, v_max])

        # result = apply_color_filter(frame, lower_color, upper_color)
        # cv2.imshow('Result', result)


        mask_red = cv2.inRange(hsv, lower_color, upper_color)
        mask_black = cv2.inRange(hsv, lower_black, upper_black)

        result = cv2.bitwise_or(mask_red, mask_black)  # 合并红色和黑色蒙版

        output = cv2.bitwise_and(frame, frame, mask=result)

        red_all = np.column_stack(np.where(output >= 245))
        red_center_x = np.mean(red_all[:, 0])
        red_center_y = np.mean(red_all[:, 1])
        print('red_x=',red_center_x,'red_y=',red_center_y)

        # 计算舵机需要转动的角度
        # angle_x = np.arctan2(target_center[0] - paper_center[0], 1000) * 180 / np.pi
        # angle_y = np.arctan2(target_center[1] - paper_center[1], 1000) * 180 / np.pi
        # print(angle_x, angle_y)


        cv2.imshow('Result', output)

        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()  # 释放摄像头
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
