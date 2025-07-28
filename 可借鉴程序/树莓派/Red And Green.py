import cv2
import numpy as np

def nothing(x):
    pass

def create_mask(image, lower_color, upper_color):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    return mask

def main():
    cap = cv2.VideoCapture(0)  # 打开摄像头

    cv2.namedWindow('Result')

    # 创建滑动条
    cv2.createTrackbar('H_min', 'Result', 0, 255, nothing)
    cv2.createTrackbar('S_min', 'Result', 0, 255, nothing)
    cv2.createTrackbar('V_min', 'Result', 0, 255, nothing)
    cv2.createTrackbar('H_max', 'Result', 0, 255, nothing)
    cv2.createTrackbar('S_max', 'Result', 0, 255, nothing)
    cv2.createTrackbar('V_max', 'Result', 0, 255, nothing)
    # #RED
    # cv2.setTrackbarPos('H_min', 'Result', 122)
    # cv2.setTrackbarPos('S_min', 'Result', 33)
    # cv2.setTrackbarPos('V_min', 'Result', 209)
    # cv2.setTrackbarPos('H_max', 'Result', 177)
    # cv2.setTrackbarPos('S_max', 'Result', 126)
    # cv2.setTrackbarPos('V_max', 'Result', 255)
    # GREEN
    cv2.setTrackbarPos('H_min', 'Result', 75)
    cv2.setTrackbarPos('S_min', 'Result', 52)
    cv2.setTrackbarPos('V_min', 'Result', 167)
    cv2.setTrackbarPos('H_max', 'Result', 204)
    cv2.setTrackbarPos('S_max', 'Result', 178)
    cv2.setTrackbarPos('V_max', 'Result', 255)



    while True:
        ret, frame = cap.read()  # 读取视频帧

        if not ret:
            break

        # 获取滑动条的当前值
        # red_h_min = cv2.getTrackbarPos('H_min', 'Result')
        # red_s_min = cv2.getTrackbarPos('S_min', 'Result')
        # red_v_min = cv2.getTrackbarPos('V_min', 'Result')
        # red_h_max = cv2.getTrackbarPos('H_max', 'Result')
        # red_s_max = cv2.getTrackbarPos('S_max', 'Result')
        # red_v_max = cv2.getTrackbarPos('V_max', 'Result')

        green_h_min = cv2.getTrackbarPos('H_min', 'Result')
        green_s_min = cv2.getTrackbarPos('S_min', 'Result')
        green_v_min = cv2.getTrackbarPos('V_min', 'Result')
        green_h_max = cv2.getTrackbarPos('H_max', 'Result')
        green_s_max = cv2.getTrackbarPos('S_max', 'Result')
        green_v_max = cv2.getTrackbarPos('V_max', 'Result')

        lower_red = np.array([122, 33, 209])
        upper_red = np.array([177, 126, 255])
        #
        lower_green = np.array([74, 75, 210])
        upper_green = np.array([160, 165, 255])


        # lower_red = np.array([red_h_min, red_s_min, red_v_min])
        # upper_red = np.array([red_h_max, red_s_max, red_v_max])
        #
        # lower_green = np.array([green_h_min, green_s_min, green_v_min])
        # upper_green = np.array([green_h_max, green_s_max, green_v_max])

        red_mask = create_mask(frame, lower_red, upper_red)
        green_mask = create_mask(frame, lower_green, upper_green)

        red = cv2.bitwise_and(frame, frame, mask=red_mask)  # 将两个蒙版应用于原始图像
        red_all = np.column_stack(np.where(red >= 245))
        red_center_x = np.mean(red_all[:, 0])
        red_center_y = np.mean(red_all[:, 1])
        print('red_x=', red_center_x, 'red_y=', red_center_y)

        green = cv2.bitwise_and(frame, frame, mask=green_mask)  # 将两个蒙版应用于原始图像
        green_all = np.column_stack(np.where(green >= 245))
        green_center_x = np.mean(green_all[:, 0])
        green_center_y = np.mean(green_all[:, 1])
        print('green_x=', green_center_x, 'green_y=', green_center_y)

        # 计算舵机需要转动的角度
        angle_x = np.arctan2(red_center_x - green_center_x, 1000) * 180 / np.pi
        angle_y = np.arctan2(red_center_y - green_center_y, 1000) * 180 / np.pi
        print(angle_x, angle_y)

        #还想再加个滤波除噪，但不会

        result = cv2.bitwise_and(frame, frame, mask=red_mask | green_mask)  # 将两个蒙版应用于原始图像


        cv2.imshow('Result', result)

        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()  # 释放摄像头
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
