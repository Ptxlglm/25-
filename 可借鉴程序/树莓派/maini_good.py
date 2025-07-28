import RPi.GPIO as GPIO
import time
import cv2 as cv 
import cv2
import numpy as np
import Micro_robot_driver_v2
import time
temp = np.array([])
cornerpixel = np.array([0,0,0,0])
cornerlast5 = np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]])
pixelflag = 0
source=29
key1=37
key2=35
key3=33
key4=31
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD) 
GPIO.setup(key1,GPIO.IN)
GPIO.setup(key2,GPIO.IN)
GPIO.setup(key3,GPIO.IN)
GPIO.setup(key4,GPIO.IN)
GPIO.setup(source,GPIO.OUT,initial=GPIO.HIGH)
GPIO.setmode(GPIO.BOARD)
	
flag1 = 1
flag2 = 1
flag3 = 1	
motor_x=4
motor_y=5
cent_y = 281.5
cent_x = 274.2
Arr_p_1_2_3_4_y = [80.2 ,479.76 ,470.6 ,89.7 ,80.2]#([[82.3,109.7], [464.5,119.6], [464.3, 496.7], [79.1,506.6]]
Arr_p_1_2_3_4_x = [84.0 ,75.0 ,461.0,467.0 ,84.0 ]
robot_use = Micro_robot_driver_v2.Micro_Robot_base(com_port_input="/dev/ttyACM0", baudrate_input=9600)
step = 0
Arr_step = 0
Arr_step_flag = 1
cap = cv2.VideoCapture(0)  # 打开摄像头
h = -13
l = 960
step_rad = 1.8/32 #一步角度
XX = int(0)
YY = int(0)
Time = 0
Time2 = 0
Time3 = 0
red_center_x = 0
red_center_y = 0
cv2.namedWindow('Result')
for i in range(40):
	ret, frame = cap.read()  # 读取视频帧
import cv2
import numpy as np
import math
def getpoint(img,corners):
    global temp,pixelflag,cornerpixel,cornerlast5
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, binary = cv.threshold(gray, 0, 255, cv.THRESH_BINARY)
    fil1 = np.array([[-2, -1, 0],  
                    [-1, 0, 1],
                    [0, 1, 2]])
    fil2 = np.array([[2, 1, 0],  
                    [1, 0, -1],
                    [0, -1, -2]])
#    fil3 = np.array([[0, -1, -2],  
#                    [1, 0, -1],
#                    [2, 1, 0]])
#    fil4 = np.array([[0, 1, 2],  
#                    [-1, 0, 1],
#                    [-2, -1, 0]])
                    
                    
    ####卷积并膨胀###       
    kernel1 = cv.getStructuringElement(cv.MORPH_RECT, (4, 4))
    kernel2 = cv.getStructuringElement(cv.MORPH_CROSS, (4, 4))
#    kernel3 = cv.getStructuringElement(cv.MORPH_CROSS, (1, 1))
    res1 = cv.filter2D(gray, -1, fil1) 
    res2 = cv.filter2D(gray, -1, fil2)  
#    res3 = cv.filter2D(gray, -1, fil3)  
#    res4 = cv.filter2D(gray, -1, fil4)  
#    res = res1 + res2 + res3 + res4
    res = res1 + res2
    res = cv.dilate(res, kernel1)
    res = cv.erode(res, kernel2)
#    res = cv.dilate(res, kernel3)
#    cv.imshow('res',res)
    if np.all(temp == 0):
        temp = res
    maxmun = 0
    summun = 0
    res[res<=254] = 0
    cornerthis= np.array([0,0,0,0])        #这次识别角的像素之和
    avglast5 = np.array(np.mean(cornerlast5, axis=0))      #上5次的角像素之和的平均值
    
    for i in range (0,4):
        ni = np.sum(res[corners[i][0][1]-20:corners[i][0][1]+20,corners[i][0][0]-20:corners[i][0][0]+20]) / 255

        if (not np.any(cornerlast5 == [0,0,0,0])) and ni- (avglast5[i])>=20:      #这次若识别值比前五次均值大20,则识别成功
            print('corner:',i)
        cornerthis[i] = ni 
#    print('this:\t',cornerthis,'\nlast5:\t',cornerlast5,'\nlast5avg\t',avglast5)
    cornerlast5 = np.delete(cornerlast5,0,0 )
    cornerlast5 = np.append(cornerlast5,[cornerthis],axis = 0)        #删除之前最早数据，将这次数据并入

 #   print("#############")
    cv.imshow('res',res)
    return res
def turn_rid(ary,h):
	y = ary
	tan_y = (y- h)/l
	#计算角度（弧度
	radians = math.atan(tan_y)
	degrees_y = math.degrees(radians)
	return degrees_y

def CZY(now_x,now_y,target_x,target_y):
	step_x=0          #x的步数
	step_y=0          #y的步数
	target=[0,0]
	# 已知矩形的四个像素点坐标
	src_points = np.array([[84.0,80.2], [467.0,89.7], [461.0, 470.6], [75.0,479.76]], dtype=np.float32)

	# 目标矩形的四个顶点坐标
	dst_points = np.array([[-250, 250], [-250, -250], [250, -250], [250, 250]], dtype=np.float32)

	# 计算仿射变换矩阵
	matrix = cv2.getPerspectiveTransform(src_points, dst_points)
	
	now = [now_x, now_y] ###########now
	src = [target_x, target_y]####################target
	
	src_point = np.array([src], dtype=np.float32)
	dst_point = cv2.perspectiveTransform(src_point.reshape(-1, 1, 2), matrix)
	dst_x, dst_y = dst_point[0][0]
	
	src_point = np.array([now], dtype=np.float32)
	dst_point = cv2.perspectiveTransform(src_point.reshape(-1, 1, 2), matrix)
	dst_x_now, dst_y_now = dst_point[0][0]
   # dst_x_now = [dst_x_now, dst_y_now]
	print(dst_x, dst_y)
	print(dst_x_now, dst_y_now)
	rid1 = turn_rid(dst_y,h)
	rid2 = turn_rid(dst_y_now,h)
	degrees_y = rid1- rid2
	rid1 = turn_rid(dst_x,0)
	rid2 = turn_rid(dst_x_now,0)
	degrees_x = rid1- rid2
	step_x = degrees_x/step_rad
	step_y = degrees_y/step_rad
	print(step_x, step_y)
	target[0]=step_x
	target[1]=-step_y
	return target

def nothing(x):
	pass
def create_mask(image, lower_color, upper_color):
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, lower_color, upper_color)
	return mask

def euclidean_distance(point1, point2):
	return np.sqrt(np.sum((point1 - point2) ** 2))

def getLocation(img1):
	OpointList = np.array(np.where(img1 == 255))
	try:
		Opoint0, Opoint1 = int(np.mean(OpointList[0][:])), int(np.mean(OpointList[1][:]))
	except:
		Opoint0, Opoint1 = -1, -1
	return np.array([Opoint1, Opoint0])

def track(points, img0):
	global first_point, flag
	point = getLocation(img0)
	if flag < 4:
		if abs(points[flag][0][0] - point[0]) <= 20 and abs(points[flag][0][1] - point[1]) <= 20:
			flag = flag + 1
			print(flag)
		# print(targetpoint)
	elif (abs(points[0][0][0] - point[0]) <= 20) and (abs(points[0][0][1] - point[1]) <= 20) and flag == 4:
		print("?")
		cap.release()  # 释放捕获器
		cv2.destroyAllWindows()  # 关闭图像窗口
		exit(0)
	# exit(0)

def getcorner(img):
	imgfinal0 = img.copy()
	imgfinal = np.array(imgfinal0)
	gray = cv.cvtColor(imgfinal, cv.COLOR_BGR2GRAY)
	ret, binary = cv.threshold(gray, 100, 200, cv.THRESH_BINARY_INV)
	kernel = cv.getStructuringElement(cv.MORPH_RECT, (10, 10))
	dil = cv.dilate(binary,kernel)         #pengzhang
	dst = binary.copy()
	
	#提取骨架
	skeleton = np.zeros(dst.shape, np.uint8)
	while (True):
		if np.sum(dst) == 0:
			break
		kernel = cv.getStructuringElement(cv.MORPH_CROSS, (7, 7))
		dst = cv.erode(dst, kernel, None, None, 1)
		open_dst = cv.morphologyEx(dst, cv.MORPH_OPEN, kernel)
		result = dst - open_dst
		skeleton = skeleton + result

	skeleton = cv.dilate(skeleton, kernel)
	
	##calculate the num of corner
	contours, hierarchy = cv.findContours(skeleton,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE)
	# drawing = cv.drawContours(skeleton,contours,-1,[255,0,0],4)
	areamax = 0
	mainobj = 0
	
	
	#qu mian ji zui da de ju xing
	for obj in contours:
		area = cv.contourArea(obj)  # 计算轮廓内区域的面积
		if area >= areamax:
			areamax = area
			mainobj = obj

	cv.drawContours(img, mainobj, -1, (255, 255, 0), 4)  # 绘制轮廓线
	perimeter = cv.arcLength(mainobj, True)  # 计算轮廓周长
	approx = cv.approxPolyDP(mainobj, 0.02 * perimeter, True)  # 获取轮廓角点坐标
	# print(approx[0])
	CornerNum = len(approx)  # 轮廓角点的数量
	x, y, w, h = cv.boundingRect(approx)  # 获取坐标值和宽度、高度

	# 轮廓对象分类
	if CornerNum == 3:
		objType = "triangle"
	elif CornerNum == 4:
		if w == h:
			objType = "Square"
		else:
			objType = "Rectangle"
	elif CornerNum > 4:
		objType = "Circle"
	else:
		objType = "N"

	cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)  # 绘制边界框
	cv.putText(img, objType, (x + (w // 2), y + (h // 2)), cv.FONT_HERSHEY_COMPLEX, 0.6, (0, 255, 0),
				1)  # 绘制文字
	approx = approx[::-1]
	if CornerNum == 4:
		cv.circle(img,(int(approx[0][0][0]),int(approx[0][0][1])),15,(0,0,255),5)
		cv.circle(img,(int(approx[1][0][0]),int(approx[1][0][1])), 15, (255, 255, 255), 5)
		cv.circle(img, (int(approx[2][0][0]),int(approx[2][0][1])), 15, (255, 255, 0), 5)
		cv.circle(img, (int(approx[3][0][0]),int(approx[3][0][1])), 15, (255, 0, 0), 5)
	cv.imshow('corners',img)

	return approx
	
while True:
	for i in range(10):
		ret, frame = cap.read()  # 读取视频帧
	if GPIO.input(37)==GPIO.HIGH and flag1 == 1:
		flag1 = 0
		Time = 0
	if GPIO.input(35)==GPIO.HIGH and flag2 == 1:
		
		flag2 = 0
		Time2 = 0
	if GPIO.input(33)==GPIO.HIGH and flag3 == 1:
		
		flag3 = 0
		Time3 = 0
	elif GPIO.input(37)==GPIO.LOW :
		flag1 = 1
		print('key1 was pressed')
		step = 0
		
		ret, frame = cap.read()  # 读取视频帧
		if not ret:
			break

		lower_red = np.array([122, 33, 209])
		upper_red = np.array([177, 126, 255])
		red_mask = create_mask(frame, lower_red, upper_red)
		red = cv2.bitwise_and(frame, frame, mask=red_mask)  # 将两个蒙版应用于原始图像
		red_all = np.column_stack(np.where(red >= 245))

		if red_all.size == 0:
			print("No red dot found")
			red_center_x = 0
			red_center_y = 0
		else:
			max_area_index = np.argmax(np.bincount(red_all[:, 0]))
			red_all = red_all[red_all[:, 0] == max_area_index]
			red_center_x = np.mean(red_all[:, 0])
			red_center_y = np.mean(red_all[:, 1])
			print('1:red_x=', red_center_x, 'red_y=', red_center_y)
			result = cv2.bitwise_and(frame, frame, mask=red_mask)
			#

		# if Time == 3:
			# YY = int(((cent_y - red_center_y) * 1.25) / 2)
			# XX = int(((cent_x - red_center_x) * 1.25) / 2)
		# else:
			# YY = int(((cent_y - red_center_y) * 1.25) / 10)
			# XX = int(((cent_x - red_center_x) * 1.25) / 10)
		# print(cent_y - red_center_y)
		# print(cent_x - red_center_x)
		# print(XX)
		# print(YY)

		# if (abs(int(cent_y - red_center_y)) <= 8):
			# XX = 0
		# if (abs(int(cent_x - red_center_x)) <= 8):
			# YY = 0
		# deta_steps_arry1=[0,0,0,0,XX,YY]
		# speed_arr=[8500,8500,8500,8500,8500,8500]
		# robot_use.on_deta_J_ALL_move(deta_steps_arry1, speed_arr)  # 电机多轴同步运动，通用函数，包含计数功能
		# robot_use.J_step_buff = [0, 0, 0, 0, 0, 0]
		# Time = Time + 1
		if abs(cent_x -red_center_x)>5 and abs(red_center_y - cent_y)>5:
			
			YY=int(CZY(red_center_x,red_center_y,cent_x,cent_y)[0])
			XX=int(CZY(red_center_x,red_center_y,cent_x,cent_y)[1])
			print("9999999999999",XX,YY)
		else:
			print("66666666666666")
			continue

		if not (XX | YY):
			
			print("stop")
			continue
		deta_steps_arry1=[0,0,0,0,XX,YY]
		speed_arr=[8500,8500,8500,8500,8500,8500]
		robot_use.on_deta_J_ALL_move(deta_steps_arry1, speed_arr)  # 电机多轴同步运动，通用函数，包含计数功能
		robot_use.J_step_buff = [0, 0, 0, 0, 0, 0]
		
		Time = Time +1 
	elif GPIO.input(35)==GPIO.LOW :
		flag2 = 1
		print('key2 was pressed')
		
		ret, frame = cap.read()  # 读取视频帧

		if not ret:
			break

		lower_red = np.array([122, 33, 209])
		upper_red = np.array([177, 126, 255])

		red_mask = create_mask(frame, lower_red, upper_red)

		red = cv2.bitwise_and(frame, frame, mask=red_mask)  # 将两个蒙版应用于原始图像

		red_all = np.column_stack(np.where(red >= 245))
		if red_all.size == 0:
			print("No red dot found")
			red_center_x = 0
			red_center_y = 0
		else:
			max_area_index = np.argmax(np.bincount(red_all[:, 0]))
			red_all = red_all[red_all[:, 0] == max_area_index]
			red_center_x = np.mean(red_all[:, 0])
			red_center_y = np.mean(red_all[:, 1])
			print('1:red_x=', red_center_x, 'red_y=', red_center_y)
			result = cv2.bitwise_and(frame, frame, mask=red_mask)
			cv2.imshow('Result', result)

		if (step >= 5):
			print("stop")
			continue
		# if Time2 == 3:
			# YY = int(((Arr_p_1_2_3_4_y[step] - red_center_y) * 1.25) / 2)
			# XX = int(((Arr_p_1_2_3_4_x[step] - red_center_x) * 1.25) / 2)
		# else:
			# YY = int((Arr_p_1_2_3_4_y[step] - red_center_y) / 10)
			# XX = int((Arr_p_1_2_3_4_x[step] - red_center_x) / 10)

		# print(XX)
		# print(YY)

		# A = (abs(Arr_p_1_2_3_4_y[step]-red_center_y)>8)&(abs(Arr_p_1_2_3_4_y[step]-red_center_y)<11)
		# B = (abs(Arr_p_1_2_3_4_x[step]-red_center_x)>8)&(abs(Arr_p_1_2_3_4_x[step]-red_center_x)<11)
		
		


		#if abs(Arr_p_1_2_3_4_x[step] -red_center_x)>10 and abs(Arr_p_1_2_3_4_y[step] - red_center_x)>10:
		if Time2 == 1:
			continue
		for i in range(0,5):
			if i == 0:
				print("44444444444444444444")
				YY=int(CZY(red_center_x,red_center_y,Arr_p_1_2_3_4_x[i],Arr_p_1_2_3_4_y[i])[0])
				XX=int(CZY(red_center_x,red_center_y,Arr_p_1_2_3_4_x[i],Arr_p_1_2_3_4_y[i])[1])
				deta_steps_arry1=[0,0,0,0,XX,YY]
				speed_arr=[6500,6500,6500,6500,6500,6500]
		
		
				robot_use.on_deta_J_ALL_move(deta_steps_arry1, speed_arr) # 电机多轴同步运动，通用函数，包含计数功能
			else:
				YY=int(CZY(Arr_p_1_2_3_4_x[i-1],Arr_p_1_2_3_4_y[i-1],Arr_p_1_2_3_4_x[i],Arr_p_1_2_3_4_y[i])[0])
				XX=int(CZY(Arr_p_1_2_3_4_x[i-1],Arr_p_1_2_3_4_y[i-1],Arr_p_1_2_3_4_x[i],Arr_p_1_2_3_4_y[i])[1])
				deta_steps_arry1=[0,0,0,0,XX,YY]
				speed_arr=[6500,6500,6500,6500,6500,6500]
		
		
				robot_use.on_deta_J_ALL_move(deta_steps_arry1, speed_arr) # 电机多轴同步运动，通用函数，包含计数功能
				



		
		
		print('2:red_x=', red_center_x, 'red_y=', red_center_y)
		Time2 =  1
	elif GPIO.input(33)==GPIO.LOW :
		if Time3==1:
			continue
		flag3 = 1
		print('key3 was pressed')
		
		#如果正确读取帧，ret为True
		ret, frame = cap.read()
		imgsize = np.shape(frame)
		temp = np.zeros([imgsize[0],imgsize[1]])
	#    img = frame.copy()
		#读取失败，则退出循环
		#图像处理-转换为灰度图
	#    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

	##################################  start  #######################################
		f1 = np.array(frame.copy()) 
		frame1 = np.array(f1)
		corners = getcorner(frame1)
		
		ret, frame = cap.read()

		f2 = np.array(frame.copy())

		frame2 = np.array(f2)
		imgsize = np.shape(frame)

		point = getpoint(frame2, corners) 
		print("corners=",corners)
		print(corners[0][0][0],corners[0][0][1])
		Arr_1_2_3_4_y = [corners[0][0][0],corners[1][0][0],corners[2][0][0],corners[3][0][0],corners[0][0][0]]
		Arr_1_2_3_4_x = [corners[0][0][1],corners[1][0][1],corners[2][0][1],corners[3][0][1],corners[0][0][1]]
		print(Arr_1_2_3_4_x)
		for i in range(0,5):
			if i == 0:
				
				print("44444444444444444444")
				YY=int(CZY(cent_x,cent_y,Arr_1_2_3_4_x[0],Arr_1_2_3_4_y[0])[0])
				XX=int(CZY(cent_x,cent_y,Arr_1_2_3_4_x[0],Arr_1_2_3_4_y[0])[1])
				deta_steps_arry1=[0,0,0,0,XX,YY]
				speed_arr=[6500,6500,6500,6500,6500,6500]
		
		
				robot_use.on_deta_J_ALL_move(deta_steps_arry1, speed_arr) # 电机多轴同步运动，通用函数，包含计数功能
			else:
				YY=int(CZY(Arr_1_2_3_4_x[i-1],Arr_1_2_3_4_y[i-1],Arr_1_2_3_4_x[i],Arr_1_2_3_4_y[i])[0])
				XX=int(CZY(Arr_1_2_3_4_x[i-1],Arr_1_2_3_4_y[i-1],Arr_1_2_3_4_x[i],Arr_1_2_3_4_y[i])[1])
				deta_steps_arry1=[0,0,0,0,XX,YY]
				speed_arr=[6500,6500,6500,6500,6500,6500]
		
		
				robot_use.on_deta_J_ALL_move(deta_steps_arry1, speed_arr) # 电机多轴同步运动，通用函数，包含计数功能
		Time3 = 1
	elif GPIO.input(31)==GPIO.LOW and Arr_step_flag == 1 :
		print('key4 was pressed', Arr_step)
		for i in range(10):
			ret, frame = cap.read()  # 读取视频帧

		lower_red = np.array([122, 33, 209])
		upper_red = np.array([177, 126, 255])
		red_mask = create_mask(frame, lower_red, upper_red)
		red = cv2.bitwise_and(frame, frame, mask=red_mask)  # 将两个蒙版应用于原始图像

		red_all = np.column_stack(np.where(red >= 245))
		if red_all.size == 0:
			print("No red dot found")
			red_center_x = 0
			red_center_y = 0
		else:
			max_area_index = np.argmax(np.bincount(red_all[:, 0]))
			red_all = red_all[red_all[:, 0] == max_area_index]
			red_center_x = np.mean(red_all[:, 0])
			red_center_y = np.mean(red_all[:, 1])
			if Arr_step == 0 :
				Arr_p_1_2_3_4_x[0] = red_center_x
				Arr_p_1_2_3_4_y[0] = red_center_y
				Arr_p_1_2_3_4_x[4] = red_center_x
				Arr_p_1_2_3_4_y[4] = red_center_y
				#print('RED DOTS', red_center_x)
				#print('RED DOTS', red_center_y)
				print(Arr_step,'Arr_step_x', Arr_p_1_2_3_4_x[Arr_step])
				print(Arr_step,'Arr_step_y', Arr_p_1_2_3_4_y[Arr_step])
			elif Arr_step==4 :
				cent_y = red_center_x
				cent_x = red_center_x
				print(Arr_step,'Arr_step_x', Arr_p_1_2_3_4_x[Arr_step])
				print(Arr_step,'Arr_step_y', Arr_p_1_2_3_4_y[Arr_step])
			else:
				Arr_p_1_2_3_4_x[Arr_step] = red_center_x
				Arr_p_1_2_3_4_y[Arr_step] = red_center_y
				print(Arr_step,'Arr_step_x', Arr_p_1_2_3_4_x[Arr_step])
				print(Arr_step,'Arr_step_y', Arr_p_1_2_3_4_y[Arr_step])
			if Arr_step == 4:
				Arr_step = 0

		Arr_step_flag = 0
	elif GPIO.input(31) == GPIO.HIGH and Arr_step_flag == 0:
		

		Arr_step = Arr_step + 1
		
		Arr_step_flag = 1

cap.release()  # 释放摄像头
cv2.destroyAllWindows()
