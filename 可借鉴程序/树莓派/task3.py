import numpy as np  #导入科学计算库numpy
import cv2 as cv  #导入opencv-python
import time

temp = np.array([])
cornerpixel = np.array([0,0,0,0])
cornerlast5 = np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]])
pixelflag = 0
        
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


        
    
    
    
#############################main function#####################################

if __name__ == '__main__':
    #打开捕获器（0-系统默认摄像头）
    cap = cv.VideoCapture(0)
    #打开成功
    if cap.isOpened():
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
        while True:
            ret, frame = cap.read()

            f2 = np.array(frame.copy())

            frame2 = np.array(f2)
            imgsize = np.shape(frame)

            point = getpoint(frame2, corners) 
 #           cv.imshow('point',point)


    ##################################  show  ########################################
#        cv.imshow('img',frame)
        #获取键盘按下那个键
            key_pressed = cv.waitKey(60)
            #如果按下esc键，就退出循环
            if key_pressed == 27:
                break
    cap.release()  #释放捕获器
    cv.destroyAllWindows() #关闭图像窗口
