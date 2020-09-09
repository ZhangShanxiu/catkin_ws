#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

class objectDetector:
    def __init__(self):
        rospy.on_shutdown(self.cleanup);

        # 创建cv_bridge
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)

        # 获取haar特征的级联表的XML文件，文件路径在launch文件中传入
        cascade_1 = rospy.get_param("~cascade_1", "")
        cascade_2 = rospy.get_param("~cascade_2", "")

        # 使用级联表初始化haar特征检测器
        self.cascade_1 = cv2.CascadeClassifier(cascade_1)
        self.cascade_2 = cv2.CascadeClassifier(cascade_2)

        # 设置级联表的参数，优化人脸识别，可以在launch文件中重新配置
        self.haar_scaleFactor  = rospy.get_param("~haar_scaleFactor", 1.2)
        self.haar_minNeighbors = rospy.get_param("~haar_minNeighbors", 2)
        self.haar_minSize      = rospy.get_param("~haar_minSize", 40)
        self.haar_maxSize      = rospy.get_param("~haar_maxSize", 60)
        self.color = (50, 255, 50)

        # 初始化订阅rgb格式图像数据的订阅者，此处图像topic的话题名可以在launch文件中重映射
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)
        # self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=1)

    def image_callback(self, data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")     
            frame = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e
        
        # 截取图像有用部分
        print(frame.shape)
        # cv2.imwrite('/home/zhx/projects/ROS/catkin_ws/src/cute_robot/robot_vision/doc/1.jpg', frame)
        ROI = frame[100:460, 200:500]
        # 创建灰度图像
        grey_image = cv2.cvtColor(ROI, cv2.COLOR_BGR2GRAY)
        self.cv_show('Image', grey_image)

        # 高斯滤波
        grey_image = cv2.GaussianBlur(grey_image, (5, 5), 0)
        self.cv_show('Guass', grey_image)

        # 创建平衡直方图，减少光线影响
        grey_image = cv2.equalizeHist(grey_image)
        self.cv_show('Hist', grey_image)

        # 边缘检测
        grey_image = cv2.Canny(grey_image, 60, 200)
        self.cv_show('Edge', grey_image)

        # 二值图像
        # ref = cv2.threshold(grey_image, 140, 255, cv2.THRESH_BINARY_INV)[1]
        # self.cv_show('Binary', ref)

        # 轮廓检测
        ref_, cnts, hierarchy = cv2.findContours(grey_image.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(cv_image,refCnts,-1,(0,0,255),3) 
        # self.cv_show('cv_image',cv_image)
        cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:5]

        # 遍历轮廓
        for c in cnts:
            # 计算轮廓近似
            # peri = cv2.arcLength(c, True)
            # C表示输入的点集
            # epsilon表示从原始轮廓到近似轮廓的最大距离，它是一个准确度参数
            # True表示封闭的
            # approx = cv2.approxPolyDP(c, 0.02 * peri, True)

            # 4个点的时候就拿出来
            # if len(approx) == 4:
                # screenCnt = approx
                # print(screenCnt)
                # break
            cv2.drawContours(ROI, c, 1, (0, 255, 0), 2)
            # cv2.drawContours(ROI, cnts, -1, (0, 255, 0), 2)
            cv_image[100:460, 200:500] = ROI
            self.cv_show('cv_image',cv_image)

        # cv2.drawContours(ROI, cnts, -1, (0, 255, 0), 1)
        # cv2.drawContours(ROI, cnts, -1, (0, 255, 0), 2)
        # cv_image[100:460, 200:500] = ROI
        # self.cv_show('cv_image',cv_image)
        # 尝试检测人脸
        # faces_result = self.detect_face(grey_image)

        # 在opencv的窗口中框出所有人脸区域
        # if len(faces_result)>0:
            # for face in faces_result: 
                # x, y, w, h = face
                # cv2.rectangle(cv_image, (x, y), (x+w, y+h), self.color, 2)
        


        # 将识别后的图像转换成ROS消息并发布
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    def detect_face(self, input_image):
        # 首先匹配正面人脸的模型
        if self.cascade_1:
            faces = self.cascade_1.detectMultiScale(input_image, 
                    self.haar_scaleFactor, 
                    self.haar_minNeighbors, 
                    cv2.CASCADE_SCALE_IMAGE, 
                    (self.haar_minSize, self.haar_maxSize))
                                         
        # 如果正面人脸匹配失败，那么就尝试匹配侧面人脸的模型
        if len(faces) == 0 and self.cascade_2:
            faces = self.cascade_2.detectMultiScale(input_image, 
                    self.haar_scaleFactor, 
                    self.haar_minNeighbors, 
                    cv2.CASCADE_SCALE_IMAGE, 
                    (self.haar_minSize, self.haar_maxSize))
        
        return faces
    
    def cv_show(self, name, image):
        cv2.imshow(name, image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("object_detector")
        objectDetector()
        rospy.loginfo("Object detector is started..")
        rospy.loginfo("Please subscribe the ROS image.")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down face detector node."
        cv2.destroyAllWindows()
