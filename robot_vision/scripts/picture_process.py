#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

class pictreProcess:
    def __init__(self):
        rospy.on_shutdown(self.cleanup);

        # 读入图片
        src = cv2.imread('/home/zhx/projects/ROS/catkin_ws/src/cute_robot/robot_vision/doc/1.jpg')
        org = src.copy()

        # 创建灰度图像
        grey_image = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        self.cv_show('gray', grey_image)

        # 高斯滤波
        # grey_image = cv2.GaussianBlur(grey_image, (5, 5), 0)
        # self.cv_show('Guass', grey_image)

        # 创建平衡直方图，减少光线影响
        # grey_image = cv2.equalizeHist(grey_image)
        # self.cv_show('Hist', grey_image)

        # 边缘检测
        edge = cv2.Canny(grey_image, 60, 200)
        self.cv_show('edge', edge)

        # 初始化卷积核
        rectKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        clseKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        roadKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))

        # 礼帽操作，突出更明亮的区域
        tophat = cv2.morphologyEx(edge, cv2.MORPH_TOPHAT, rectKernel) 
        self.cv_show('tophat',tophat) 

        # sobel算子
        # gradX = cv2.Sobel(grey_image, ddepth=cv2.CV_32F, dx=1, dy=0,ksize=-1)
        # gradX = np.absolute(gradX)
        # (minVal, maxVal) = (np.min(gradX), np.max(gradX))
        # gradX = (255 * ((gradX - minVal) / (maxVal - minVal)))
        # grey_image = gradX.astype("uint8")
        # self.cv_show('gradX',grey_image)

        # 通过闭操作（先膨胀，再腐蚀）将数字连在一起
        close = cv2.morphologyEx(tophat, cv2.MORPH_CLOSE, clseKernel) 
        self.cv_show('close',close)

        # 梯度运算
        # grey_image = cv2.morphologyEx(grey_image, cv2.MORPH_GRADIENT, rectKernel) 
        # self.cv_show('gradient',grey_image)

        # 二值图像
        # ref = cv2.threshold(grey_image, 140, 255, cv2.THRESH_BINARY_INV)[1]
        # self.cv_show('Binary', ref)

        # 轮廓检测
        ref_, cnts, hierarchy = cv2.findContours(close.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(cv_image,refCnts,-1,(0,0,255),3) 
        # self.cv_show('cv_image',cv_image)
        cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:5]

        # 遍历轮廓
        for (i, c) in enumerate(cnts):
            # 计算轮廓近似
            peri = cv2.arcLength(c, True)
            # C表示输入的点集
            # epsilon表示从原始轮廓到近似轮廓的最大距离，它是一个准确度参数
            # True表示封闭的
            approx = cv2.approxPolyDP(c, 0.03 * peri, True)

            # 4个点的时候就拿出来
            if len(approx) == 4:
                screenCnt = approx
                print(screenCnt)
                break
            # cv2.drawContours(src, cnts, i, (0, 255, 0), 2)
            # cv2.drawContours(ROI, cnts, -1, (0, 255, 0), 2)
            # self.cv_show('cv_image',src)

        # cv2.drawContours(ROI, cnts, -1, (0, 255, 0), 1)
        cv2.drawContours(src, [screenCnt], -1, (0, 255, 0), 2)
        # cv_image[100:460, 200:500] = ROI
        self.cv_show('src',src)

        # 透视变换
        warped = self.four_point_transform(org, screenCnt.reshape(4, 2))
        warped = cv2.resize(warped, (0, 0), fx = 2, fy = 2)
        self.cv_show('warped',warped)

        # 转换为灰度图
        gray_warped = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)

        # 边缘检测
        edge_warped = cv2.Canny(gray_warped, 15, 20)
        self.cv_show('edge_warped', edge_warped)

        # 通过闭操作（先膨胀，再腐蚀）将数字连在一起
        close_warped = cv2.morphologyEx(edge_warped, cv2.MORPH_CLOSE, clseKernel) 
        self.cv_show('close_warped',close_warped)

        # 轮廓检测
        ref_, cnts, hierarchy = cv2.findContours(close_warped.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(warped, cnts, -1, (0, 255, 0), 2)
        # self.cv_show('warped',warped)

        # 遍历轮廓
        for c in cnts:
            # 计算外接矩形
            (x, y, w, h) = cv2.boundingRect(c)

            # 画出矩形轮廓
            if h / w > 10 and h > 400:
                cv2.rectangle(warped, (x, y), (x + w, y + h), (0, 0, 255), 1)
                self.cv_show('warped', warped)

        # 遍历轮廓
        # for c in cnts:
            # 计算外接矩形
	        # (x, y, w, h) = cv2.boundingRect(c)
            # cv2.rectangle(warped, (x - 5, y - 5), (x + w + 5, y + h + 5), (0, 0, 255), 1)
            # cv2.rectangle(warped, (x - 5, y - 5), (x + w + 5, y + h + 5), (0, 0, 255), 1)
            # self.cv_show('warped',warped)

        # 高斯滤波
        blur_warped = cv2.GaussianBlur(warped, (3, 3), 0)
        self.cv_show('blur_warped', blur_warped)

        # 梯度运算
        # warped = cv2.morphologyEx(warped, cv2.MORPH_GRADIENT, roadKernel) 
        # self.cv_show('warped',warped)

        # sobel算子
        # gradX = cv2.Sobel(warped, ddepth=cv2.CV_32F, dx=1, dy=0,ksize=-1)
        # gradX = np.absolute(gradX)
        # (minVal, maxVal) = (np.min(gradX), np.max(gradX))
        # gradX = (255 * ((gradX - minVal) / (maxVal - minVal)))
        # warped = gradX.astype("uint8")
        # self.cv_show('gradX',warped)

        # 通过闭操作（先膨胀，再腐蚀）将数字连在一起
        # warped = cv2.morphologyEx(warped, cv2.MORPH_CLOSE, rectKernel) 
        # self.cv_show('close',warped)

        # 创建平衡直方图，减少光线影响
        # warped = cv2.equalizeHist(warped)
        # self.cv_show('Hist', warped)

        # 二值化
        binary = cv2.threshold(blur_warped, 140, 255, cv2.THRESH_BINARY)[1]
        self.cv_show('binary',binary)

        # 边缘检测
        edge_binary = cv2.Canny(binary, 50, 150)
        self.cv_show('edge_binary', edge_binary)

        # 轮廓检测
        ref_, cnts, hierarchy = cv2.findContours(edge_binary.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        # 遍历轮廓
        for c in cnts:
            # 计算外接矩形
            (x, y, w, h) = cv2.boundingRect(c)

            # 画出矩形轮廓
            if h / w > 2 and h / w < 5:
                cv2.rectangle(warped, (x, y), (x + w, y + h), (0, 0, 255), 1)
                self.cv_show('warped', warped)

    
    # 退出时清空显示
    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

    # 显示图片
    def cv_show(self, name, image):
        cv2.imshow(name, image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def order_points(self, pts):
        # 一共4个坐标点
        rect = np.zeros((4, 2), dtype = "float32")
        # print(pts)
        # 按顺序找到对应坐标0123分别是 左上，右上，右下，左下
        # 计算左上，右下
        s = pts.sum(axis = 1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]

        # 计算右上和左下
        diff = np.diff(pts, axis = 1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        # print(rect)

        return rect

    # 透视变换
    def four_point_transform(self, image, pts):
        # 获取输入坐标点
        rect = self.order_points(pts)
        (tl, tr, br, bl) = rect

        # 计算输入的w和h值
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))

        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))

        # 变换后对应坐标位置
        dst = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype = "float32")

        # 计算变换矩阵
        M = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))

        # 返回变换后结果
        return warped

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("picture_process")
        pictreProcess()
        # rospy.loginfo("picture processor is started..")
        # rospy.loginfo("Please subscribe the ROS image.")
        # rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down face detector node."
        cv2.destroyAllWindows()
