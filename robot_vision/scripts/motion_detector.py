#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from cute_teleop.msg import pixel
import pyrealsense2 as rs


class motionDetector:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        # 创建cv_bridge
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)

        # 发布位置信息
        self.pixel_info_pub = rospy.Publisher('/mouse_pixel', pixel, queue_size=1)

        # 设置参数：最小区域、阈值
        # self.minArea   = rospy.get_param("~minArea",   500)
        self.minArea   = rospy.get_param("~minArea",   100)
        self.threshold = rospy.get_param("~threshold", 25)

        self.firstFrame = None
        self.text = "Unoccupied"

        # 初始化订阅rgb格式图像数据的订阅者，此处图像topic的话题名可以在launch文件中重映射
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)

        # 用于发布像素信息
        self.pixel = pixel()

        # 用于计算三维坐标的首次进入标志位
        self.first = True
        self.x = 0
        self.y = 0

        self.FlagCreateBar = True


    def image_callback(self, data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")     
            frame = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e

        # 创建灰度图像
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        # 使用两帧图像做比较，检测移动物体的区域
        if self.firstFrame is None:
            self.firstFrame = gray
            return  
        frameDelta = cv2.absdiff(self.firstFrame, gray)

        thresh = cv2.threshold(frameDelta, self.threshold, 255, cv2.THRESH_BINARY)[1]

        cv2.namedWindow("frame", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("frame", frame)

        if self.FlagCreateBar:
            self.FlagCreateBar = False
            cv2.createTrackbar("thresh", "frame", self.threshold, 255, self.nothing)

        self.threshold = cv2.getTrackbarPos('thresh', "frame")
        
        k = cv2.waitKey(1) & 0xff
        if k == ord('s') and self.first:
            self.first = False
            box = cv2.selectROI("frame", frame, fromCenter=False, showCrosshair=True)
            (x0, y0, w0, h0) = [int(v) for v in box]
            cv2.rectangle(frame, (x0, y0), (x0 + w0, y0 + h0), (0, 255, 0), 1)
            self.x = x0
            self.y = y0

        thresh = cv2.dilate(thresh, None, iterations=2)
        binary, cnts, hierarchy= cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in cnts:
            # 如果检测到的区域小于设置值，则忽略
            # if cv2.contourArea(c) < self.minArea:
            #    continue 

            if cv2.contourArea(c) > 400 or cv2.contourArea(c) < 100:
               continue 

            # 在输出画面上框出识别到的物体
            (x, y, w, h) = cv2.boundingRect(c)

            if x - self.x >= 30 or self.x - x >= 30 or y - self.y >= 30 or self.y - y >= 30:
                continue

            if h / w > 1.2 or w / h > 1.2:
               continue 

            cv2.rectangle(frame, (x, y), (x + w, y + h), (50, 255, 50), 2)
            self.text = "Occupied"

            # 发布像素坐标
            self.pixel.x = x
            self.pixel.y = y
            self.x = x
            self.y = y
            self.pixel_info_pub.publish(self.pixel)

            # 在输出画面上打印面积
            cv2.putText(frame, "{}".format(cv2.contourArea(c)), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            # self.coordinate_map(x, y)
        # 在输出画面上打当前状态和时间戳信息
        cv2.putText(frame, "Status: {}".format(self.text), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 将识别后的图像转换成ROS消息并发布
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def nothing(self, x):
        pass

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("motion_detector")
        rospy.loginfo("motion_detector node is started...")
        rospy.loginfo("Please subscribe the ROS image.")

        # Declare pointcloud object, for calculating pointclouds and texture mappings
        # pc = rs.pointcloud()
        # We want the points object to be persistent so we can display the last cloud when a frame drops
        # points = rs.points()
        
        # pipeline = rs.pipeline()
        # config = rs.config()
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        # Start streaming
        # pipe_profile = pipeline.start(config)

        motionDetector()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down motion detector node."
        cv2.destroyAllWindows()
        # pipeline.stop()


