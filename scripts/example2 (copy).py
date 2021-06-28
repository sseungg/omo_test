#!/usr/bin/env python
# -*- coding: cp949 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
global sum_slope

class HoughLine():
    def __init__(self):
        self.sub_image = "raw"

        if self.sub_image == "compressed":
            self._sub = rospy.Subscriber('usb_cam/image_raw/compressed', CompressedImage, self.callback, queue_size = 1)
        else:
            self._sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback, queue_size = 1)

        self.bridge = CvBridge()

    def callback(self, image_msg):
        if self.sub_image == "compressed":
            np_arr = np.fromstring(image_msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IREAD_COLOR)

        elif self.sub_image == "raw":
            image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        height, width = image.shape[:2]

        vertices = np.array([[(50, height), (width/2-45, height/2+60), (width/2+45, height/2+60), (width-50, height)]], dtype = np.int32)
        #vertices1 = np.array([[(50, height), (width/2-45, height/2+60), (width/2, height/2+60), (width/2, height)]], dtype = np.int32)
        #vertices2 = np.array([[(width/2, height), (width/2, height/2+60), (width/2+45, height/2+60), (width-50, height)]], dtype = np.int32)

        def grayscale(img):
            return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        def gaussian_blur(img, kernel_size):
            return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

        def canny(img, low_threshold, high_threshold):
            return cv2.Canny(img, low_threshold, high_threshold)

        def region_of_interest(img, vertices, color3=(0, 190, 190), color1=255):
            mask = np.zeros_like(img)

            if len(img.shape) > 2:
                color = color3
            else:
                color = color1

            cv2.fillPoly(mask, vertices, color)

            ROI_img = cv2.bitwise_and(img, mask)
            return ROI_img

        def draw_lines(img, lines, color=(0, 0, 255), thickness=3):
            if lines is None:
                return

            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(img, (x1, y1), (x2, y2), color, thickness)

            # 기울기 구하고 해당 영역에서 검출된 직선의 기울기값을 모두 더함
            slope = (y2 - y1) / (x2 - x1)
            sum_slope += slope
            sum_slope count ++

        def hough_lines(img, rho, theta, threshold, minLineLength, maxLineGap):
            lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength, maxLineGap)
            line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype = np.uint8)
            draw_lines(line_img, lines)
            return line_img

        def find_slope(sum_slope, count):
            average_slope = sum_slope / count

        slope1 = find_slope()

        gray_img = grayscale(image)
        blur_img = gaussian_blur(gray_img, 3)
        canny_img = canny(blur_img, 70, 210)
        ROI_img = region_of_interest(canny_img, vertices)
        hough_img = hough_lines(ROI_img, 1, 1*np.pi/180, 30, 10, 20)

        result = cv2.addWeighted(image, 1, hough_img, 1, 0)
        cv2.imshow('result', result), cv2.waitKey(1)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('hough')
    node = HoughLine()
    node.main()
