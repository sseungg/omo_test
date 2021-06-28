#!/usr/bin/env python
# -*- coding: cp949 -*-
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

class FitLine():
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

        def grayscale(img):
            return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        def gaussian_blur(img, kernel_size):
            return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

        def canny(img, low_threshold, high_threshold):
            return cv2.Canny(img, low_threshold, high_threshold)

        def region_of_interest(img, vertices, color3=(255, 255, 255), color1=255):
            mask = np.zeros_like(img)

            if len(img.shape) > 2:
                color = color3
            else:
                color = color1

            cv2.fillPoly(mask, vertices, color)
            ROI_img = cv2.bitwise_and(img, mask)
            return ROI_img

        def draw_lines(img, lines, color=(255, 0, 0), thickness=3):
            if lines is None:
                return
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(img, (x1, y1), (x2, y2), color, thickness)

        def draw_fit_line(img, lines, color=(255, 0, 0), thickness=10):
            cv2.line(img, (line[0], line[1]), (line[2], line[3]), color, thickness)

        def hough_lines(img, rho, threshold, min_line_len, max_line_gap):
            lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLinelength = min_line_len, maxLineGap = max_line_gap)
            #line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype = np.uint8)
            #draw_lines(line_img, lines)
            return lines

        def get_fitline(img, f_liens):
            lines = np.squeeze(f_lines)
            lines = lines.reshape(lines.shape[0]*2, 2)
            rows,cols = img.shape[:2]
            output = cv2.fitLine(lines, cv2.DIST_L2, 0, 0.01, 0.01)
            vx, vy, x, y = output[0], output[1], output[2], output[3]
            x1, y1 = int(((img.shape[0]-1)-y/vy*vx + x), img.shape[1]-1
            x2, y2 = int(((img.shape[0]/2+200)-y)/vy*vx + x), int(img.shape[0]/2+100)

            result = [x1, y1, x2, y2]
            return result

        gray_img = grayscale(image)
        blur_img = gaussian_blur(gray_img, 3)
        canny_img = canny(blur_img, 3)

        vertices = np.array([[(50, height), (width/2-45, height/2+60), (width/2+45, height/2+60), (width-50, height)]], dtype = np.int32)
        ROI_img = region_of_interest(canny_img, vertices)

        line_arr = hough_lines(ROI_img, 1, 1*np.pi/180, 30, 10, 20)
        line_arr = np.squeeze(line_arr)

        cv2.imshow('result', result_img), cv2.waitKey(1)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('fitline')
    node = FitLine()
    node.main()
