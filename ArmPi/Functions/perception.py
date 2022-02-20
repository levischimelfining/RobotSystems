#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/levi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *


class Perception:

    def __init__(self):
        self.rect = None
        self.size = (640, 480)
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255)}
        t1 = 0
        reroi = ()
        self.center_list = []
        last_x, last_y = 0, 0
        self.draw_color = self.range_rgb["black"]
        self.detect_color = []
        self.color_list = []

    def run(self, img, target_color='blue'):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert image to LAB space

        area_max = 0
        areaMaxContour = 0

        for i in color_range:
            if i in target_color:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][
                    1])  # Bitwise operations on the original image and mask
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # open operation
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # closed operation
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # find the outline
                areaMaxContour, area_max = self.getAreaMaxContour(contours)  # find the largest contour

        if area_max > 2500:  # have found the largest area
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))

            roi = getROI(box)  # get roi region
            get_roi = True

            img_centerx, img_centery = getCenter(rect, roi, self.size,
                                                 square_length)  # Get the coordinates of the center of the block
            world_x, world_y = convertCoordinate(img_centerx, img_centery, self.size)  # Convert to real world coordinates

            cv2.drawContours(img, [box], -1, self.range_rgb[detect_color], 2)
            cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[detect_color], 1)  # draw center point
        return img

    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # iterate over all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate the contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # The contour with the largest area is valid only if the area is greater than 300 to filter out the noise
                    area_max_contour = c

        return area_max_contour, contour_area_max  # returns the largest contour

    def color_sort(self, img, target_color=('red', 'green', 'blue')):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert image to LAB space

        color_area_max = None
        max_area = 0
        areaMaxContour_max = 0

        for i in color_range:
            if i in target_color:
                frame_mask = cv2.inRange(frame_lab, color_range[i][0],
                                         color_range[i][1])  # Bitwise operations on the original image and mask
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # open operation
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # closed operation
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # find the outline
                areaMaxContour, area_max = self.getAreaMaxContour(contours)  # find the largest contour
                if areaMaxContour is not None:
                    if area_max > max_area:  # find the largest area
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
        if max_area > 2500:  # have found the largest area
            rect = cv2.minAreaRect(areaMaxContour_max)
            box = np.int0(cv2.boxPoints(rect))

            roi = getROI(box)  # get roi region
            get_roi = True
            img_centerx, img_centery = getCenter(rect, roi, self.size,
                                                 square_length)  # Get the coordinates of the center of the object

            world_x, world_y = convertCoordinate(img_centerx, img_centery, self.size)  # Convert to real world coordinates

            cv2.drawContours(img, [box], -1, self.range_rgb[color_area_max], 2)
            cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[color_area_max], 1)  # draw center point

            if color_area_max == 'red':  # red max
                color = 1
            elif color_area_max == 'green':  # green max
                color = 2
            elif color_area_max == 'blue':  # blue max
                color = 3
            else:
                color = 0
            self.color_list.append(color)

            if len(self.color_list) == 3:  # multiple judgments
                # take the average
                color = int(round(np.mean(np.array(self.color_list))))

                if color == 1:
                    self.detect_color = 'red'
                    self.draw_color = self.range_rgb["red"]
                elif color == 2:
                    self.detect_color = 'green'
                    self.draw_color = self.range_rgb["green"]
                elif color == 3:
                    self.detect_color = 'blue'
                    self.draw_color = self.range_rgb["blue"]
                else:
                    self.detect_color = 'None'
                    self.draw_color = self.range_rgb["black"]

            cv2.putText(img, "Color: " + self.detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        self.draw_color, 2)
            return img

if __name__ == '__main__':
    my_camera = Camera.Camera()
    my_camera.camera_open()

    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            perception = Perception()
            Frame = perception.color_sort(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
