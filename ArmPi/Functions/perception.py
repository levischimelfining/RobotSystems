#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/levi/ArmPi/')
import cv2
import Camera
import time
import math
import threading
import numpy as np
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
from CameraCalibration.CalibrationConfig import *
import HiwonderSDK.Board as Board


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

        self.draw_color = self.range_rgb["black"]
        self.color_list = []
        self.color_area_max = None
        self.max_area = 0
        self.areaMaxContour_max = 0
        self.area_max = 0
        self.t1 = 0
        self.roi = ()

    start_count_t1 = True
    detect_color = 'None'
    center_list = []
    world_X, world_Y = 0, 0
    world_x, world_y = 0, 0
    last_x, last_y = 0, 0
    rotation_angle = 0
    count = 0
    action_finish = True

    def run(self, img, target_color='red'):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert image to LAB space

        area_max = 0
        areaMaxContour = 0

        if not Motion.start_pick_up:
            for i in color_range:
                if i in target_color:
                    Perception.detect_color = i
                    frame_mask = cv2.inRange(frame_lab, color_range[Perception.detect_color][0], color_range[Perception.detect_color][
                        1])  # Bitwise operations on the original image and mask
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # open operation
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # closed operation
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[
                        -2]  # find the outline
                    areaMaxContour, area_max = self.getAreaMaxContour(contours)  # find the largest contour
            if area_max > 2500:  # have found the largest area
                rect = cv2.minAreaRect(areaMaxContour)
                box = np.int0(cv2.boxPoints(rect))
                roi = getROI(box)  # get roi region
                get_roi = True

                img_centerx, img_centery = getCenter(rect, roi, self.size,
                                                     square_length)  # Get the coordinates of the center of the block
                Perception.world_x, Perception.world_y = convertCoordinate(img_centerx, img_centery,
                                                     self.size)  # Convert to real world coordinates

                cv2.drawContours(img, [box], -1, self.range_rgb[Perception.detect_color], 2)
                cv2.putText(img, '(' + str(Perception.world_x) + ',' + str(Perception.world_y) + ')',
                            (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[Perception.detect_color], 1)  # draw center point
                distance = math.sqrt(pow(Perception.world_x - Perception.last_x, 2) + pow(Perception.world_y - Perception.last_y,
                                                                    2))  # Compare the last coordinates to determine whether to move
                Perception.last_x, Perception.last_y = Perception.world_x, Perception.world_y
                Motion.track = True

                if Motion.action_finish:
                    if distance < 0.3:
                        Perception.center_list.extend((Perception.world_x, Perception.world_y))
                        Perception.count += 1
                        if self.start_count_t1:
                            Perception.start_count_t1 = False
                            self.t1 = time.time()
                        if time.time() - self.t1 > 1.5:
                            Perception.rotation_angle = rect[2]
                            Perception.start_count_t1 = True
                            Perception.world_X, Perception.world_Y = np.mean(
                                np.array(Perception.center_list).reshape(Perception.count, 2), axis=0)
                            Perception.count = 0
                            Perception.center_list = []
                            Motion.start_pick_up = True
                    else:
                        self.t1 = time.time()
                        Perception.start_count_t1 = True
                        Perception.count = 0
                        Perception.center_list = []

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

        for i in self.range_rgb:
            if i in target_color:
                frame_mask = cv2.inRange(frame_lab, self.range_rgb[i][0],
                                         self.range_rgb[i][1])  # Bitwise operations on the original image and mask
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # open operation
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # closed operation
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # find the outline
                areaMaxContour, area_max = self.getAreaMaxContour(contours)  # find the largest contour
                if areaMaxContour is not None:
                    if self.area_max > self.max_area:  # find the largest area
                        max_area = area_max
                        self.color_area_max = i
                        areaMaxContour_max = areaMaxContour
        if self.max_area > 2500:  # have found the largest area
            rect = cv2.minAreaRect(areaMaxContour_max)
            box = np.int0(cv2.boxPoints(rect))

            self.roi = getROI(box)  # get roi region
            get_roi = True
            img_centerx, img_centery = getCenter(rect, self.roi, self.size,
                                                 square_length)  # Get the coordinates of the center of the object

            Perception.world_x, Perception.world_y = convertCoordinate(img_centerx, img_centery, self.size)  # Convert to real world coordinates

            cv2.drawContours(img, [box], -1, self.range_rgb[self.color_area_max], 2)
            cv2.putText(img, '(' + str(Perception.world_x) + ',' + str(Perception.world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[self.color_area_max], 1)  # draw center point

            if self.color_area_max == 'red':  # red max
                color = 1
            elif self.color_area_max == 'green':  # green max
                color = 2
            elif self.color_area_max == 'blue':  # blue max
                color = 3
            else:
                color = 0
            self.color_list.append(color)

            if len(self.color_list) == 3:  # multiple judgments
                # take the average
                color = int(round(np.mean(np.array(self.color_list))))

                if color == 1:
                    Perception.detect_color = 'red'
                    self.draw_color = self.range_rgb["red"]
                elif color == 2:
                    Perception.detect_color = 'green'
                    self.draw_color = self.range_rgb["green"]
                elif color == 3:
                    Perception.detect_color = 'blue'
                    self.draw_color = self.range_rgb["blue"]
                else:
                    Perception.detect_color = 'None'
                    self.draw_color = self.range_rgb["black"]

            cv2.putText(img, "Color: " + Perception.detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        self.draw_color, 2)
            distance = math.sqrt(pow(Perception.world_x - Perception.last_x, 2) + pow(Perception.world_y - Perception.last_y,
                                                                2))  # Compare the last coordinates to determine whether to move
            Perception.last_x, Perception.last_y = Perception.world_x, Perception.world_y

            if Motion.action_finish:
                if distance < 0.3:
                    Perception.center_list.extend((Perception.world_x, Perception.world_y))
                    Perception.count += 1
                    if self.start_count_t1:
                        Perception.start_count_t1 = False
                        self.t1 = time.time()
                    if time.time() - self.t1 > 1.5:
                        Perception.rotation_angle = rect[2]
                        Perception.start_count_t1 = True
                        Motion.world_X, Perception.world_Y = np.mean(np.array(Perception.center_list).reshape(Perception.count, 2), axis=0)
                        Perception.count = 0
                        Perception.center_list = []
                        Motion.start_pick_up = True
                else:
                    self.t1 = time.time()
                    Perception.start_count_t1 = True
                    Perception.count = 0
                    Perception.center_list = []

            return img


class Motion:

    def __init__(self):
        self.coordinate = {
            'red': (-15 + 0.5, 12 - 0.5, 4),
            'green': (-15 + 0.5, 6 - 0.5, 4),
            'blue': (-15 + 0.5, 0 - 0.5, 4)}
        self.servo1 = 500
        self.stop = False
        self.get_roi = False
        self.first_move = True
        self.isRunning = False
        self.detect_color = Perception.detect_color
        self.target_color = ()
        self.unreachable = False

    track = False
    action_finish = True
    start_pick_up = False

    # reset variables
    def reset(self):
        Perception.count = 0
        self.stop = False
        self.track = False
        self.get_roi = False
        Perception.center_list = []
        self.first_move = True
        self.target_color = ()
        self.detect_color = Perception.detect_color
        Motion.action_finish = True
        Motion.start_pick_up = False
        Perception.start_count_t1 = True

    # initial position
    def init_move(self):
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    # start color tracking
    def start(self):
        self.reset()
        self.isRunning = True

    # stop color tracking
    def stop(self):
        self.stop = True
        self.isRunning = False

    # exit color tracking
    def exit(self):
        self.stop = True
        self.isRunning = False

    def set_buzzer(self, timer):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(timer)
        Board.setBuzzer(0)

    def set_rgb(self, color):
        if color == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif color == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()
        elif color == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()


    # move arm
    def move(self):
        while True:
            if self.isRunning:
                if self.first_move and Motion.start_pick_up:  # When an object is first detected
                    Motion.action_finish = False
                    self.set_rgb(self.detect_color)
                    self.set_buzzer(0.1)
                    # Do not fill in the running time parameter, adaptive running time
                    result = AK.setPitchRangeMoving((Perception.world_X, Perception.world_Y - 2, 5), -90, -90, 0)
                    if result == False:
                        self.unreachable = True
                    else:
                        self.unreachable = False
                    time.sleep(result[2] / 1000)  # The third item of the returned parameter is the time
                    Motion.start_pick_up = False
                    self.first_move = False
                    Motion.action_finish = True
                elif not self.first_move and not self.unreachable:  # Not the first time an object has been detected
                    self.set_rgb(self.detect_color)
                    if Motion.track:  # If it is the tracking phase
                        if not self.isRunning:  # stop and exit flag detection
                            continue
                        AK.setPitchRangeMoving((Perception.world_x, Perception.world_y - 2, 5), -90, -90, 0, 20)
                        time.sleep(0.02)
                        Motion.track = False
                        print(Motion.start_pick_up)
                    if Motion.start_pick_up:  # If the object has not moved for a while, start gripping
                        Motion.action_finish = False
                        print("TEST 2")
                        if not self.isRunning:  # stop and exit flag detection
                            continue
                        Board.setBusServoPulse(1, self.servo1 - 280, 500)  # gripper open

                        # Calculate the angle by which the gripper needs to be rotated
                        servo2_angle = getAngle(Perception.world_X, Perception.world_Y, Perception.rotation_angle)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.8)

                        if not self.isRunning:
                            continue
                        AK.setPitchRangeMoving((Perception.world_X, Perception.world_Y, 2), -90, -90, 0, 1000)  # lower the altitude
                        time.sleep(2)

                        if not self.isRunning:
                            continue
                        Board.setBusServoPulse(1, self.servo1, 500)  # Gripper closed
                        time.sleep(1)

                        if not self.isRunning:
                            continue
                        Board.setBusServoPulse(2, 500, 500)
                        AK.setPitchRangeMoving((Perception.world_X, Perception.world_Y, 12), -90, -90, 0, 1000)  # The robotic arm is raised
                        time.sleep(1)

                        if not self.isRunning:
                            continue
                        # Sort and place object on reference color
                        result = AK.setPitchRangeMoving((self.coordinate[Perception.detect_color][0], self.coordinate[Perception.detect_color][1], 12),
                                                        -90, -90, 0)
                        time.sleep(result[2] / 1000)

                        if not self.isRunning:
                            continue
                        servo2_angle = getAngle(self.coordinate[Perception.detect_color][0], self.coordinate[Perception.detect_color][1], -90)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        if not self.isRunning:
                            continue
                        AK.setPitchRangeMoving(
                            (self.coordinate[Perception.detect_color][0], self.coordinate[Perception.detect_color][1], self.coordinate[Perception.detect_color][2] + 3),
                            -90, -90, 0, 500)
                        time.sleep(0.5)

                        if not self.isRunning:
                            continue
                        AK.setPitchRangeMoving((self.coordinate[Perception.detect_color]), -90, -90, 0, 1000)
                        time.sleep(0.8)

                        if not self.isRunning:
                            continue
                        Board.setBusServoPulse(1, self.servo1 - 200, 500)  # Gripper open, drop object
                        time.sleep(0.8)

                        if not self.isRunning:
                            continue
                        AK.setPitchRangeMoving((self.coordinate[Perception.detect_color][0], self.coordinate[Perception.detect_color][1], 12), -90, -90,
                                               0, 800)
                        time.sleep(0.8)

                        self.init_move()  # return to original position
                        time.sleep(1.5)

                        detect_color = 'None'
                        self.first_move = True
                        self.get_roi = False
                        Motion.action_finish = True
                        self.start_pick_up = False
                        self.set_rgb(detect_color)
                    else:
                        time.sleep(0.01)
            else:
                if self.stop:
                    self.stop = False
                    Board.setBusServoPulse(1, self.servo1 - 70, 300)
                    time.sleep(0.5)
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                    time.sleep(1.5)
                time.sleep(0.01)

if __name__ == '__main__':

    perception = Perception()
    motion = Motion()
    AK = ArmIK()

    motion.init_move()
    motion.start()

    my_camera = Camera.Camera()
    my_camera.camera_open()

    th = threading.Thread(target=motion.move)
    th.setDaemon(True)
    th.start()

    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            perception = Perception()
            Frame = perception.run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
