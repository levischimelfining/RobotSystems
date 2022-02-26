#!/usr/bin/python3
# coding=utf8
import sys

sys.path.append('/home/levi/ArmPi/')
import cv2
from perception import Perception
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *


class Motion:

    def __init__(self):
        self.coordinate = {
            'red': (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5, 1.5),
            'blue': (-15 + 0.5, 0 - 0.5, 1.5)}
        self.servo1 = 500
        self.count = 0
        self.track = False
        self.stop = False
        self.get_roi = False
        self.center_list = []
        self.first_move = True
        self.isRunning = False
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True
        self.target_color = ()

    # reset variables
    def reset(self):
        self.count = 0
        self.stop = False
        self.track = False
        self.get_roi = False
        self.center_list = []
        self.first_move = True
        self.target_color = ()
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True

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
                if self.first_move and self.start_pick_up:  # When an object is first detected
                    self.action_finish = False
                    self.set_rgb(detect_color)
                    self.set_buzzer(0.1)
                    result = AK.setPitchRangeMoving((world_X, world_Y - 2, 5), -90, -90,
                                                    0)  # Do not fill in the running time parameter, adaptive running time
                    if result == False:
                        unreachable = True
                    else:
                        unreachable = False
                    time.sleep(result[2] / 1000)  # The third item of the returned parameter is the time
                    start_pick_up = False
                    first_move = False
                    action_finish = True
                elif not first_move and not unreachable:  # Not the first time an object has been detected
                    set_rgb(detect_color)
                    if track:  # If it is the tracking phase
                        if not __isRunning:  # stop and exit flag detection
                            continue
                        AK.setPitchRangeMoving((world_x, world_y - 2, 5), -90, -90, 0, 20)
                        time.sleep(0.02)
                        track = False
                    if start_pick_up:  # If the object has not moved for a while, start gripping
                        action_finish = False
                        if not __isRunning:  # stop and exit flag detection
                            continue
                        Board.setBusServoPulse(1, servo1 - 280, 500)  # gripper open
                        # Calculate the angle by which the gripper needs to be rotated
                        servo2_angle = getAngle(world_X, world_Y, rotation_angle)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.8)

                        if not __isRunning:
                            continue
                        AK.setPitchRangeMoving((world_X, world_Y, 2), -90, -90, 0, 1000)  # lower the altitude
                        time.sleep(2)

                        if not __isRunning:
                            continue
                        Board.setBusServoPulse(1, servo1, 500)  # Gripper closed
                        time.sleep(1)

                        if not __isRunning:
                            continue
                        Board.setBusServoPulse(2, 500, 500)
                        AK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000)  # The robotic arm is raised
                        time.sleep(1)

                        if not __isRunning:
                            continue
                        # Sort and place object on reference color
                        result = AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12),
                                                        -90, -90, 0)
                        time.sleep(result[2] / 1000)

                        if not __isRunning:
                            continue
                        servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        if not __isRunning:
                            continue
                        AK.setPitchRangeMoving(
                            (coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3),
                            -90, -90, 0, 500)
                        time.sleep(0.5)

                        if not __isRunning:
                            continue
                        AK.setPitchRangeMoving((coordinate[detect_color]), -90, -90, 0, 1000)
                        time.sleep(0.8)

                        if not __isRunning:
                            continue
                        Board.setBusServoPulse(1, servo1 - 200, 500)  # Gripper open, drop object
                        time.sleep(0.8)

                        if not __isRunning:
                            continue
                        AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90,
                                               0, 800)
                        time.sleep(0.8)

                        self.init_move()  # return to original position
                        time.sleep(1.5)

                        detect_color = 'None'
                        first_move = True
                        get_roi = False
                        action_finish = True
                        start_pick_up = False
                        set_rgb(detect_color)
                    else:
                        time.sleep(0.01)
            else:
                if _stop:
                    _stop = False
                    Board.setBusServoPulse(1, servo1 - 70, 300)
                    time.sleep(0.5)
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                    time.sleep(1.5)
                time.sleep(0.01)

if __name__ == '__main__':
    AK = ArmIK()
    my_camera = Camera.Camera()
    my_camera.camera_open()

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
