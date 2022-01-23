import cv2
frame = cv2.imread('/home/pi/DeepPiCar/driver/data/road1_240x320.png')
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np


class Image:

    def detect_edges(frame):
        lower_blue = np.array([60, 40, 40])
        upper_blue = np.array([150, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        # filter for blue lane lines
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        show_image("hsv", hsv)
        lower_blue = np.array([60, 40, 40])
        upper_blue = np.array([150, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        show_image("blue mask", mask)

        # detect edges
        edges = cv2.Canny(mask, 200, 400)

        return edges

    def region_of_interest(edges):
        height, width = edges.shape
        mask = np.zeros_like(edges)

        # only focus bottom half of the screen
        polygon = np.array([[
            (0, height * 1 / 2),
            (width, height * 1 / 2),
            (width, height),
            (0, height),
        ]], np.int32)

        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)
        return cropped_edges

    def detect_line_segments(cropped_edges):
        # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
        rho = 1  # distance precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
        min_threshold = 10  # minimal of votes
        line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold,
                                        np.array([]), minLineLength=8, maxLineGap=4)

        return line_segments

#init camera
print("start color detect")
camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 24
rawCapture = PiRGBArray(camera, size=camera.resolution)

for frame in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
    img = frame.array
    img,img_2,img_3 =  color_detect(img,'red')
    cv2.imshow("video", img)
    cv2.imshow("mask", img_2)
    cv2.imshow("morphologyEx_img", img_3)
    rawCapture.truncate(0)

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        camera.close()
        break