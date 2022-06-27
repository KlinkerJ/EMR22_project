#!/usr/bin/env python3

import torch
import cv2 #sudo apt-get install python3-opencv
import time
import rospy
from move_robot import move_robot, move_cartesian

def setup(camera_id = 0, yolo_version = "yolov5s"):
    #Model
    model = torch.hub.load('ultralytics/yolov5', yolo_version)  # or yolov5n - yolov5x6, custom

    #CV2
    vid = cv2.VideoCapture(camera_id)
    time.sleep(1) #Let the webcam initialize
    ret, frame = vid.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_height = frame.shape[0]
    frame_width  = frame.shape[1]
    print("Frame Dimensions:", frame_height, frame_width)

    center_point = [frame_width / 2, frame_height / 2]
    print("Center-Point:", center_point)
    return vid, model, center_point, frame_width, frame_height

def run(camera_id = 0, yolo_version = "yolov5s", object_str = "bottle", debug = False):
    vid, model, center_point, frame_width, frame_height = setup(camera_id = camera_id, yolo_version = yolo_version)

    run = True

    while run:
            ret, frame = vid.read()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) #Convert Colors
            #Detection
            results = model(frame)
            res = results.pandas().xyxy[0]
            if object_str in list(res.name):
                #detected object -> calculate position
                index = list(res.name).index(object_str)
                x_min, y_min = list(res.xmin)[index], list(res.ymin)[index]
                x_max, y_max = list(res.xmax)[index], list(res.ymax)[index]
                x_mid = x_min + 0.5 * (x_max - x_min)
                y_mid = y_min + 0.5 * (y_max - y_min)
                if debug: print("Object Position:", x_mid, y_mid)
                #delta from Center
                delta_x, delta_y = x_mid - center_point[0], center_point[1] - y_mid
                #positive x -> object right from Center
                #positive y -> object higher than Center
                print("Delta from Center:", delta_x, delta_y)

                #control
                #Factor and reverse
                p = 0.001

                move_cartesian(p * 0, p * -delta_x, p * delta_y, wait=False)

                if debug:
                    run = False
                    results.show()
                    run = True

if __name__ == "__main__":
    try:
        #Init node
        rospy.init_node('yolo_move_node', anonymous=False)
        object_str = rospy.get_param('~object') or "bottle"
        camera_id = int(rospy.get_param('~cameraid')) or 0
        yolo_version = rospy.get_param('~yolo') or "yolov5s"
        run(camera_id = camera_id, yolo_version = yolo_version, object_str = object_str, debug = False)
    except rospy.ROSInterruptException:
        pass
