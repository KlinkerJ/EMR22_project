#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
    

def talker(camera_id = 0, yolo_version = "yolov5s", object_str = "bottle", debug = False):
    pub = rospy.Publisher('/yolo_prediction_img', Image, queue_size=1)

    # init yolo model
    model = torch.hub.load('ultralytics/yolov5', yolo_version)

    # init camera and cv2
    bridge = CvBridge()
    cap = cv2.VideoCapture(camera_id)
    while not cap.isOpened():
        rospy.sleep(1)
    print("VideoCapture.isOpened: ", cap.isOpened())

    # get frame info (shape and centerpoint)
    ret, frame = cap.read()
    frame_height = frame.shape[0]
    frame_width  = frame.shape[1]
    print("Frame Dimensions: ", frame_height, frame_width)
    center_point = [frame_width / 2, frame_height / 2]
    print("Center-Point: ", center_point)

    # run node
    while not rospy.is_shutdown():
        # read image
        ret, frame = cap.read()
        if not ret:
            break

        # get model prediction
        results = model(frame)
        res = results.pandas().xyxy[0]
        if object_str in list(res.name):
                #detected object --> calculate position
                index = list(res.name).index(object_str)
                x_min, y_min = list(res.xmin)[index], list(res.ymin)[index]
                x_max, y_max = list(res.xmax)[index], list(res.ymax)[index]
                x_mid = x_min + 0.5 * (x_max - x_min)
                y_mid = y_min + 0.5 * (y_max - y_min)

                #delta from Center
                delta_x, delta_y = x_mid - center_point[0], center_point[1] - y_mid
                #positive x -> ball right from Center
                #positive y -> ball higher than Center
                
                # draw bottle center point and object name on frame
                frame = cv2.circle(frame,(int(x_mid), int(y_mid)), 10, (0,0,255), -1)
                frame = cv2.putText(frame, object_str, (int(x_mid)+20, int(y_mid)+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)

                if debug:
                    print("Object Position:", x_mid, y_mid)
                    print("Delta from Center:", delta_x, delta_y)

        # pub img
        msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
        pub.publish(msg)

        if rospy.is_shutdown():
            cap.release()

if __name__ == '__main__':
    try:
        #Init node
        rospy.init_node('yolo_image_node', anonymous=False)
        object_str = rospy.get_param('~object') or "bottle"
        camera_id = int(rospy.get_param('~cameraid')) or 0
        yolo_version = rospy.get_param('~yolo') or "yolov5s"
        talker(camera_id = camera_id, yolo_version = yolo_version, object_str = object_str, debug = False)
    except rospy.ROSInterruptException:
        pass