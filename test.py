import torch
import cv2 #sudo apt-get install python3-opencv
import time

#Model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # or yolov5n - yolov5x6, custom


#CV2
vid = cv2.VideoCapture(0)
time.sleep(1) #Let the webcam initialize
ret, frame = vid.read()
frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
frame_height = frame.shape[0]
frame_width  = frame.shape[1]
print("Frame Dimensions:", frame_height, frame_width)

center_point = [frame_width / 2, frame_height / 2]
print("Center-Point:", center_point)

ball_detected = False
Debug = True

while not ball_detected:
    ret, frame = vid.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) #Convert Colors
    #Detection
    results = model(frame)
    res = results.pandas().xyxy[0]
    if 'sports ball' in list(res.name):
        #detected sports ball -> calculate position
        index = list(res.name).index('sports ball')
        x_min, y_min = list(res.xmin)[index], list(res.ymin)[index]
        x_max, y_max = list(res.xmax)[index], list(res.ymax)[index]
        x_mid = x_min + 0.5 * (x_max - x_min)
        y_mid = y_min + 0.5 * (y_max - y_min)
        print("Ball Position:", x_mid, y_mid)
        #delta from Center
        delta_x, delta_y = x_mid - center_point[0], center_point[1] - y_mid
        #positive x -> ball right from Center
        #positive y -> ball higher than Center
        print("Delta from Center:", delta_x, delta_y)

        if Debug:
            ball_detected = True
            results.show()
