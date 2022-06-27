import cv2

camera_ids = [0, 1, 2, 3, 4, 5] #Ubuntu 20.04

for camera_id in camera_ids:
    try:
        vid = cv2.VideoCapture(camera_id)
        ret, frame = vid.read()
        print("Found camera with id: " + str(camera_id))
    except Exception as e:
        continue