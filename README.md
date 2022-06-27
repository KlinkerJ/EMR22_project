# EMR22
Repository for Embedded Robotics 2022 by Jonas Klinker (@KlinkerJ) and Lennart Fuhrig (@lennart2810).

# Installation
1. Clone the repository to the `catkin_ws`
2. Install the required dependencies
```python3 -m pip install -r requirements.txt```
3. Build the package
```catkin build emr22_project```
# Usage

## Find the correct webcam
You may have to use a different webcam-id for your webcam / robot-mounted cam. In our scripts, we are using the camera id 0. If that is not the right id, you can change it in `nodes/yolo_img_pub.py` and `nodes/yolo_move_robot.py`.

## Run only yolov5 and view the results in Rqt
1. Start our script
```roslaunch emr22_project yolo_view.launch```
2. Navigate to Image_View in Rqt.

## Run yolov5 and move the robot
1. Launch MoveIt and Simulation or use your own command for the real robot
```roslaunch ur5_moveit_config demo_gazebo.launch```
2. Start our script
```roslaunch emr22_project yolo_move_robot.launch```

**Warning**: You should try our control system in simulation first and adapt Kp in `nodes/yolo_move_robot.py`!

**Hint**: You can not view the detected objects in Rqt if you called the move-Script! 

**Hint**: Our script reacts only to the object class `bottle`! You can change this as you want to one of the Yolov5-Object-Classes in `nodes/yolo_img_pub.py` and `nodes/yolo_move_robot.py`! The scripts can only recognize one object!
