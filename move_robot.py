import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import tf

#Init

# First initialize moveit_ Command and rospy nodes:
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface',
                anonymous=True)
# Instantiate the robot commander object,
# which is used to control the whole robot
robot = moveit_commander.RobotCommander()

# Instantiate the MoveGroupCommander object.
group_name = "ur5_arm"
group = moveit_commander.MoveGroupCommander(group_name)
group_name_gripper = "gripper"
group_gripper = moveit_commander.MoveGroupCommander(group_name_gripper)

# Create a Publisher.
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

#listener = tf.TransformListener()
#(trans,rot) = listener.lookupTransform('/world', '/object_9', rospy.Time())

def move_robot(x,y,z, debug=False):
    if debug:
        input("Start")
    group.set_planner_id("RRTConnect")
    #We can set RRTConnect here, however:
    #we have do define the parameters like in RVIZ because otherwise the robot is doing crazy things as always
    #longest_valid_segment: 0.005
    #projection_evaluator: joints(shoulder_pan_joint,shoulder_lift_joint)
    #range: 0
    pose_goal = group.get_current_pose()
    pose_goal.pose.position.x  +=x # from tf-Tree
    pose_goal.pose.position.y  +=y # from tf-Tree
    #pose_goal.pose.position.z  +=z# from tf-Tree

    print(" going to ", pose_goal.pose.position)
    #input("confirm moving ur3_arm to this position")
    group.set_pose_target(pose_goal)
    group.go(wait=False)
    #sucess = group.go(wait=True)
    #print("suc?", sucess)
    group.stop()
    group.clear_pose_targets()

    return

if __name__ == "__main__":
    move_robot(0.1, 0.0, 0.0, True)