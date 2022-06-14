import copy
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import tf
import time

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
    group.set_planner_id("EMR")
    group.allow_replanning(True)
    group.set_goal_tolerance(0.005)
    group.set_num_planning_attempts(3)
    group.set_planning_time(1)
    #We can set RRTConnect here, however:
    #we have do define the parameters like in RVIZ because otherwise the robot is doing crazy things as always
    #longest_valid_segment: 0.005
    #projection_evaluator: joints(shoulder_pan_joint,shoulder_lift_joint)
    #range: 0
    pose_goal = group.get_current_pose()
    pose_goal.pose.position.x  +=x # from tf-Tree
    pose_goal.pose.position.y  +=y # from tf-Tree
    pose_goal.pose.position.z  +=z# from tf-Tree

    print(" going to ", pose_goal.pose.position)
    #input("confirm moving ur3_arm to this position")
    t1 = time.time()
    group.set_pose_target(pose_goal)
    group.go(wait=True)
    print(t1 - time.time())
    #sucess = group.go(wait=True)
    #print("suc?", sucess)
    group.stop()
    group.clear_pose_targets()

    return

def move_cartesian(x,y,z, debug=False):

    if debug:
        input("Start")
    group.set_planner_id("EMR")
    group.allow_replanning(True)
    group.set_goal_tolerance(0.005)
    group.set_num_planning_attempts(1)
    group.set_planning_time(1)

    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.x +=x
    wpose.position.y +=y 
    wpose.position.z +=z  
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    t1 = time.time()
    group.execute(plan, wait=True)
    print(t1 - time.time())
    #sucess = group.go(wait=True)
    #print("suc?", sucess)
    group.stop()
    group.clear_pose_targets()
    return

if __name__ == "__main__":
    #move_robot(0.1, 0.1, 0.1, True)
    move_cartesian(+0.2, +0.3, -0.2, True)