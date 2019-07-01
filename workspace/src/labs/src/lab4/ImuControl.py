#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed at UC
# Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu) and Greg Marcil (grmarcil@berkeley.edu). The cloud
# services integation with ROS was developed by Kiet Lam
# (kiet.lam@berkeley.edu). The web-server app Dator was based on an open source
# project by Bruce Wootton
# ---------------------------------------------------------------------------

# README: This node serves as an outgoing messaging bus from odroid to arduino
# Subscribes: steering and motor commands on 'ecu'
# Publishes: combined ecu commands as 'ecu_pwm'

from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
from barc.msg import ECU
from numpy import pi, sign, sin,cos
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list



import rospy
import time
import numpy

motor_pwm = 0.0
servo_pwm = 0.0
shoulder=0.0
bicep=0.0


# update
def callback_function(data):
    global shoulder_angle,bicep_angle,forearm_angle,wrist_angle,gripper_angle,initialized
    if 'arm_bicep_joint' in data.name:
        shoulder_angle = data.position[3]
        bicep_angle =  data.position[0]
        forearm_angle =  data.position[4]
        wrist_angle =  data.position[1]
        gripper_angle =  data.position[2]
        initialized = True

def callback_function2(data):
    global shoulder, bicep
    global pub1_shoulder,pub2_bicep,pub3_forearm,pub4_wrist,pub5_gripper
    global shoulder_angle,bicep_angle,forearm_angle,wrist_angle,gripper_angle,initialized
    global counter,tstart

    #print(data.linear_acceleration.x, data.linear_acceleration.y)

    #set targets
   #  if 0<=data.linear_acceleration.x<=10:
   #      bicep=(data.linear_acceleration.x*10.0*(pi/180))
   #  elif 0>data.linear_acceleration.x>=-10:
   #      bicep=(data.linear_acceleration.x*(10.0)*(pi/180))
   # # print("this is the bicep:", bicep)
   #  if 0<=data.linear_acceleration.y<=10:
   #      shoulder=(data.linear_acceleration.y*10.0*(pi/180))
   #  elif 0>data.linear_acceleration.y>=-10:
   #      shoulder=(data.linear_acceleration.y*10.0*(pi/180))
   # # print("this is the shoulder:", shoulder)
   
    #pass a global variable from subsruber to joint states
   

    
    #min and max bounds 
    # print(shoulder,shoulder_angle,y)
    # print(bicep,bicep_angle,x)


   # print(x,y)
    if counter>0: 
        if -4<=data.linear_acceleration.x<=4:
            pass
        if 9<=data.linear_acceleration.x<=10:
             pub5_gripper.publish(Float64(-1.5))
        if -9>=data.linear_acceleration.x>=-10:
             pub5_gripper.publish(Float64(0))
        else:
            x= bicep_angle + sign(data.linear_acceleration.x)*5*(pi/180)
            x=min(max(x,-pi/2),pi/2)
            pub2_bicep.publish(Float64(numpy.around(x,2)))
        if -4<=data.linear_acceleration.y<=4:
            pass
        else:
            y= shoulder_angle + sign(data.linear_acceleration.y)*2*(pi/180)
            y=min(max(y,-pi/2),pi/2)
            pub1_shoulder.publish(Float64(numpy.around(y,2)))

        # pub2_bicep.publish(Float64(numpy.around(sin(time.time()-tstart)/2,2)))
       
        #I want these to be constant/not moving
        pub3_forearm.publish(Float64(0))
        pub4_wrist.publish(Float64(0))
       
def callback_function1(data):
    global ecu_pub, motor_pwm, servo_pwm

    if data.linear_acceleration.x>=4 and data.linear_acceleration.x<=10:
        motor_pwm=((data.linear_acceleration.x-4)*(1.0/9))
    elif data.linear_acceleration.x<=-4 and data.linear_acceleration.x>=-10:
        motor_pwm=((data.linear_acceleration.x+4)*(1.0/9))
    else:
        motor_pwm=0

    if data.linear_acceleration.y>=4 and data.linear_acceleration.y<=10:
        servo_pwm=((data.linear_acceleration.y-4)*(1.0/9))
    elif data.linear_acceleration.y<=-4 and data.linear_acceleration.y>=-10:
        servo_pwm=((data.linear_acceleration.y+4)*1.0/9)  
    else:
        servo_pwm =0

    # print(data.linear_acceleration)
    ecu_cmd.linear.x=motor_pwm
    ecu_cmd.angular.z=servo_pwm
    ecu_pub.publish(ecu_cmd)

def arduino_interface():
    global ecu_pub, motor_pwm, servo_pwm, ecu_cmd, shoulder, bicep
    global pub1_shoulder,pub2_bicep,pub3_forearm,pub4_wrist,pub5_gripper
    global shoulder_angle,bicep_angle,forearm_angle,wrist_angle,gripper_angle,initialized
    global counter, tstart

    init_node('arduino_interface')
    
    tstart = time.time()
    shoulder_angle = 0
    bicep_angle = 0
    forearm_angle = 0
    wrist_angle = 0
    gripper_angle = 0
    counter=0
    initialized = False
    # set node rate
    loop_rate   = 50
    dt          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    
    ecu_cmd = Twist()

    time_prev = time.time()
    
    #publishers
    ecu_pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size = 1)

    pub1_shoulder = rospy.Publisher('arm_shoulder_pan_joint/command',Float64, queue_size = 1)
    pub2_bicep = rospy.Publisher('arm_bicep_joint/command',Float64, queue_size = 1)
    pub3_forearm = rospy.Publisher('arm_forearm_joint/command',Float64, queue_size = 1)
    pub4_wrist = rospy.Publisher('arm_wrist_flex_joint/command',Float64, queue_size = 1)
    pub5_gripper = rospy.Publisher('gripper_joint/command',Float64, queue_size = 1)

    


    #subscribers
    imu_sub1 =Subscriber('imu/data1', Imu, callback_function1)
 
    imu_sub2 =Subscriber('imu/data2', Imu, callback_function2)

    jointState_sub = rospy.Subscriber('/joint_states', JointState, callback_function)
    tutorial = MoveGroupPythonIntefaceTutorial()

    while not rospy.is_shutdown():
        if counter==0 and initialized:
            raw_input()
            tutorial.go_to_joint_state([0,0,0,0,0])
            counter+=1 

        # wait
        rate.sleep()
    ###########################################################################
    #Tony's Code

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "pincher_arm"
        # group_name = "phantomx_pincher_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Available Planning Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()

        print "============ Printing end effector position"
        print move_group.get_current_pose()
        print ""

        move_group.set_goal_orientation_tolerance(180*pi/180)
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


    def go_to_joint_state(self,angles):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
        ## thing we want to do is move it to a slightly better configuration.
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = move_group.get_current_joint_values()
        # print joint_goal
        # raw_input()


        joint_goal[0] = angles[0]
        joint_goal[1] = angles[1]
        joint_goal[2] = angles[2]
        joint_goal[3] = angles[3]
        joint_goal[4] = angles[4]

        # joint_goal[0] = pi/2
        # joint_goal[1] = -pi/3
        # joint_goal[2] = -pi/3
        # joint_goal[3] = 0
        # joint_goal[4] = 0
        # joint_goal[5] = pi/3
        # joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def go_to_pose_goal(self, pos):
        # pos is [x,y,z] vector
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:

        pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.position.x = 0.223681620425
        # pose_goal.position.y = 0.198604744269
        # pose_goal.position.z = 0.146275787147

        # pose_goal.orientation.x = -0.0172435324159
        # pose_goal.orientation.y = 0.0453920549799
        # pose_goal.orientation.z = 0.354700815292
        # pose_goal.orientation.w = 0.933718133896

        # q = quaternion_from_euler(0,pi/4,0)
        # print "Quaternion"
        # print(q)
        # move_group.set_pose_target(pose_goal)
        # move_group.set_position_target([0.35326288981682941,-0.064228324711322768,0.024627257720643725])
        if len(pos)>3:
            # move_group.set_pose_target([pos[0],pos[1],pos[2],pos[3],pos[4],pos[5]])
            move_group.set_position_target([pos[0],pos[1],pos[2]])
        else:
            move_group.set_position_target([pos[0],pos[1],pos[2]])
        # move_group.set_position_target(pos)

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # print(plan)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return plan#all_close(pose_goal, current_pose, 0.01)


    def plan_cartesian_path(self, pos, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        print("WPOSE")
        print(wpose)

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = pos[0]
        pose_goal.position.y = pos[1]
        pose_goal.position.z = pos[2]+0.1

        pose_goal.orientation.x = -0.02991
        pose_goal.orientation.y = 0.6877
        pose_goal.orientation.z = 0.0315
        pose_goal.orientation.w = 0.724
        print(pose_goal)
        waypoints.append(pose_goal)
        pose_goal.position.z =  pos[2]
        print(pose_goal)
        waypoints.append(pose_goal)

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL


    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);

        ## END_SUB_TUTORIAL


    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL


    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
          # Test if the box is in attached objects
          attached_objects = scene.get_attached_objects([box_name])
          is_attached = len(attached_objects.keys()) > 0

          # Test if the box is in the scene.
          # Note that attaching the box will remove it from known_objects
          is_known = box_name in scene.get_known_object_names()

          # Test if we are in the expected state
          if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

          # Sleep so that we give other threads time on the processor
          rospy.sleep(0.1)
          seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL





#############################################################
if __name__ == '__main__':
    try:
        arduino_interface()
    except ROSInterruptException:
        pass
