#!/usr/bin/env python
import sys
import math
import rospy
import copy
import tf
import numpy
import moveit_commander 
import moveit_msgs.msg
import std_msgs.msg 
import geometry_msgs.msg 
import roslib; roslib.load_manifest('robotiq_c_model_control')
from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg
from robotiq_c_model_control.msg import _CModel_robot_input  as inputMsg
from apriltags_ros.msg import * 
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import *
from maskrcnn.msg import int_list
from rospy import init_node, is_shutdown

##___GLOBAL VARIABLES___###
velocity = 0.05 #velocity scaling factor (0, 1.0] - Safe value for a real robot is ~0.05

##___INITIALIZATION___###
moveit_commander.roscpp_initialize(sys.argv) #initialize the moveit commander
rospy.init_node('move_group_python_interface_tutorial', anonymous=True) #initialize rospy 
robot = moveit_commander.RobotCommander() #Instantiate a RobotCommander object
scene = moveit_commander.PlanningSceneInterface() #Instantiate a Planning SceneInterface Object
group = moveit_commander.MoveGroupCommander("manipulator") #Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. 
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory) #Create DisplayTrajectory publisher which is used to publish trajectories (RVIZ visual)

##__Publishers&Listeners__##
pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output)


#############################################################################################################################################################################################################
####____GRIPPER CONTROL____####
#############################################################################################################################################################################################################
###___Activate gripper___###
def gactive(pub):
  command = outputMsg.CModel_robot_output();
  command.rACT = 1
  command.rGTO = 1
  command.rSP  = 50
  command.rFR  = 150					
  pub.publish(command)
  rospy.sleep(0.5)
  return command

###___Reset gripper___###
def greset(pub):
  command = outputMsg.CModel_robot_output();
  command.rACT = 0
  pub.publish(command)
  rospy.sleep(0.5)

###___Set position of gripper___###
def gposition(pub,command, position):   ##0=open, 255=close
  #rospy.sleep(0.5)
  command = outputMsg.CModel_robot_output();
  command.rACT = 1
  command.rGTO = 1
  command.rSP  = 50
  command.rFR  = 150					
  command.rPR = position
  pub.publish(command)
  return command


###___Pick-up Object___###
## This function manipulates gripper and grabs object
## distance is the distance to dive before gripping and velocity is the speed of the motion. It rises 10cm after grabbing object
def pickup(command, down_distance, up_distance, vel):
    rospy.sleep(0.5)
    gposition(pub, command, 70) #increment gripper width
    rospy.sleep(1)
    

    resolution = 0.05 #resolution is interpreted as 1/resolution = number of interpolated points in the path
    pose_target = group.get_current_pose().pose
    x_1 = pose_target.position.x
    y_1 = pose_target.position.y
    z_1 = pose_target.position.z
    x_2 = x_1
    y_2 = y_1
    z_2 = z_1 + down_distance
    direction_vector = [x_2-x_1, y_2-y_1, z_2-z_1]
    pose_target = group.get_current_pose().pose #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"
    waypoints = []
    waypoints.append(pose_target)
    t = 0 # counter/increasing variabe for the parametric equation of straight line      
    while t <= 1.01:
        pose_target.position.x = x_1 + direction_vector[0]*t
        pose_target.position.y = y_1 + direction_vector[1]*t
        pose_target.position.z = z_1 + direction_vector[2]*t
        t += resolution 
        
        waypoints.append(copy.deepcopy(pose_target))
         
    del waypoints[:1]
    rospy.sleep(0.5)
    plan_execute_waypoints(waypoints, 0.03)

    command = outputMsg.CModel_robot_output();
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 50
    command.rFR  = 10						##force need to be adjusted later
    command.rPR = 150
    pub.publish(command)
    rospy.sleep(1)

    
    pose_target = group.get_current_pose().pose
    x_1 = pose_target.position.x
    y_1 = pose_target.position.y
    z_1 = pose_target.position.z
   
    x_2 = x_1
    y_2 = y_1
    z_2 = z_1 + up_distance
    direction_vector = [x_2-x_1, y_2-y_1, z_2-z_1]
    pose_target = group.get_current_pose().pose #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"
    waypoints = []
    waypoints.append(pose_target)
    t = 0 # counter/increasing variabe for the parametric equation of straight line      
    while t <= 1.01:
        pose_target.position.x = x_1 + direction_vector[0]*t
        pose_target.position.y = y_1 + direction_vector[1]*t
        pose_target.position.z = z_1 + direction_vector[2]*t
        t += resolution 
        
        waypoints.append(copy.deepcopy(pose_target))
         
    del waypoints[:1]
    rospy.sleep(0.5)
    plan_execute_waypoints(waypoints, 0.03)


#############################################################################################################################################################################################################
####____MOTION PLAN____####
#############################################################################################################################################################################################################
###___Linear Path___###
## This function makes the end-effector travel in a straight path 
def linear_path(axis_world, distance, vel):
    resolution = 0.05 #resolution is interpreted as 1/resolution = number of interpolated points in the path
    pose_target = group.get_current_pose().pose
    x_1 = pose_target.position.x
    y_1 = pose_target.position.y
    z_1 = pose_target.position.z
    if axis_world is 'x':
        x_2 = x_1 + distance
        y_2 = y_1
        z_2 = z_1
    if axis_world is 'y':
        x_2 = x_1
        y_2 = y_1 + distance
        z_2 = z_1
    if axis_world is 'z':
        x_2 = x_1
        y_2 = y_1
        z_2 = z_1 + distance
    direction_vector = [x_2-x_1, y_2-y_1, z_2-z_1]
    pose_target = group.get_current_pose().pose #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"
    waypoints = []
    waypoints.append(pose_target)
    t = 0 # counter/increasing variabe for the parametric equation of straight line      
    while t <= 1.01:
        pose_target.position.x = x_1 + direction_vector[0]*t
        pose_target.position.y = y_1 + direction_vector[1]*t
        pose_target.position.z = z_1 + direction_vector[2]*t
        t += resolution 
        
        waypoints.append(copy.deepcopy(pose_target))
         
    del waypoints[:1]
    plan_execute_waypoints(waypoints, vel)

###___JOINT VALUE MANIPULATION___###
## Manipulate by assigning joint values
def assign_joint_value(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5):
    group.set_max_velocity_scaling_factor(velocity)
    group_variable_values = group.get_current_joint_values() #create variable that stores joint values

    #Assign values to joints
    group_variable_values[0] = joint_0
    group_variable_values[1] = joint_1
    group_variable_values[2] = joint_2
    group_variable_values[3] = joint_3
    group_variable_values[4] = joint_4
    group_variable_values[5] = joint_5

    group.set_joint_value_target(group_variable_values) #set target joint values for 'manipulator' group
 
    plan1 = group.plan() #call plan function to plan the path (visualize on rviz)
    group.go(wait=True) #execute plan on real/simulation (gazebo) robot 
    #rospy.sleep(2) #sleep 2 seconds

def plan_execute_waypoints(waypoints, velocity):
    (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0) #parameters(waypoints, resolution_1cm, jump_threshold)
    plan= group.retime_trajectory(robot.get_current_state(), plan3, velocity) #parameter that changes velocity
    group.execute(plan)

#############################################################################################################################################################################################################
####____STATUS____####
#############################################################################################################################################################################################################


###___STATUS ROBOT___###
def manipulator_status():
    #You can get a list with all the groups of the robot like this:
    print "Robot Groups:"
    print robot.get_group_names()

    #You can get the current values of the joints like this:
    print "Current Joint Values:"
    print group.get_current_joint_values()

    #You can also get the current Pose of the end effector of the robot like this:
    print "Current Pose:"
    print group.get_current_pose()

    #Finally you can check the general status of the robot like this:
    print "Robot State:"
    print robot.get_current_state()

#############################################################################################################################################################################################################
####____OBJECT TRACKING____####
#############################################################################################################################################################################################################

def grasp_check():
    position = rospy.wait_for_message('/chatter', int_list, timeout = None)
    cpu_time = position.data[-1]
    position = numpy.delete(position.data, -1)
    position = numpy.reshape(position, (-1,7))
    if numpy.count_nonzero(position==4) != 0:
	grasp = "True"
    else:
	grasp = "False"
    return grasp

def object_positions():
    position = rospy.wait_for_message('/chatter', int_list, timeout = None)
    cpu_time = position.data[-1]
    position = numpy.delete(position.data, -1)
    position = numpy.reshape(position, (-1,7))
    gripper_position = numpy.zeros(7)
    cube_position = numpy.zeros((1,7))
    cylinder_position = numpy.zeros((1,7))
    if numpy.count_nonzero(position==1) == 1:
	gripper_index = numpy.argwhere(position == 1)
	gripper_position = position[gripper_index[0][0]]  
    if numpy.count_nonzero(position==2) != 0:
	cube_index = numpy.argwhere(position ==2)
	cube_position = position[cube_index[:,0]]
    if numpy.count_nonzero(position==3) != 0:
	cylinder_index = numpy.argwhere(position ==3)
	cylinder_position = position[cylinder_index[:,0]]
    if numpy.count_nonzero(position==4) != 0:
	grasp = True
    else:
	grasp = False
    print "Before sorting"
    print cube_position
    print cylinder_position
    #SORT BASED ON FIRST COLUMN AND RETURN
    cube_position = numpy.asarray(sorted(cube_position, key=lambda a_entry: a_entry[0]))  
    cylinder_position = numpy.asarray(sorted(cylinder_position, key=lambda a_entry: a_entry[0]))  
    
    print "after sorting"
    print cube_position
    print cylinder_position
    return gripper_position, cube_position, cylinder_position, cpu_time, grasp
    
def autonomous_tracking_x(object='cylinder', threshold=10, K_p=0.0015, vel=0.1):
    # GET ERROR VALUES	
    gripper_position, cube_position, cylinder_position, cpu_time, grasp = object_positions()
    #rospy.sleep(cpu_time)
    gripper_position, cube_position, cylinder_position, cpu_time, grasp = object_positions()
    if object=='cube':
	x_error = gripper_position[0] - cube_position[0,0]
    if object=='cylinder':
	x_error = gripper_position[0] - cylinder_position[0,0]
    # Move gripper while error is greater than threshold
    while numpy.absolute(x_error) > threshold:
 	
	#x_error will be negative if object is on the left, positive if object is on the right
	if object == 'cube' and gripper_position[6] != 0 and cube_position[0,6] != 0:	
	    linear_path('x', x_error * K_p, vel)
	    rospy.sleep(cpu_time)
	if object == 'cylinder' and gripper_position[6] != 0 and cylinder_position[0,6] != 0:	
	    linear_path('x', x_error * K_p, vel)
	    rospy.sleep(cpu_time)	
	gripper_position, cube_position, cylinder_position, cpu_time, grasp = object_positions()
    	if object=='cube':
	    x_error = gripper_position[0] - cube_position[0,0]
        elif object=='cylinder':
	    x_error = gripper_position[0] - cylinder_position[0,0]
        print x_error

def autonomous_tracking_z_cube(object='cylinder', threshold=50, K_p=0.0005, vel=0.1):
    # GET ERROR VALUES	
    gripper_position, cube_position, cylinder_position, cpu_time, grasp = object_positions()
    #rospy.sleep(cpu_time)
    gripper_position, cube_position, cylinder_position, cpu_time, grasp = object_positions()

    if object=='cube':
	z_distance = cube_position[0,5] - gripper_position[4]
    if object=='cylinder':
	z_distance = cylinder_position[0,5] - gripper_position[4] 

    # Move gripper while error is greater than threshold
    while z_distance > threshold:
 	
	#x_error will be negative if object is on the left, positive if object is on the right
	if object == 'cube' and gripper_position[6] != 0 and cube_position[0,6] != 0:	
	    linear_path('z', -z_distance * K_p, vel)
	    rospy.sleep(cpu_time)
	if object == 'cylinder' and gripper_position[6] != 0 and cylinder_position[0,6] != 0:	
	    linear_path('z', -z_distance * K_p, vel)
	    rospy.sleep(cpu_time)	
	gripper_position, cube_position, cylinder_position, cpu_time, grasp = object_positions()
	if object=='cube':
	    z_distance = cube_position[0,5] - gripper_position[4]
        elif object=='cylinder':
	    z_distance = cylinder_position[0,5] - gripper_position[4]
        print z_distance

# x, z, width, height, max, min, id 

def autonomous_tracking_z_cylinder(object='cylinder', threshold=10, K_p=0.0005, vel=0.1, offset = 100):
    # GET ERROR VALUES	
    gripper_position, cube_position, cylinder_position, cpu_time, grasp = object_positions()
    #rospy.sleep(cpu_time)
    gripper_position, cube_position, cylinder_position, cpu_time, grasp = object_positions()

    if object=='cube':
	z_distance = cube_position[0,1] - gripper_position[1]
    if object=='cylinder':
	z_distance = cylinder_position[0,1] - gripper_position[1] 
    error = z_distance - offset
    # Move gripper while error is greater than threshold
    while error > threshold:
	#x_error will be negative if object is on the left, positive if object is on the right
	if object == 'cube' and gripper_position[6] != 0 and cube_position[0,6] != 0:	
	    linear_path('z', -error * K_p, vel)
	    rospy.sleep(cpu_time)
	if object == 'cylinder' and gripper_position[6] != 0 and cylinder_position[0,6] != 0:	
	    linear_path('z', -error * K_p, vel)
	    rospy.sleep(cpu_time)	
	gripper_position, cube_position, cylinder_position, cpu_time, grasp = object_positions()
	if object=='cube':
	    z_distance = cube_position[0,1] - gripper_position[1]
        elif object=='cylinder':
	    z_distance = cylinder_position[0,1] - gripper_position[1]
        error = z_distance - offset
        print error

   


#############################################################################################################################################################################################################
####____MAIN____####
#############################################################################################################################################################################################################
###___Initiate node; subscribe to topic; call callback function___###
def manipulator_arm_control():
    cube_count = 3
    cylinder_count = 1
    tape_count = 2
    assign_joint_value(0.025, -2.114, -1.034, -4.706, -1.569, 1.597) # Home
    
    ###___PICK UP YELLOW CYLINDER___###
    i=0
    while i < cylinder_count:
        command = gactive(pub)
        rospy.sleep(0.5) 
        gposition(pub, command, 220)
        assign_joint_value(0.025, -2.114, -1.034, -4.706, -1.569, 1.597)  # Home
        autonomous_tracking_x(object='cylinder',  vel=0.05)
        autonomous_tracking_z_cylinder(object='cylinder', threshold=10, K_p=0.0009, vel=0.05, offset = 130)
        pickup(command, -0.08, 0.3, 0.05)
        grasping1 = grasp_check()
        grasping2 = grasp_check()
        print grasping1, grasping2
        if grasping1 == "True" or grasping2 == "True": 
            i = i + 1       
            assign_joint_value(-0.06813699403871709, -1.313883129750387, -2.182196919118063, -4.455761377011434, -1.6252854506122034, 1.5197675228118896) # Cylinder Box
            rospy.sleep(0.5)
            gposition(pub,command, 50)
            rospy.sleep(0.5)
    ###___PICK UP CUBE___###
    i = 0
    while i < cube_count:
        command = gactive(pub)
        rospy.sleep(0.5) 
        gposition(pub, command, 220)
        assign_joint_value(0.025, -2.114, -1.034, -4.706, -1.569, 1.597)  # Home
        autonomous_tracking_x(object='cube',  vel=0.05)
        autonomous_tracking_z_cube(object='cube', threshold=40, K_p=0.0009, vel=0.05)
        pickup(command, -0.06, 0.3, 0.05)
        rospy.sleep(4)
        grasping1 = grasp_check()
        grasping2 = grasp_check()
        print grasping1, grasping2
        if grasping1 == "True" or grasping2 == "True":
            i = i + 1
            print "in loop!"
            assign_joint_value(-0.00415, -2.52740, -0.53002, -4.1466, -1.6106, 1.563035) #Cube Box
            rospy.sleep(0.5)
            gposition(pub,command, 50)
            rospy.sleep(0.5)
            

    ###___PICK UP TAPE CYLINDER___###
    i=0
    while i < tape_count:
        command = gactive(pub)
        rospy.sleep(0.5) 
        gposition(pub, command, 220)
        assign_joint_value(0.025, -2.114, -1.034, -4.706, -1.569, 1.597)  # Home
        autonomous_tracking_x(object='cylinder',  vel=0.05)
        autonomous_tracking_z_cylinder(object='cylinder', threshold=10, K_p=0.0009, vel=0.05, offset = 120)
        pickup(command, -0.06, 0.3, 0.05)
        grasping1 = grasp_check()
        grasping2 = grasp_check()
        print grasping1, grasping2
        if grasping1 == "True" or grasping2 == "True":
            i = i + 1
            assign_joint_value(-0.06813699403871709, -1.313883129750387, -2.182196919118063, -4.455761377011434, -1.6252854506122034, 1.5197675228118896) # Cylinder Box
            rospy.sleep(0.5)
            gposition(pub,command, 50)
            rospy.sleep(0.5)
 


    assign_joint_value(0.025, -2.114, -1.034, -4.706, -1.569, 1.597)  # Home
    manipulator_status() #debug
    rospy.spin()



###___MAIN___###
if __name__ == '__main__':

    try:
        
        manipulator_arm_control()
        
        moveit_commander.roscpp_shutdown() #shut down the moveit_commander

    except rospy.ROSInterruptException: pass
