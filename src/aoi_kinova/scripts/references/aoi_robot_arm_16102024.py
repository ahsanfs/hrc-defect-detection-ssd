#!/usr/bin/env python3

import rospy
from flask import Flask, request, render_template_string
import json

import actionlib
from kinova_msgs.srv import AddPoseToCartesianTrajectory, AddPoseToCartesianTrajectoryRequest, ClearTrajectories, ClearTrajectoriesRequest
from kinova_msgs.msg import KinovaPose,SetFingersPositionAction,SetFingersPositionGoal,FingerPosition
import requests
from std_msgs.msg import String
import time

kinova_kartesian_pos=""
kinova_reach_goal=False

currentFingerPosition =[0.0,0.0,0.0]
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger

# _debug_mode=False

# app = Flask(__name__)
# data_received = {}
current_status = "000000000000"  # Store the current status string from /sub/aoi/spain/status

def getCurrentFingerPosition():
    # wait to get current position
    topic_address = '/j2n6s300_driver/out/tool_pose'
    rospy.Subscriber(topic_address, FingerPosition, setCurrentFingerPosition)
    rospy.wait_for_message(topic_address, FingerPosition)
    # print('obtained current position ')


def setCurrentFingerPosition(feedback):
    global currentFingerPosition
    currentFingerPosition[0] = feedback.finger1
    currentFingerPosition[1] = feedback.finger2
    currentFingerPosition[2] = feedback.finger3
    # if _debug_mode:
    rospy.loginfo(rospy.get_caller_id() + "kinova_finger_pos:\n %s \n", feedback)

def unitParser(unit_, finger_value_, relative_):
    """ Argument unit """
    global currentFingerPosition

    # transform between units
    if unit_ == 'turn':
        # get absolute value
        if relative_:
            finger_turn_absolute_ = [finger_value_[i] + currentFingerPosition[i] for i in range(0, len(finger_value_))]
        else:
            finger_turn_absolute_ = finger_value_

        finger_turn_ = finger_turn_absolute_
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_turn_]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn_]

    elif unit_ == 'mm':
        # get absolute value
        finger_turn_command = [x/1000 * finger_maxTurn / finger_maxDist for x in finger_value_]
        if relative_:
            finger_turn_absolute_ = [finger_turn_command[i] + currentFingerPosition[i] for i in range(0, len(finger_value_))]
        else:
            finger_turn_absolute_ = finger_turn_command

        finger_turn_ = finger_turn_absolute_
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_turn_]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn_]
    elif unit_ == 'percent':
        # get absolute value
        finger_turn_command = [x/100.0 * finger_maxTurn for x in finger_value_]
        if relative_:
            finger_turn_absolute_ = [finger_turn_command[i] + currentFingerPosition[i] for i in
                                     range(0, len(finger_value_))]
        else:
            finger_turn_absolute_ = finger_turn_command

        finger_turn_ = finger_turn_absolute_
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_turn_]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn_]
    else:
        raise Exception("Finger value have to be in turn, mm or percent")

    return finger_turn_, finger_meter_, finger_percent_
       
def kinova_set_fingger(finger_pos_percent):
    global kinova_reach_goal
    # getCurrentFingerPosition()
    finger_turn, finger_meter, finger_percent = unitParser("percent", finger_pos_percent, False)
    
    positions_temp1 = [max(0.0, n) for n in finger_turn]
    positions_temp2 = [min(n, finger_maxTurn) for n in positions_temp1]
    positions = [float(n) for n in positions_temp2]
    rospy.wait_for_service('/j2n6s300_driver/in/clear_trajectories')
    
    try:
        clear_pose = rospy.ServiceProxy('/j2n6s300_driver/in/clear_trajectories', ClearTrajectories)
        #set_finger_pos = rospy.ServiceProxy('/j2n6s300_driver/fingers_action/finger_positions', SetFingersPositionAction)
        client = actionlib.SimpleActionClient('/j2n6s300_driver/fingers_action/finger_positions', SetFingersPositionAction)

        request_clear = ClearTrajectoriesRequest()

        client.wait_for_server()
        goal = SetFingersPositionGoal()
        goal.fingers.finger1 = positions[0]
        goal.fingers.finger2 = positions[1]
        goal.fingers.finger3 = positions[2]
        #set_finger_pos(goal)
        # print(goal.fingers.finger1)
        response_clear = clear_pose(request_clear)
        client.send_goal(goal)
        
        while not kinova_reach_goal:
            # getCurrentFingerPosition()
            # print(goal.fingers)
            kinova_check_fingger_reach_goal(goal.fingers.finger1,goal.fingers.finger2,goal.fingers.finger3)
        # print(kinova_reach_goal)
        kinova_reach_goal=False
        
        '''
        if client.wait_for_result(rospy.Duration(5.0)):
            rospy.logwarn("\n%s\n", str(client.get_result()))
        else:
            client.cancel_all_goals()
            rospy.logwarn('\n        the gripper action timed-out\n')
          #  return None
        '''

    except rospy.ServiceException as e:
        rospy.logerr("action call failed: %s", e)
        
def kinova_check_fingger_reach_goal(goal_Fingger_1,goal_Fingger_2,goal_Fingger_3,offset_=100):
    global kinova_reach_goal
    if goal_Fingger_1-offset_< currentFingerPosition[0] and currentFingerPosition[0]<goal_Fingger_1+offset_ and goal_Fingger_2-offset_< currentFingerPosition[1] and currentFingerPosition[1]<goal_Fingger_2+offset_ and goal_Fingger_3-offset_< currentFingerPosition[2] and currentFingerPosition[2]<goal_Fingger_3+offset_ :
        kinova_reach_goal=True    
            
def set_kinova_kartesian_pos(msg):
    global kinova_kartesian_pos
    kinova_kartesian_pos=msg
    #if _debug_mode:
    #    rospy.loginfo(rospy.get_caller_id() + "kinova_kartesian_pos:\n %s \n", msg)
       
def kinova_move(k_x,k_y,k_z,r_deg_x,r_deg_y,r_deg_z):
    global kinova_reach_goal
    # Wait for the service to be available
    rospy.wait_for_service('/j2n6s300_driver/in/clear_trajectories')
    rospy.wait_for_service('/j2n6s300_driver/in/add_pose_to_Cartesian_trajectory')

    try:
        # Create a service proxy
        clear_pose = rospy.ServiceProxy('/j2n6s300_driver/in/clear_trajectories', ClearTrajectories)
        add_pose_service = rospy.ServiceProxy('/j2n6s300_driver/in/add_pose_to_Cartesian_trajectory', AddPoseToCartesianTrajectory)
        
        # Clear traj
        request_clear = ClearTrajectoriesRequest()
        # Create a request object
        request = AddPoseToCartesianTrajectoryRequest()
        request.X = k_x/1000
        request.Y = -k_y/1000
        request.Z = k_z/1000
        request.ThetaX = r_deg_x*3.14/180
        request.ThetaY = r_deg_y*3.14/180
        request.ThetaZ = r_deg_z*3.14/180

        # Call the service
        response_clear = clear_pose(request_clear)
        response = add_pose_service(request)

        while not kinova_reach_goal:
            rospy.Subscriber("/j2n6s300_driver/out/cartesian_command", KinovaPose,set_kinova_kartesian_pos)
            rospy.wait_for_message("/j2n6s300_driver/out/cartesian_command", KinovaPose)
            kinova_check_reach_goal(request.X,request.Y,request.Z,request.ThetaX,request.ThetaY,request.ThetaZ)
        print(kinova_reach_goal)
        kinova_reach_goal=False
        rospy.loginfo("Service call successful: %s", response)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        
def kinova_check_reach_goal(goal_X,goal_Y,goal_Z,goal_ThetaX,goal_ThetaY,goal_ThetaZ):
    global kinova_reach_goal
    if goal_X-0.010< kinova_kartesian_pos.X and kinova_kartesian_pos.X<goal_X+0.010 and goal_Y-0.010< kinova_kartesian_pos.Y and kinova_kartesian_pos.Y<goal_Y+0.010 and goal_Z-0.010< kinova_kartesian_pos.Z and kinova_kartesian_pos.Z<goal_Z+0.010 and goal_ThetaX-0.010< kinova_kartesian_pos.ThetaX and kinova_kartesian_pos.ThetaX<goal_ThetaX+0.010 and goal_ThetaY-0.010< kinova_kartesian_pos.ThetaY and kinova_kartesian_pos.ThetaY<goal_ThetaY+0.010 and goal_ThetaZ-0.010< kinova_kartesian_pos.ThetaZ and kinova_kartesian_pos.ThetaZ<goal_ThetaZ+0.010:
    
        kinova_reach_goal=True
        
#### Homing        
def kinova_homing_c():
    kinova_set_fingger([0.0,0.0,0.0])
    kinova_move(-10.0,300,150,175,1,-90)
    kinova_set_fingger([90.0,90.0,90.0])
#### Retract


## go to
def kinova_ssd1():
    # kinova_set_fingger([0.0,0.0,0.0])
    kinova_move(110.0,580,100,175,1,-90)

def kinova_ssd2():
    # kinova_set_fingger([0.0,0.0,0.0])
    kinova_move(30.0,580,100,175,1,-90)

def kinova_ssd3():
    # kinova_set_fingger([0.0,0.0,0.0])
    kinova_move(-40.0,580,100,175,1,-90)

def kinova_ssd4():
    # kinova_set_fingger([0.0,0.0,0.0])
    kinova_move(-120.0,580,100,175,1,-90)


def kinova_ssd5():
    # kinova_set_fingger([0.0,0.0,0.0])
    kinova_move(110.0,480,100,175,1,-90)

def kinova_ssd6():
    # kinova_set_fingger([0.0,0.0,0.0])
    kinova_move(30.0,480,100,175,1,-90)

def kinova_ssd7():
    # kinova_set_fingger([0.0,0.0,0.0])
    kinova_move(-40.0,480,100,175,1,-90)

def kinova_ssd8():
    # kinova_set_fingger([0.0,0.0,0.0])
    # kinova_move(110.0,480,100,175,1,-90)
    kinova_move(-120.0,480,100,175,1,-90)


def kinova_ssd9():
    # kinova_set_fingger([0.0,0.0,0.0])
    # kinova_move(-120.0,580,100,175,1,-90)
    kinova_move(110.0,380,100,175,1,-90)

def kinova_ssd10():
    # kinova_set_fingger([0.0,0.0,0.0])
    # kinova_move(-40.0,580,100,175,1,-90)
    kinova_move(30.0,380,100,175,1,-90)

def kinova_ssd11():
    # kinova_set_fingger([0.0,0.0,0.0])
    kinova_move(-40.0,380,100,175,1,-90)

def kinova_ssd12():
    # kinova_set_fingger([0.0,0.0,0.0])
    kinova_move(-120.0,380,100,175,1,-90)




def currentPOI(data):
    global current_status
    current_status = data.data  # Update the current status string
    rospy.loginfo(f"Current POI status updated: {current_status}")

def selectedPOI(data):
    global current_status
    status_string = data.data  # Get the 12-bit binary string from /robot/poi
    rospy.loginfo(f"Received POI status: {status_string}")
    
    # Ensure that we have a valid 12-bit string
    if len(status_string) == 12:
        # Reverse the string so that index 0 represents SSD 1 and index 11 represents SSD 12
        # status_string = status_string[::-1]
        
        # Find the first '1' in the string to determine which SSD to inspect
        for i, bit in enumerate(status_string):
            if bit == '1':
                if current_status[i] == '0':  # Check if the SSD hasn't been inspected yet
                    # Call the appropriate SSD function based on the bit position
                    if i == 0:
                        kinova_ssd1()
                    elif i == 1:
                        kinova_ssd2()
                    elif i == 2:
                        kinova_ssd3()
                    elif i == 3:
                        kinova_ssd4()
                    elif i == 4:
                        kinova_ssd5()
                    elif i == 5:
                        kinova_ssd6()
                    elif i == 6:
                        kinova_ssd7()
                    elif i == 7:
                        kinova_ssd8()
                    elif i == 8:
                        kinova_ssd9()
                    elif i == 9:
                        kinova_ssd10()
                    elif i == 10:
                        kinova_ssd11()
                    elif i == 11:
                        kinova_ssd12()

                    # Update the current status for the SSD to '1'
                    new_status_list = list(current_status)
                    new_status_list[i] = '1'
                    current_status = ''.join(new_status_list)

                    # Publish the updated status
                    pub.publish(String(data=current_status[::-1]))  # Publish the updated status
                    break  # Only inspect the first '1' we encounter
                else:
                    # If the SSD has already been inspected, log a warning
                    rospy.logwarn(f"SSD {12 - i} has already been inspected.")
                    break

if __name__ == '__main__':

    # try:
    # Initialize the ROS node
    rospy.init_node('kinova_llm_node_controller')
    subscriber = rospy.Subscriber('/sub/aoi/spain/status', String, currentPOI)
    subscriber = rospy.Subscriber('/robot/ssd', String, selectedPOI)
    pub = rospy.Publisher('/update_states_GUI', SSDStates, queue_size=10)
    
    # print("Ping")
    print("Ros strated")
    kinova_homing_c()
    rospy.spin()
        
    # except rospy.ROSInterruptException:
    #     pass
    # app.run(host="127.0.0.3",port=5000,debug=True)


