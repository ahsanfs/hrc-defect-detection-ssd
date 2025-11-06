#!/usr/bin/env python3

import rospy
from kinova_msgs.srv import AddPoseToCartesianTrajectory, AddPoseToCartesianTrajectoryRequest, ClearTrajectories, ClearTrajectoriesRequest
from kinova_msgs.msg import KinovaPose, SetFingersPositionAction, SetFingersPositionGoal, FingerPosition, SSDStates
import actionlib
from std_msgs.msg import Header
from kinova_msgs.msg import SSDStates  # Assuming SSDStates is your custom message
import time

kinova_kartesian_pos = ""
kinova_reach_goal = False
current_status = [0] * 12  # Store the current status for each SSD as a list of integers

currentFingerPosition = [0.0, 0.0, 0.0]
finger_maxDist = 18.9 / 2 / 1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger

def kinova_check_reach_goal(goal_X,goal_Y,goal_Z,goal_ThetaX,goal_ThetaY,goal_ThetaZ):
    global kinova_reach_goal
    if goal_X-0.010< kinova_kartesian_pos.X and kinova_kartesian_pos.X<goal_X+0.010 and goal_Y-0.010< kinova_kartesian_pos.Y and kinova_kartesian_pos.Y<goal_Y+0.010 and goal_Z-0.010< kinova_kartesian_pos.Z and kinova_kartesian_pos.Z<goal_Z+0.010 and goal_ThetaX-0.010< kinova_kartesian_pos.ThetaX and kinova_kartesian_pos.ThetaX<goal_ThetaX+0.010 and goal_ThetaY-0.010< kinova_kartesian_pos.ThetaY and kinova_kartesian_pos.ThetaY<goal_ThetaY+0.010 and goal_ThetaZ-0.010< kinova_kartesian_pos.ThetaZ and kinova_kartesian_pos.ThetaZ<goal_ThetaZ+0.010:
    
        kinova_reach_goal=True
        
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

def kinova_set_finger(finger_pos_percent):
    global kinova_reach_goal
    finger_turn, finger_meter, finger_percent = unitParser("percent", finger_pos_percent, False)
    positions_temp1 = [max(0.0, n) for n in finger_turn]
    positions_temp2 = [min(n, finger_maxTurn) for n in positions_temp1]
    positions = [float(n) for n in positions_temp2]
    rospy.wait_for_service('/j2n6s300_driver/in/clear_trajectories')
    
    try:
        clear_pose = rospy.ServiceProxy('/j2n6s300_driver/in/clear_trajectories', ClearTrajectories)
        client = actionlib.SimpleActionClient('/j2n6s300_driver/fingers_action/finger_positions', SetFingersPositionAction)
        request_clear = ClearTrajectoriesRequest()
        client.wait_for_server()
        goal = SetFingersPositionGoal()
        goal.fingers.finger1 = positions[0]
        goal.fingers.finger2 = positions[1]
        goal.fingers.finger3 = positions[2]
        response_clear = clear_pose(request_clear)
        client.send_goal(goal)
        
        while not kinova_reach_goal:
            kinova_check_finger_reach_goal(goal.fingers.finger1, goal.fingers.finger2, goal.fingers.finger3)
        kinova_reach_goal = False
        
    except rospy.ServiceException as e:
        rospy.logerr("Action call failed: %s", e)
        
def kinova_check_finger_reach_goal(goal_Finger_1, goal_Finger_2, goal_Finger_3, offset_=100):
    global kinova_reach_goal
    if goal_Finger_1 - offset_ < currentFingerPosition[0] < goal_Finger_1 + offset_ and \
       goal_Finger_2 - offset_ < currentFingerPosition[1] < goal_Finger_2 + offset_ and \
       goal_Finger_3 - offset_ < currentFingerPosition[2] < goal_Finger_3 + offset_:
        kinova_reach_goal = True

def kinova_move(k_x, k_y, k_z, r_deg_x, r_deg_y, r_deg_z):
    global kinova_reach_goal
    rospy.wait_for_service('/j2n6s300_driver/in/add_pose_to_Cartesian_trajectory')
    
    try:
        clear_pose = rospy.ServiceProxy('/j2n6s300_driver/in/clear_trajectories', ClearTrajectories)
        add_pose_service = rospy.ServiceProxy('/j2n6s300_driver/in/add_pose_to_Cartesian_trajectory', AddPoseToCartesianTrajectory)
        request_clear = ClearTrajectoriesRequest()
        request = AddPoseToCartesianTrajectoryRequest()
        request.X = k_x / 1000
        request.Y = -k_y / 1000
        request.Z = k_z / 1000
        request.ThetaX = r_deg_x * 3.14 / 180
        request.ThetaY = r_deg_y * 3.14 / 180
        request.ThetaZ = r_deg_z * 3.14 / 180
        response_clear = clear_pose(request_clear)
        response = add_pose_service(request)
        
        while not kinova_reach_goal:
            rospy.Subscriber("/j2n6s300_driver/out/cartesian_command", KinovaPose, set_kinova_kartesian_pos)
            rospy.wait_for_message("/j2n6s300_driver/out/cartesian_command", KinovaPose)
            kinova_check_reach_goal(request.X, request.Y, request.Z, request.ThetaX, request.ThetaY, request.ThetaZ)
        kinova_reach_goal = False
        rospy.loginfo("Service call successful: %s", response)
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def set_kinova_kartesian_pos(msg):
    global kinova_kartesian_pos
    kinova_kartesian_pos = msg


#### Homing        
def kinova_homing_c():
    kinova_set_finger([0.0,0.0,0.0])
    kinova_move(-10.0,300,150,175,1,-90)
    kinova_set_finger([90.0,90.0,90.0])
#### Retract

def kinova_ssd1(): kinova_move(110.0, 580, 100, 175, 1, -90)
def kinova_ssd2(): kinova_move(30.0, 580, 100, 175, 1, -90)
def kinova_ssd3(): kinova_move(-40.0, 580, 100, 175, 1, -90)
def kinova_ssd4(): kinova_move(-120.0, 580, 100, 175, 1, -90)
def kinova_ssd5(): kinova_move(110.0, 480, 100, 175, 1, -90)
def kinova_ssd6(): kinova_move(30.0, 480, 100, 175, 1, -90)
def kinova_ssd7(): kinova_move(-40.0, 480, 100, 175, 1, -90)
def kinova_ssd8(): kinova_move(-120.0, 480, 100, 175, 1, -90)
def kinova_ssd9(): kinova_move(110.0, 380, 100, 175, 1, -90)
def kinova_ssd10(): kinova_move(30.0, 380, 100, 175, 1, -90)
def kinova_ssd11(): kinova_move(-40.0, 380, 100, 175, 1, -90)
def kinova_ssd12(): kinova_move(-120.0, 380, 100, 175, 1, -90)

def currentPOI(data):
    global current_status
    current_status = list(data.id)
    # rospy.loginfo(f"Current POI status updated: {current_status}")

def selectedPOI(data):
    global current_status
    status_list = list(data.id)  # List of integers representing the SSD states
    rospy.loginfo(f"Received POI status: {status_list}")

    # Loop through each SSD and perform actions based on the state
    for i, bit in enumerate(status_list):
        if bit == 2 and current_status[i] == 0:  # Check if SSD needs inspection
            if i == 0:
                kinova_ssd12()
            elif i == 1:
                kinova_ssd11()
            elif i == 2:
                kinova_ssd10()
            elif i == 3:
                kinova_ssd9()
            elif i == 4:
                kinova_ssd8()
            elif i == 5:
                kinova_ssd7()
            elif i == 6:
                kinova_ssd6()
            elif i == 7:
                kinova_ssd5()
            elif i == 8:
                kinova_ssd4()
            elif i == 9:
                kinova_ssd3()
            elif i == 10:
                kinova_ssd2()
            elif i == 11:
                kinova_ssd1()

            current_status[i] = 2  # Mark the SSD as inspected

            # Publish the updated status
            updated_status_msg = SSDStates()
            updated_status_msg.header = Header(stamp=rospy.Time.now())
            updated_status_msg.id = current_status
            pub.publish(updated_status_msg)

            break  # Only inspect the first SSD that needs it
        else:
            rospy.logwarn(f"SSD {i+1} has already been inspected or is not ready for inspection.")

if __name__ == '__main__':
    rospy.init_node('kinova_llm_node_controller')

    rospy.Subscriber('/current_states_GUI', SSDStates, currentPOI)

    rospy.Subscriber('/robot/ssd', SSDStates, selectedPOI)

    pub = rospy.Publisher('/update_states_GUI', SSDStates, queue_size=10)

    kinova_homing_c()

    rospy.spin()
