#!/usr/bin/env python3

import rospy
import actionlib
from inspection_system.msg import YoloDetectionAction, YoloDetectionGoal

def done_cb(state, result):
    rospy.loginfo("Detected Objects: %s", result.detected_objects)
    rospy.loginfo("Average Confidence Score: %.2f", result.confidence_score)
    rospy.loginfo("Classification: %s", result.classification)

def webcam_client():
    rospy.init_node('yolo_webcam_action_client')
    client = actionlib.SimpleActionClient("yolo_webcam_detection", YoloDetectionAction)
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()

    goal = YoloDetectionGoal()
    goal.dummy = "start"
    client.send_goal(goal, done_cb=done_cb)

    rospy.loginfo("Sent webcam detection goal. Waiting for result...")
    client.wait_for_result()

if __name__ == "__main__":
    webcam_client()
