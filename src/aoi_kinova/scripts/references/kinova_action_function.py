#!/usr/bin/env python3

import rospy
import actionlib
import cv2
from kinova_msgs.msg import ArmPoseAction, ArmPoseGoal
from geometry_msgs.msg import PoseStamped

def send_pose(prefix, pose_values, label=""):
    action_address = '/' + prefix + 'driver/pose_action/tool_pose'
    client = actionlib.SimpleActionClient(action_address, ArmPoseAction)

    rospy.loginfo(f"[{label}] Waiting for pose action server...")
    client.wait_for_server()

    goal = ArmPoseGoal()
    goal.pose.header.frame_id = prefix + "link_base"
    goal.pose.header.stamp = rospy.Time.now()

    goal.pose.pose.position.x = pose_values[0]
    goal.pose.pose.position.y = pose_values[1]
    goal.pose.pose.position.z = pose_values[2]
    goal.pose.pose.orientation.x = pose_values[3]
    goal.pose.pose.orientation.y = pose_values[4]
    goal.pose.pose.orientation.z = pose_values[5]
    goal.pose.pose.orientation.w = pose_values[6]

    rospy.loginfo(f"[{label}] Sending pose: {pose_values}")
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(20.0)):
        return client.get_result()
    else:
        print('        the joint angle action timed-out')
        client.cancel_all_goals()
        return None


if __name__ == '__main__':
    rospy.init_node('send_multiple_poses_node')

    robot_prefix = "j2n6s300_"
# -0.220
    poses = {
        "ssd_case_1": [-0.220, -0.380, 0.045, 1.0, 0, 0, 0],
        "ssd_case_2": [-0.140, -0.380, 0.045, 1.0, 0, 0, 0],
        "ssd_case_3": [-0.060, -0.380, 0.045, 1.0, 0, 0, 0],
        "ssd_case_4": [0.020, -0.380, 0.045, 1.0, 0, 0, 0],
        "ssd_case_5": [-0.220, -0.490, 0.055, 1.0, 0, 0, 0],
        "ssd_case_6": [-0.140, -0.490, 0.055, 1.0, 0, 0, 0],
        "ssd_case_7": [-0.060, -0.490, 0.055, 1.0, 0, 0, 0],
        "ssd_case_8": [0.020, -0.490, 0.055, 1.0, 0, 0, 0],
        "ssd_case_9": [-0.220, -0.600, 0.065, 1.0, 0, 0, 0],
        "ssd_case_10": [-0.140, -0.600, 0.065, 1.0, 0, 0, 0],
        "ssd_case_11": [-0.060, -0.600, 0.065, 1.0, 0, 0, 0],
        "ssd_case_12": [0.020, -0.600, 0.065, 1.0, 0, 0, 0],
        "ssd_case_home": [0.000, -0.300, 0.045, 1.0, 0, 0, 0]
    }

    # Open webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("Camera could not be opened")
        exit()

    for label, pose in poses.items():
        rospy.loginfo(f"[{label}] Displaying camera feed. Press any key to continue...")
        for _ in range(30):  # Display a few frames for the user to see
            ret, frame = cap.read()
            if ret:
                cv2.imshow("YOLO Webcam", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        cv2.waitKey(0)  # Wait for keypress before sending pose
        cv2.destroyAllWindows()
        result = send_pose(robot_prefix, pose, label)
        rospy.sleep(1.0)  # Optional delay between poses

    cap.release()
    cv2.destroyAllWindows()
