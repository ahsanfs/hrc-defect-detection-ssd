#!/usr/bin/env python3

import rospy
import actionlib
import cv2
import numpy as np
import rospkg
import os
from inspection_system.msg import YoloDetectionAction, YoloDetectionResult
import time
from kinova_msgs.msg import SSDStates
from std_msgs.msg import Header

class YoloWebcamActionServer:
    def __init__(self):
        # rospack = rospkg.RosPack()
        # pkg_path = rospack.get_path('iri_ssd_case_inspection_robot_behavior_main')
        pkg_path = "/home/hucenrotia/spain_taiwan_ws/src/iri_ssd_case_inspection_robot_behavior/txt/target_objects_robot.txt"
        # txt_path = os.path.join(pkg_path, 'txt', 'target_objects_robot.txt')
        with open(pkg_path, 'r') as f:
            self.case_order = [
                int(line.strip().split('_')[-1]) - 1
                for line in f
                if line.strip()
            ]
        rospy.loginfo(f"[YoloInspector] loaded case order: {self.case_order}")
        self._next_idx = 0

        self.inspection_pub = rospy.Publisher('/update_states_inspection', SSDStates, queue_size=10)

        self.server = actionlib.SimpleActionServer("yolo_webcam_detection", YoloDetectionAction, self.execute, False)
        self.server.start()
        rospy.loginfo("YOLO Webcam Action Server Started")

        self.y_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config'))
        print("Path", self.y_path)
        self.labels_path = os.path.join(self.y_path, "obj.names")
        self.config_path = os.path.join(self.y_path, "1730_1.cfg")
        self.weights_path = os.path.join(self.y_path, "1730_1.weights")

        self.LABELS = open(self.labels_path).read().strip().split("\n")
        self.COLORS = np.random.randint(0, 255, size=(len(self.LABELS), 3), dtype="uint8")

        self.net = cv2.dnn.readNetFromDarknet(self.config_path, self.weights_path)
        ln = self.net.getLayerNames()
        self.ln = [ln[i - 1] for i in self.net.getUnconnectedOutLayers()]

        self.conf_threshold = 0.5
        self.nms_threshold = 0.3

        os.makedirs(os.path.expanduser("~/yolo_detections"), exist_ok=True)

    def publish_inspection_result(self, slot_index, classification):
        """Map classification result to SSD state and publish."""
        if classification == "bad":
            state_value = 2
        elif classification == "ask_human_help":
            state_value = 5
        elif classification == "good":
            state_value = 4
        else:
            state_value = 0  # fallback

        ids = [0] * 12
        ids[slot_index] = state_value

        msg = SSDStates()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.id = ids

        self.inspection_pub.publish(msg)
        rospy.loginfo(f"[Inspection] Published state: {ids}")


    def execute(self, goal):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            rospy.logerr("Webcam not accessible.")
            self.server.set_aborted()
            return

        detected_objects = []
        confidence_scores = []
        frame_counter = 0
        saved_frame_dir = os.path.expanduser("~/yolo_detections")

        while not rospy.is_shutdown() and frame_counter < 20:
            ret, frame = cap.read()
            if not ret:
                break

            crop_width_percent = 0.25 # 30% left+right
            crop_height_percent = 0.10  # 15% top+bottom

            (H_full, W_full) = frame.shape[:2]
            x_start = int(W_full * crop_width_percent)
            x_end = int(W_full * (1 - crop_width_percent))
            y_start = int(H_full * crop_height_percent)
            y_end = int(H_full * (1 - crop_height_percent))

            frame = frame[y_start:y_end, x_start:x_end]

            (H, W) = frame.shape[:2]
            blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
            self.net.setInput(blob)
            layer_outputs = self.net.forward(self.ln)

            boxes, confidences, classIDs = [], [], []

            for output in layer_outputs:
                for detection in output:
                    scores = detection[5:]
                    classID = np.argmax(scores)
                    confidence = scores[classID]
                    if confidence > self.conf_threshold:
                        box = detection[0:4] * np.array([W, H, W, H])
                        (centerX, centerY, width, height) = box.astype("int")
                        x = int(centerX - width / 2)
                        y = int(centerY - height / 2)

                        boxes.append([x, y, int(width), int(height)])
                        confidences.append(float(confidence))
                        classIDs.append(classID)

            idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.nms_threshold)

            if len(idxs) > 0:
                for i in idxs.flatten():
                    label = self.LABELS[classIDs[i]]
                    conf = confidences[i]
                    detected_objects.append(label)
                    confidence_scores.append(conf)

                    x, y, w, h = boxes[i]
                    color = [int(c) for c in self.COLORS[classIDs[i]]]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                    cv2.putText(frame, f"{label}: {conf:.2f}", (x, y - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Save frame
            save_path = os.path.join(saved_frame_dir, f"frame_{frame_counter:02d}.jpg")
            cv2.imwrite(save_path, frame)

            # Show frame
            cv2.imshow("YOLO Webcam", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

            frame_counter += 1

        cap.release()
        # time.sleep(5) # removeeeee
        # cv2.destroyAllWindows()

        # Final results
        avg_conf = sum(confidence_scores) / len(confidence_scores) if confidence_scores else 0.0
        # 70/85, 90, 95
        if avg_conf >= 0.95:
            classification = "bad"
        elif avg_conf >= 0.40:
            classification = "ask_human_help"
        else:
            classification = "good"

        result = YoloDetectionResult()
        result.detected_objects = ", ".join(set(detected_objects)) if detected_objects else "None"
        result.confidence_score = avg_conf
        result.classification = classification

        # Example: Assume we know the slot to inspect (e.g., 4 for SSD 5)
        # slot_index = 4 
        slot_index = goal.slot_index
        print("Slot_index", slot_index)
        slot_index = int(slot_index.strip().split('_')[-1]) - 1
        
        
        # slot_index = self.case_order[self._next_idx]
        # rospy.loginfo(f"[YoloInspector] inspection #{self._next_idx+1} → slot_index={slot_index}")
        # self._next_idx = (self._next_idx + 1) % len(self.case_order)
 
        self.publish_inspection_result(slot_index, classification)


        rospy.loginfo("Sent results — Avg Confidence: %.2f | Classification: %s", avg_conf, classification)
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('yolo_webcam_action_server')
    server = YoloWebcamActionServer()
    rospy.spin()


# what about if the last ssd is bad (human side)
# update the arduino, so once the next state is 111111111111, so change the last state of 7 to 3.
# then save the finished all inspection, on json, give the timestamp as header as well