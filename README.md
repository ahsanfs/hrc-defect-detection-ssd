# Human–Robot Collaborative Defect Detection System (Kinova + Vision)

This workspace (`spain_taiwan_ws`) integrates the **Kinova Jaco2 robot**, **Webcam**,  **Smart Tray**, **defect inspection system**, and **iri_ssd_case_inspection_robot_behavior** for collaborative SSD case inspection. The **iri_ssd_case_inspection_robot_behavior** is published on another repo.
The system uses ROS to coordinate the robot, camera, and visual inspection logic.

---

## Prerequisites

Before running, ensure:

- The **camera is correctly mounted** on the Kinova end-effector.  
- The **USB camera** and **USB smart tray** are both connected to the PC.  
- The workspace has been built successfully (ROS Noetic):
  ```bash
  cd ~/spain_taiwan_ws
  catkin_make
  ```

---

## Usage

You will need several terminals (or tabs).  
Make sure each terminal sources the workspace environment. Also, make sure every tab to source the setup.bash.

---

### **Tab 1 – Start ROS Master**
```bash
roscore
```

---

### **Tab 2 – Environment setup**
```bash
cd ~/spain_taiwan_ws
source devel/setup.bash
```

---

### **Tab 3 – Launch Kinova robot**
```bash
cd ~/spain_taiwan_ws
source devel/setup.bash
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2n6s300
```

---

### **Tab 4 – Run smart tray logic**
```bash
cd ~/spain_taiwan_ws/src/aoi_kinova/scripts
python3 box_main.py
```

---

### **Tab 5 – Launch GUI interface**
```bash
cd ~/spain_taiwan_ws/src/aoi_kinova/scripts
python3 gui_main.py
```

---

### **Tab 6 – Close the gripper (prevent camera obstruction)**
```bash
cd ~/spain_taiwan_ws/src/aoi_kinova/scripts
rosrun kinova_demo fingers_action_client.py j2n6s300 percent -- 80 80 90
```

---

### **Tab 7 – Start inspection system server**
```bash
cd ~/spain_taiwan_ws/
rosrun inspection_system yolo_action_server.py
```

---

### **Tab 8 – Run inspection behavior controller**
```bash
cd ~/spain_taiwan_ws/
rosrun iri_ssd_case_inspection_robot_behavior iri_ssd_case_inspection_robot_behavior
```

---

## Notes

- Ensure all USB devices are recognized (`lsusb`) before running.  
- If the robot fails to connect, check that `roscore` is running and the correct `kinova_robotType` is used.  
- The `inspection_system` server must be running before launching the behavior node.  
- The GUI (`gui_main.py`) provides monitoring status of SSD states.

---

## Folder Structure

```
spain_taiwan_ws/
├── src/
│   ├── aoi_kinova/
│   │   └── scripts/
│   │       ├── box_main.py
│   │       ├── gui_main.py
│   ├── inspection_system/
│   └── iri_ssd_case_inspection_robot_behavior/
└── data_extraction_bag/
```

---

## ✍️ Author
**Ahsan F. S. (hucenrotia)**  
Laboratory for Human–Robot Collaboration, Spain–Taiwan Project  