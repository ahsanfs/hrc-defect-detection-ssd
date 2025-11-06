import rospy
from std_msgs.msg import Header
from kinova_msgs.msg import SSDStates
import ttkbootstrap as ttk
from ttkbootstrap.constants import *
import threading
import time
import json
from datetime import datetime

# Status Mapping --> adding two another action --> 
STATUS_MAPPING = {
    '0': {'label': "Not Inspected", 'color': SECONDARY},
    '1': {'label': "H done - Good", 'color': SUCCESS},
    '4': {'label': "R done - Good", 'color': SUCCESS},
    '3': {'label': "H done - Bad", 'color': DANGER},
    '2': {'label': "R done - Bad", 'color': DANGER}, 
    '5': {'label': "H help", 'color': WARNING},
    '6': {'label': "R to do", 'color': DARK},
    '7': {'label': "Empty", 'color': LIGHT}
}

class ButtonControllerNode:
    def __init__(self, root, buttons, labels):
        rospy.init_node('aoi', anonymous=True)
        
        self.publisher = rospy.Publisher('/current_states_GUI', SSDStates, queue_size=10)
        self.subscriber = rospy.Subscriber('/update_states_box', SSDStates, self.listener_callback)
        self.inspection_subscriber = rospy.Subscriber('/update_states_inspection', SSDStates, self.inspection_callback)

        self.waiting_for_reset = False
        self.box_state = [0] * len(buttons)
        self.inspection_state = [0] * len(buttons)
        self.final_state = [0] * len(buttons)
        self.locked = [False] * len(buttons)
        self.last_index_with_7 = -1
        self.seven_history = []
        self.first_seven_index = None
        
        self.prev_state = [0] * len(buttons)
        self.last_uninspected_index = None
        self.waiting_for_human_check = False


        self.root = root
        self.buttons = buttons
        self.labels = labels
        self.button_status = ['0'] * len(buttons)

        style = ttk.Style()
        style.configure('CustomRed.TButton', background='#FFFFFF', bordercolor='red', lightcolor='#FF0000', darkcolor='#0000FF', borderwidth=5, foreground='#000000')
        style.configure('CustomBlue.TButton', relief="solid", bordercolor='red', borderwidth=5)
        style.configure('Large.TButton', background='#808080', foreground='#FFFFFF', font=("Helvetica", 20))
        style.configure('custom.TButton', background='red', foreground='white', font=('Helvetica', 20))
        style.configure('TButton', font=('Helvetica', 20))

        # Start periodic publishing in a separate thread
        self.start_periodic_publishing()

    def apply_status_7_rule(self, new_state):
        # Track new 7 arrivals
        for i, val in enumerate(new_state):
            if val == 7 and i not in self.seven_history:
                self.seven_history.append(i)

        # If two or more, change the first-arrived one
        if len(self.seven_history) >= 2:
            first_index = self.seven_history.pop(0)  # remove and get oldest
            if new_state[first_index] == 7:  # only overwrite if still 7
                new_state[first_index] = 3
            # clear all history after correction
            self.seven_history.clear()

        return new_state


    def update_combined_state(self):
        """Merge box and inspection states and update GUI."""

        # Check if we're in reset waiting mode
        if self.waiting_for_reset:
            # Track any new 7 during waiting and convert it unless it becomes 1
            if all(state == 0 for state in self.box_state):
                # Save log before full reset
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                log_data = {
                    "timestamp": timestamp,
                    "ros_time": str(rospy.Time.now().to_sec()),
                    "final_state_before_reset": self.final_state
                }
                with open("log.json", "a") as f:
                    f.write(json.dumps(log_data) + "\n")

                time.sleep(1)

                rospy.loginfo("Arduino box reset detected — final state reset complete.")
                self.final_state = [0] * len(self.buttons)
                self.inspection_state = [0] * len(self.buttons)
                self.locked = [False] * len(self.buttons)
                self.waiting_for_reset = False

            # Skip other processing while waiting
            return

        # Step 2: Normal state merging
        for i in range(len(self.buttons)):
            current = self.final_state[i]
            proposed = max(self.box_state[i], self.inspection_state[i])
            if current in [0, 7]:  # Only allow updates on fresh or empty
                self.final_state[i] = proposed
                self.locked[i] = proposed in [1, 2, 3, 4, 5, 6]

        self.final_state = self.apply_status_7_rule(self.final_state)

        # Print if only 1 SSD is left uninspected
        zero_indices = [i for i, s in enumerate(self.final_state) if s == 0]
        if len(zero_indices) == 1:
            self.last_uninspected_index = zero_indices[0]
            rospy.loginfo(f"Only one SSD left uninspected: SSD {self.last_uninspected_index+1}")
        
        # Track transition 0 -> 7
        if self.last_uninspected_index is not None and self.final_state[self.last_uninspected_index] == 7:
            rospy.loginfo(f"SSD {self.last_uninspected_index+1} moved to state 7 (human checking)")
            self.waiting_for_human_check = True

        # Handle decision after 7
        if self.waiting_for_human_check and self.last_uninspected_index is not None:
            ssd_state = self.final_state[self.last_uninspected_index]
            if ssd_state == 0:
                rospy.logwarn(f"SSD {self.last_uninspected_index+1} reverted to 0 after 7 — marking as 3 (bad)")
                self.final_state[self.last_uninspected_index] = 3
                self.waiting_for_human_check = False
            elif ssd_state == 1:
                rospy.loginfo(f"SSD {self.last_uninspected_index+1} confirmed as good (1)")
                self.waiting_for_human_check = False
                self.last_uninspected_index = None
                
                
        # Trigger waiting state ONLY if all are != 0 AND != 7
        if all(state != 0 and state != 7 for state in self.final_state):
            rospy.loginfo("All active states filled (no 0 or 7) — entering reset wait mode.")
            self.waiting_for_reset = True

        # Update GUI
        for i, status in enumerate(self.final_state):
            self.update_button_status(i, str(status), from_subscriber=False)

    def listener_callback(self, msg):
        self.box_state = list(msg.box)
        self.update_combined_state()

    def inspection_callback(self, msg):
        self.inspection_state = list(msg.id)
        self.update_combined_state()
    
    def update_button_status(self, index, status, from_subscriber=False):
        """Update the button status and its corresponding label."""
        status_info = STATUS_MAPPING.get(status, {'label': "Unknown", 'color': 'custom.TButton'})

        if status == '6':
            self.buttons[index].config(style='CustomRed.TButton')
        else:
            self.buttons[index].config(bootstyle=status_info['color'])

        self.labels[index]["text"] = status_info['label']
        self.button_status[index] = status

        if from_subscriber:
            self.publish_button_states()

    def publish_button_states(self):
        """Publish the current button states to the '/current_state_GUI' topic."""
        # status_ids = [int(status) for status in self.button_status]
        status_ids = self.final_state


        custom_msg = SSDStates()
        custom_msg.header = Header()
        custom_msg.header.stamp = rospy.Time.now()
        custom_msg.id = status_ids
        
        self.publisher.publish(custom_msg)
        rospy.loginfo(f"Published current GUI state: {status_ids}")

    def start_periodic_publishing(self):
        """Start a periodic publishing thread to continuously publish the current state."""
        def periodic_publish():
            rate = rospy.Rate(1)  # Publish every 1 second
            while not rospy.is_shutdown():
                self.publish_button_states()
                rate.sleep()

        # Run the periodic publishing in a separate thread
        threading.Thread(target=periodic_publish, daemon=True).start()

def on_button_click(index, controller):
    """Handle button clicks to cycle through the status."""
    current_status = controller.button_status[index]
    try:
        current_status_int = int(current_status)
    except ValueError:
        current_status_int = 1

    # Increment status (cycle through 1 to 7)
    next_status = str(current_status_int + 1) if current_status_int < 8 else '1'
    
    # Update the button status
    controller.update_button_status(index, next_status)

def toggle_fullscreen(event=None):
    """Toggle fullscreen on F11 key press."""
    root.attributes("-fullscreen", not root.attributes("-fullscreen"))
    return "break"

def main():
    global root
    root = ttk.Window()
    root.title("Simple Counter Spain")
    root.geometry("800x600")

    buttons = []
    labels = []

    # Layout for buttons
    button_order = [
        [0, 1, 2, 3],
        [4, 5, 6, 7],
        [8, 9, 10, 11]
    ]

    for row in button_order:
        row_frame = ttk.Frame(root)
        row_frame.pack(padx=10, pady=10, expand=True)
        for i in row:
            button = ttk.Button(row_frame, text=f"SSD {i+1}", bootstyle=SECONDARY, width=20)
            button.grid(row=0, column=row.index(i), padx=10, pady=10, ipady=100, ipadx=30)
            buttons.append(button)
            
            label = ttk.Label(row_frame, text="Not Inspected", font=("Helvetica", 20))
            label.grid(row=1, column=row.index(i), padx=10, pady=10)
            labels.append(label)

    # Initialize the controller node
    button_controller_node = ButtonControllerNode(root, buttons, labels)
    
    # Bind the buttons to their corresponding click handlers
    for i, button in enumerate(buttons):
        button.config(command=lambda i=i: on_button_click(i, button_controller_node))

    # Bind F11 for fullscreen toggle
    root.bind("<F11>", toggle_fullscreen)
    
    # Run the ROS spin loop in a separate thread
    threading.Thread(target=lambda: rospy.spin(), daemon=True).start()

    # Start the GUI main loop
    root.mainloop()

if __name__ == '__main__':
    main()
