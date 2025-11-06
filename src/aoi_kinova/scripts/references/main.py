import rospy
from std_msgs.msg import Header
from kinova_msgs.msg import SSDStates
import ttkbootstrap as ttk
from ttkbootstrap.constants import *
import threading

# Status Mapping
STATUS_MAPPING = {
    '0': {'label': "Not Inspected", 'color': SECONDARY},
    '1': {'label': "H done - Good", 'color': SUCCESS},
    '2': {'label': "R done - Good", 'color': SUCCESS},
    '3': {'label': "H done - Bad", 'color': DANGER},
    '4': {'label': "R done - Bad", 'color': DANGER}, 
    '5': {'label': "H help", 'color': WARNING},
    '6': {'label': "R to do", 'color': DARK},
    '7': {'label': "Empty", 'color': LIGHT}
}

class ButtonControllerNode:
    def __init__(self, root, buttons, labels):
        rospy.init_node('aoi', anonymous=True)

        # self.publisher = rospy.Publisher('/pub/aoi/spain/status', SSDStates, queue_size=10)
        # self.subscriber = rospy.Subscriber('/sub/aoi/spain/status', SSDStates, self.listener_callback)

        self.publisher = rospy.Publisher('/current_states_GUI', String, queue_size=10)
        self.subscriber = rospy.Subscriber('/update_states_GUI', String, self.listener_callback)

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

    def listener_callback(self, msg):
        """Callback when new states are received from the /update_states_GUI topic."""
        ids = msg.id
        rospy.loginfo(f'Received state update: {ids}')

        for i, status in enumerate(ids):
            status_str = str(status)
            self.update_button_status(i, status_str, from_subscriber=True)

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
        status_ids = [int(status) for status in self.button_status]

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
            button = ttk.Button(row_frame, text=f"SSD {i+1}", bootstyle=SECONDARY, width=20, state=DISABLED)
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
