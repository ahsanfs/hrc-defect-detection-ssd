import rospy
from std_msgs.msg import Header
from kinova_msgs.msg import SSDStates  # Assuming SSDStates is your custom message
import serial
import time

# Serial connection settings
serial_port = '/dev/ttyACM0'
baud_rate = 115200
ser = serial.Serial(serial_port, baud_rate, timeout=1)
time.sleep(2)

current_gui_state = []

def ros_callback(data):
    """ROS callback to handle incoming data and publish it."""
    binary_string = data.data
    rospy.loginfo(f"Received data: {binary_string}")
    id_list = [int(bit) for bit in binary_string]
    check_and_publish_ssd_state(id_list)


def check_and_publish_ssd_state(id_list):
    """Check current state and publish only if there are changes."""
    global current_gui_state
    print(current_gui_state)
    print(id_list)
    
    # for i in enumerate(current_gui_state):
    #     # for y in enumerate(id_list):
    #     if current_gui_state[i] == 0 and id_list[i] == 7:
    #         print(id_list[i])
    #         current_gui_state[i] = id_list[i]
    #         print("Change to empty")
    #     elif current_gui_state[i] == 7 and id_list[i] == 1:
    #         print(id_list[i])
    #         current_gui_state[i] = id_list[i]
    #         print("Change to inpsected")
    #     elif current_state_GUI[i] == id_list[i]:
    #         print("Nothing change")
    #     else:
    #         print("Unknown")

    if current_gui_state != id_list:  # Compare with the current state from /update_states_GUI
        rospy.loginfo(f"State has changed. Publishing new state: {id_list}")
        publish_ssd_state(id_list)
    else:
        rospy.loginfo("State has not changed, no need to publish.")




def publish_ssd_state(id_list):
    """Publish the custom SSDState message."""
    ssd_state_msg = SSDStates()
    ssd_state_msg.header = Header()
    ssd_state_msg.header.stamp = rospy.Time.now()
    ssd_state_msg.box = id_list
    
    pub.publish(ssd_state_msg)


def read_from_arduino():
    """Reads data from Arduino and processes it."""
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            try:
                data = ser.readline().decode().strip()
                if data:
                    parse_data(data)
            except serial.SerialException as e:
                rospy.logerr(f"Serial error: {e}")
                break
        time.sleep(0.1)


def parse_data(data):
    """Parses the incoming data from Arduino and processes the status."""
    try:
        current_status, saved_status = data.split(' ')
        rospy.loginfo(f"Current Status: {current_status}")
        rospy.loginfo(f"Saved Status: {saved_status}")

        # Convert the saved_status string into a list of integers
        id_list = [int(bit) for bit in saved_status]
        # print(id_list)
        
        # Check the state and publish if necessary
        check_and_publish_ssd_state(id_list)

    except ValueError:
        rospy.logwarn(f"Received unexpected data format: {data}")


# def update_gui_state(data):
#     global current_gui_state
#     current_gui_state = data.id
    # print("Current State: " + current_gui_state)


if __name__ == '__main__':
    rospy.init_node('arduino_state_publisher', anonymous=True)

    # Publisher for the SSDStates message
    pub = rospy.Publisher('/update_states_box', SSDStates, queue_size=10)

    # Subscriber to get updates from the /update_states_GUI topic
    # rospy.Subscriber('/current_states_GUI', SSDStates, update_gui_state)

    try:
        read_from_arduino()
    except rospy.ROSInterruptException:
        pass
    finally:
        ser.close()  # Ensure serial connection is closed when the script is stopped


# Update, if the state from box change, but it still has 7, and if receive another 7, change the previous 7 as 3