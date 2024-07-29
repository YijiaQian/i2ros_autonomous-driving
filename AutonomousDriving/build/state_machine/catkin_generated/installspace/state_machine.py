#!/usr/bin/env python3

# Imports the rospy library, which is ROS’s Python client library
import rospy

# Smach is a library used for implementing finite state machines (FSM)
import smach

# Provides additional functionalities to use smach within a ROS environment
import smach_ros

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
from traffic_light_detector_pkg.msg import TrafficLightState

# A class that inherits from smach.State. It defines the behavior of the robot in the "drive" state
class DriveState(smach.State):
    
    # Defines the constructor, which is the initialization method for a Python class. It is automatically called when the class is instantiated
    def __init__(self):
        
        # Initialize the constructor of the smach.State class, setting the possible outcome "stop" of a state in the state machine. The outcomes parameter defines the possible transition results when the state finishes
        smach.State.__init__(self, outcomes=["stop"])

    # The execute method defines the actions that need to be performed when the state machine enters this state. "userdata" is a dictionary that allows passing data between states. In the smach state machine, userdata is used to share information between different states
    def execute(self, userdata):
        
        # Declare v and omega as global variables here
        global v, omega
        
        # Get the current ROS time in seconds and assign it to the current_time variable
        current_time = rospy.get_time()
        
        # Log entry into the drive state with timestamp, target speed, and angular velocity
        rospy.loginfo("Entering Drive state at time: %.2f with target speed: %.2f and angular velocity: %.2f" % (current_time, v.data, omega.data))
        
        # Create a ROS Rate object with a frequency of 50Hz and assign it to the rate variable. Can change
        rate = rospy.Rate(50)
   
        # Execute the loop while the signal value is false and ROS is not shutdown
        while not signal_value and not rospy.is_shutdown():
            
            # Publish the target linear velocity v to the target linear velocity topic
            target_v_pub.publish(v)
            
            # Publish the target angular velocity omega to the target angular velocity topic
            target_omega_pub.publish(omega)
            
            # Control the loop to meet the specified frequency
            rate.sleep()

        # Return the output state of the state machine as "stop". Transition to the stop state when the signal becomes False
        return "stop"


class StopState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["drive"])

    def execute(self, userdata):
        
        global v, omega
        current_time = rospy.get_time()
        rospy.loginfo("Entering Stop state at time: %.2f with target speed: %.2f and angular velocity: %.2f" % (current_time, v.data, omega.data))

        rate = rospy.Rate(50)

        # Execute the loop while the signal value is true and ROS is not shutdown
        while signal_value and not rospy.is_shutdown():
            v = Float64()
            
            # Set the data member of the v object to 0.0 in the stop state
            v.data = 0.0
            target_v_pub.publish(v)
            
            omega = Float64()
            omega.data = 0.0
            target_omega_pub.publish(omega)
            
            rate.sleep()

        # Return the output state of the state machine as "drive". Transition to the drive state when the signal becomes True
        return "drive"

# Define a callback function to handle target twist messages
def target_twist_callback(target_twist_msg):
    
    # Assign the linear velocity value (linear.y) from the received target twist message to the data member of v
    v.data = target_twist_msg.linear.x
    
    # Assign the angular velocity value (angular.z) from the received target twist message to the data member of omega
    omega.data = target_twist_msg.angular.z


class TrafficLight:
    def __init__(self) -> None:
        
        self.signal_sub = rospy.Subscriber("traffic_light_state", TrafficLightState, self.signal_callback)
        
        self.last_signal_color = "Unknown"
        
        # Initialize the counter variable to 0
        self.counter = 0

    def signal_callback(self, signal_msg):
      
        global signal_value

        if self.last_signal_color == signal_msg.color:
            self.counter += 1
        else:
            self.last_signal_color = signal_msg.color
            self.counter = 0
            
        if self.counter > 10:
            if signal_msg.color == "Red":
                signal_value = True
            elif signal_msg.color == "Green":
                signal_value = False
            else:
                pass
            self.counter = 0


def main():
    
    # Initializes a ROS node with the name "smach"
    rospy.init_node("smach")

    # Declare the signal_value variable as a global variable
    global signal_value
    
    # Initialize the signal_value variable to False
    signal_value = False
    
    # Create an instance of the TrafficLight class
    traffic_light = TrafficLight()

    # Declare the v and omega variables as global variables
    global v, omega
    
    # Create Float64 objects for linear and angular velocities
    v = Float64()
    omega = Float64()
    
    # ROS Subscriber for "cmd_vel" topic, calling target_twist_callback() on received Twist messages
    twist_sub = rospy.Subscriber("cmd_vel", Twist, target_twist_callback)

    # Declare target_v_pub and target_omega_pub as global variables
    global target_v_pub, target_omega_pub
    
    # Create a publisher for the topic "target_linear_velocity" with message type Float64 and queue size 10
    target_v_pub = rospy.Publisher("target_linear_velocity", Float64, queue_size=10)
    
    # Create a publisher for the topic "target_angular_velocity" with message type Float64 and queue size 10
    target_omega_pub = rospy.Publisher("target_angular_velocity", Float64, queue_size=10)

    # Create a state machine named 'sm' with no specified outcomes
    sm = smach.StateMachine(outcomes=[])

    with sm:
        
        # Add the "DRIVE" state to the state machine. DriveState() represents the state object for driving.
        # Transitions={"stop": "STOP"} specifies that when the "DRIVE" state ends with outcome "stop", transition to state "STOP"
        smach.StateMachine.add("DRIVE", DriveState(), transitions={"stop": "STOP"})
        
        # Add the "STOP" state to the state machine. StopState() represents the state object for stopping.
        # Transitions={"drive": "DRIVE"} specifies that when the "STOP" state ends with outcome "drive", transition to state "DRIVE"
        smach.StateMachine.add("STOP", StopState(), transitions={"drive": "DRIVE"})

    # Create an introspection server named 'server_name' to visualize the state machine 'sm' under the namespace '/SM_ROOT'
    ## sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    ## sis.start()

    # Executes the state machine `sm` and stores the result of the execution in `outcome`.
    outcome = sm.execute()

    # Keeps Python from exiting until this node is stopped by ROS
    rospy.spin()

if __name__ == "__main__":
    main()
