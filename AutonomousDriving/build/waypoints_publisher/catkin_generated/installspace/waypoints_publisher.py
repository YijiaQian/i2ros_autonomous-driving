#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from math import sqrt

current_pose = None  # Define as global variable outside of functions

def pose_callback(actual_pose):
    global current_pose
    current_pose = actual_pose.pose

def main():
    rospy.init_node('waypoints_publisher')
    pose_pub = rospy.Publisher('my_goal', PoseStamped, queue_size=10)  # Publishing PoseStamped instead of Pose
    rospy.Subscriber('pose_est', PoseStamped, pose_callback)

    frequency = rospy.get_param('~frequency', 10)
    poses = rospy.get_param('~poses')
    
    rospy.loginfo("Frequency set to: %d Hz", frequency)
    rospy.loginfo("Number of poses received: %d", len(poses))

    rate = rospy.Rate(frequency)
    rospy.sleep(3)

    pose_index = 0

    while not rospy.is_shutdown() and pose_index < len(poses):
        target_pose = Pose()
        target_pose.position = Point(**poses[pose_index]['position'])
        target_pose.orientation = Quaternion(**poses[pose_index]['orientation'])
        
        pose_stamped = PoseStamped()  # Creating PoseStamped message
        pose_stamped.header = Header(stamp=rospy.Time.now(), frame_id="world")  # Including timestamp and frame_id
        pose_stamped.pose = target_pose

        if current_pose:
            distance = sqrt((current_pose.position.x - target_pose.position.x) ** 2 +
                            (current_pose.position.y - target_pose.position.y) ** 2 +
                            (current_pose.position.z - target_pose.position.z) ** 2)

            if distance <= 1:
                rospy.loginfo("Close enough to waypoint %d, moving to next waypoint", pose_index + 1)
                pose_index += 1
                if pose_index >= len(poses):
                    rospy.loginfo("All poses have been processed")
                    break
            pose_pub.publish(pose_stamped)  # Publishing PoseStamped
        else:
            rospy.loginfo("Waiting for current_pose to be initialized...")
        rate.sleep()

if __name__ == '__main__':
    main()
