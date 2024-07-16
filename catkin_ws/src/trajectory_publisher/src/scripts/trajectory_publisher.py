#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from grid.py import Grid, a_star_search
import math
import tf
from tf.transformations import euler_from_quaternion

class PathPlanner:
    def __init__(self, grid):
        self.grid = grid

    def plan_path(self, start, goal):
        return a_star_search(self.grid, start, goal)

def create_path_message(path):
    path_msg = Path()
    path_msg.header.frame_id = "map"

    for waypoint in path:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.position.z = 0
        pose.pose.orientation.w = 1.0  # Assuming no rotation
        path_msg.poses.append(pose)

    return path_msg

def trajectory_publisher():
    rospy.init_node('trajectory_publisher', anonymous=True)
    pub = rospy.Publisher('/car/trajectory', Path, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Define your grid environment
    grid = Grid(10, 10)  # Example grid size 10x10
    # Set obstacles
    grid.set_obstacle(3, 3)
    grid.set_obstacle(3, 4)
    grid.set_obstacle(3, 5)

    path_planner = PathPlanner(grid)
    start = (0, 0)
    goal = (7, 7)

    while not rospy.is_shutdown():
        path = path_planner.plan_path(start, goal)
        path_msg = create_path_message(path)
        pub.publish(path_msg)
        rate.sleep()

class TrajectoryPublisher:
    def __init__(self):
        rospy.init_node('trajectory_publisher', anonymous=True)
        self.path_sub = rospy.Subscriber('/car/path', Path, self.path_callback)
        self.cmd_vel_pub = rospy.Publisher('/car/cmd_vel', Twist, queue_size=10)
        self.current_path = None
        self.current_pose = None
        self.pose_sub = rospy.Subscriber('/car/pose', PoseStamped, self.pose_callback)
        self.rate = rospy.Rate(10)  # 10 Hz

    def path_callback(self, msg):
        self.current_path = msg
        self.publish_trajectory()

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def publish_trajectory(self):
        if self.current_path is None or self.current_pose is None:
            return

        for pose in self.current_path.poses:
            twist = self.calculate_twist(pose)
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

    def calculate_twist(self, target_pose):
        twist = Twist()
        if self.current_pose is None:
            return twist

        dx = target_pose.pose.position.x - self.current_pose.position.x
        dy = target_pose.pose.position.y - self.current_pose.position.y

        distance = math.sqrt(dx**2 + dy**2)
        target_yaw = math.atan2(dy, dx)

        _, _, current_yaw = euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ])

        yaw_error = target_yaw - current_yaw

        # Simple proportional controller
        twist.linear.x = min(1.0, distance)  # Cap the speed
        twist.angular.z = yaw_error

        return twist

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tp = TrajectoryPublisher()
        tp.run()
    except rospy.ROSInterruptException:
        pass
