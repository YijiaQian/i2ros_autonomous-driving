#include <ros/ros.h>

#include <ros/console.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <math.h>
#include <std_msgs/Float64.h>

#define PI M_PI


#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf provides useful functions to convert tf 
// messages to eigen types and vice versa, have a look to the documentation:
// http://docs.ros.org/melodic/api/eigen_conversions/html/namespacetf.html
#include <eigen_conversions/eigen_msg.h>

class controllerNode{
  ros::NodeHandle nh;

  ros::Subscriber current_state, desired_Lvelocity, desired_Avelocity;
  ros::Publisher car_commands;
  ros::Timer timer;

	// Controller internals (you will have to set them below)
  double kp1, kp2, kp3, kd1, kd2, kd3;
  double lastErrorAcc = 0,errorAcc,lastErrorBreak = 0,errorBreak,lastErrorTurn = 0,errorTurn;
  double signalAcc = 0.1,signalTurn = 0,signalBreak = 0;
  double target_linear_velocity = 0.0, target_angular_velocity = 0.0, linear_velocity = 0.0, angular_velocity = 0.0;



  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle

  double hz;             // frequency of the main control loop

public:
  controllerNode():hz(1000.0){
      
      current_state = nh.subscribe("current_state_est", 1, &controllerNode::onCurrentState, this);
      desired_Lvelocity = nh.subscribe("target_linear_velocity", 1, &controllerNode::onDesiredLvelocity, this);
      desired_Avelocity = nh.subscribe("target_angular_velocity", 1, &controllerNode::onDesiredAvelocity, this);
      car_commands = nh.advertise<mav_msgs::Actuators>("car_commands", 1);
      timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
      nh.getParam("/params/kp1", kp1);
      nh.getParam("/params/kp2", kp2);
      nh.getParam("/params/kp3", kp3);
      nh.getParam("/params/kd1", kd1);
      nh.getParam("/params/kd2", kd2);
      nh.getParam("/params/kd3", kd3);
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
    ROS_INFO("Received current state message.");

    linear_velocity = cur_state.twist.twist.linear.x;
    angular_velocity = cur_state.twist.twist.angular.z;
    /*
    x << cur_state.pose.pose.position.x,cur_state.pose.pose.position.y,cur_state.pose.pose.position.z;
    v << cur_state.twist.twist.linear.x,cur_state.twist.twist.linear.y,cur_state.twist.twist.linear.z;
    omega << cur_state.twist.twist.angular.x,cur_state.twist.twist.angular.y,cur_state.twist.twist.angular.z;
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen (cur_state.pose.pose.orientation, q);
    R = q.toRotationMatrix();
    // Rotate omega
    omega = R.transpose()*omega;
    */
    //ROS_INFO("Current linear velocity: [%.2f, %.2f, %.2f]", v.x(), v.y(), v.z());
    //ROS_INFO("Current angular velocity: [%.2f, %.2f, %.2f]", omega.x(), omega.y(), omega.z());
  }

   void onDesiredLvelocity(const std_msgs::Float64::ConstPtr& msg){
    ROS_INFO("Received DesiredLvelocity.");
    target_linear_velocity = msg->data;
   }

   void onDesiredAvelocity(const std_msgs::Float64::ConstPtr& msg){
    ROS_INFO("Received DesiredAvelocity.");
    target_angular_velocity = msg->data;
   }

  void controlLoop(const ros::TimerEvent& t){
    ROS_INFO("Car_commands");
    //signalAcc
    if (target_linear_velocity >= linear_velocity){
      errorAcc = target_linear_velocity - linear_velocity;
      signalAcc = kp1 * errorAcc + kd1 * (lastErrorAcc - errorAcc);
      if (signalAcc > 0.5){
        signalAcc = 0.5;
      }
      if (signalAcc < 0){
        signalAcc = 0;
      }
      lastErrorAcc = errorAcc;
    }
    // signalTurn
      errorTurn = target_angular_velocity - angular_velocity;
      signalTurn = kp2 * errorTurn + kd2 * (lastErrorAcc - lastErrorTurn);
      lastErrorTurn = errorTurn;
       if (signalTurn > 0.5){
        signalTurn = 0.5;
      }
      if (signalTurn < (-0.5)){
        signalTurn = - 0.5;
      }
    /*
    //signalBreak
    if (target_linear_velocity < linear_velocity){
      errorBreak = target_linear_velocity - linear_velocity;
      signalAcc = kp3 * errorBreak + kd3 * (lastErrorBreak - errorBreak);
      if (signalBreak > 0.5){
        signalBreak = 0.5;
      }
      if (signalBreak <0){
        signalBreak = 0;
      }
      lastErrorBreak = errorBreak;
    }
    */

    mav_msgs::Actuators msg;
    msg.angular_velocities.resize(4);
    msg.angular_velocities[0] = signalAcc; // Acceleration
    msg.angular_velocities[1] = signalTurn;  // Turning angle
    msg.angular_velocities[2] = signalBreak;  // Breaking
    msg.angular_velocities[3] = 0;

    car_commands.publish(msg);

  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}
