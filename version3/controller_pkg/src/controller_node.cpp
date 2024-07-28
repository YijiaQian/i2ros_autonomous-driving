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
  /*
  // Controller internals (you will have to set them below)
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
*/
  double hz;                        // frequency of the main control loop
  double k1, k2, k3,kd1, kd2, kd3,kf1, kf2;  // coefficient of PD controll
  double last_err1=0,last_err2=0;  //error for PD controll
  double cur_linVel_x=0,cur_linVel_y=0,cur_linVel=0,cur_angVel_z=0;
  double tar_linVel=0, tar_angVel=0;
  double signal_acc=0, signal_turn=0, signal_angVel=0,signal_bre=0;
  double L=4;   //car length

public:
  controllerNode():hz(10.0){
      
      current_state = nh.subscribe("current_state_est", 1, &controllerNode::onCurrentState, this);
      desired_Lvelocity = nh.subscribe("target_linear_velocity", 1, &controllerNode::onDesiredLvelocity, this);
      desired_Avelocity = nh.subscribe("target_angular_velocity", 1, &controllerNode::onDesiredAvelocity, this);
      car_commands = nh.advertise<mav_msgs::Actuators>("car_commands", 1);
      timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
      
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
    cur_linVel_x = cur_state.twist.twist.linear.x;
    cur_linVel_y = cur_state.twist.twist.linear.y;
    cur_angVel_z = cur_state.twist.twist.angular.z;

    nh.getParam("/params/kf1", kf1);
    nh.getParam("/params/kf2", kf2);
    nh.getParam("/params/k1", k1);
    nh.getParam("/params/k2", k2);
    nh.getParam("/params/k3", k3);
    nh.getParam("/params/kd1", kd1);
    nh.getParam("/params/kd2", kd2);
    nh.getParam("/params/kd3", kd3);
    //ROS_INFO(",cur_poseX,cur_posY,curVelX,curVelY,curAng : [%.2f, %.2f,%.2f, %.2f, %.2f]", cur_state.pose.pose.position.x,cur_state.pose.pose.position.y,cur_linVel_x, cur_linVel_y,cur_angVel_z);
  }

void onDesiredLvelocity(const std_msgs::Float64::ConstPtr& msg){
    
    tar_linVel = msg->data;
    //ROS_INFO("Received DesiredLvelocity:[%.2f]", tar_linVel);

   }

void onDesiredAvelocity(const std_msgs::Float64::ConstPtr& msg){
    
    tar_angVel = msg->data;
    tar_angVel = -tar_angVel;
    //ROS_INFO("Received DesiredAvelocity: [%.2f]", tar_angVel);
   }


  void controlLoop(const ros::TimerEvent& t){
    cur_linVel = std::sqrt(cur_linVel_x * cur_linVel_x + cur_linVel_y * cur_linVel_y);  //current velocity

    tar_linVel= kf1 * tar_linVel;                  //Feedfoward control
    tar_angVel= kf2 * tar_angVel; 

    double linVel_err = tar_linVel - cur_linVel;    //Error
    double linVel_err_dot = linVel_err - last_err1; // Calculate derivative of errors
    last_err1 = linVel_err;                         // Update last errors

     // for linear velocity (acceleration and break)
     if(linVel_err>=0){
        signal_acc = k1 * linVel_err - kd1 * linVel_err_dot; 
        if(signal_acc<0){
          signal_acc=0;
        }
        if (signal_acc > 2.5) {
        signal_acc = 2.5;
        }
     }else{
        signal_acc=0;
        signal_bre=k3 * linVel_err - kd3 * linVel_err_dot;
        if (signal_bre < -2.5) {
          signal_bre = -2.5;
        }
        if(signal_bre>0){
          signal_bre=0;
        }
       }
  
    // Calculate turning angle based on current velocity and target angular velocity
    double angVel_err = tar_angVel - cur_angVel_z;
    double angVel_err_dot = angVel_err - last_err2;
    last_err2 = angVel_err;

    if (cur_linVel != 0) {
        signal_angVel=k2*angVel_err+kd2*angVel_err_dot;
        signal_turn = -(atan(L * signal_angVel / cur_linVel));
        
        if(tar_angVel=0){
          signal_turn=0;
        }

        if (signal_turn < -1) {
        signal_turn = -1;
        } else if (signal_turn > 1) {
          signal_turn = 1;
        }

      } else {
        signal_turn = 0;
    }

    mav_msgs::Actuators msg;

    msg.angular_velocities.resize(4);

    msg.angular_velocities[0] = signal_acc;  // Acceleration
    msg.angular_velocities[1] = signal_turn;  // Turning angle
    msg.angular_velocities[2] = signal_bre;  // Breaking
    msg.angular_velocities[3] = 0;

    car_commands.publish(msg);
    //ROS_INFO("Acc,bre,tVel,cVel,err,errDot[%.2f],[%.2f],[%.2f],[%.2f],[%.2f],[%.2f]", signal_acc,signal_bre,tar_linVel,cur_linVel,linVel_err,linVel_err_dot);
    //ROS_INFO("turn,tVel,cVel,err,errDot[%.2f],[%.2f],[%.2f],[%.2f],[%.2f]", signal_turn,tar_angVel,cur_angVel_z,angVel_err,angVel_err_dot);
  }

};
int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}
