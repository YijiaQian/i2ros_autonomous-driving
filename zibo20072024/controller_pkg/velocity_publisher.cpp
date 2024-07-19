#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "velocity_publisher");
  ros::NodeHandle nh;

  ros::Publisher linear_velocity_pub = nh.advertise<std_msgs::Float64>("target_linear_velocity", 1);
  ros::Publisher angular_velocity_pub = nh.advertise<std_msgs::Float64>("target_angular_velocity", 1);

  ros::Rate loop_rate(10); // 10 Hz

  std_msgs::Float64 lin_vel_msg;
  std_msgs::Float64 ang_vel_msg;

  ros::Time start_time = ros::Time::now();
  
  ros::Duration duration2(2.0);
  ros::Duration duration3(3.0);
  ros::Duration duration4(4.0);
  ros::Duration duration8(8.0);

/* test1 --succeed
  // -x forward
  while (ros::ok() && (ros::Time::now() - start_time) < duration8) {
    lin_vel_msg.data = -5;
    ang_vel_msg.data = 0.0;

    linear_velocity_pub.publish(lin_vel_msg);
    angular_velocity_pub.publish(ang_vel_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  start_time = ros::Time::now();
  // stop
  while (ros::ok() && (ros::Time::now() - start_time) < duration4) {
    lin_vel_msg.data = 0.0;
    ang_vel_msg.data = 0.0;

    linear_velocity_pub.publish(lin_vel_msg);
    angular_velocity_pub.publish(ang_vel_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
*/
   

  start_time = ros::Time::now();
  // turn
  while (ros::ok() && (ros::Time::now() - start_time) < duration8) {
    lin_vel_msg.data = -5.0;
    ang_vel_msg.data = 5.0;

    linear_velocity_pub.publish(lin_vel_msg);
    angular_velocity_pub.publish(ang_vel_msg);

    ros::spinOnce();
    loop_rate.sleep();

  }

/*
   start_time = ros::Time::now();
  // y forward
  while (ros::ok() && (ros::Time::now() - start_time) < duration2) {
    lin_vel_msg.data = -5.0;
    ang_vel_msg.data = 0.0;

    linear_velocity_pub.publish(lin_vel_msg);
    angular_velocity_pub.publish(ang_vel_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
*/
  return 0;
}
