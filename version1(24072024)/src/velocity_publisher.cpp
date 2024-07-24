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
  ros::Duration duration12(12.0);
  ros::Duration duration16(16.0);
  ros::Duration duration17(17.0);
  ros::Duration duration18(17.0);

  double v1,v2,v3,v4,a1,a2,a3,a4;
   nh.getParam("/params/v1", v1);
    nh.getParam("/params/v2", v2);
    nh.getParam("/params/v3", v3);
    nh.getParam("/params/v4", v4);
    nh.getParam("/params/a1", a1);
    nh.getParam("/params/a2", a2);
    nh.getParam("/params/a3", a3);
    nh.getParam("/params/a4", a4);

/*
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

//acc
 while (ros::ok() && (ros::Time::now() - start_time) < duration8) {
    lin_vel_msg.data = v1;
    ang_vel_msg.data = 0.0;

    linear_velocity_pub.publish(lin_vel_msg);
    angular_velocity_pub.publish(ang_vel_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  //break
  start_time = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start_time) < duration8) {
    lin_vel_msg.data = v2;
    ang_vel_msg.data = 0.0;

    linear_velocity_pub.publish(lin_vel_msg);
    angular_velocity_pub.publish(ang_vel_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // turn
  start_time = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start_time) < duration18) {
    lin_vel_msg.data = v2;
    ang_vel_msg.data = a1;
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
