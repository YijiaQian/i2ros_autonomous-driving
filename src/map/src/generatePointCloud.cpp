#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class DepthToPointCloud
{
public:
    DepthToPointCloud()
        : it_(nh_)
    {
        // 订阅深度图像和RGB图像
        image_sub_ = it_.subscribe("/unity_ros/OurCar/Sensors/RGBCameraRight/image_raw", 1, &DepthToPointCloud::imageCb, this);
        depth_sub_ = it_.subscribe("/unity_ros/OurCar/Sensors/DepthCamera/image_raw", 1, &DepthToPointCloud::depthCb, this);
        
        // 点云发布器
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("pointcloud", 1);
        cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud2", 1);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            rgb_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
            //ROS_INFO("Received RGB image.");
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void depthCb(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            depth_image_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
            //ROS_INFO("Received depth image.");
            generatePointCloud();
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void generatePointCloud()
    {
        if (rgb_image_.empty() || depth_image_.empty())
        {
            ROS_WARN("RGB or Depth image not received yet!");
            return;
        }

        sensor_msgs::PointCloud cloud;
        cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = "Quadrotor/Sensors/DepthCamera";

        sensor_msgs::ChannelFloat32 channel;
        channel.name = "rgb";
        for (int m = 0; m < depth_image_.rows; m++)
        {
            for (int n = 0; n < depth_image_.cols; n++)
            {
                ushort d = depth_image_.ptr<ushort>(m)[n];
                if (d == 0) continue;

                // 计算点云原始坐标
                Eigen::Vector3f point(
                    (n - camera_cx) * double(d) / camera_factor / camera_fx,
                    (m - camera_cy) * double(d) / camera_factor / camera_fy,
                    double(d) / camera_factor
                );

                // 应用旋转矩阵，沿X轴旋转-90度
                Eigen::Matrix3f rotation = Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitX()).toRotationMatrix();
                Eigen::Vector3f transformed_point = rotation * point;

                geometry_msgs::Point32 p;
                p.x = transformed_point[0];
                p.y = transformed_point[1];
                p.z = transformed_point[2];

                cloud.points.push_back(p);

                // 处理RGB数据
                uint8_t r = rgb_image_.ptr<uchar>(m)[n*3 + 2];
                uint8_t g = rgb_image_.ptr<uchar>(m)[n*3 + 1];
                uint8_t b = rgb_image_.ptr<uchar>(m)[n*3];
                uint32_t rgb = (r << 16) | (g << 8) | b;
                float rgb_float = *reinterpret_cast<float*>(&rgb);
                channel.values.push_back(rgb_float);
            }
        }

        cloud.channels.push_back(channel);

        // 将PointCloud转换为PointCloud2
        sensor_msgs::PointCloud2 cloud2;
        sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);

        // 将点云转换为ROS消息并发布
        cloud_pub_.publish(cloud);
        cloud2_pub_.publish(cloud2);
        //ROS_INFO("Point cloud published.");
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;
    ros::Publisher cloud_pub_;
    ros::Publisher cloud2_pub_;
    cv::Mat rgb_image_;
    cv::Mat depth_image_;

    static constexpr double camera_factor = 1000.0;
    static constexpr double camera_cx = 160.0;
    static constexpr double camera_cy = 120.0;
    static constexpr double camera_fx = 120.0;
    static constexpr double camera_fy = 120.0;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "generatePointCloud");
    DepthToPointCloud d2p;
    ros::spin();
    return 0;
}
