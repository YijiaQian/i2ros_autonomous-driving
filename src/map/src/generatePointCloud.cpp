#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参
const double camera_factor = 1000;
const double camera_cx = 160;
const double camera_cy = 120;
const double camera_fx = 120.00000000000001;
const double camera_fy = 120.00000000000001;

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
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            rgb_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
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

        PointCloud::Ptr cloud(new PointCloud);
        for (int m = 0; m < depth_image_.rows; m++)
        {
            for (int n = 0; n < depth_image_.cols; n++)
            {
                ushort d = depth_image_.ptr<ushort>(m)[n];
                if (d == 0)
                    continue;

                PointT p;
                p.z = double(d) / camera_factor;
                p.x = (n - camera_cx) * p.z / camera_fx;
                p.y = (m - camera_cy) * p.z / camera_fy;

                p.b = rgb_image_.ptr<uchar>(m)[n*3];
                p.g = rgb_image_.ptr<uchar>(m)[n*3 + 1];
                p.r = rgb_image_.ptr<uchar>(m)[n*3 + 2];

                cloud->points.push_back(p);
            }
        }

        cloud->height = 1;
        cloud->width = cloud->points.size();
        cloud->is_dense = false;

        // 将点云转换为ROS消息并发布
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.frame_id = "Quadrotor/Sensors/DepthCamera";
        cloud_pub_.publish(output);

        ROS_INFO("Point cloud published.");
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;
    ros::Publisher cloud_pub_;
    cv::Mat rgb_image_;
    cv::Mat depth_image_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_to_pointcloud");
    DepthToPointCloud d2p;
    ros::spin();
    return 0;
}
