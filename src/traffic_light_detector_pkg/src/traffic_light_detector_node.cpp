#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <traffic_light_detector_pkg/TrafficLightState.h>

class TrafficLightDetector {
public:
    TrafficLightDetector()
    {
        semantic_sub_ = nh_.subscribe("/unity_ros/OurCar/Sensors/SemanticCamera/image_raw", 1, &TrafficLightDetector::semanticCallback, this);
        rgb_sub_ = nh_.subscribe("/unity_ros/OurCar/Sensors/RGBCameraRight/image_raw", 1, &TrafficLightDetector::rgbCallback, this);
        traffic_light_pub_ = nh_.advertise<traffic_light_detector_pkg::TrafficLightState>("traffic_light_state", 1);
    }

    void semanticCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            semantic_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void rgbCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            rgb_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
            detectTrafficLight();
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber semantic_sub_;
    ros::Subscriber rgb_sub_;
    ros::Publisher traffic_light_pub_;
    cv::Mat semantic_image_;
    cv::Mat rgb_image_;

    void detectTrafficLight()
    {
        if (semantic_image_.empty() || rgb_image_.empty()) {
            return;
        }

        // 定义图像的中间宽度区域
        int rows = semantic_image_.rows;
        int cols = semantic_image_.cols;
        int mid_x = cols / 2;
        int width = cols / 4; // 中间区域宽度为图像宽度的1/4
        cv::Rect middle_region(mid_x - width / 2, 0, width, rows); // 覆盖整个高度

        // 假设语义图像中交通灯的标注为特定的颜色，例如黄色
        // 这里以黄色为例
        cv::Mat mask;
        cv::inRange(semantic_image_(middle_region), cv::Scalar(0, 200, 200), cv::Scalar(50, 255, 255), mask); // 修改为黄色
        cv::Mat output;
        cv::bitwise_and(rgb_image_(middle_region), rgb_image_(middle_region), output, mask);

        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        for (size_t i = 0; i < contours.size(); ++i) {
            cv::Rect rect = cv::boundingRect(contours[i]);
            cv::Mat traffic_light_region = rgb_image_(middle_region)(rect);
            std::string color = getTrafficLightColor(traffic_light_region);
            ROS_INFO("Detected traffic light color: %s", color.c_str());

            // 发布交通灯状态消息
            traffic_light_detector_pkg::TrafficLightState msg;
            msg.color = color;
            traffic_light_pub_.publish(msg);
        }
    }

    std::string getTrafficLightColor(const cv::Mat& region)
    {
        // int margin = 1; // 设置边缘裁剪的宽度
        // cv::Rect center_rect(margin, margin, region.cols - 2 * margin, region.rows - 2 * margin);
        // cv::Mat center_region = region(center_rect);
        // cv::Mat hsv;
        // cv::cvtColor(center_region, hsv, cv::COLOR_BGR2HSV);

        cv::Mat hsv;
        cv::cvtColor(region, hsv, cv::COLOR_BGR2HSV);

        cv::Mat red_mask1, red_mask2, yellow_mask, green_mask;
        cv::inRange(hsv, cv::Scalar(0, 50, 50), cv::Scalar(12, 255, 255), red_mask1);
        cv::inRange(hsv, cv::Scalar(168, 50, 50), cv::Scalar(180, 255, 255), red_mask2); // 增加红色的上限范围
        cv::Mat red_mask = red_mask1 | red_mask2; // 合并两个红色范围
        cv::inRange(hsv, cv::Scalar(18, 100, 100), cv::Scalar(32, 255, 255), yellow_mask);
        cv::inRange(hsv, cv::Scalar(30, 50, 50), cv::Scalar(90, 255, 255), green_mask);

        int red_count = cv::countNonZero(red_mask);
        int yellow_count = cv::countNonZero(yellow_mask);
        int green_count = cv::countNonZero(green_mask);

    //     if (red_count > yellow_count && red_count > green_count) {
    //         return "Red";
    //     } else if (yellow_count > red_count && yellow_count > green_count) {
    //         return "Yellow";
    //     } else if (green_count > red_count && green_count > yellow_count) {
    //         return "Green";
    //     } else {
    //         return "Unknown";
    //     }
    // }
        if (red_count > (green_count+2)) {
            return "Red";
        } else if (green_count > (red_count+2)) {
            return "Green";
        } else {
            return "Unknown";
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traffic_light_detector_node");
    TrafficLightDetector detector;
    ros::spin();
    return 0;
}
