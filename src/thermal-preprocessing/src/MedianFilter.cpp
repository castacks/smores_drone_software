//
// Created by ranai on 9/9/25.
//

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"

class MedianFilterNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    cv_bridge::CvImagePtr ros2cv;

    void medianFilter(cv::Mat& dst, cv::Mat src, int k)
    {
        cv::medianBlur(src, dst, k);
    }


    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg){
        cv::Mat pubImg;
        auto cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        medianFilter(pubImg, cv_ptr->image, 5);
        sensor_msgs::msg::Image::SharedPtr pubMsg = cv_ptr->toImageMsg();
        image_pub->publish(*pubMsg);
    }

public:
    MedianFilterNode():Node("median_filter_node"){

        image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/thermal_left/image", 10, std::bind(&MedianFilterNode::image_callback, this, std::placeholders::_1));
        image_pub = this->create_publisher<sensor_msgs::msg::Image>("/thermal_left/medianed", 1);
    }
};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MedianFilterNode>());
    rclcpp::shutdown();
    return 0;
}