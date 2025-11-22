//
// Created by smores on 4/8/25.
//

#include "rclcpp/rclcpp.hpp"
#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/calib3d.hpp>
// #include <NvInferRuntime.h>
#include <fstream>
#include "cv_bridge/cv_bridge.h"
// #include "yaml-cpp/yaml.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;
#define BASELINE 0.24262f;
#define LP1 21000
#define LP99 21600


class PCLPub : public rclcpp::Node {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr disparity_subscriber;

    // bool started = false;
    // rclcpp::Time last_processed_time;
    // rclcpp::Duration min_dt = rclcpp::Duration::from_seconds(5);
    // void *left_buffer, *right_buffer, *disparity_buffer;

    cv::Mat K1, K2, D1, D2, R, T, R1, R2, P1, P2, Q, leftMapX, leftMapY, rightMapX, rightMapY;

public:
    PCLPub()
        : Node("pcl_pub") {
        rclcpp::QoS qos = rclcpp::QoS(10);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
        K1 = (cv::Mat_<double>(3, 3) <<
              405.57512495273403, 0.0, 313.2365778080362,
              0.0, 405.5592192044737, 237.85962939282086,
              0.0, 0.0, 1.0);

        K2 = (cv::Mat_<double>(3, 3) <<
              402.67129522383914, 0, 311.84481417550376,
              0, 402.4522895606074, 241.23817260805384,
              0.0, 0.0, 1.0);

        D1 = (cv::Mat_<double>(1, 4) << -0.3448173534502492,
              0.09834339989635991,
              0.0006913356388736054,
              -0.0001326767741732132);

        D2 = (cv::Mat_<double>(1, 4) << -0.3397900304621515,
              0.095616923755259,
              0.0011394239795359034,
              -0.0010376635277968291);

        R = (cv::Mat_<double>(3, 3) <<
             1, 0, 0,
             0, 1, 0,
             0, 0, 1);

        T = (cv::Mat_<double>(3, 1) << -0.24262, 0, 0);
        // RCLCPP_INFO(this->get_logger(), "StereoRectify");
        cv::stereoRectify(K1, D1, K2, D2, cv::Size(640, 512), R, T, R1, R2, P1, P2, Q);
        // RCLCPP_INFO(this->get_logger(), "StereoRectify");
        cv::initUndistortRectifyMap(K1, D1, R1, P1, cv::Size(640, 512), CV_32FC1, leftMapX, leftMapY);
        // RCLCPP_INFO(this->get_logger(), "StereoRectify");
        cv::initUndistortRectifyMap(K2, D2, R2, P2, cv::Size(640, 512), CV_32FC1, rightMapX, rightMapY);


        disparity_subscriber = this->create_subscription<sensor_msgs::msg::Image>("/disparity_synced", 10, std::bind(&PCLPub::disparity_callback, this, _1));
        points_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("foundation_points", 10);
        depth_publisher = this->create_publisher<sensor_msgs::msg::Image>("/depth_img", 10);
        disparity_publisher = this->create_publisher<sensor_msgs::msg::Image>("/disparity_viz", 10);
    }

    void disparity_callback(const sensor_msgs::msg::Image::ConstSharedPtr &disparity) {
        // cv::Mat cvDisparity;
        const cv_bridge::CvImagePtr cv_disparity = cv_bridge::toCvCopy(disparity, disparity->encoding);
        cv::Mat depthMap;
        cv::Mat points3D;
        depthMap.create(cv_disparity->image.rows, cv_disparity->image.cols, CV_32FC1);
        cv::reprojectImageTo3D(cv_disparity->image, points3D, Q, true); // handleMissingValues = true

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = disparity->header.stamp;
        cloud_msg.header.frame_id = "thermal_left_frame";
        cloud_msg.height = cv_disparity->image.rows;
        cloud_msg.width = cv_disparity->image.cols;
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(cv_disparity->image.rows * cv_disparity->image.cols);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (int v = 0; v < cv_disparity->image.rows; ++v) {
            for (int u = 0; u < cv_disparity->image.cols; ++u, ++iter_x, ++iter_y, ++iter_z) {
                cv::Vec3f point = points3D.at<cv::Vec3f>(v, u);
                *iter_x = point[0];
                *iter_y = point[1];
                *iter_z = point[2];
            }
        }

        RCLCPP_INFO(this->get_logger(), "The disparity image has coi = %d and size = %dx%d", points3D.channels(), points3D.cols, points3D.rows);
        cv::extractChannel(points3D, depthMap, 2);
        cv_bridge::CvImage depthImageBridge = cv_bridge::CvImage(disparity->header, "", depthMap);

        cv::Mat normalizedDisparity = cv_disparity->image.clone();
        normalizedDisparity *= 255.0 / 155.0;
        cv::min(normalizedDisparity, 255.0, normalizedDisparity);
        cv::max(normalizedDisparity, 0.0, normalizedDisparity);
        normalizedDisparity.convertTo(normalizedDisparity, CV_8UC1);
        cv_bridge::CvImage disparity_message(disparity->header, "mono8", normalizedDisparity);
        sensor_msgs::msg::Image::SharedPtr disparityMessage = disparity_message.toImageMsg();
        disparityMessage->header.stamp = disparity->header.stamp;
        
        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing");
        disparity_publisher->publish(*disparityMessage);
        points_publisher->publish(cloud_msg);
        depth_publisher->publish(*depthImageBridge.toImageMsg());
        // cv::imshow("Disparity", normalizedDisparity);
        // cv::waitKey(0);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCLPub>());
    rclcpp::shutdown();
    return 0;
}
