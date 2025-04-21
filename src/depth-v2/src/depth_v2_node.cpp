//
// Created by smores on 4/8/25.
//

#include "rclcpp/rclcpp.hpp"
#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/calib3d.hpp>
#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "cv_bridge/cv_bridge.h"
#include "yaml-cpp/yaml.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;
#define BASELINE 0.24262f;
#define LP1 21000
#define LP99 21600

class Logger : public nvinfer1::ILogger {
public:
    // Override the log method to handle TensorRT messages
    void log(Severity severity, const char *msg) noexcept override {
        // Filter out info-level messages; adjust the severity threshold as needed
        if (severity <= Severity::kWARNING) {
            std::cout << "[TensorRT] " << msg << std::endl;
        }
    }
};

void imageToFloatNCHW(cv::Mat inputImage, void *destinationBuffer) {
    cv::Mat floatImage;
    inputImage.convertTo(floatImage, CV_32FC1);
    for (int i = 0; i < 3; ++i) {
        auto status = cudaMemcpy(static_cast<char *>(destinationBuffer) + i * 640 * 512 * sizeof(float32_t),
                                 floatImage.data,
                                 640 * 512 * sizeof(float32_t), cudaMemcpyHostToDevice);
        if (status != cudaSuccess) {
            std::cout << "FAIL IN CHW\n";
        }
    }
}

nvinfer1::ICudaEngine *loadEngine(const std::string &engine_file, nvinfer1::IRuntime *runtime) {
    std::ifstream file(engine_file, std::ios::binary);
    if (!file) {
        std::cerr << "Error opening engine file: " << engine_file << std::endl;
        return nullptr;
    }
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> engine_data(size);
    file.read(engine_data.data(), size);
    return runtime->deserializeCudaEngine(engine_data.data(), size);
}

class FoundationStereoDepthNode : public rclcpp::Node {
    message_filters::Subscriber<sensor_msgs::msg::Image> left_subscriber;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_subscriber;

    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image> > > sync;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher;


    nvinfer1::ICudaEngine *engine;
    nvinfer1::IExecutionContext *context;
    void *left_buffer, *right_buffer, *disparity_buffer;
    std::vector<void *> bindings;

    // void getCamIntrinsicsFromYaml(const YAML::Node camIntrinsicsYaml, cv::Mat &cameraIntrinsics) {
    //     const auto &intrinsics = camIntrinsicsYaml["cam0"]["intrinsics"];
    //
    //     float fx = intrinsics[0].as<float>();
    //     float fy = intrinsics[1].as<float>();
    //     float cx = intrinsics[2].as<float>();
    //     float cy = intrinsics[3].as<float>();
    //
    //     cameraIntrinsics = (cv::Mat_<float>(3, 3) << fx, 0, cx,
    //                         0, fy, cy,
    //                         0, 0, 1);
    // }
    //
    // void getCamDistortionCoeffs(const YAML::Node camIntrinsicsYaml, cv::Mat &cameraDistCoeffs) {
    //     const auto &dist_coeffs = camIntrinsicsYaml["cam0"]["distortion_coeffs"];
    //
    //     float k1 = dist_coeffs[0].as<float>();
    //     float k2 = dist_coeffs[1].as<float>();
    //     float r1 = dist_coeffs[2].as<float>();
    //     float r2 = dist_coeffs[3].as<float>();
    //
    //     cameraDistCoeffs = (cv::Mat_<float>(1, 4) << k1, k2, r1, r2);
    // }

    cv::Mat K1, K2, D1, D2, R, T, R1, R2, P1, P2, Q, leftMapX, leftMapY, rightMapX, rightMapY;

    // void getDepth(cv::Mat &disparity) {
    //     cv::Mat depth;
    //     cv::divide(cv::Mat(), disparity, depth, cameraIntrinsics.at<float>(0, 0) * 0.24);
    // }

public:
    FoundationStereoDepthNode()
        : Node("foundation_stereo_depth") {
        rclcpp::QoS qos = rclcpp::QoS(10);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
        Logger logger;
        // this->declare_parameter<std::string>("left_cam_name", "");
        // this->declare_parameter<std::string>("right_cam_name", "");
        // this->declare_parameter<std::string>("left_cam_intrinsics_file", "");
        // this->declare_parameter<std::string>("right_cam_intrinsics_file", "");
        // std::string left_cam_intrinsics_file = this->get_parameter("left_cam_intrinsics_file").as_string();
        // std::string right_cam_intrinsics_file = this->get_parameter("right_cam_intrinsics_file").as_string();
        // if (left_cam_intrinsics_file.empty() || right_cam_intrinsics_file.empty()) {
        // RCLCPP_ERROR(this->get_logger(), "camera intrinsics file not provided!");
        // return;
        // }
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


        nvinfer1::IRuntime *runtime = nvinfer1::createInferRuntime(logger);
        engine = loadEngine("/external/foundation_stereo.engine", runtime);
        context = engine->createExecutionContext();
        cudaMalloc(&left_buffer, 640 * 512 * 3 * 4);
        cudaMalloc(&right_buffer, 640 * 512 * 3 * 4);
        cudaMalloc(&disparity_buffer, 640 * 512 * 4);
        RCLCPP_INFO(this->get_logger(), "Created engine and allocated device buffers");
        // string subscription_topic = "/thermal_" + cam + "/image";
        left_subscriber.subscribe(this, "/thermal_left/image", qos.get_rmw_qos_profile());
        right_subscriber.subscribe(this, "/thermal_right/image", qos.get_rmw_qos_profile());
        sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::
            ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> > >(
            message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                sensor_msgs::msg::Image>(10), left_subscriber, right_subscriber);
        // sync->setAgePenalty(1);
        sync->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
        sync->registerCallback(std::bind(&FoundationStereoDepthNode::stereoDepthCallback, this, _1, _2));
        disparity_publisher = this->create_publisher<sensor_msgs::msg::Image>("disparity", 10);
        points_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("foundation_points", 10);
    }

    void stereoDepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left,
                             const sensor_msgs::msg::Image::ConstSharedPtr &right) {
        RCLCPP_INFO(this->get_logger(), "Received Image Pair");
        const cv_bridge::CvImagePtr cv_left = cv_bridge::toCvCopy(left, left->encoding);
        const cv_bridge::CvImagePtr cv_right = cv_bridge::toCvCopy(right, right->encoding);
        // RCLCPP_INFO_STREAM(this->get_logger(), left->encoding);


        cv::Mat rectifiedLeft;
        cv::remap(cv_left->image, rectifiedLeft, leftMapX, leftMapY, cv::INTER_LINEAR);
        cv::Mat rectifiedLeftF32;
        rectifiedLeft.convertTo(rectifiedLeftF32, CV_32F);
        cv::Mat sortedLeft;
        cv::sort(rectifiedLeftF32.reshape(1, 1), sortedLeft, cv::SORT_EVERY_ROW + cv::SORT_ASCENDING);
        float32_t lp1Left = sortedLeft.at<float32_t>(int(0.01 * sortedLeft.cols));
        float32_t lp99Left = sortedLeft.at<float32_t>(int(0.99 * sortedLeft.cols));
        rectifiedLeftF32 -= lp1Left;
        cv::threshold(rectifiedLeftF32, rectifiedLeftF32, lp99Left - lp1Left, lp99Left - lp1Left, cv::THRESH_TRUNC);
        rectifiedLeftF32 *= (255.0 / (lp99Left - lp1Left));
        cv::Mat leftVisualImage;
        rectifiedLeftF32.convertTo(leftVisualImage, CV_8U);
        // cv::imshow("Rectified Left", leftVisualImage);

        cv::Mat rectifiedRight;
        cv::remap(cv_right->image, rectifiedRight, rightMapX, rightMapY, cv::INTER_LINEAR);
        cv::Mat rectifiedRightF32;
        rectifiedRight.convertTo(rectifiedRightF32, CV_32F);
        cv::Mat sortedRight;
        cv::sort(rectifiedRightF32.reshape(1, 1), sortedRight, cv::SORT_EVERY_ROW + cv::SORT_ASCENDING);
        float32_t lp1Right = sortedRight.at<float32_t>(int(0.01 * sortedRight.cols));
        float32_t lp99Right = sortedRight.at<float32_t>(int(0.99 * sortedRight.cols));
        rectifiedRightF32 -= lp1Right;
        cv::threshold(rectifiedRightF32, rectifiedRightF32, lp99Right - lp1Right, lp99Right - lp1Right,
                      cv::THRESH_TRUNC);
        rectifiedRightF32 *= (255.0 / (lp99Right - lp1Right));
        cv::Mat rightVisualImage;
        rectifiedRightF32.convertTo(rightVisualImage, CV_8U);
        // cv::imshow("Rectified Right", rightVisualImage);

        imageToFloatNCHW(rectifiedLeftF32, left_buffer);
        imageToFloatNCHW(rectifiedRightF32, right_buffer);

        bindings = {left_buffer, right_buffer, disparity_buffer};
        context->executeV2(bindings.data());
        auto disparity = cv::Mat(512, 640, CV_32FC1);
        cudaMemcpy(disparity.data, disparity_buffer, 640 * 512 * 4, cudaMemcpyDeviceToHost);
        cv::Mat points3D;
        cv::reprojectImageTo3D(disparity, points3D, Q, true); // handleMissingValues = true

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = left->header.stamp;
        cloud_msg.header.frame_id = "thermal_left_frame";
        cloud_msg.height = disparity.rows;
        cloud_msg.width = disparity.cols;
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(disparity.rows * disparity.cols);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (int v = 0; v < disparity.rows; ++v) {
            for (int u = 0; u < disparity.cols; ++u, ++iter_x, ++iter_y, ++iter_z) {
                cv::Vec3f point = points3D.at<cv::Vec3f>(v, u);
                *iter_x = point[0];
                *iter_y = point[1];
                *iter_z = point[2];
            }
        }

        cv::Mat normalizedDisparity = disparity.clone();
        normalizedDisparity *= 255.0 / 155.0;
        cv::min(normalizedDisparity, 255.0, normalizedDisparity);
        cv::max(normalizedDisparity, 0.0, normalizedDisparity);
        normalizedDisparity.convertTo(normalizedDisparity, CV_8UC1);
        // auto avg_time = (left->header.stamp.nanosec + right->header.stamp.nanosec) / 2;
        cv_bridge::CvImage disparity_message(left->header, "mono8", normalizedDisparity);
        sensor_msgs::msg::Image::SharedPtr disparityMessage = disparity_message.toImageMsg();
        disparityMessage->header.stamp = left->header.stamp;
        // disparity = K1[0, 0] * 0.24262 / disp
        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing");
        disparity_publisher->publish(*disparityMessage);
        points_publisher->publish(cloud_msg);
        // cv::imshow("Disparity", normalizedDisparity);
        // cv::waitKey(0);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FoundationStereoDepthNode>());
    rclcpp::shutdown();
    return 0;
}
