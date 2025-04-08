//
// Created by smores on 4/8/25.
//

#include "rclcpp/rclcpp.hpp"
#include<iostream>
#include<opencv2/opencv.hpp>

#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "cv_bridge/cv_bridge.h"

using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;

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


    nvinfer1::ICudaEngine *engine;
    nvinfer1::IExecutionContext *context;
    void *left_buffer, *right_buffer, *disparity_buffer;
    std::vector<void *> bindings;

public:
    FoundationStereoDepthNode()
        : Node("foundation_stereo_depth") {
        rclcpp::QoS qos = rclcpp::QoS(10);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
        Logger logger;
        nvinfer1::IRuntime *runtime = nvinfer1::createInferRuntime(logger);
        engine = loadEngine("/external/foundation_stereo.engine", runtime);
        context = engine->createExecutionContext();
        cudaMalloc(&left_buffer, 640 * 512 * 3 * 4);
        cudaMalloc(&right_buffer, 640 * 512 * 3 * 4);
        cudaMalloc(&disparity_buffer, 640 * 512 * 4);
        RCLCPP_INFO(this->get_logger(), "Created engine and allocated device buffers");
        // string subscription_topic = "/thermal_" + cam + "/image";
        left_subscriber.subscribe(this, "/thermal_left/preprocd_image", qos.get_rmw_qos_profile());
        right_subscriber.subscribe(this, "/thermal_right/preprocd_image", qos.get_rmw_qos_profile());
        sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::
            ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> > >(
            message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                sensor_msgs::msg::Image>(50), left_subscriber, right_subscriber);
        // sync->setAgePenalty(1);
        sync->setMaxIntervalDuration(rclcpp::Duration(1, 0));
        sync->registerCallback(std::bind(&FoundationStereoDepthNode::stereoDepthCallback, this, _1, _2));
        disparity_publisher = this->create_publisher<sensor_msgs::msg::Image>("disparity", 10);
    }

    void stereoDepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left,
                             const sensor_msgs::msg::Image::ConstSharedPtr &right) {
        RCLCPP_INFO(this->get_logger(), "Received Image Pair");
        const cv_bridge::CvImagePtr cv_left = cv_bridge::toCvCopy(left, left->encoding);
        const cv_bridge::CvImagePtr cv_right = cv_bridge::toCvCopy(right, right->encoding);

        imageToFloatNCHW(cv_left->image, left_buffer);
        imageToFloatNCHW(cv_right->image, right_buffer);

        bindings = {left_buffer, right_buffer, disparity_buffer};
        context->executeV2(bindings.data());
        auto disparity = cv::Mat(512, 640, CV_32FC1);
        cudaMemcpy(disparity.data, disparity_buffer, 640 * 512 * 4, cudaMemcpyDeviceToHost);
        cv::Mat normalizedDisparity = disparity.clone();
        normalizedDisparity *= 255.0 / 155.0;
        cv::min(normalizedDisparity, 255.0, normalizedDisparity);
        cv::max(normalizedDisparity, 0.0, normalizedDisparity);
        normalizedDisparity.convertTo(normalizedDisparity, CV_8UC1);
        auto avg_time = (left->header.stamp.nanosec + right->header.stamp.nanosec) / 2;
        cv_bridge::CvImage disparity_message(left->header, "mono8", normalizedDisparity);
        sensor_msgs::msg::Image::SharedPtr disparityMessage = disparity_message.toImageMsg();
        disparity_publisher->publish(*disparityMessage);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FoundationStereoDepthNode>());
    rclcpp::shutdown();
    return 0;
}
