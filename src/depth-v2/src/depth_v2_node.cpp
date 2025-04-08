//
// Created by smores on 4/8/25.
//

#include "rclcpp/rclcpp.hpp"
#include<iostream>
// #include <opencv4/opencv2/opencv.hpp>
#include<opencv2/opencv.hpp>
// #include <opencv4/opencv2/cudaimgproc.hpp>
// #include <opencv4/opencv2/cudawarping.hpp>
#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>
#include <fstream>
// Function to preprocess the image
void preprocessImage(const cv::Mat& input_image, float* gpu_input, const nvinfer1::Dims& dims) {
    // Ensure the input image is in the correct format
    cv::Mat resized_image;
    cv::resize(input_image, resized_image, cv::Size(dims.d[2], dims.d[1]));

    // Convert to float and normalize
    cv::Mat float_image;
    resized_image.convertTo(float_image, CV_32FC3, 1.0 / 255);

    // Mean normalization (example values, use the mean used during training)
    cv::Scalar mean(0.485, 0.456, 0.406);
    cv::Scalar std(0.229, 0.224, 0.225);
    cv::subtract(float_image, mean, float_image);
    cv::divide(float_image, std, float_image);

    // Convert HWC to CHW
    std::vector<cv::Mat> chw_planes(3);
    cv::split(float_image, chw_planes);
    for (int i = 0; i < 3; ++i) {
        cudaMemcpy(gpu_input + i * dims.d[1] * dims.d[2], chw_planes[i].data,
                   dims.d[1] * dims.d[2] * sizeof(float), cudaMemcpyHostToDevice);
    }
}
int main(int argc, char* argv){
    std::cout<<"Hello, World!\n";
}