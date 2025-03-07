#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.h>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d.hpp"
#include "yaml-cpp/yaml.h"

#define UNUSED(x) (void)x;

using std::placeholders::_1;
using namespace std;
using namespace cv;
using namespace YAML;


/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
void preprocess(const Mat &image, Mat &output, double_t fraction) {
  image.convertTo(output, CV_64F);
  size_t numElements = output.total();
  vector<double_t> elements(output.begin<double_t>(), output.end<double_t>());
  // copy(, elements.begin());
  std::sort(elements.begin(), elements.end());
  double_t hist_min = elements[numElements * fraction];
  double_t hist_max = elements[numElements * (1 - fraction)];
  output = (output - hist_min) / (hist_max - hist_min);
  threshold(output, output, 1, 1, THRESH_TRUNC);
  threshold(output, output, 0, 0, THRESH_TOZERO);
  // output = output * 255;
  output *= 255;
  output.convertTo(output, CV_8U);
  equalizeHist(output, output);
}

/**
 * @param source: input image
 * @param output: output image
 * @param fraction: percentile of pixels to be removed from the histogram
 * @param cameraIntrinsics: camera intrinsics matrix loaded from YAML file given in the launch file
 * @param cameraDistCoeffs: camera distortion coefficients loaded from YAML file given in the launch file
 */
void fastPreprocess(const Mat& source, Mat& output, double_t fraction, cv::Mat& cameraIntrinsics, cv::Mat& cameraDistCoeffs){
  
  UNUSED(cameraIntrinsics);
  UNUSED(cameraDistCoeffs);

  float range[] = {0, 65536};
  int histSize = 16384;
  const float* histRange[] = { range };
  Mat hist;
  calcHist(&source, 1, 0, Mat(), hist, 1, &histSize, histRange, true, false);
  int total_elements = source.total();
  int lb = total_elements * fraction;
  int hb = total_elements * (1.-fraction);

  // cout<<lb<<" "<<hb<<endl;
  int lv = 0, hv = 0;
  bool lvfound = false, hvfound = false;
  int accum = 0;
  for(int i = 0; i < histSize; i++){
    accum += hist.at<float>(i);
    // cout<<accum<<" ";
    if(!lvfound && accum > lb){
      lv = i;
      lvfound = true;
    }
    if(!hvfound && accum > hb){
      hv = i;
      hvfound = true;
    }
  }
  // cout<<endl;
  int lpixelval = lv * (65536 / histSize);
  int hpixelval = hv * (65536 / histSize);
  // cout<<lpixelval<<" "<<hpixelval<<" "<<hpixelval - lpixelval<<endl;
  source.convertTo(output, CV_32F);

  output -= lpixelval;
  threshold(output, output, hpixelval-lpixelval, hpixelval-lpixelval, THRESH_TRUNC);
  output *= (256.0/(hpixelval - lpixelval));
  output.convertTo(output, CV_8U);
  // cv::undistort(temp, output, cameraIntrinsics, cameraDistCoeffs);
}

class ThermalPreprocessingNode : public rclcpp::Node
{
  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv::Mat cameraDistCoeffs;
    cv::Mat cameraIntrinsics;

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg){
      // std::cout<<"Hello, world!!"<<std::endl;

      cv_bridge::CvImagePtr cv_ptr;
        try
        {   
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            // std::cout<<"Hello, World inside cv_ptr"<<std::endl;
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        Mat betterImage;
        // preprocess(cv_ptr->image, betterImage, 0.1);
        fastPreprocess(cv_ptr->image, betterImage, 0.01, this->cameraIntrinsics, this->cameraDistCoeffs);
        // cout<<msg->header.stamp.sec<<endl;
        cv_bridge::CvImage preprocessed_image_message(msg->header, "mono8", betterImage);
        sensor_msgs::msg::Image::SharedPtr imageMessage = preprocessed_image_message.toImageMsg();
        publisher_->publish(*imageMessage);
    }

    void getCamIntrinsicsFromYaml(const YAML::Node camIntrinsicsYaml, cv::Mat& cameraIntrinsics){
      const auto& intrinsics = camIntrinsicsYaml["cam0"]["intrinsics"];

      float fx = intrinsics[0].as<float>();
      float fy = intrinsics[1].as<float>();
      float cx = intrinsics[2].as<float>();
      float cy = intrinsics[3].as<float>();

      cameraIntrinsics = (cv::Mat_<float>(3, 3) << fx,  0, cx,
                                                    0, fy, cy,
                                                    0,  0,  1);

    }

    void getCamDistortionCoeffs(const YAML::Node camIntrinsicsYaml, cv::Mat& cameraDistCoeffs){
      const auto& dist_coeffs = camIntrinsicsYaml["cam0"]["distortion_coeffs"];

      float k1 = dist_coeffs[0].as<float>();
      float k2 = dist_coeffs[1].as<float>();
      float r1 = dist_coeffs[2].as<float>();
      float r2 = dist_coeffs[3].as<float>();

      cameraDistCoeffs = (cv::Mat_<float>(1, 4) << k1, k2, r1, r2);
    }

  public:
    ThermalPreprocessingNode()
    : Node("thermal_preprocessing"){

      this->declare_parameter<std::string>("cam", "");
      this->declare_parameter<std::string>("camIntrinsicsFile", "");
      
      std::string cam = this->get_parameter("cam").as_string();
      std::string camIntrinsicsFile = this->get_parameter("camIntrinsicsFile").as_string();
        
        if (camIntrinsicsFile.empty()) {
              RCLCPP_ERROR(this->get_logger(), "No camera intrinsics file provided!");
            return;
        }
        
      string subscription_topic = "/thermal_" + cam + "/image";
      string publisher_topic = "/thermal_" + cam + "/preprocd_image";
      
      RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", subscription_topic.c_str());
      RCLCPP_INFO(this->get_logger(), "Publishing to: %s", publisher_topic.c_str());

      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subscription_topic, 10, std::bind(&ThermalPreprocessingNode::image_callback, this, _1));
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>(publisher_topic, 10);

      RCLCPP_INFO(this->get_logger(), "Loading YAML: %s", camIntrinsicsFile.c_str());
      YAML::Node camIntrinsicsYaml = YAML::LoadFile(camIntrinsicsFile);

      getCamIntrinsicsFromYaml(camIntrinsicsYaml, cameraIntrinsics);
      getCamDistortionCoeffs(camIntrinsicsYaml, cameraDistCoeffs);

    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalPreprocessingNode>());
  rclcpp::shutdown();
  return 0;
}