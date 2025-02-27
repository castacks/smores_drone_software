#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.h>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc.hpp"
// #include "opencv2/highgui.hpp"
using std::placeholders::_1;
using namespace std;
using namespace cv;
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

void fastPreprocess(const Mat& source, Mat& output, double_t fraction){
  float range[] = {0, 65536};
  int histSize = 16384;
  const float* histRange[] = { range };
  Mat hist;
  calcHist(&source, 1, 0, Mat(), hist, 1, &histSize, histRange, true, false);
  int total_elements = source.total();
  int lb = total_elements * fraction;
  int hb = total_elements * (1.-fraction);

  // cout<<lb<<" "<<hb<<endl;
  int lv, hv = 0;
  bool lvfound, hvfound = false;
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

  /** TODO 1. Install yaml-cpp and import yaml.h
      TODO 2. Have different subscribers for left and right image
      TODO 3. Load Yaml scripts and read necessary values
      TODO 4. call cv2.undistort
   */
  //  cv::Mat cameraIntMat = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  //  cv::Mat distCoeffs = (cv::Mat1d(1, 5) << k1, k2, p1, p2, k3);
  //  cv::undistort(output, output, cameraIntMat, distCoeffs, newCamMat)
}
class ThermalPreprocessingNode : public rclcpp::Node
{
  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
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
        fastPreprocess(cv_ptr->image, betterImage, 0.01);
        // cout<<msg->header.stamp.sec<<endl;
        cv_bridge::CvImage preprocessed_image_message(msg->header, "mono8", betterImage);
        sensor_msgs::msg::Image::SharedPtr imageMessage = preprocessed_image_message.toImageMsg();
        publisher_->publish(*imageMessage);
    }

  public:
    ThermalPreprocessingNode()
    : Node("thermal_preprocessing"){
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>("thermal_raw", 10, std::bind(&ThermalPreprocessingNode::image_callback, this, _1));
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("preprocessed", 10);
      // cv::namedWindow("ImageDisplay");
    }

  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalPreprocessingNode>());
  rclcpp::shutdown();
  return 0;
}