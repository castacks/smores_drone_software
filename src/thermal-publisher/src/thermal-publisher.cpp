#include "rclcpp/rclcpp.hpp"
#include <asm/types.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "rawBoson.h"

class ThermalPublisher : public rclcpp::Node {
private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  std::string devicePath;

public:
  ThermalPublisher() : Node("thermal_publisher") {
    // std::vector<std::string> serialList = {"flir_boson_serial_34564", "flir_boson_serial_69703"};
    // std::vector<int> syncList = {0, 0};
    // int count = 0;
    // for (std::string serialPort: serialList) {
    //   char serialPortRoot[8192];
    //   char *serialResult = realpath(("/dev/" + serialPort).c_str(), serialPortRoot);
    //   set_sync_mode(syncList[count++], serialPortRoot);
    //   int returnedSyncMode = get_sync_mode(serialPortRoot);
    //   std::cout << "return sync mode: " << returnedSyncMode << std::endl;
    //   switch (returnedSyncMode) {
    //     case 0:
    //       RCLCPP_INFO(this->get_logger(), " !!!!!!!!!!!!!!!returns mode setting result: disabled.");
    //       break;
    //
    //     case 1:
    //       std::cout << serialPort << " !!!!!!!!!!!!!!!!returns mode setting result: master." << std::endl;
    //       break;
    //
    //     case 2:
    //       std::cout << serialPort << " !!!!!!!!!!!!!!!!returns mode setting result: slave." << std::endl;
    //       break;
    //   }
    // }

    this->declare_parameter("device_path", "/dev/flir_boson_video_34564");
    devicePath = this->get_parameter("device_path").as_string();
    std::string serialPort = "flir_boson_serial_" + devicePath.substr(devicePath.length() - 5, 5);
    RCLCPP_INFO(this->get_logger(), "Serial Path: %s", serialPort.c_str());
    char serialPortRoot[8192];
    char *serialResult = realpath(("/dev/" + serialPort).c_str(), serialPortRoot);
    set_sync_mode(0, serialPortRoot);
    int returnedSyncMode = get_sync_mode(serialPortRoot);
    RCLCPP_INFO(this->get_logger(), "Serial Path: %s, returned sync mode: %d", serialPort.c_str(), returnedSyncMode);
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("thermal_image", 10);

    auto fd = open(devicePath.c_str(), O_RDWR);
    if (fd < 0) {
      RCLCPP_FATAL(this->get_logger(), "Could not open device %s", devicePath.c_str());
      rclcpp::shutdown();
    } else {
      RCLCPP_INFO(this->get_logger(), "Opened Device %s", devicePath.c_str());
      // rclcpp::shutdown();
    }

    struct v4l2_capability cap = {};
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to query capabilities for %s, Error: %s", devicePath.c_str(),
                   strerror(errno));
      rclcpp::shutdown();
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
      RCLCPP_FATAL(this->get_logger(), "Device does not support Video Capture %s", devicePath.c_str());
      rclcpp::shutdown();
    }

    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 640; // Set your desired width
    fmt.fmt.pix.height = 512; // Set your desired height
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16; // 16-bit grayscale
    // fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
      RCLCPP_FATAL(this->get_logger(), "Could not set format for %s, error: %s", devicePath.c_str(), strerror(errno));
      rclcpp::shutdown();
    }
    RCLCPP_DEBUG(this->get_logger(), "Opened and set-up camera %s", devicePath.c_str());

    struct v4l2_requestbuffers req = {};
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
      RCLCPP_FATAL(this->get_logger(), "Could not set request buffers for %s, error: %s", devicePath.c_str(),
                   strerror(errno));
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "Requested Buffers");
    struct buffer {
      void *start;
      size_t length;
    } *buffers;

    buffers = static_cast<buffer *>(calloc(req.count, sizeof(*buffers)));

    for (int i = 0; i < req.count; ++i) {
      struct v4l2_buffer buf = {};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;

      if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
        RCLCPP_FATAL(this->get_logger(), "Could not query buffer for %s, error: %s", devicePath.c_str(),
                     strerror(errno));
        rclcpp::shutdown();
      }

      buffers[i].length = buf.length;
      buffers[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

      if (buffers[i].start == MAP_FAILED) {
        RCLCPP_FATAL(this->get_logger(), "Could not map buffer for %s, error: %s", devicePath.c_str(),
                     strerror(errno));
        rclcpp::shutdown();
      }
    }

    for (int i = 0; i < req.count; ++i) {
      struct v4l2_buffer buf = {0};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;

      if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
        RCLCPP_FATAL(this->get_logger(), "Could not queue buffer for %s, error: %s", devicePath.c_str(),
                     strerror(errno));
        rclcpp::shutdown();
      }
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
      RCLCPP_FATAL(this->get_logger(), "Could not start streamin %s, error: %s", devicePath.c_str(), strerror(errno));
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "Started Streaming");
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    double prev_v4l2_time = 0;
    double prev_os_time = 0;
    while (true) {
      if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
        perror("Failed to dequeue buffer");
        break;
      }
      auto osTime = this->now();
      timeval v4l2Time = buf.timestamp;
      double v4l2Seconds = static_cast<double>(v4l2Time.tv_sec) + static_cast<double>(v4l2Time.tv_usec) * 0.000001;
      double v4l2_td = v4l2Seconds - prev_v4l2_time;
      double os_td = osTime.seconds() - prev_os_time;
      // RCLCPP_INFO(this->get_logger(),
      //            "OS Time: %f (%f), v4l2 Time: %f (%f), sequence number: %d, timecode.frames: %d", osTime.seconds(),
      //            os_td,
      //            v4l2Seconds, v4l2_td, buf.sequence, buf.timecode.frames);
      prev_os_time = osTime.seconds();
      prev_v4l2_time = v4l2Seconds;
      sensor_msgs::msg::Image imgMessage;
      imgMessage.encoding = sensor_msgs::image_encodings::MONO16;
      imgMessage.width = 640;
      imgMessage.height = 512;
      imgMessage.step = 640 * 2;
      imgMessage.is_bigendian = false;
      imgMessage.data.insert(imgMessage.data.begin(), static_cast<char *>(buffers[buf.index].start),
                             static_cast<char *>(buffers[buf.index].start) + buf.bytesused);
      imgMessage.header.stamp = this->now();
      this->image_publisher_->publish(imgMessage);
      // RCLCPP_INFO(this->get_logger(), "Received a buffer of size %d", buf.bytesused);

      if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
        perror("Failed to requeue buffer");
        break;
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ThermalPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
