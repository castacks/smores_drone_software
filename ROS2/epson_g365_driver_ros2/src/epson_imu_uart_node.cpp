//==============================================================================
//
// 	epson_imu_uart_node.cpp
//     - ROS node for Epson IMU sensor evaluation
//     - This program initializes the Epson IMU and publishes ROS messages in
//       ROS topic /epson_imu as convention per [REP 145]
//       (http://www.ros.org/reps/rep-0145.html).
//     - If the IMU model supports quaternion output (currently supported only
//     by
//       G330/G365/G366) orientation fields are updated in published topic
//       /epson_imu/data
//     - If the IMU model does not support quaternion output (currently
//       G320/G354/G364/G370/V340) orientation fields do not update in published
//       topic /epson_imu/data_raw
//
//  [This software is BSD-3
//  licensed.](http://opensource.org/licenses/BSD-3-Clause)
//
//  Original Code Development:
//  Copyright (c) 2019, Carnegie Mellon University. All rights reserved.
//
//  Additional Code contributed:
//  Copyright (c) 2019, 2023, Seiko Epson Corp. All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//  THE POSSIBILITY OF SUCH DAMAGE.
//
//==============================================================================

#include <chrono>
#include <memory>

#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <termios.h>
#include <iostream>
#include <string>

#include "hcl.h"
#include "hcl_gpio.h"
#include "hcl_uart.h"
#include "sensor_epsonCommon.h"

//=========================================================================
// Timestamp Correction
//
// This assumes that the ROS time is sync'ed to external 1PPS pulse sent to
// Epson IMU GPIO2_EXT pin and the IMU External Reset Counter function is
// enabled and ROS latency of calling rclcpp::Clock().now() is
// less than 0.020 seconds. Otherwise the reset counter may overflow and the
// timestamp correction will not be reliable. The IMU reset count returned
// is already converted to nsecs units
//=========================================================================

class TimeCorrection {
 private:
#if defined G320PDG0 || defined G354PDH0 || defined G364PDC0 || \
    defined G364PDCA || defined V340PDD0
  // Counter freq = 46875Hz, Max Count = 65535/46875 * 1e9
  const int64_t MAX_COUNT = 1398080000;
  const int64_t ALMOST_ROLLOVER = 1340000000;
#else
  // Counter freq = 62500Hz, Max Count = 65535/62500 * 1e9
  const int64_t MAX_COUNT = 1048560000;
  const int64_t ALMOST_ROLLOVER = 1010000000;
#endif
  const int64_t ONE_SEC_NSEC = 1000000000;
  const int64_t HALF_SEC_NSEC = 500000000;

  int64_t count_corrected;
  int64_t count_corrected_old;
  int64_t count_old;
  int64_t count_diff;
  int32_t time_sec_current;
  int32_t time_sec_old;
  int64_t time_nsec_current;
  bool rollover;
  bool flag_imu_lead;

 public:
  TimeCorrection();
  rclcpp::Time get_stamp(int);
};

TimeCorrection::TimeCorrection() {
  count_corrected = 0;
  count_old = 0;
  count_diff = 0;
  time_sec_current = 0;
  time_sec_old = 0;
  time_nsec_current = 0;
  count_corrected_old = 0;
  rollover = false;
  flag_imu_lead = false;
}

//=========================================================================
// TimeCorrection::get_stamp
//
// Returns the timestamp based on time offset from most recent 1PPS signal.
// Epson IMU has a free-running upcounter that resets on active 1PPS signal.
// Counter value is embedded in the sensor data at the time of sampling.
// Time stamp is corrected based on reset counter retrieved from embedded
// sensor data.
//=========================================================================

rclcpp::Time TimeCorrection::get_stamp(int count) {
  rclcpp::Time now = rclcpp::Clock().now();
  time_sec_current = now.seconds();
  time_nsec_current = now.nanoseconds();
  // std::cout.precision(20);

  count_diff = count - count_old;
  if (count > ALMOST_ROLLOVER) {
    rollover = true;
  }
  if (count_diff < 0) {
    if (rollover) {
      count_diff = count + (MAX_COUNT - count_old);
      std::cout << "Warning: time_correction enabled but IMU reset counter "
                   "rollover detected. If 1PPS not connected to IMU GPIO2/EXT "
                   "pin, disable time_correction."
                << std::endl;
    } else {
      count_diff = count;
      count_corrected = 0;
    }
    rollover = 0;
  }
  count_corrected = (count_corrected + count_diff) % ONE_SEC_NSEC;
  if ((time_sec_current != time_sec_old) && (count_corrected > HALF_SEC_NSEC)) {
    time_sec_current = time_sec_current - 1;
  } else if (((count_corrected - count_corrected_old) < 0) &&
             (time_nsec_current > HALF_SEC_NSEC)) {
    time_sec_current = time_sec_current + 1;
    flag_imu_lead = 1;
  } else if (flag_imu_lead && (time_nsec_current > HALF_SEC_NSEC)) {
    time_sec_current = time_sec_current + 1;
  } else {
    flag_imu_lead = 0;
  }
  rclcpp::Time ros_time = rclcpp::Time(time_sec_current, count_corrected);
  time_sec_old = time_sec_current;
  count_old = count;
  count_corrected_old = count_corrected;

  return ros_time;
}

// Global Serial Port File Handle
int fd_serial;

//=========================================================================
// Epson IMU ROS2 C++ Node
//
// 1. Retrieves node parameters from launch file otherwise uses defaults.
// 2. Creates a publisher for IMU messages
// 3. Sets up communication and initializes IMU
// 4. Starts an internal timer to periodically check for incoming IMU burst data
// 5. Formats and publishes IMU messages until <CTRL-C>
//=========================================================================

using namespace std;
using namespace std::chrono_literals;

class ImuNode : public rclcpp::Node {
 public:
  explicit ImuNode(const rclcpp::NodeOptions &op) : Node("epson_node", op) {
    ParseParams();

    // publisher
    imu_data_pub_ =
        this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_.c_str(), 10);

    Init();
    // Poll to check for IMU burst read data @ 4000Hz
    // (atleast 2x the highest output rate 2000Hz)
    std::chrono::milliseconds ms((int)(1000.0 / poll_rate_));
    timer_ = this->create_wall_timer(ms, std::bind(&ImuNode::Spin, this));
  }

  ~ImuNode() {
    sensorStop();
    uartRelease(fd_serial);
    gpioRelease();
    seRelease();
    RCLCPP_INFO(this->get_logger(), "Cleanup and shutown completed.");
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub_;

  // IMU configuration settings
  struct EpsonOptions options_ = {};

  // Buffer for scaled values of IMU read burst
  struct EpsonData epson_data_ = {};

  std::string port_;
  std::string frame_id_;
  std::string imu_topic_;
  double poll_rate_;

  // time_correction function requires 1PPS connection to IMU GPIO2_EXT pin
  // cannot be used with ext_trigger at the same time
  bool time_correction_;

  std::string prod_id_;
  std::string serial_id_;

  TimeCorrection tc;

  // NOTE: It is recommended to change IMU settings by launch files instead of
  //       modifying directly here in the source code
  void ParseParams() {
    std::string key;
    port_ = "/dev/ttyUSB0";
    frame_id_ = "imu_link";
    imu_topic_ = "/epson_imu/data_raw";
    poll_rate_ = 4000.0;
    time_correction_ = false;
    // ext_trigger function requires external trigger signal to IMU GPIO2_EXT
    // pin cannot be used with time_correction at the same time
    bool ext_trigger_en_ = false;
    bool output_32bit_ = false;

    // Initialize defaults of IMU settings
    options_.ext_sel =
        0;  // 0 = Sample Counter 1=Reset Counter 2=External Trigger
    options_.drdy_on = true;
    options_.drdy_pol = 1;  // 1 = active HIGH 0=active LOW
    options_.dout_rate = CMD_RATE125;
    options_.filter_sel = CMD_FLTAP32;
    options_.flag_out = true;
    options_.temp_out = true;
    options_.gyro_out = true;
    options_.accel_out = true;
    options_.qtn_out =
        false;  // Can be set true only for G365PDC1, G365PDF1, G325PDF1
    options_.count_out = true;  // Must be enabled for time_correction function
    options_.checksum_out = true;
    options_.atti_mode =
        1;  // 1=Euler mode 0=Inclination mode (Euler mode is typical)
    options_.atti_profile =
        0;  // Can only be nonzero only for G365PDC1, G365PDF1, G325PDF1
            // 0=General 1=Vehicle 2=Construction Machinery

    // Read parameters
    key = "serial_port";
    if (this->get_parameter(key, port_)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t\t%s", key.c_str(), port_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%s", key.c_str(),
                  port_.c_str());
    }

    key = "frame_id";
    if (this->get_parameter(key, frame_id_)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t\t%s", key.c_str(),
                  frame_id_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%s", key.c_str(),
                  frame_id_.c_str());
    }

    key = "time_correction_en";
    if (this->get_parameter(key, time_correction_)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(), time_correction_);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  time_correction_);
    }

    key = "ext_trigger_en";
    if (this->get_parameter(key, ext_trigger_en_)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(), ext_trigger_en_);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  ext_trigger_en_);
    }

    key = "burst_polling_rate";
    if (this->get_parameter(key, poll_rate_)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%.1f", key.c_str(), poll_rate_);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%.1f",
                  key.c_str(), poll_rate_);
    }

    key = "imu_dout_rate";
    if (this->get_parameter(key, options_.dout_rate)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(),
                  options_.dout_rate);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.dout_rate);
    }

    key = "imu_filter_sel";
    if (this->get_parameter(key, options_.filter_sel)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(),
                  options_.filter_sel);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.filter_sel);
    }

    key = "quaternion_output_en";
    if (this->get_parameter(key, options_.qtn_out)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(), options_.qtn_out);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.qtn_out);
    }

    // if quaternion is enabled then change topic to /imu/data
    imu_topic_ = (static_cast<bool>(options_.qtn_out) == false)
                     ? "/epson_imu/data_raw"
                     : "/epson_imu/data";

    key = "imu_topic";
    if (this->get_parameter(key, imu_topic_)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t\t%s", key.c_str(),
                  imu_topic_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%s", key.c_str(),
                  imu_topic_.c_str());
    }

    key = "output_32bit_en";
    if (this->get_parameter(key, output_32bit_)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(), output_32bit_);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  output_32bit_);
    }
    if (output_32bit_) {
      options_.temp_bit = true;
      options_.gyro_bit = true;
      options_.accel_bit = true;
      options_.qtn_bit = true;
    }

    key = "atti_profile";
    if (this->get_parameter(key, options_.atti_profile)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t\t%d", key.c_str(),
                  options_.atti_profile);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.atti_profile);
    }

    // Send warning if both time_correction & ext_trigger are enabled
    if (time_correction_ && ext_trigger_en_) {
      RCLCPP_ERROR(this->get_logger(),
                   "Time correction & ext_trigger both enabled");
      RCLCPP_WARN(this->get_logger(),
                  "Ext_trigger will be disabled. 1PPS should be connected to "
                  "GPIO2/EXT");
      options_.ext_sel = 1;
    } else if (!time_correction_ && ext_trigger_en_) {
      options_.ext_sel = 2;
      RCLCPP_INFO(this->get_logger(),
                  "Trigger should be connected to GPIO2/EXT");
    } else if (time_correction_ && !ext_trigger_en_) {
      options_.ext_sel = 1;
      RCLCPP_INFO(this->get_logger(), "1PPS should be connected to GPIO2/EXT");
    }
  }

  void Init() {
    std::chrono::seconds sec(1);
    rclcpp::Rate one_sec(sec);

    while (rclcpp::ok() && !seInit()) {
      RCLCPP_WARN(this->get_logger(),
                  "Retry to initialize the Seiko Epson HCL "
                  "layer in 1 second period...");
      one_sec.sleep();
    }

    while (rclcpp::ok() && !gpioInit()) {
      RCLCPP_WARN(this->get_logger(),
                  "Retry to initialize the GPIO layer in 1 second period...");
      one_sec.sleep();
    }

    do {
      fd_serial = uartInit(port_.c_str(), BAUD_460800);
      if (fd_serial == -1) {
        RCLCPP_WARN(this->get_logger(),
                    "Retry to open port: %s in 1 second period...",
                    port_.c_str());
      }
      one_sec.sleep();
    } while (rclcpp::ok() && (fd_serial == -1));

    while (rclcpp::ok() && !InitImu(options_)) {
      RCLCPP_WARN(this->get_logger(), "Retry to initialize the IMU...");
      one_sec.sleep();
    }

    IdentifyBuild();
    IdentifyDevice();
    int result = get_prod_id().compare(BUILD_FOR);
    if (result == 0) {
      RCLCPP_INFO(this->get_logger(), "OK: Build matches detected device");
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "*** Build *mismatch* with detected device ***");
      RCLCPP_WARN(
          this->get_logger(),
          "*** Check the CMakeLists.txt for setting a compatible IMU_MODEL \
                    variable, modify as necessary, and rebuild the driver ***");
    }
    sensorStart();
    RCLCPP_INFO(this->get_logger(), "Sensor started...");
  }

  bool InitImu(const struct EpsonOptions &options_) {
    RCLCPP_INFO(this->get_logger(), "Checking sensor power on status...");
    if (!sensorPowerOn()) {
      RCLCPP_WARN(this->get_logger(),
                  "Error: failed to power on Sensor. Exiting...");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Initializing Sensor...");
    if (!sensorInitOptions(options_)) {
      RCLCPP_WARN(this->get_logger(),
                  "Error: could not initialize Epson Sensor. Exiting...");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Epson IMU initialized.");
    return true;
  }

  std::string get_prod_id() {
    unsigned short prod_id1 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID1, false);
    unsigned short prod_id2 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID2, false);
    unsigned short prod_id3 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID3, false);
    unsigned short prod_id4 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID4, false);

    char myarray[] = {
        static_cast<char>(prod_id1), static_cast<char>(prod_id1 >> 8),
        static_cast<char>(prod_id2), static_cast<char>(prod_id2 >> 8),
        static_cast<char>(prod_id3), static_cast<char>(prod_id3 >> 8),
        static_cast<char>(prod_id4), static_cast<char>(prod_id4 >> 8)};
    std::string prod_id(myarray);
    return prod_id;
  }

  std::string get_serial_id() {
    unsigned short ser_num1 =
        registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM1, false);
    unsigned short ser_num2 =
        registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM2, false);
    unsigned short ser_num3 =
        registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM3, false);
    unsigned short ser_num4 =
        registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM4, false);

    char myarray[] = {
        static_cast<char>(ser_num1), static_cast<char>(ser_num1 >> 8),
        static_cast<char>(ser_num2), static_cast<char>(ser_num2 >> 8),
        static_cast<char>(ser_num3), static_cast<char>(ser_num3 >> 8),
        static_cast<char>(ser_num4), static_cast<char>(ser_num4 >> 8)};
    std::string ser_num(myarray);
    return ser_num;
  }

  void IdentifyBuild() {
    RCLCPP_INFO(this->get_logger(), "Compiled for:\t%s", BUILD_FOR);
  }

  void IdentifyDevice() {
    RCLCPP_INFO(this->get_logger(), "Reading device info...");
    RCLCPP_INFO(this->get_logger(), "PRODUCT ID:\t%s", get_prod_id().c_str());
    RCLCPP_INFO(this->get_logger(), "SERIAL ID:\t%s", get_serial_id().c_str());
  }

  void PubImuData() {
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();

    for (int i = 0; i < 9; i++) {
      imu_msg->orientation_covariance[i] = 0;
      imu_msg->angular_velocity_covariance[i] = 0;
      imu_msg->linear_acceleration_covariance[i] = 0;
    }
    imu_msg->orientation_covariance[0] = -1;
    imu_msg->header.frame_id = frame_id_;

    while (rclcpp::ok()) {
      // Call to read and post process IMU stream
      // Will return 0 if data incomplete or checksum error
      if (sensorDataReadBurstNOptions(options_, &epson_data_)) {
        if (!time_correction_) {
          imu_msg->header.stamp = this->now();
        } else {
          imu_msg->header.stamp = tc.get_stamp(epson_data_.count);
        }

        // Linear acceleration
        imu_msg->linear_acceleration.x = epson_data_.accel_x;
        imu_msg->linear_acceleration.y = epson_data_.accel_y;
        imu_msg->linear_acceleration.z = epson_data_.accel_z;

        // Angular velocity
        imu_msg->angular_velocity.x = epson_data_.gyro_x;
        imu_msg->angular_velocity.y = epson_data_.gyro_y;
        imu_msg->angular_velocity.z = epson_data_.gyro_z;

        // Orientation
        imu_msg->orientation.x = epson_data_.qtn1;
        imu_msg->orientation.y = epson_data_.qtn2;
        imu_msg->orientation.z = epson_data_.qtn3;
        imu_msg->orientation.w = epson_data_.qtn0;

        imu_data_pub_->publish(*imu_msg);
      } else {
        RCLCPP_WARN(
            this->get_logger(),
            "Warning: Checksum error or incorrect delimiter bytes in imu_msg "
            "detected");
      }
    }
  }

  void Spin() { PubImuData(); }
};  // end of class

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Force flush of the stdout buffer, which ensures a sync of all console
  // output even from a launch file.
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  auto imu_node = std::make_shared<ImuNode>(options);

  rclcpp::spin(imu_node);
  rclcpp::shutdown();
  return 0;
}
