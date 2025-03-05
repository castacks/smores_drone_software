extern "C"
{
#include "hcl.h"
#include "hcl_gpio.h"
#include "hcl_uart.h"
#include "sensor_epsonCommon.h"
}
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <termios.h>
#include <string>
#include <iostream>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
using namespace std;

int comPort;
string serial_port;
int baudrate;

//=========================================================================
//------------------------ IMU Initialization -----------------------------
//=========================================================================

bool init(const struct EpsonOptions &options)
{
  ROS_INFO("Initializing HCL layer...");
  if (!seInit())
  {
    ROS_ERROR("Error: could not initialize the Seiko Epson HCL layer. Exiting...");
    return false;
  }

  ROS_INFO("Initializing GPIO interface...");
  if (!gpioInit())
  {
    ROS_ERROR("Error: could not initialize the GPIO layer. Exiting...");
    seRelease();
    return false;
  }

  ROS_INFO("Initializing UART interface...");
  comPort = uartInit(serial_port.c_str(), baudrate);
  if (comPort == -1)
  {
    ROS_ERROR("Error: could not initialize UART interface. Exiting...");
    gpioRelease();
    seRelease();
    return false;
  }

  ROS_INFO("Resetting sensor...");
  // This is required for NVIDIA Jetson TK1 but root cause is still TBD.
  // Current assumption is that when the serial port is first opened, there are
  // corrupted characters sent on first serial messages.
  // So sending sensor stop may work around this condition
  // Investigation TBD
  // sensorDummyWrite();
  sensorRecoverFromGarbageInit();
  sensorReset();
  sensorStop();

  ROS_INFO("Checking sensor NOT_READY status...");
  if (!sensorPowerOn())
  {
    ROS_ERROR("Error: failed to power on Sensor. Exiting...");
    uartRelease(comPort);
    gpioRelease();
    seRelease();
    return false;
  }
  printf("...done.");

  ROS_INFO("Initializing Sensor...");
  if (!sensorInitOptions(options))
  {
    ROS_ERROR("Error: could not initialize Epson Sensor. Exiting...");
    uartRelease(comPort);
    gpioRelease();
    seRelease();
    return false;
  }

  ROS_INFO("...Epson IMU initialized.");
  return true;
}

//=========================================================================
//----------------------- Timestamp Correction ----------------------------
//=========================================================================

class TimeCorrection
{
private:
  int MAX_COUNT;
  int ALMOST_ROLLOVER;
  int ONE_SEC_NSEC;
  int HALF_SEC_NSEC;

  int32_t count_corrected;
  int32_t count_old;
  int32_t count_diff;
  int32_t time_current;
  int32_t time_old;
  int32_t time_nsec_current;
  int32_t count_corrected_old;
  bool rollover;
  bool flag_imu_lead;

public:
  TimeCorrection();
  ros::Time get_stamp(int, bool &);
};

TimeCorrection::TimeCorrection()
{
#if defined G365PDC0 || defined G365PDF0 || defined G370PDC0 || defined G370PDF0 || defined G325PDF0
  MAX_COUNT = 1048560000;
  ALMOST_ROLLOVER = 1010000000;
#else // G354/G364PDC0/G364PDCA/V340
  MAX_COUNT = 1398080000;
  ALMOST_ROLLOVER = 1340000000;
#endif
  ONE_SEC_NSEC = 1000000000;
  HALF_SEC_NSEC = 500000000;

  count_corrected = 0;
  count_old = 0;
  count_diff = 0;
  time_current = 0;
  time_old = 0;
  time_nsec_current = 0;
  count_corrected_old = 0;
  rollover = false;
  flag_imu_lead = false;
}

ros::Time TimeCorrection::get_stamp(int count, bool &rollover_event)
{
  // This assumes that the ROS time is sync'ed to 1PPS pulse sent to IMU
  // and the ROS latency from IMU sample to calling ros::Time::now() is
  // less than 0.020 seconds, otherwise the time in seconds can be off by 1
  // The IMU count is already converted to nsecs units (should always because
  // less than ONE_SEC_NSEC (1e9)
  time_current = ros::Time::now().toSec();
  time_nsec_current = ros::Time::now().nsec;
  std::cout.precision(20);
  rollover_event = false;

  count_diff = count - count_old;
  if (count > ALMOST_ROLLOVER)
  {
    rollover = true;
  }
  if (count_diff < 0)
  {
    rollover_event = true;
    if (rollover)
    {
      count_diff = count + (MAX_COUNT - count_old);
      std::cout << "rollover at: " << ros::Time::now().toSec() << "\n";
    }
    else
    {
      count_diff = count;
      count_corrected = 0;
      std::cout << "reset at: " << ros::Time::now().toSec() << "\n";
    }
#ifdef DEBUG
    std::cout << "cnt: " << count << "\t";
    std::cout << "cnt_o: " << count_old << "\t";
    std::cout << "cnt_d: " << count_diff << "\t";
    std::cout << "cntc: " << count_corrected << "\t";
    std::cout << "cntc_o: " << count_corrected_old << "\t";
    std::cout << "tm_c: " << time_current << "\t";
    std::cout << "tm_o: " << time_old << "\t";
    std::cout << "rovr: " << rollover << "\t";
    std::cout << "imu_ld: " << flag_imu_lead << std::endl;
#endif
    rollover = 0;
  }
  count_corrected = (count_corrected + count_diff) % ONE_SEC_NSEC;
  if (time_current != time_old && count_corrected > HALF_SEC_NSEC)
  {
#ifdef DEBUG
    std::cout << "tm_c != tm_o"
              << "\t";
    std::cout << "cnt: " << count << "\t";
    std::cout << "cnt_o: " << count_old << "\t";
    std::cout << "cnt_d: " << count_diff << "\t";
    std::cout << "cntc: " << count_corrected << "\t";
    std::cout << "cntc_o: " << count_corrected_old << "\t";
    std::cout << "tm_c: " << time_current << "\t";
    std::cout << "tm_o: " << time_old << "\t";
    std::cout << "rovr: " << rollover << "\t";
    std::cout << "imu_ld: " << flag_imu_lead << std::endl;
#endif
    time_current = time_current - 1;
  }
  else if ((count_corrected - count_corrected_old) < 0 && time_nsec_current > HALF_SEC_NSEC)
  {
#ifdef DEBUG
    std::cout << "cntc < cntc_o"
              << "\t";
    std::cout << "cnt: " << count << "\t";
    std::cout << "cnt_o: " << count_old << "\t";
    std::cout << "cnt_d: " << count_diff << "\t";
    std::cout << "cntc: " << count_corrected << "\t";
    std::cout << "cntc_o: " << count_corrected_old << "\t";
    std::cout << "tm_c: " << time_current << "\t";
    std::cout << "tm_o: " << time_old << "\t";
    std::cout << "rovr: " << rollover << "\t";
    std::cout << "imu_ld: " << flag_imu_lead << std::endl;
#endif
    time_current = time_current + 1;
    flag_imu_lead = 1;
  }
  else if (flag_imu_lead && time_nsec_current > HALF_SEC_NSEC)
  {
#ifdef DEBUG
    std::cout << "imu_ld = 1"
              << "\t";
    std::cout << "cnt: " << count << "\t";
    std::cout << "cnt_o: " << count_old << "\t";
    std::cout << "cnt_d: " << count_diff << "\t";
    std::cout << "cntc: " << count_corrected << "\t";
    std::cout << "cntc_o: " << count_corrected_old << "\t";
    std::cout << "tm_c: " << time_current << "\t";
    std::cout << "tm_o: " << time_old << "\t";
    std::cout << "rovr: " << rollover << "\t";
    std::cout << "imu_ld: " << flag_imu_lead << std::endl;
#endif
    time_current = time_current + 1;
  }
  else
  {
    flag_imu_lead = 0;
  }
  ros::Time time;
  time.nsec = count_corrected;
  time.sec = time_current;
  time_old = time_current;
  count_old = count;
  count_corrected_old = count_corrected;

  return time;
}

#include <math.h>

//=========================================================================
//------------------------------ Main -------------------------------------
//=========================================================================

int main(int argc, char **argv)
{
  ros::init(argc, argv, "epson_imu_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle np("~");

  np.param<string>("port", serial_port, "/dev/ttyUSB0");

  struct EpsonOptions options;
  int time_correction = 0;

  // Recommended to change these parameters via .launch file instead of
  // modifying source code below directly
  np.param("ext_sel", options.ext_sel, 0);
  np.param("ext_pol", options.ext_pol, 0);
  np.param("drdy_on", options.drdy_on, 0);
  np.param("drdy_pol", options.drdy_pol, 0);

  np.param("dout_rate", options.dout_rate, 3);
  np.param("filter_sel", options.filter_sel, 5);

  np.param("flag_out", options.flag_out, 1);
  np.param("temp_out", options.temp_out, 1);
  np.param("gyro_out", options.gyro_out, 1);
  np.param("accel_out", options.accel_out, 1);
  np.param("gyro_delta_out", options.gyro_delta_out, 0);
  np.param("accel_delta_out", options.accel_delta_out, 0);
  np.param("atti_out", options.atti_out, 0);
  np.param("gpio_out", options.gpio_out, 0);
  np.param("count_out", options.count_out, 1);
  np.param("checksum_out", options.checksum_out, 1);

  np.param("temp_bit", options.temp_bit, 1);
  np.param("gyro_bit", options.gyro_bit, 1);
  np.param("accel_bit", options.accel_bit, 1);
  np.param("gyro_delta_bit", options.gyro_delta_bit, 1);
  np.param("accel_delta_bit", options.accel_delta_bit, 1);
  np.param("atti_bit", options.atti_bit, 1);

  np.param("invert_xgyro", options.invert_xgyro, 0);
  np.param("invert_ygyro", options.invert_ygyro, 0);
  np.param("invert_zgyro", options.invert_zgyro, 0);
  np.param("invert_xaccel", options.invert_xaccel, 0);
  np.param("invert_yaccel", options.invert_yaccel, 0);
  np.param("invert_zaccel", options.invert_zaccel, 0);

  np.param("dlt_ovf_en", options.dlt_ovf_en, 0);
  np.param("dlt_range_ctrl", options.dlt_range_ctrl, 8);

  np.param("atti_mode", options.atti_mode, 1);
  np.param("atti_conv", options.atti_conv, 0);

  np.param("time_correction", time_correction, 0);

  // The baudrate value should be set the the same setting as currently
  // flashed value in the IMU UART_CTRL BAUD_RATE register
  baudrate = 460800;

  bool successful_init = init(options);
  for (int i = 0; i < 100; ++i)
  {
    if (successful_init)
      break;
    successful_init = init(options);
  }
  sensorStart();

  struct EpsonData epson_data;
  TimeCorrection tc;

  sensor_msgs::Imu imu_msg;
  for (int i = 0; i < 9; i++)
  {
    imu_msg.orientation_covariance[i] = 0;
    imu_msg.angular_velocity_covariance[i] = 0;
    imu_msg.linear_acceleration_covariance[i] = 0;
  }
  imu_msg.orientation_covariance[0] = -1;

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("epson_imu", 1);
  ros::Publisher rpy_pub = nh.advertise<geometry_msgs::Vector3Stamped>("epson_imu_rpy", 1);
  ros::Publisher rollover_event_pub = nh.advertise<std_msgs::Bool>("debug/imu_rollover_event", 1);
  ros::Publisher error_stamp_pub = nh.advertise<std_msgs::Float32>("debug/imu_error_stamp", 1);
  tf2::Quaternion myQuaternion;

  ros::Time latest_time = ros::Time::now();
  std_msgs::Bool rollover_event_msg;
  double last_correct_stamp = 0;
  double last_correct_time = 0;
  int cycles_since_last_correct_stamp = 0;
  double trigger_error_margin = 0.01;
  double trigger_correct_margin = 0.01;
  double accept_it_timeout = 3.0;
  double expected_period = 1. / 200;
  bool in_error_state = false;
  double load_time = ros::Time::now().toSec();
  while (ros::ok())
  {
    if (sensorDataReadBurstNOptions(options, &epson_data))
    {
      auto now = ros::Time::now().toSec();
      imu_msg.header.frame_id = "epson";
      bool rollover_event;
      if (!time_correction)
        imu_msg.header.stamp = ros::Time::now();
      else
        imu_msg.header.stamp = tc.get_stamp(epson_data.count, rollover_event);

      if (last_correct_stamp > 1e3 && std::abs(load_time - now) > 3)
      {
        auto orig_imu_stamp = imu_msg.header.stamp.toSec();
        auto prediction = last_correct_stamp + cycles_since_last_correct_stamp * expected_period;

        if (!in_error_state && std::abs(orig_imu_stamp - prediction) > trigger_error_margin)
        {
          ROS_ERROR("Entering error state :( %3.3f, %3.3f", orig_imu_stamp, prediction);
          in_error_state = true;
        }
        if (in_error_state)
        {
          if (std::abs(orig_imu_stamp - prediction) <= trigger_correct_margin || std::abs(now - last_correct_stamp) > accept_it_timeout)
          {
            in_error_state = false;
          }
          else
          {
            ROS_ERROR_THROTTLE(1, "In Error State :( %3.3f, %3.3f", orig_imu_stamp, prediction);
            ++cycles_since_last_correct_stamp;
            imu_msg.header.stamp = ros::Time(prediction);
            std_msgs::Float32 bad_stamp_msg;
            bad_stamp_msg.data = orig_imu_stamp;
            error_stamp_pub.publish(bad_stamp_msg);
          }
        }
        if (!in_error_state)
        {
          last_correct_stamp = orig_imu_stamp;
          cycles_since_last_correct_stamp = 1;
          last_correct_time = now;
        }
      }
      else
      {
        last_correct_stamp = imu_msg.header.stamp.toSec();
        cycles_since_last_correct_stamp = 1;
        last_correct_time = now;
      }

      if (rollover_event)
      {
        rollover_event_msg.data = true;
        rollover_event_pub.publish(rollover_event_msg);
      }
      // change to vel && acc

      imu_msg.angular_velocity.x = -epson_data.gyro_x;
      imu_msg.angular_velocity.y = -epson_data.gyro_y;
      imu_msg.angular_velocity.z = epson_data.gyro_z;
      imu_msg.linear_acceleration.x = -epson_data.accel_x;
      imu_msg.linear_acceleration.y = -epson_data.accel_y;
      imu_msg.linear_acceleration.z = epson_data.accel_z;

      // swap roll and pitch because G365 rotation order is yaw -> roll -> pitch
      // reverse polarity roll to maintain Right Hand Rule
      // myQuaternion.setRPY(epson_data.pitch, -epson_data.roll, epson_data.yaw); // Original

      double roll, pitch, yaw;
      roll = -epson_data.roll;
      pitch = -epson_data.pitch;
      yaw = epson_data.yaw;
      double t1 = 0.5 * epson_data.yaw;
      double t2 = 0.5 * epson_data.roll;
      double t3 = 0.5 * epson_data.pitch;
      myQuaternion[3] = -sin(t1) * sin(t2) * sin(t3) + cos(t1) * cos(t2) * cos(t3);
      myQuaternion[0] = -sin(t1) * sin(t3) * cos(t2) + sin(t2) * cos(t1) * cos(t3);
      myQuaternion[1] = sin(t1) * sin(t2) * cos(t3) + sin(t3) * cos(t1) * cos(t2);
      myQuaternion[2] = sin(t1) * cos(t2) * cos(t3) + sin(t2) * sin(t3) * cos(t1);

      // for flipping the imu
      // tf2::Quaternion q_rot2;
      // q_rot2.setRPY(M_PI, 0, 0);
      // end

      tf2::Quaternion q_new = myQuaternion; //*q_rot2;
      q_new.normalize();
      // this is the normal setting

      imu_msg.orientation.x = q_new[0];
      imu_msg.orientation.y = q_new[1];
      imu_msg.orientation.z = q_new[2];
      imu_msg.orientation.w = q_new[3];

      imu_pub.publish(imu_msg);

      // publish roll, pitch, yaw angles
      geometry_msgs::Vector3Stamped rpy;
      tf::Quaternion q(q_new[0], q_new[1], q_new[2], q_new[3]);
      tf::Matrix3x3 m(q);
      // double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      rpy.header = imu_msg.header;

      rpy.vector.x = roll;
      rpy.vector.y = pitch;
      rpy.vector.z = yaw;
      rpy_pub.publish(rpy);
    }
  }

  sensorStop();
  uartRelease(comPort);
  gpioRelease();
  seRelease();

  return 0;
}
