//==============================================================================
//
// 	sensor_epsonCommon.h - Epson IMU sensor specific definitions common 
//                      for all IMU models
//
//
//  THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT, 
//  SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT
//  SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE OR CLAIM, ARISING FROM OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OF THE SOFTWARE.
//
//==============================================================================
#ifndef EPSONCOMMON_H_
#define EPSONCOMMON_H_

#include<math.h>
#include<stdio.h>

#define TRUE (1)
#define FALSE (0)

#ifdef G354
#include "sensor_epsonG354.h"
#elif G364PDC0
#include "sensor_epsonG364PDC0.h"
#elif G364PDCA
#include "sensor_epsonG364PDCA.h"
#elif G320
#include "sensor_epsonG320.h"
#elif G365PDC0
#include "sensor_epsonG365PDC0.h"
#elif G365PDF0
#include "sensor_epsonG365PDF0.h"
#elif G370PDC0
#include "sensor_epsonG370PDC0.h"
#elif G370PDF0
#include "sensor_epsonG370PDF0.h"
#elif G325PDF0
#include "sensor_epsonG325PDF0.h"
#else /* V340 */
#include "sensor_epsonV340.h"
#endif

#define DELAY_EPSON_RESET           10                                     // Milliseconds Reset Pulse Width
#define EPSON_POWER_ON_DELAY        800                                    // Milliseconds
#define EPSON_FLASH_TEST_DELAY      5                                      // Milliseconds
#define EPSON_SELF_TEST_DELAY       80                                     // Milliseconds
#define EPSON_FILTER_DELAY          1                                      // Milliseconds

#define EpsonStall()                seDelayMicroSecs(EPSON_STALL)          // Required delay between bus cycles for serial timings

struct EpsonOptions{
  
  // MSC_CTRL
  int ext_sel;
  int ext_pol;
  int drdy_on;
  int drdy_pol;
  
  // SMPL_CTRL
  int dout_rate;

  // FILTER_CTRL
  int filter_sel;
  
  // BURST_CTRL1
  int flag_out, temp_out, gyro_out, accel_out, gyro_delta_out, accel_delta_out, atti_out;
  int gpio_out, count_out, checksum_out;
  
  // BURST_CTRL2
  int temp_bit, gyro_bit, accel_bit, gyro_delta_bit, accel_delta_bit, atti_bit;
  
  // POL_CTRL
  int invert_xgyro, invert_ygyro, invert_zgyro, invert_xaccel, invert_yaccel, invert_zaccel;

  //DLT_CTRL
  int dlt_ovf_en;
  int dlt_range_ctrl;

  //ATTI_CTRL
  int atti_mode;
  int atti_conv;
  
  //LPF_CTRL
  int accl_lpf_fc;
  int gyro_lpf_fc;
};

struct EpsonData {
  unsigned short ndflags;
  float temperature;
  float gyro_x, gyro_y, gyro_z;
  float accel_x, accel_y, accel_z;
  float gyro_delta_x, gyro_delta_y, gyro_delta_z;
  float accel_delta_x, accel_delta_y, accel_delta_z;
  float roll, pitch, yaw;
  unsigned short gpio;
  int count;
};

unsigned int sensorDataByteLength(struct EpsonOptions);
int sensorDataReadyOptions(struct EpsonOptions);
void populateEpsonData(struct EpsonOptions, struct EpsonData*);
int sensorDataReadBurstNOptions(struct EpsonOptions, struct EpsonData*);
int sensorInitOptions(struct EpsonOptions);
int sensorHWReset(void);
int sensorPowerOn(void);
void registerDump(void);
void registerWriteByte(unsigned char, unsigned char, unsigned char, unsigned int);
unsigned short registerRead16(unsigned char, unsigned char, unsigned int);
void sensorStart(void);
void sensorStop(void);
void sensorReset(void);
int sensorFlashTest(void);
int sensorSelfTest(void);
void sensorDummyWrite(void);
void sensorRecoverFromGarbageInit(void);
#endif /* EPSONCOMMON_H_ */
