//==============================================================================
//
// 	main_csvlogger.c - Epson IMU sensor test application
//                   - This program initializes the Epson IMU and 
//                     sends sensor output to CSV file
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
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "hcl.h"
#include "hcl_gpio.h"
#include "sensor_epsonCommon.h"
#include "main_helper.h"

#include "hcl_uart.h"
#include <termios.h>

int comPort;
// Modify below as needed for hardware 
const char *IMUSERIAL = "/dev/ttyUSB0";

// The baudrate value should be set the the same setting as currently 
// flashed value in the IMU UART_CTRL BAUD_RATE register
const int IMUBAUD = 460800;

// Specify the number of samples to readout before exiting the program
const unsigned int NUM_SAMPLES = 100;


int main(int argc, char *argv[])
{
  unsigned int sample = 0;

  // Stores the post-processed sensor data    
  struct EpsonData epson_data;

  // Specify IMU options
  struct EpsonOptions options = {
  
  .ext_sel = 1,  // 0 = Sample Counter 1=Reset Counter 2=External Trigger
  .ext_pol = 0,
  .drdy_on = 0,
  .drdy_pol = 0,
  .dout_rate = CMD_RATE125,
  .filter_sel = CMD_FLTAP32,
  .flag_out = 1,
  .temp_out = 1,
  .gyro_out = 1,
  .accel_out = 1,
  .gyro_delta_out = 0,
  .accel_delta_out = 0,
  .atti_out = 0,   // Only valid for G365PDC0, G365PDF0, G325PDF0 otherwise ignored
  .gpio_out = 0,
  .count_out = 1,
  .checksum_out = 1,

  // Set 0=16bit 1=32bit sensor output
  .temp_bit = 1,
  .gyro_bit = 1,
  .accel_bit = 1,
  .gyro_delta_bit = 1,
  .accel_delta_bit = 1,
  .atti_bit = 1,

  // Set 0=normal 1=reverse polarity
  .invert_xgyro = 0,
  .invert_ygyro = 0,
  .invert_zgyro = 0,
  .invert_xaccel = 0,
  .invert_yaccel = 0,
  .invert_zaccel = 0,
  
  .dlt_ovf_en = 0,
  .dlt_range_ctrl = 8,

  // NOTE:Only valid for G365PDC0, G365PDF0, G325PDF0 otherwise ignored
  .atti_mode = 1, // 0=Inclination mode 1=Euler mode
  .atti_conv = 0  // Attitude Conversion Mode
  };

  // 1) Initialize the Seiko Epson HCL layer
  printf("\r\nInitializing HCL layer...");
  if (!seInit())
  {
    printf("\r\nError: could not initialize the Seiko Epson HCL layer. Exiting...\r\n");
    return -1;
  }
  printf("...done.\r\n");

  // 2) Initialize the GPIO interfaces, For GPIO control of pins SPI CS, RESET, DRDY
  printf("\r\nInitializing GPIO interface...");
  if (!gpioInit())
  {
    printf("\r\nError: could not initialize the GPIO layer. Exiting...\r\n");
    seRelease();
    return -1;
  }
  printf("...done.");

  // 3) Initialize UART Interface
  printf("\r\nInitializing UART interface...");
  comPort = uartInit(IMUSERIAL, IMUBAUD);
  if(comPort == -1)
  {   
    printf("\r\nError: could not initialize UART interface. Exiting...\r\n");
    gpioRelease();
    seRelease();
    return -1;
  }
  printf("...done.");

  // 4) Power on sequence - force sensor to config mode, HW reset sensor
  //      Check for errors
  printf("\r\nChecking sensor NOT_READY status...");
  if(!sensorPowerOn())
  {
    printf("\r\nError: failed to power on Sensor. Exiting...\r\n");
    uartRelease(comPort);
    gpioRelease();
    seRelease();
    return -1;
  }
  printf("...done.");
  
  // Initialize sensor with desired settings
  printf("\r\nInitializing Sensor...");
  if(!sensorInitOptions(options))
  {
    printf("\r\nError: could not initialize Epson Sensor. Exiting...\r\n");
    uartRelease(comPort);
    gpioRelease();
    seRelease();
    return -1;
  }
  else
  {
    printf("...Epson IMU initialized.");        
  }
  // Initialize text files for data logs
  const time_t date   = time(NULL);                   // Functions for obtaining and printing time and date
  struct tm tm        = *localtime(&date);
  char  EpsonlogName[128];
  
  // Create Epson IMU Data Log
  sprintf(EpsonlogName, "EpsonLog_%4d-%02d-%02d_T%02d-%02d-%02d.csv", tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  FILE *EpsonLog     = fopen(EpsonlogName, "w");
  fprintf(EpsonLog,"Date: ");
  fprintf(EpsonLog, "%s", ctime(&date));
  printf("\r\n...Epson IMU Logging.\r\n");
  sensorStart();
  printHeaderRow(EpsonLog, options);
  while (1)
  {
    // For SPI interface, check if DRDY pin asserted
    // For UART interface, check if UART recv buffer contains a sensor sample packet
    if(sensorDataReadyOptions(options))
    {
      sensorDataReadBurstNOptions(options, &epson_data);       
      printSensorRow(EpsonLog, options, &epson_data, sample);
      sample++;
    }
    if (sample > (NUM_SAMPLES-1))
      break;
  }
  
  const time_t end = time(NULL);                    // Functions for obtaining and printing time and data
  fprintf(EpsonLog, "\r\nEnd: ");
  fprintf(EpsonLog, "%s", ctime(&end));
  
  sensorStop();
  seDelayMS(1000);
  uartRelease(comPort);
  gpioRelease();
  seRelease();
  fclose(EpsonLog);
  printf("\r\n");
  return 0;
}
