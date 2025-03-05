//==============================================================================
//
// 	main_regdump.c - Epson IMU sensor test application
//                 - This program reads all registers values for debug purpose
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

#include "hcl_uart.h"
#include <termios.h>

int comPort;
// Modify below as needed for hardware 
const char *IMUSERIAL = "/dev/ttyUSB0";

// The baudrate value should be set the the same setting as currently 
// flashed value in the IMU UART_CTRL BAUD_RATE register
const int IMUBAUD = 460800;


int main(int argc, char *argv[])
{   

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
  sensorDummyWrite();
  printf("...done.");

  // Incase, the IMU is currently in sampling mode, force config mode before
  // attempting to read from registers
  sensorStop();
  registerDump();
  uartRelease(comPort);
  gpioRelease();
  seRelease();
  printf("\r\n");

  return 0;
}
