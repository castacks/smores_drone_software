//==============================================================================
//
// 	main_regdump.c - Epson IMU sensor test application
//                 - This program reads all registers values for debug purpose
//
//
//  THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  NONINFRINGEMENT, SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A
//  PARTICULAR PURPOSE. IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE
//  OR CLAIM, ARISING FROM OR IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE
//  SOFTWARE.
//
//==============================================================================
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "hcl.h"
#include "hcl_gpio.h"
#include "hcl_uart.h"
#include "sensor_epsonCommon.h"

int fd_serial;

// Modify below as needed for hardware
const char* IMUSERIAL = "/dev/ttyUSB0";

int main(int argc, char* argv[]) {
  char prod_id[9];  // Device Product ID
  char ser_id[9];   // Device Serial ID

  // 1) Initialize the Seiko Epson HCL layer
  printf("\r\nInitializing HCL layer...");
  if (!seInit()) {
    printf(
        "\r\nError: could not initialize the Seiko Epson HCL layer. "
        "Exiting...\r\n");
    return -1;
  }
  printf("...done.\r\n");

  // 2) Initialize the GPIO interfaces, For GPIO control of pins SPI CS, RESET,
  // DRDY
  printf("\r\nInitializing GPIO interface...");
  if (!gpioInit()) {
    printf("\r\nError: could not initialize the GPIO layer. Exiting...\r\n");
    seRelease();
    return -1;
  }
  printf("...done.\r\n");

  // 3) Initialize UART Interface
  //    The baudrate value should be set the the same setting as currently
  //    flashed value in the IMU UART_CTRL BAUD_RATE register
  printf("\r\nInitializing UART interface...");
  fd_serial = uartInit(IMUSERIAL, BAUD_460800);
  if (fd_serial == -1) {
    printf("\r\nError: could not initialize UART interface. Exiting...\r\n");
    gpioRelease();
    seRelease();
    return -1;
  }
  printf("...done.\r\n");

  // 4) Print out which model executable was compiled and identify model
  printf("\r\nCompiled for:\t" BUILD_FOR);
  printf("\r\nReading device info...");
  if (strcmp(BUILD_FOR, getProductId(prod_id)) != 0) {
    printf("\r\n*** Build *mismatch* with detected device ***");
    printf(
        "\r\n*** Ensure you specify a compatible 'MODEL=' variable when "
        "running make when  rebuilding the driver ***\r\n");
  }
  printf("\r\nPRODUCT ID:\t%s", getProductId(prod_id));
  printf("\r\nSERIAL ID:\t%s", getSerialId(ser_id));

  // Incase, the IMU is currently in sampling mode, force config mode before
  // attempting to read from registers
  sensorStop();
  registerDump();
  uartRelease(fd_serial);
  gpioRelease();
  seRelease();
  printf("\r\n");

  return 0;
}
