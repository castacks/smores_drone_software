Disclaimer:
--------------
THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT,
SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE OR CLAIM, ARISING FROM OR
IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE SOFTWARE.

Test machine:
--------------
  UART Interface,
    Ubuntu 14 running in Oracle VirtualBox on Core i7 Win 8.1 PC
    Raspberry Pi 2 Model B v1.1 (with Epson IMU USB evalboard)
    Raspberry Pi Model B+ v1.2 (with Epson IMU USB evalboard)

Requirements:
--------------
  For using IMU UART interface, this should work on any generic unix (POSIX) serial port with gcc

Important for UART Interface:
-----------------------------
1. The application assumes that the Epson IMU is connected to serial tty (UART) either through USB evaluation board or
   directly to the UART port on an embedded system:

    Edit the source files:
    Specify the proper serial port on host system:
        const char *IMUSERIAL = "/dev/ttyxxx";
    Specify the # of samples to capture in main_csvlogger.c or main_screen.c:
        const unsigned int NUM_SAMPLES = xxxx;

    Compile the application:
        A. Run "make" specifying the <target> with "MODEL=" parameter.
           Supported <target> options are:
                screen or
                csvlogger or
                regdump or
                all
           Supported "MODEL=" parameters are:
                G354
                G364PDC0
                G364PDCA
                G365PDC0
                G365PDF0
                G370PDC0
                G370PDF0
                G325PDF0
                G320
                V340
           If "MODEL=" is not specified then it assumes MODEL=G354
           For example:
                make screen MODEL=G320
                or
                make csvlogger MODEL=V340
                or
                make regdump MODEL=G364PDC0
                or
                make clean  <-- recommended before creating new builds
        C. The executable will be in the found in the same folder as the Makefile and source files.

NOTE: Modify the EpsonOptions struct and configure sensor configuration settings in main() function in main_xxxx.c

NOTE: Any references to GPIO interface are placeholders and currently do nothing.
      The end user is required to connect the low level code to these functions to make use of them.

NOTE: Any references to SPI interface is not relevant for this package. The source code is shared with a common 
      code base so references to SPI interface can be ignored.

How to run the program:
-----------------------
1. Run the executable from console (may require root access to execute if regular user can not access TTY)
   sudo ./<exe filename>

2. The default csvlogger program creates CSV log of sensor data in a processed scaled log file for 1000 samples:
       Output date rate = 125 Hz (For all other models)
       Filter Tap = Moving Average TAP16/TAP32 (For all other models)

       Output = For G365PDC0/G365PDF0/G325PDF0 is 32-bit Gyro X,Y,Z Accel X,Y,Z Roll,Pitch,Yaw ResetCounter Checksum
                For G354/G364PDCA/G364PDC0/G320/G370PDC0/G370PDF0 is 32-bit Gyro X,Y,Z Accel X,Y,Z ResetCounter Checksum
                For V340 is 16-bit Gyro X,Y,Z Accel X,Y,Z Sample Counter

File Listing:
--------------
epson_imu_driver_node.cpp   - ROS Driver C++ to C wrapper
                            - This built by ROS catkin_make environment

hcl.h                       - Dummy abstraction layer header (work in progress) which defines delay() functions
hcl_linux.c                 - Abstraction layer for Linux
hcl_gpio.c                  - Abstraction layer for GPIO control functions typically for connection to RESET, DRDY, SCS#
                              This a dummy assignment of pins RESET, DRDY, SCS#
                              Modify or replace if GPIO pins are to be used
hcl_gpio.h                  - Header for GPIO abstraction
hcl_uart.c                  - Abstraction layer specific for UART IF which uses standard unix termios library calls
hcl_uart.h                  - Header for UART IF abstraction
main_csvlogger.c            - Test application - Initialize IMU, and read sensor data to CSV log file
main_regdump.c              - Test application - Output register settings to console for debug purpose
main_screen.c               - Test application - Initialize IMU, and read sensor data to console
main_helper.c               - Helper functions for console utilities
main_helper.h               - Header for helper functions for console utilities
Makefile                    - For make utility to compile test applications
readme.txt                  - This file.
sensor_epsonCommon.c        - Common functions for Epson IMU
sensor_epsonCommon.h        - Header for common C functions of Epson IMU

*** Modify sensorInit() to customize IMU configuration in sensor_epsonImuxxxx.c ***
sensor_epsonG320.c          - Model specific functions for Epson M-G320
sensor_epsonG320.h          - Model specific header for Epson M-G320
sensor_epsonG354.c          - Model specific functions for Epson M-G354
sensor_epsonG354.h          - Model specific header for Epson M-G354
sensor_epsonG364.c          - Model specific functions for Epson M-G364
sensor_epsonG364PDC0.h      - Model specific header for Epson M-G364PDC0
sensor_epsonG364PDCA.h      - Model specific header for Epson M-G364PDCA
sensor_epsonV340.c          - Model specific functions for Epson M-G340
sensor_epsonG340.h          - Model specific header for Epson M-V340
sensor_epsonG365.c          - Model specific functions for Epson M-G365
sensor_epsonG365PDC0.h      - Model specific header for Epson M-G365PDC0
sensor_epsonG365PDF0.h      - Model specific header for Epson M-G365PDF0
sensor_epsonG370.c          - Model specific functions for Epson M-G370
sensor_epsonG370PDC0.h      - Model specific header for Epson M-G370PDC0
sensor_epsonG370PDF0.h      - Model specific header for Epson M-G370PDF0
sensor_epsonG325.c          - Model specific functions for Epson M-G325
sensor_epsonG325PDF0.h      - Model specific header for Epson M-G325PDF0
***********************************************************************************
sensor_epsonUart.c          - UART specific functions

Change Record:
--------------
2017-02-07  v1.0    - Initial release
2018-04-27  v1.1    - Code cleanup, no functional changes
2018-11-30  v1.2    - Added support for G365/G370
2019-06-06  v1.3    - Added G325 support, Refactor the IMU initialization and sensor processing routine to use struct
