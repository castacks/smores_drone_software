# Setup instructions for IMP365 on Jetson orin with phoenix pro drone

1) clone this repository into the src folder of a Ros2 workspace
2) build the repo with the following command
```
  colcon build --packages-select epson_imu_driver_ros2 --symlink-install
  source install/setup.bash

```
3) run the imu driver with the following command
```
  ros2 launch epson_imu_driver_ros2 epson_g365_fixed_launch.py

```

Following are the generic instructions for other IMU types

** This repo is deprecated and read-only **

** Please use ess_imu_driver2 instead **

# README for Epson IMU Driver using UART interface for ROS2 Node

<!---toc start-->

- [README for Epson IMU Driver using UART interface for ROS2 Node](#readme-for-epson-imu-driver-using-uart-interface-for-ros2-node)
  - [What is this repository for?](#what-is-this-repository-for)
  - [What kind of hardware or software will I likely need?](#what-kind-of-hardware-or-software-will-i-likely-need)
  - [How do I use the driver?](#how-do-i-use-the-driver)
  - [How do I use the driver if usleep() is not supported for time delays?](#how-do-i-use-the-driver-if-usleep-is-not-supported-for-time-delays)
  - [How do I use the driver with GPIOs to control IMU RESET#, DRDY, EXT pins?](#how-do-i-use-the-driver-with-gpios-to-control-imu-reset-drdy-ext-pins)
  - [How do I build, install, run this ROS2 package?](#how-do-i-build-install-run-this-ros2-package)
    - [Example console output of *colcon build* for G366PDG0:](#example-console-output-of-colcon-build-for-g366pdg0)
    - [Example console output of launching ROS2 node for G366PDG0:](#example-console-output-of-launching-ros2-node-for-g366pdg0)
  - [What does this ROS2 IMU Node Publish as messages?](#what-does-this-ros2-imu-node-publish-as-messages)
    - [Without Quaternion Output](#without-quaternion-output)
      - [ROS Topic Message *data_raw*](#ros-topic-message-data_raw)
    - [With Quaternion Output](#with-quaternion-output)
      - [ROS Topic Message data](#ros-topic-message-data)
  - [Why am I seeing high latencies or slower than expected IMU data rates when using USB-UART bridges?](#why-am-i-seeing-high-latencies-or-slower-than-expected-imu-data-rates-when-using-usb-uart-bridges)
    - [Modifying *latency_timer* by sysfs mechanism](#modifying-latency_timer-by-sysfs-mechanism)
    - [Modifying *low_latency* flag using *setserial* utility](#modifying-low_latency-flag-using-setserial-utility)
  - [Package Contents](#package-contents)
  - [License](#license)
  - [References](#references)

<!---toc end-->

## What is this repository for?

- This code provides interface between Epson IMU and ROS2 using the UART interface.
- The UART connection can be either direct or by USB-serial converter such as FTDI bridge ICs.
- The src/epson_imu_uart_node.cpp is the ROS2 C++ wrapper used to communicate with ROS2
- The other source files in src/ are based on the Linux C driver originally released by Epson:
  [Epson IMU UART-only Linux User-space Driver Example](https://vdc.epson.com/imu-products/imu-inertial-measurement-units)
- Information about ROS2, and tutorials can be found: [ROS.org](https://index.ros.org/doc/ros2/)

## What kind of hardware or software will I likely need?

This will likely require the following:

- Epson USB evaluation board or equivalent FTDI USB-Serial interface to connect to ROS host (tty/serial) [See Evaluation Boards](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)
- Epson IMU (G320/G330/G354/G364/G365/G366/G370/V340) [IMU models](https://global.epson.com/products_and_drivers/sensing_system/imu/)
- ROS2 Dashing or later (via download) [ROS.org](https://index.ros.org/doc/ros2/Installation/#installationguide)
- This software was developed and tested on the following:

```
  ROS2:        Dashing, Eloquent, Foxy
  Description: Ubuntu 18.04.4 LTS, Ubuntu 20.04 LTS
```

## How do I use the driver?

- This code assumes that the user is familiar with building ROS2 packages using the colcon build process.
- **NOTE:** This is *NOT* detailed instructions describing step by step procedures on how to build and install this ROS driver.
- Please refer to the ROS.org website for more detailed instructions on the ROS package build process. [ROS.org](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/)
- **NOTE:** At bare minimum, you must modify the *CMakeLists.txt*, or any time you change to a different IMU model, then build this package with colcon.
- If the IMU model is unchanged, then subsequent changes to IMU settings can be done by instead editing the IMU model specific launch.py file located in the launch/ folder
- The IMU model specific launch.py file should only be used in conjunction with the same colcon built executable of the same matching the IMU model.
- **NOTE:** Do not just switch launch.py files without modifying the CMakeLists.txt & rebuilding the executable to match the IMU model. Do not mix IMU model launch files without the matching IMU model colcon built binaries.

## How do I use the driver if usleep() is not supported for time delays?

- **NOTE:** In the hcl_linux.c, there are empty functions for time delays in millisecond and microseconds using seDelayMS() and seDelayMicroSecs(), respectively.
- On embedded Linux platforms, the user may need modify and redirect to platform specific delay routines if usleep() is not supported.
- For example on RaspberryPi, the time delay functions for millisecond and microseconds can be redirected to WiringPi library delay() and delayMicroseconds(), respectively.
- If a hardware delay is not available from a library, then a software delay loop is possible but not preferred.

## How do I use the driver with GPIOs to control IMU RESET#, DRDY, EXT pins?

- Because this driver connects to the IMU using the UART interface, the use of GPIO pins for connecting to the IMU RESET#, EXT, or DRDY is purely optional, and mainly intended for embedded Linux platforms (non-PC based).
- When possible, connecting the RESET# is recommended to force Hardware Reset during every IMU initialization, for better robustness.
- Although this code does not implement GPIO functions, this code is structured for the user to easily redirect GPIO control to low-level hardware GPIO function calls for ease of implementation.
- There is no standard method to implement GPIO connections on embedded Linux platform, but the following files typically need changes:

```
  src/hcl_linux.c
  src/hcl_gpio.c
  src/hcl_gpio.h
```

- Typically, an external library needs to be invoked to initialize & enable GPIO HW functions on the user's embedded platform.

- This typically requires changes to hcl_linux.c

  - add #include to external library near the top of hcl_linux.c
  - add the initialization call inside the seInit() function in hcl_linux.c

For example on an Raspberry Pi, the following changes can be made to hcl_linux.c:

```
...

  #include <stdint.h>
  #include <stdio.h>
  #include <wiringPi.h>  // <== Added external library
  #include "hcl.h"

  int seInit(void)
  {
    // Initialize wiringPi libraries                                                   // <== Added
    printf("\r\nInitializing libraries...");                                           // <== Added
    if(wiringPiSetupGpio() != 0) {                                                     // <== Added external library initialization
      printf("\r\nError: could not initialize wiringPI libraries. Exiting...\r\n");    // <== Added
      return NG;                                                                       // <== Added
    }                                                                                  // <== Added
    printf("...done.");

    return OK;
  }

...
```

- Typically, the GPIO pins need to be assigned according to pin numbering specific to the embedded HW platform.
- This typically requires changes to hcl_gpio.h

For example on an Raspberry Pi, the following changes to hcl_gpio.h with the following pin mapping:

```
    Epson IMU                   Raspberry Pi
    ---------------------------------------------------
    EPSON_RESET                 RPI_GPIO_P1_15 (GPIO22) Output
    EPSON_DRDY                  RPI_GPIO_P1_18 (GPIO24) Input
```

```
...

  // Prototypes for generic GPIO functions
  int gpioInit(void);
  int gpioRelease(void);

  void gpioSet(uint8_t pin);
  void gpioClr(uint8_t pin);
  uint8_t gpioGetPinLevel(uint8_t pin);

  #define RPI_GPIO_P1_15              22                    // <== Added
  #define RPI_GPIO_P1_18              24                    // <== Added

  #define EPSON_RESET                 RPI_GPIO_P1_15        // <== Added
  #define EPSON_DRDY                  RPI_GPIO_P1_18        // <== Added
...
```

- Typically, the external library will have GPIO pin control functions such as set_output(), set_input(), set(), reset(), read_pin_level(), etc...

- This requires changes to hcl_gpio.c

  - Redirect function calls in hcl_gpio.c for gpioInit(), gpioRelease(), gpioSet(), gpioClr(), gpioGetPinLevel() to equivalent external library pin control functions.
  - For example on an Raspberry Pi, the following changes to hcl_gpio.c:

```
  #include <stdint.h>
  #include <wiringPi.h>                         // <== Added external library

  #include "hcl.h"
  #include "hcl_gpio.h"
...

  int gpioInit(void)
  {
    pinMode(EPSON_RESET, OUTPUT);               // <== Added external call RESET Output Pin
    pinMode(EPSON_DRDY, INPUT);                 // <== Added external call DRDY Input Pin
    pullUpDnControl(EPSON_DRDY, PUD_OFF) ;      // <== Added external call Disable any internal pullup or pulldown resistance

    return OK;
  }

...

  int gpioRelease(void)
  {
    return OK;
  }

...

  void gpioSet(uint8_t pin)
  {
    digitalWrite(pin, HIGH);                    // <== Added external call set pin HIGH
  }

  ...

  void gpioClr(uint8_t pin)
  {
    digitalWrite(pin, LOW);                     // <== Added external call set pin LOW
  }

  ...

  uint8_t gpioGetPinLevel(uint8_t pin)
  {
    return (digitalRead(pin));                  // <== Added external call to return pin state of input pin
  }

  ...
```

## How do I build, install, run this ROS2 package?

The Epson IMU ROS2 driver is designed for building in the standard ROS2 colcon build environment.
Therefore, a functional colcon workspace in ROS2 is a prerequisite.
Refer to the ROS2 Tutorials for more info: [ROS2 Tutorial](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/)

For more information on ROS & colcon setup refer to:
[Installing and Configuring ROS Environment](https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/).

1. Place this package (including folders) into a new folder within your colcon workspace "src" folder.
   For example, we recommend using the folder name "ess_imu_ros2_uart_driver"

```
   <colcon_workspace>/src/ess_imu_ros2_uart_driver/
```

2. Modify the CMakeLists.txt to select the desired Epson IMU model that is attached to the ROS system.
   Refer to the comment lines inside the CMakeLists.txt for additional info.
   **NOTE:** You *MUST* re-build using *colcon build* when changing IMU models or after any changes in the *CMakeLists.txt*

3. From the colcon_workspace folder run *colcon build* to build all ROS2 packages located in the \<colcon_workspace>/src/ folder.
   Re-run the above *colcon build* command to rebuild the driver after making any changes to the *CMakeLists.txt*, any of the .c or .cpp or .h source files.

```
   <colcon_workspace>/colcon build --packages-select ess_imu_ros2_uart_driver --symlink-install
```

**NOTE:** It is not necessary to re-run *colcon build* if changes are only made to the launch.py files.

**NOTE:** It is recommended to change IMU settings by editing the parameters in the launch.py file, wherever possible, instead of modifying the .c or .cpp source files directly

4. Reload the current ROS environment variables that may have changed after the colcon build process.

```
   From the <colcon_workspace>: . install/setup.bash
```

5. Modify the appropriate launch file for the IMU model in the launch/ folder to set your desired IMU configure parameter options at runtime:

| Parameter            | Comment                                                                                                                                                      |
| -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| serial_port          | specifies the serial port name i.e. /dev/ttyUSB0                                                                                                             |
| frame_id             | specifies the value in the IMU message frame_id field                                                                                                        |
| imu_topic            | specifies the topic name for publishing IMU messages                                                                                                         |
| burst_polling_rate   | specifies the driver internal polling rate for the serial port input buffer (typically does not need changing)                                               |
| imu_dout_rate        | specifies the IMU output data rate                                                                                                                           |
| imu_filter_sel       | specifies the IMU filter setting                                                                                                                             |
| quaternion_output_en | enables or disables quaternion output (supported only be G330PDG0, G365PDC1, G365PDF1, G366PDG0)                                                             |
| atti_profile         | specifies the attitude motion profile (supported only be G330PDG0, G365PDC1, G365PDF1, G366PDG0)                                                             |
| output_32bit_en      | enables or disables outputing sensor data in 32 or 16 bit                                                                                                    |
| time_correction_en   | enables time correction function using IMU counter reset function & external 1PPS connection to IMU GPIO2/EXT pin. **Cannot be used when ext_trigger_en=1.** |
| ext_trigger_en       | enables triggering of IMU samples using IMU external trigger function on IMU GPIO2/EXT pin. \*\*Cannot be used when time_correction_en=1                     |

**NOTE:** The ROS2 launch file passes IMU configuration settings to the IMU at runtime.
Therefore, rebuilding with *colcon build* when changing the launch file is not required

**NOTE:** For previous ROS2 versions (Dashing or Eloquent) the launch.py needs to be modified to replace namespace parameter "executable" to "node_executable" because a change in ROS2 Foxy

6. To start the Epson IMU ROS2 driver use the appropriate launch.py file (located in launch/) from console. All parameters are described in the inline comments of the launch file. The launch file contains parameters for configuring IMU settings at runtime.

   For example, for the Epson G366PDG0 IMU:

```
   <colcon_workspace>/ros2 launch ess_imu_ros2_uart_driver g330_g365_g366_launch.py
```

| Launch File                  | Description                                                                                                                  |
| ---------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| g320_g354_g364_launch.py     | For G320PDG0/G354PDH0/G364PDC0/G364PDCA outputs to ROS topic imu/data_raw (**gyro, accel, but no quaternion orientation**)   |
| g330_g365_g366_launch.py     | For G330PDG0/G365PDx1/G366PDG0 outputs to ROS topic imu/data (**gyro, accel data, including quaternion orientation**)        |
| g330_g365_g366_raw_launch.py | For G330PDG0/G365PDx1/G366PDG0 outputs to ROS topic imu/data_raw (**gyro, accel data, but no quaternion orientation**)       |
| g370_launch.py               | For G370PDF1/G370PDS0/G370PDG0/G370PDT0 , outputs to ROS topic imu/data_raw (**gyro, accel, but no quaternion orientation**) |
| v340.launch.py               | For V340PDD0, outputs to ROS topic imu/data_raw (**gyro, accel, but no quaternion orientation**)                             |

### Example console output of *colcon build* for G366PDG0:

```
guest@guest-VirtualBox:~/dev_ws$ colcon build --symlink-install --packages-select ess_imu_ros2_uart_driver
Starting >>> ess_imu_ros2_uart_driver
Finished <<< ess_imu_ros2_uart_driver [11.0s]

Summary: 1 package finished [11.4s]
```

### Example console output of launching ROS2 node for G370PDS0:

```
guest@guest-VirtualBox:~/dev_ws$ ros2 launch ess_imu_ros2_uart_driver g370_launch.py
[INFO] [launch]: All log files can be found below /home/guest/.ros/log/2023-06-01-11-23-03-495344-guest-VirtualBox-5874
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [ess_imu_uart_node-1]: process started with pid [5876]
[ess_imu_uart_node-1] [INFO] [1685643783.688859098] [epson_node]: serial_port:		/dev/ttyUSB0
[ess_imu_uart_node-1] [INFO] [1685643783.690021597] [epson_node]: frame_id:		imu_link
[ess_imu_uart_node-1] [INFO] [1685643783.690654926] [epson_node]: time_correction_en:	0
[ess_imu_uart_node-1] [INFO] [1685643783.691051459] [epson_node]: ext_trigger_en:	0
[ess_imu_uart_node-1] [INFO] [1685643783.691633076] [epson_node]: burst_polling_rate:	4000.0
[ess_imu_uart_node-1] [INFO] [1685643783.692141342] [epson_node]: imu_dout_rate:	4
[ess_imu_uart_node-1] [INFO] [1685643783.692657785] [epson_node]: imu_filter_sel:	6
[ess_imu_uart_node-1] [WARN] [1685643783.692678935] [epson_node]: Not specified param quaternion_output_en. Set default value:	0
[ess_imu_uart_node-1] [INFO] [1685643783.692694952] [epson_node]: imu_topic:		/epson_imu/data_raw
[ess_imu_uart_node-1] [INFO] [1685643783.692721186] [epson_node]: output_32bit_en:	1
[ess_imu_uart_node-1] [WARN] [1685643783.692734187] [epson_node]: Not specified param atti_profile. Set default value:	0
[ess_imu_uart_node-1] Attempting to open port.../dev/ttyUSB0
[ess_imu_uart_node-1] [INFO] [1685643784.696642847] [epson_node]: Checking sensor power on status...
[ess_imu_uart_node-1] [INFO] [1685643785.550607398] [epson_node]: Initializing Sensor...
[ess_imu_uart_node-1] [INFO] [1685643785.594485625] [epson_node]: Epson IMU initialized.
[ess_imu_uart_node-1] [INFO] [1685643785.595783585] [epson_node]: Compiled for:	G370PDS0
[ess_imu_uart_node-1] [INFO] [1685643785.597116331] [epson_node]: Reading device info...
[ess_imu_uart_node-1] [INFO] [1685643785.662866685] [epson_node]: PRODUCT ID:	G370PDS0
[ess_imu_uart_node-1] [INFO] [1685643785.726509623] [epson_node]: SERIAL ID:	W0000015
[ess_imu_uart_node-1] [INFO] [1685643785.789930079] [epson_node]: OK: Build matches detected device
[ess_imu_uart_node-1]
[ess_imu_uart_node-1] [INFO] [1685643785.791155944] [epson_node]: Sensor started...
```

## What does this ROS2 IMU Node Publish as messages?

The Epson IMU ROS2 driver will publish messages as convention per [REP 145](http://www.ros.org/reps/rep-0145.html).

- For IMU models such as G320/G354/G364/G370/V340, the IMU messages will only update the fields for angular rate (gyro) and linear acceleration (accel) data.
- For IMU models G330/G365/G366, it depends on the enabling/disabling of the internal attitude function with quaternion output:
  - IMU messages will only update *angular_velocity* (gyro) and *linear_acceleration* (accel) fields when quaternion output is *disabled*
  - IMU messages will update update *angular_velocity* (gyro), *linear_acceleration* (accel), and *orientation* field using the internal extended Kalman Filter when the quaternion output is *enabled*

### Without Quaternion Output

For non-quaternion output models, the ROS2 driver will publish to the following ROS topic:

```
/epson_imu/data_raw <-- orientation field will not contain valid data & should be ignored
```

#### ROS Topic Message *data_raw*

```
---
header:
  stamp:
    sec: 1685643703
    nanosec: 402378465
  frame_id: imu_link
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0
orientation_covariance:
- -1.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: -0.0002180843148380518
  y: 7.883128273533657e-05
  z: 6.580488116014749e-05
angular_velocity_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
linear_acceleration:
  x: 0.07670824229717255
  y: -0.056904129683971405
  z: -9.818550109863281
linear_acceleration_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
---
```

### With Quaternion Output

For quaternion output models, the ROS2 driver will publish to the following ROS topic:

```
/epson_imu/data <-- orientation, angular_velocity, linear_acceleration fields will be updating
```

#### ROS Topic Message data

```
---
header:
  stamp:
    sec: 1685644337
    nanosec: 660167310
  frame_id: imu_link
orientation:
  x: 0.9999865293502808
  y: -0.00023142248392105103
  z: 0.002697369083762169
  w: -0.004425106570124626
orientation_covariance:
- -1.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: 0.00032043419196270406
  y: -0.0005722396308556199
  z: 0.0002733084256760776
angular_velocity_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
linear_acceleration:
  x: 0.04799626022577286
  y: -0.07659434527158737
  z: -9.828039169311523
linear_acceleration_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
---
```

## Why am I seeing high latencies or slower than expected IMU data rates when using USB-UART bridges?

- If your connection between the Epson IMU UART and the Linux host is by FTDI (or similar USB-UART bridge devices,
  the default *latency_timer* setting in the FTDI driver may be too large (i.e. typically 16msec).
- There are 2 recommended methods to reduce this value to 1msec.

### Modifying *latency_timer* by sysfs mechanism

The example below reads the *latency_timer* setting for /dev/ttyUSB0 which returns 16msec.
Then, the *latency_timer* is set to 1msec, and confirmed by readback.

**NOTE**: May require root (sudo su) access on your system to modify.

```
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
16
echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
1
```

### Modifying *low_latency* flag using *setserial* utility

The example below sets the *low_latency* flag for /dev/ttyUSB0.
This will have the same effect as setting the *latency_timer* to 1msec.
This can be confirmed by running the *setserial* command again.

```
user@user-VirtualBox:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0

user@user-VirtualBox:~$ setserial /dev/ttyUSB0 low_latency

user@user-VirtualBox:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0, Flags: low_latency
```

## Package Contents

The Epson IMU ROS2 driver-related sub-folders & root files are:

```
launch/        <== various example launch files for Epson IMU models
src/           <== source code for ROS2 node C++ wrapper, IMU Linux C driver, and additional README_src.md specifically for building and using the IMU Linux C driver as stand-alone (without ROS support)
CHANGELOG.rst  <== summarizes major changes
CMakeLists.txt <== cmake build script for colcon
LICENSE.txt    <== description of the applicable licenses
package.xml    <== colcon package description
README.md      <== general README for the ROS2 driver
```

## License

Refer to *LICENSE.txt* in this package

## References

1. https://index.ros.org/doc/ros2/
2. https://github.com/technoroad/ADI_IMU_TR_Driver_ROS2
