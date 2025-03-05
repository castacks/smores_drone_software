# README for Epson IMU Driver for ROS

## What is this repository for?

* This code provides interface between Epson IMU (current default=G365PDF0 or G320/G354/G364/G370/V340) and ROS.

* The epson_imu_driver_node.cpp is the C++ wrapper used to communicate with ROS

* The other source files in src/ are based on the C driver released by Epson:
  [Epson IMU UART-only Linux User-space Driver Example](https://vdc.epson.com/imu-products/imu-inertial-measurement-units)

## How do I use the driver?

1. Place the source files in its own directory in your catkin workspace src/ folder.
   Modify the CMakeLists.txt in this package to select the IMU model in the add_definitions -D macro assignment for gcc in Line 125
   - For example, "-DV340" for M-V340 
   - For example, "-DG364PDC0" for M-G364PDC0
   
2. Run "catkin_make" at the root of your catkin workspace to build or rebuild the driver after making any changes to
   the CMakeLists.txt or any of the .c or .cpp or .h source files.
   - NOTE: It is recommended to configure the parameters in the launch file, if possible, instead of modifying the source files directly

3. Modify the appropriate launch file in the launch/ folder to set your desired IMU environment and configure parameter options:
   - *serial port* in Line 7
   - *dout_rate* in Line 61
   - *filter_sel* in Line 95
   - "publisher topic" in Line 138 etc...

4. Run the appropriate launch file:
   i.e.
```   
roslaunch epson_imu_driver epson_g365.launch
```
   - The launch file contains parameters for configuring:
     - the type of data to output from the IMU
     - the update rate, etc. 
   - All parameters are described in the launch file.

*NOTE:*

When using G365 with *attitude output / quaternion orientation*, the X, Y, Z axis designation will differ from the labelling on the IMU marking.
This is necessary to adhere to ROS axis convention as defined ROS REP103.

IMU Marking   | ROS Orientation
------------- | ---------------
Y+            | X+ (Roll)
X-            | Y+ (Pitch)
Z+            | Z+ (Yaw)


### License ###
[This software is BSD-3 licensed.](http://opensource.org/licenses/BSD-3-Clause)

Original Code Development:
Copyright (c) 2019, Carnegie Mellon University. All rights reserved.

Additional Code contributed:
Copyright (c) 2019, Seiko Epson Corp. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.