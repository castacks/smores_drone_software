cmake_minimum_required(VERSION 3.5)
project(epson_imu_driver_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(rosidl_default_generators REQUIRED)


# Refer to the README_src.md inside the src folder for more details about IMU macro
# Uncomment the imu_model to build for
# NOTE: Change IMU models require rebuilding binaries with colcon
#set(IMU_MODEL "G320PDG0")
#set(IMU_MODEL "G330PDG0")
#set(IMU_MODEL "G354PDH0")
#set(IMU_MODEL "G364PDC0")
#set(IMU_MODEL "G364PDCA")
#set(IMU_MODEL "G365PDC1")
#set(IMU_MODEL "G365PDF1")
set(IMU_MODEL "G365PDF0")
#set(IMU_MODEL "G366PDG0")
#set(IMU_MODEL "G370PDF1")
#set(IMU_MODEL "G370PDS0")
#set(IMU_MODEL "G370PDG0")
#set(IMU_MODEL "G370PDT0")
#set(IMU_MODEL "V340PDD0")

add_definitions(-D${IMU_MODEL})
add_definitions(-DBUILD_FOR=\"${IMU_MODEL}\")
message(STATUS "-- Building for IMU Model: ${IMU_MODEL}")

# Specify DEBUG macro to enable any debug code by adding "-DDEBUG" in add_definitions()
#add_definitions(-DDEBUG)

# Create filelist macro of C sources for library
set(lib_sources
  src/hcl_gpio.c
  src/hcl_linux.c
  src/hcl_uart.c
  src/sensor_epsonCommon.c
  src/sensor_epsonUart.c
)

# Add IMU model specific C source to filelist macro
if (IMU_MODEL STREQUAL "G320PDG0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG320.c)
elseif (IMU_MODEL STREQUAL "G330PDG0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG330_G366.c)
elseif (IMU_MODEL STREQUAL "G354PDH0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG354.c)
elseif (IMU_MODEL STREQUAL "G364PDC0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG364.c)
elseif (IMU_MODEL STREQUAL "G364PDCA")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG364.c)
elseif (IMU_MODEL STREQUAL "G365PDC1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG365.c)
elseif (IMU_MODEL STREQUAL "G365PDF0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG365.c)
elseif (IMU_MODEL STREQUAL "G365PDF1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG365.c)
elseif (IMU_MODEL STREQUAL "G366PDG0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG330_G366.c)
elseif (IMU_MODEL STREQUAL "G370PDF1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG370.c)
elseif (IMU_MODEL STREQUAL "G370PDS0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG370.c)
elseif (IMU_MODEL STREQUAL "G370PDG0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG370.c)
elseif (IMU_MODEL STREQUAL "G370PDT0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG370.c)
elseif (IMU_MODEL STREQUAL "V340PDD0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonV340.c)
else()
  message(FATAL_ERROR "**** Invalid IMU Model")
endif()

# Declare static library for Epson IMU functions
add_library(ess_imu_ros2_uart_driver STATIC
  ${lib_sources}
)

# Declare C++ executable
add_executable(epson_imu_driver_ros2_node src/epson_imu_uart_node.cpp)

# Link Epson IMU C Library to the ROS node
target_link_libraries(epson_imu_driver_ros2_node
  ess_imu_ros2_uart_driver
)

# Link C++ Library to the ROS node
ament_target_dependencies(epson_imu_driver_ros2_node
  rclcpp
  std_msgs
  sensor_msgs
)

# Mark executables and/or libraries for installation
install(
   TARGETS epson_imu_driver_ros2_node
   DESTINATION lib/${PROJECT_NAME}
 )

# Mark directories for installation
install(
   DIRECTORY launch DESTINATION share/${PROJECT_NAME}
)

#ament_export_include_directories(include)
#ament_export_dependencies(std_msgs)
ament_package()
