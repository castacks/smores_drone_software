cmake_minimum_required(VERSION 3.8)
project(thermal-preprocessing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror -pedantic-errors)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

message(STATUS "Got OpenCV" + OPENCV_VERSION)

add_executable(thermal-preprocessing src/thermal-preprocessing-node.cpp)
ament_target_dependencies(thermal-preprocessing rclcpp std_msgs cv_bridge sensor_msgs OpenCV yaml-cpp)

target_include_directories(thermal-preprocessing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  target_include_directories(thermal-preprocessing PRIVATE
  ${YAML_CPP_INCLUDE_DIR}
)

target_link_libraries(thermal-preprocessing
  ${YAML_CPP_LIBRARIES}
)

target_compile_features(thermal-preprocessing PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS thermal-preprocessing
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()


