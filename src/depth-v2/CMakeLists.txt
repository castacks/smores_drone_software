cmake_minimum_required(VERSION 3.8)
project(depth-v2)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})

#find_package(TensorRT REQUIRED)

set(TENSORRT_INCLUDE_DIR /usr/include/aarch64-linux-gnu/NvInfer)
set(TENSORRT_LIBRARY_DIR /usr/lib/aarch64-linux-gnu/)

include_directories(${TENSORRT_INCLUDE_DIR})
link_directories(${TENSORRT_LIBRARY_DIR})

add_executable(depth-v2-node src/depth_v2_node.cpp)
target_link_libraries(depth-v2-node nvinfer nvinfer_plugin ${OpenCV_LIBS} ${CUDA_LIBRARIES})

ament_target_dependencies(depth-v2-node rclcpp std_msgs)
install(TARGETS
        depth-v2-node
        DESTINATION lib/${PROJECT_NAME})

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  # comment the line when a copyright and license is added to all source files
#  set(ament_cmake_copyright_FOUND TRUE)
#  # the following line skips cpplint (only works in a git repo)
#  # comment the line when this package is in a git repo and when
#  # a copyright and license is added to all source files
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()
#
ament_package()
