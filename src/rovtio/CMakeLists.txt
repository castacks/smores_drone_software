cmake_minimum_required(VERSION 3.14.4)
project(rovtio)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()
set(ROVIO_NMAXFEATURE 25 CACHE STRING "Number of feature for ROVIO")
set(ROVIO_NCAM 1 CACHE STRING "Number of enabled cameras")
set(ROVIO_NLEVELS 4 CACHE STRING "Number of image leavels for the features")
set(ROVIO_PATCHSIZE 6 CACHE STRING "Size of patch (edge length in pixel)")
set(ROVIO_NPOSE 0 CACHE STRING "Additional estimated poses for external pose measurements")
add_definitions(-DROVIO_NMAXFEATURE=${ROVIO_NMAXFEATURE})
add_definitions(-DROVIO_NCAM=${ROVIO_NCAM})
add_definitions(-DROVIO_NLEVELS=${ROVIO_NLEVELS})
add_definitions(-DROVIO_PATCHSIZE=${ROVIO_PATCHSIZE})
add_definitions(-DROVIO_NPOSE=${ROVIO_NPOSE})


if (0)
    find_package(OpenMP REQUIRED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc features2d)

if (MAKE_SCENE)
    message(STATUS "Building ROVTIO with openGL Scene Visualization")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAKE_SCENE=1")
    find_package(GLEW REQUIRED)
    include_directories(${GLEW_INCLUDE_DIRS})

    find_package(GLUT REQUIRED)
    include_directories(${GLUT_INCLUDE_DIRS})
    link_directories(${GLUT_LIBRARY_DIRS})
    add_definitions(${GLUT_DEFINITIONS})

    find_package(OpenGL REQUIRED)
    include_directories(${OpenGL_INCLUDE_DIRS})
    link_directories(${OpenGL_LIBRARY_DIRS})
    add_definitions(${OpenGL_DEFINITIONS})
endif ()

find_package(PkgConfig)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)        # replaces ROS1 tf
find_package(tf2_ros REQUIRED)    # if tf2 buffer or broadcaster needed
find_package(tf2_geometry_msgs REQUIRED)    # if tf2 buffer or broadcaster needed
find_package(yaml-cpp REQUIRED)   # use system yaml-cpp library
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc features2d)
# add_subdirectory(lightweight_filtering)
find_package(lightweight_filtering REQUIRED)
message(STATUS "lightweight_filtering_INCLUDE_DIRS = ${lightweight_filtering_INCLUDE_DIRS}")

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kindr REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/SrvResetToPose.srv"
        DEPENDENCIES std_msgs geometry_msgs
)

if (MAKE_SCENE)
    add_library(${PROJECT_NAME}_core SHARED src/Camera.cpp src/FeatureCoordinates.cpp src/FeatureDistance.cpp src/Scene.cpp)
else ()
    add_library(${PROJECT_NAME}_core SHARED src/Camera.cpp src/FeatureCoordinates.cpp src/FeatureDistance.cpp)
endif ()
target_link_libraries(${PROJECT_NAME}_core PUBLIC ${OpenMP_EXE_LINKER_FLAGS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIBRARY} ${OpenCV_LIBRARIES}
        rclcpp::rclcpp
        ${sensor_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${visualization_msgs_TARGETS}
        ${tf2_geometry_msgs_TARGETS}
        cv_bridge::cv_bridge
        tf2::tf2
        tf2_ros::tf2_ros
        yaml-cpp)

target_include_directories(${PROJECT_NAME}_core PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" ${lightweight_filtering_INCLUDE_DIRS} ${kindr_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/rovio_node.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>" ${lightweight_filtering_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)
install(TARGETS ${PROJECT_NAME}_core
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
