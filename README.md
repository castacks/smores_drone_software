# Local Source Build for ROS2 Humble

This is a local source build for ROS2 humble with support for:
- OpenCV
- PCL Library
- Image compression
The build is made for MRSD lab system running fedora with gnu 14 but should be extendable to Orin.

## Steps to build

1. Clone the repository
        
        ```
        cd ~ 

        git clone -b Ros2_humble_source_build https://github.com/castacks/smores_drone_software.git

        mv smores_drone_software ros2_humble
        ```

2. Install dependencies:

        ```
        sudo dnf install cmake \
        cppcheck \
        eigen3-devel \
        gcc-c++ \
        liblsan 
        libXaw-devel \
        libyaml-devel \
        make \
        opencv-devel \
        patch \
        python3-colcon-common-extensions \
        python3-coverage \
        python3-devel \
        python3-empy \
        python3-nose \
        python3-pip \
        python3-pydocstyle \
        python3-pyparsing \
        python3-pytest \
        python3-pytest-cov \
        python3-pytest-mock \
        python3-pytest-runner \
        python3-rosdep \
        python3-setuptools \
        python3-vcstool \
        poco-devel \
        poco-foundation \
        python3-flake8 \
        python3-flake8-import-order \
        redhat-rpm-config \
        uncrustify \
        wget

        ```

3. Setup rosdep:

        ```
        sudo rosdep init
        
        rosdep update
        
        rosdep install --from-paths src --ignore-src -y --skip-keys "asio cyclonedds fastcdr fastrtps ignition-cmake2 ignition-math6 python3-babeltrace python3-mypy rti-connext-dds-6.0.1 urdfdom_headers"

        sudo dnf install opencv opencv-devel boost-devel python3-numpy
        
        ```

4. Setup environment variables:

        ```
        pip install empy==3.3.4
        export RPM_ARCH=$(uname -m) 
        export RPM_PACKAGE_RELEASE=1.0 
        export RPM_PACKAGE_VERSION=1.0 
        export RPM_PACKAGE_NAME=qt_gui_cpp

        ```
5. Build ROS:

        ```
        cd ~/ros2_humble/
        colcon build --symlink-install --cmake-args -DTHIRDPARTY_Asio=ON -DPython3_EXECUTABLE=/usr/bin/python3 --no-warn-unused-cli

        ```

4. Add local source to bashrc:

        ```
        echo '. ~/ros2_humble/install/local_setup.bash' >> ~/.bashrc

        ```
## obtaining updated core source files
        ```
        vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src
        
        ```
