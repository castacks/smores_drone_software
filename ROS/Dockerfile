FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu20.04

RUN useradd -ms /bin/bash user && \
    usermod -aG sudo user

RUN apt-get update
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y gnupg2 curl lsb-core vim wget python3-pip libpng16-16 libjpeg-turbo8 libtiff5

# Installing ROS-noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt update
RUN apt-get install -y ros-noetic-desktop \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

RUN rosdep init && rosdep update && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Intalling python-catkin
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get update && \
    apt-get install -y python3-catkin-tools \
                       software-properties-common \
                        # Base tools
                        cmake \
                        build-essential \
                        git \
                        unzip \
                        pkg-config \
                        python3-dev \
                        # OpenCV dependencies
                        python3-numpy \
                        # Pangolin dependencies
                        libgl1-mesa-dev \
                        libglew-dev \
                        libpython3-dev \
                        libeigen3-dev \
                        apt-transport-https \
                        ca-certificates\
                        software-properties-common \
                        python3-dev \
                        python3-numpy \
                        python-dev \
                        python-numpy \
                        libavcodec-dev \
                        libavformat-dev \
                        libswscale-dev \
                        libgstreamer-plugins-base1.0-dev \
                        libgstreamer1.0-dev \
                        libgtk-3-dev \
                        libsuitesparse-dev \
                        libboost-all-dev \
                        libyaml-cpp-dev \
                        libtbb-dev \
                        libcanberra-gtk-module \
                        libcanberra-gtk3-module \
                        ros-noetic-hector-trajectory-server

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bash_profile

RUN cd /tmp && git clone https://github.com/opencv/opencv.git && \
    cd opencv && \
    git checkout 4.4.0 && \
    mkdir build && cd build && \
    cmake -j4 -D CMAKE_BUILD_TYPE=Release \
                -D BUILD_EXAMPLES=OFF  \
                -D BUILD_DOCS=OFF \
                -D BUILD_PERF_TESTS=OFF \
                -D BUILD_TESTS=OFF \
                -D CMAKE_BUILD_TYPE=Release \
                -D WITH_CUDA=OFF \
                -D WITH_GTK=ON \
                -D WITH_GTK3=ON \
                -D WITH_GTK_2_X=OFF \
                -DENABLE_PRECOMPILED_HEADERS=OFF \
                -D CMAKE_INSTALL_PREFIX=/usr/local  .. && \
    make -j16 && make install && \
    cd / && rm -rf /tmp/opencv

USER user
