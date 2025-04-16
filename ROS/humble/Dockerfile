FROM nvcr.io/nvidia/pytorch:24.05-py3
ENV LANG=en_US.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

ARG ssh_prv_key
ARG ssh_pub_key

RUN apt-get update && apt-get install -y software-properties-common curl locales && \
    add-apt-repository universe && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt-get update && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y ros-humble-desktop-full ros-humble-cv-bridge ros-humble-foxglove-bridge x11-apps libeigen3-dev libceres-dev git \
    build-essential ros-dev-tools unzip ninja-build libgtk2.0-dev pkg-config libgtk2.0-dev libgtk-3-dev libcanberra-gtk-module libcanberra-gtk3-module libxkbcommon-x11-0 vulkan-tools

RUN mkdir /opencv-build-temp && cd /opencv-build-temp && \
    wget -O /opencv-build-temp/opencv.zip https://github.com/opencv/opencv/archive/4.x.zip && \
    wget -O /opencv-build-temp/opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip && \
    unzip /opencv-build-temp/opencv.zip && \
    unzip /opencv-build-temp/opencv_contrib.zip && \
    mkdir -p /opencv-build-temp/build && cd /opencv-build-temp/build && \
    cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x -GNinja \
    -D CMAKE_BUILD_TYPE=Release \
    -D BUILD_EXAMPLES=OFF  \
    -D BUILD_DOCS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D BUILD_TESTS=OFF \
    -D CMAKE_BUILD_TYPE=Release \
    -D WITH_CUDA=OFF \
    -D WITH_GTK=ON \
    -D WITH_GTK3=ON \
    -D WITH_GTK_2_X=OFF \
    -D BUILD_QT=ON \
    -D BUILD_JAVA=OFF \
    -DENABLE_PRECOMPILED_HEADERS=OFF \
    -D BUILD_opencv_python3=ON && \
    ninja install && \
    cd / && rm -rf /opencv-build-temp/

RUN mkdir -p /root/.ssh && \
        chmod 700 /root/.ssh  && \
        echo "$ssh_prv_key" > /root/.ssh/id_rsa && \
        echo "$ssh_pub_key" > /root/.ssh/id_rsa.pub && \
        chmod 600 /root/.ssh/id_rsa && \
        chmod 600 /root/.ssh/id_rsa.pub  && \
        cd /workspace && \
        mkdir smores_drone_software && \
        echo "StrictHostKeyChecking no" >> /etc/ssh/ssh_config


############################ UNCOMMENT BELOW FOR MAST3R #######################################

# RUN git clone --recursive https://github.com/naver/mast3r --branch mast3r_sfm /mast3r
# WORKDIR /mast3r/dust3r
# RUN pip install -r requirements.txt #&& pip install -r requirements_optional.txt

# WORKDIR /mast3r/dust3r/croco/models/curope/
# RUN python setup.py build_ext --inplace


# WORKDIR /mast3r
# RUN pip install -r requirements.txt
# RUN mkdir -p checkpoints/ && \
#     wget https://download.europe.naverlabs.com/ComputerVision/MASt3R/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth -P checkpoints/ && \
#     wget https://download.europe.naverlabs.com/ComputerVision/MASt3R/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric_retrieval_trainingfree.pth -P checkpoints/ && \
#     wget https://download.europe.naverlabs.com/ComputerVision/MASt3R/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric_retrieval_codebook.pkl -P checkpoints/

############################ UNCOMMENT ABOVE FOR MAST3R #######################################

# ############################ UNCOMMENT BELOW FOR MOGE + MADPOSE #######################################
RUN apt-get remove -y python3-blinker 
    # cd /workspace && \
    # wget -O model.pt https://huggingface.co/Ruicheng/moge-vitl/resolve/main/model.pt?download=true
# ############################ UNCOMMENT ABOVE FOR MOGE + MADPOSE #######################################


########################### UNCOMMENT FOR MAPPING #############################
RUN apt-get -y install libosmesa6-dev
###############################################################################

# RUN git clone https://github.com/jenicek/asmk /asmk
# RUN pip install cython faiss-gpu
# WORKDIR /asmk/cython
# RUN cythonize *.pyx

# WORKDIR /asmk
# RUN pip install .

# COPY no-push/.Xauthority /root/.Xauthority
COPY ROS/humble/ros_entrypoint.sh /
COPY pip-requirements.txt /workspace
RUN pip3 install -r pip-requirements.txt && \
    pip3 install git+https://github.com/EasternJournalist/utils3d.git#egg=utils3d && \
    chmod +x /ros_entrypoint.sh && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "export ROS_WS_DIR=/workspace/smores_drone_software" >> ~/.bashrc
    
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
