FROM ros:humble-ros-base

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libopencv-dev \
    libomp-dev \
    protobuf-compiler \
    libprotobuf-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Clone and build NCNN from source
WORKDIR /tmp
RUN git clone --depth 1 --branch 20240727 https://github.com/Tencent/ncnn.git && \
    cd ncnn && \
    git submodule update --init && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DNCNN_VULKAN=OFF \
          -DNCNN_BUILD_EXAMPLES=OFF \
          -DNCNN_BUILD_TOOLS=OFF \
          -DNCNN_BUILD_BENCHMARK=OFF \
          -DNCNN_SHARED_LIB=ON \
          -DNCNN_ARM_NEON=ON \
          .. && \
    make -j$(nproc) && \
    make install && \
    cd /tmp && rm -rf ncnn

# Set up workspace
WORKDIR /workspace

# Copy ROS2 workspace
COPY ros2_ws /workspace/ros2_ws

# Setup entrypoint script
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
