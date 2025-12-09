FROM ubuntu:jammy

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update

RUN apt-get -y dist-upgrade

RUN apt-get -y --no-install-recommends install \
    build-essential \
    ca-certificates \
    curl \
    doxygen \
    g++ \
    gdb \
    git \
    gnupg \
    gpg \
    jq \
    less \
    libabsl-dev \
    libboost-all-dev \
    libgrpc++-dev \
    libprotobuf-dev \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    libgtk-3-dev \
    lsb-release \
    ninja-build \
    pkg-config \
    protobuf-compiler-grpc \
    software-properties-common \
    sudo \
    libjsoncpp-dev \
    wget

# Install CMake 3.x from Kitware's official repository
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | \
    gpg --dearmor - | \
    tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null && \
    echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/kitware.list >/dev/null && \
    apt-get update && \
    apt-get install -y --no-install-recommends cmake=3.30.* cmake-data=3.30.*


RUN bash -c 'wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|apt-key add -'
RUN apt-add-repository -y 'deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy-19 main'
RUN apt-add-repository -y 'deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy-19 main'
RUN apt-get update
RUN apt-get -y --no-install-recommends install -t llvm-toolchain-jammy-19 \
    clang-19 \
    clang-format-19 \
    clang-tidy-19 \
    clangd-19 \
    clang-tools-19 \
    lldb-19

RUN mkdir -p /root/opt/src

# Install Eigen
RUN apt install -y libeigen3-dev

# Install xtl 0.8.1 (required by xtensor, header-only)
RUN cd /root/opt/src && \
    git clone https://github.com/xtensor-stack/xtl.git && \
    cd xtl && \
    git checkout 0.8.1 && \
    cmake -S . -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -G Ninja && \
    cmake --build build -- -j4 && \
    cmake --install build --prefix /usr/local && \
    rm -rf /root/opt/src/xtl

# Install xtensor 0.27.1 (compatible with clang-19, header-only)
RUN cd /root/opt/src && \
    git clone https://github.com/xtensor-stack/xtensor.git && \
    cd xtensor && \
    git checkout 0.27.1 && \
    cmake -S . -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -G Ninja && \
    cmake --build build -- -j4 && \
    cmake --install build --prefix /usr/local && \
    rm -rf /root/opt/src/xtensor

# Install Viam C++ SDK from source. If you change the
# version here, change it in the top level CMakeLists.txt as well.
RUN cd /root/opt/src && \
    git clone https://github.com/viamrobotics/viam-cpp-sdk && \
    cd viam-cpp-sdk && \
    git checkout releases/v0.25.0 && \
    cmake -S . -B build \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DVIAMCPPSDK_USE_DYNAMIC_PROTOS=ON \
        -DVIAMCPPSDK_OFFLINE_PROTO_GENERATION=ON \
        -DVIAMCPPSDK_BUILD_EXAMPLES=OFF \
        -DVIAMCPPSDK_BUILD_TESTS=OFF \
        -G Ninja && \
    cmake --build build --target all -- -j4 && \
    cmake --install build --prefix /usr/local && \
    rm -rf /root/opt/src/viam-cpp-sdk
