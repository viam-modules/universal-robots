FROM debian:bookworm

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update

RUN apt-get -y dist-upgrade

RUN apt-get -y --no-install-recommends install \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    doxygen \
    g++ \
    gdb \
    git \
    gnupg \
    gpg \
    less \
    libabsl-dev \
    libboost-all-dev \
    libgrpc++-dev \
    libprotobuf-dev \
    libssl-dev \
    libxtensor-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    libgtk-3-dev \
    ninja-build \
    pkg-config \
    protobuf-compiler-grpc \
    software-properties-common \
    sudo \
    wget


RUN bash -c 'wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|apt-key add -'
RUN apt-add-repository -y 'deb http://apt.llvm.org/bookworm/ llvm-toolchain-bookworm-15 main'
RUN apt-add-repository -y 'deb http://apt.llvm.org/bookworm/ llvm-toolchain-bookworm-15 main'
RUN apt-get update
RUN apt-get -y --no-install-recommends install -t llvm-toolchain-bookworm-15 \
    clang-15 \
    clang-format-15 \
    clang-tidy-15

RUN mkdir -p /root/opt/src

# Install appimage-builder deps
RUN apt install -y \
    binutils \
    coreutils \
    desktop-file-utils \
    fakeroot \
    fuse \
    libgdk-pixbuf2.0-dev \
    patchelf \
    python3-pip python3-setuptools \
    squashfs-tools \
    strace \
    vim \
    util-linux zsync

RUN pip3 install -U pip setuptools urllib3==1.26.12 requests==2.26.0 --break-system-packages

# Install appimage-builder
RUN pip3 install --break-system-packages git+https://github.com/AppImageCrafters/appimage-builder.git@61c8ddde9ef44b85d7444bbe79d80b44a6a5576d

# Install Go
RUN apt install -y golang-go

# Install GTest
RUN apt install -y libgtest-dev

# Install Eigen
RUN apt install -y libeigen3-dev

# Install Viam C++ SDK from source. If you change the
# version here, change it in the top level CMakeLists.txt as well.
RUN cd /root/opt/src && \
    git clone https://github.com/viamrobotics/viam-cpp-sdk && \
    cd viam-cpp-sdk && \
    git checkout releases/v0.13.2 && \
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
