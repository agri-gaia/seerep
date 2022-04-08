FROM ros:noetic

ENV DEBIAN_FRONTEND="noninteractive"
ENV HDF5_REL="https://github.com/BlueBrain/HighFive"
ENV PB_REL="https://github.com/protocolbuffers/protobuf/releases"
ENV PB_VER="3.15.6"

#Install general required tools
RUN apt-get -qq update && apt-get -qq install -y --no-install-recommends -y \
        bash-completion \
        autoconf \
        automake \
        libtool \
        curl \
        make \
        g++ \
        gdb \
        unzip \
        git \
        sudo \
        ssh \
        build-essential \
        pkg-config \
        python3-catkin-tools \
        python3-osrf-pycommon \
        libfile-pushd-perl \
        ros-noetic-vision-msgs \
        libgeographic-dev \
        libfile-pushd-perl \
        python3-pip \
        && rm -rf /var/lib/apt/lists/*
RUN pip3 install --upgrade pip
#install seerep dependencies from apt
RUN apt-get -qq update && apt-get -qq install -y --no-install-recommends -y \
        ros-noetic-tf \
        ros-noetic-vision-msgs \
        libgeographic-dev \
        && rm -rf /var/lib/apt/lists/*

# Install pre-commit hooks to /root/.cache/pre-commit/
RUN pip3 install pre-commit \
        && apt-get update -qq && apt-get -qq install -y --no-install-recommends \
                ruby shellcheck \
                clang-format-10 \
                python3-catkin-lint \
        && rm -rf /var/lib/apt/lists/*

####################################################
#Install cmake
####################################################
RUN curl -s https://apt.kitware.com/kitware-archive.sh -o kitware-archive.sh \
        && chmod +x kitware-archive.sh \
        && /kitware-archive.sh \
        && rm /kitware-archive.sh \
        && apt-get update -qq && apt-get -qq install --no-install-recommends -y cmake \
        && rm -rf /var/lib/apt/lists/*


####################################################
#Install HighFive
####################################################
RUN apt-get -qq update && apt-get -qq install --no-install-recommends -y -o=Dpkg::Use-Pty=0 \
        libhdf5-dev \
        libhdf5-mpi-dev \
        libboost-all-dev \
        libeigen3-dev \
        && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 $HDF5_REL.git \
        && mkdir HighFive/build \
        && cmake -DHIGHFIVE_EXAMPLES=OFF -DHIGHFIVE_PARALLEL_HDF5=ON -DHIGHFIVE_USE_EIGEN=ON -B /HighFive/build -S /HighFive \
        && make -j"$(nproc)" -C /HighFive/build \
        && make install -C /HighFive/build \
        && rm -rf HighFive

####################################################
#install protobuf via precompiled binaries from github
####################################################
RUN curl -LO $PB_REL/download/v$PB_VER/protobuf-cpp-$PB_VER.zip \
        && unzip protobuf-cpp-$PB_VER.zip \
        && cd /protobuf-$PB_VER/ \
        && ./configure --prefix=/usr \
        && make -j"$(nproc)" -C /protobuf-$PB_VER/ \
        && make check -j"$(nproc)" -C /protobuf-$PB_VER/ \
        && make install -C /protobuf-$PB_VER/ \
        && ldconfig \
        && cd .. \
        && rm protobuf-cpp-$PB_VER.zip \
        && rm -rf protobuf-$PB_VER
#Updating python proto stuff
RUN pip3 install --upgrade protobuf

####################################################
#install flatbuffer
####################################################
RUN wget https://github.com/google/flatbuffers/archive/refs/tags/v2.0.0.tar.gz \
        && tar -xf v2.0.0.tar.gz \
        && mkdir flatbuffers-2.0.0/build \
        && cd flatbuffers-2.0.0/build \
        && cmake .. \
        && make -j"$(nproc)" -C /flatbuffers-2.0.0/build \
        && make install -C /flatbuffers-2.0.0/build \
        && cd ../.. \
        && rm -rf flatbuffers-2.0.0 v2.0.0.tar.gz

####################################################
#install gRPC
####################################################

#Clone the grpc repo
RUN git clone --recurse-submodules --depth 1 -b v1.35.0 https://github.com/grpc/grpc \
        && mkdir -p grpc/cmake/build \
        && cd grpc/cmake/build \
        && cmake -DgRPC_INSTALL=ON -DgRPC_BUILD_TESTS=OFF ../.. \
        && make -j"$(nproc)" \
        && make install \
        && cd ../../.. \
        && rm -rf grpc

#Install python3 grpcio_tools'
RUN pip3 install grpcio-tools

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
        && echo "source /opt/ros/noetic/setup.zsh" >> /root/.zshrc \
        && echo "[[ -e /seerep/devel/setup.bash ]] && source /seerep/devel/setup.bash" >> /root/.bashrc \
        && echo "[[ -e /seerep/devel/setup.zsh ]] && source /seerep/devel/setup.zsh" >> /root/.zshrc

RUN echo 'export PYTHONPATH="${PYTHONPATH}:/seerep/build/seerep-msgs:/seerep/build/seerep-com"' >> /root/.bashrc

# add non root user
RUN useradd -u 1000 --create-home --shell /bin/bash docker && echo "docker:docker" | chpasswd && adduser docker sudo

RUN mkdir -p /home/docker/.vscode-server/extensions \
    && chown -R docker /home/docker/.vscode-server \
    && mkdir -p /seerep \
    && chown -R docker /seerep

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/docker/.bashrc \
        && echo "source /opt/ros/noetic/setup.zsh" >> /home/docker/.zshrc \
        && echo "[[ -e /seerep/devel/setup.bash ]] && source /seerep/devel/setup.bash" >> /home/docker/.bashrc \
        && echo "[[ -e /seerep/devel/setup.zsh ]] && source /seerep/devel/setup.zsh" >> /home/docker/.zshrc

RUN echo 'export PYTHONPATH="${PYTHONPATH}:/seerep/build/seerep-msgs:/seerep/build/seerep-com"' >> /home/docker/.bashrc

ENTRYPOINT [ "/bin/sh", "-c", "while sleep 1000; do :; done" ]