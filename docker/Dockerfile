FROM ros:noetic

ENV DEBIAN_FRONTEND="noninteractive"
ENV HDF5_REL="https://github.com/BlueBrain/HighFive"
ENV PB_REL="https://github.com/protocolbuffers/protobuf/releases"
ENV PB_VER="3.15.6"

RUN echo '#Install other required tools'

RUN apt-get -qq update && apt-get -qq install -y -o=Dpkg::Use-Pty=0 \
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
        libpcl-dev \
        ros-noetic-pcl-conversions \
        ros-noetic-tf \
        libfile-pushd-perl

RUN echo '# Install pre-commit hooks to /root/.cache/pre-commit/'
RUN apt-get update -qq && apt-get install -y -qq --no-install-recommends \
    git \
    python3-pip \
    ruby shellcheck \
    clang-format-10 \
    python3-catkin-lint

RUN pip3 install --upgrade pip
RUN pip3 install pre-commit

RUN echo '####################################################'
RUN echo '#Install cmake'
RUN echo '####################################################'
RUN curl -s https://apt.kitware.com/kitware-archive.sh -o kitware-archive.sh
RUN chmod +x kitware-archive.sh
RUN /kitware-archive.sh
RUN apt-get update -q && apt-get install -qy cmake


RUN echo '####################################################'
RUN echo '#Install HighFive'
RUN echo '####################################################'
RUN apt-get -qq update && apt-get -qq install -y -o=Dpkg::Use-Pty=0 \
        libhdf5-dev \
        libhdf5-mpi-dev \
        libboost-all-dev \
        libeigen3-dev


RUN git clone --depth 1 $HDF5_REL.git
RUN mkdir HighFive/build
RUN cmake -DHIGHFIVE_EXAMPLES=OFF -DHIGHFIVE_PARALLEL_HDF5=ON -DHIGHFIVE_USE_EIGEN=ON -B /HighFive/build -S /HighFive
RUN make -j"$(nproc)" -C /HighFive/build
RUN make install -C /HighFive/build

RUN echo '#remove HighFive Code'
RUN rm -rf HighFive

RUN echo '####################################################'
RUN echo '#install protobuf via precompiled binaries from github'
RUN echo '####################################################'


RUN curl -LO $PB_REL/download/v$PB_VER/protobuf-cpp-$PB_VER.zip
RUN unzip protobuf-cpp-$PB_VER.zip
RUN cd /protobuf-$PB_VER/ && ./configure --prefix=/usr

RUN make -j"$(nproc)" -C /protobuf-$PB_VER/
RUN make check -j"$(nproc)" -C /protobuf-$PB_VER/
RUN make install -C /protobuf-$PB_VER/
RUN ldconfig

RUN echo '#remove protobuf code'
RUN rm protobuf-cpp-$PB_VER.zip
RUN rm -rf protobuf-$PB_VER

RUN echo '####################################################'
RUN echo '#install flatbuffer'
RUN echo '####################################################'
RUN wget https://github.com/google/flatbuffers/archive/refs/tags/v2.0.0.tar.gz
RUN tar -xf v2.0.0.tar.gz
RUN mkdir flatbuffers-2.0.0/build
RUN cd flatbuffers-2.0.0/build && cmake ..
RUN make -j"$(nproc)" -C /flatbuffers-2.0.0/build
RUN make install -C /flatbuffers-2.0.0/build

RUN rm -rf flatbuffers-2.0.0 v2.0.0.tar.gz

RUN echo '####################################################'
RUN echo '#install gRPC'
RUN echo '####################################################'

RUN echo '#Clone the grpc repo'
RUN git clone --recurse-submodules --depth 1 -b v1.35.0 https://github.com/grpc/grpc

RUN echo '#Build and install gRPC and Protocol Buffers'
RUN cd grpc && mkdir -p cmake/build
RUN cd grpc/cmake/build && cmake -DgRPC_INSTALL=ON -DgRPC_BUILD_TESTS=OFF ../..
RUN cd grpc/cmake/build && make -j"$(nproc)"
RUN cd grpc/cmake/build && make install

RUN echo '#remove gRPC code'
RUN rm -rf grpc

RUN echo '#Install python3 grpcio_tools'
RUN pip3 install grpcio-tools

RUN echo '#Updating python proto stuff'
RUN pip3 install --upgrade protobuf

# remove apt lists so that they are not saved in the image layers
RUN rm -rf /var/lib/apt/lists/*

# add non root user
RUN useradd -u 1001 --create-home --shell /bin/bash docker && echo "docker:docker" | chpasswd && adduser docker sudo

WORKDIR /home/docker/seerep_ws
RUN mkdir -p src/seerep

COPY seerep-com src/seerep/seerep-com
COPY seerep-hdf5 src/seerep/seerep-hdf5
COPY seerep-msgs src/seerep/seerep-msgs
COPY seerep-ros src/seerep/seerep-ros
COPY seerep-srv src/seerep/seerep-srv
COPY util src/seerep/util
COPY start_server.sh start_server.sh

RUN chown -R docker:docker /home/docker/seerep_ws

USER docker

RUN /bin/bash -c ". /opt/ros/noetic/setup.bash; catkin build"
RUN /bin/bash -c ". /home/docker/seerep_ws/devel/setup.bash"

EXPOSE 9090

ENTRYPOINT ["/bin/bash", "start_server.sh"]







