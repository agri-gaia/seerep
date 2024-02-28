FROM ros:noetic

ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1
ENV DEBIAN_FRONTEND="noninteractive"
ENV HDF5_REL="https://github.com/BlueBrain/HighFive"
ENV PB_REL="https://github.com/protocolbuffers/protobuf/releases"
ENV PB_VER="21.12"
ENV DOX_REL="https://github.com/doxygen/doxygen/archive/refs/tags"
ENV DOX_VER="Release_1_9_3"

#Install general required tools
RUN apt-get -qq update && apt-get -qq install -y --no-install-recommends -y \
        software-properties-common \
        bash-completion \
        autoconf \
        automake \
        libtool \
        wget \
        curl \
        make \
        g++ \
        gdb \
        unzip \
        git \
        libcgal-dev \
        sudo \
        ssh \
        vim \
        build-essential \
        pkg-config \
        libproj-dev \
        python3-catkin-tools \
        python3-osrf-pycommon \
        libfile-pushd-perl \
        ros-noetic-vision-msgs \
        ros-noetic-geographic-msgs \
        libgeographic-dev \
        libfile-pushd-perl \
        python3-pip \
        flex \
        bison \
        libjsoncpp-dev \
        libcurl4-openssl-dev \
        && rm -rf /var/lib/apt/lists/*


COPY requirements.txt /tmp/requirements.txt

# Not sure why the path must be updated directly after upgrading pip.
# Setting it before via the Dockerfile ENV command does not work.

# Without updating the path, pip cannot find open3d versions > 0.13.0
# (likely due to the upgraded pip not getting found).

# Ignoring PyYAML is necessary because an upgrade of a package requires an uninstall of the old version.
# However, PyYAML is a distutil package, for which uninstall was deprecated with pip 10.0.0.
# With --ignore-installed, the uninstall is skipped and the new version is installd on top of the old one.
RUN pip3 install --upgrade pip \
        && PATH=/home/docker/.local/bin:${PATH} \
        && pip3 install -r /tmp/requirements.txt --ignore-installed PyYAML --no-cache-dir \
        && rm /tmp/requirements.txt

        #Install seerep dependencies from apt
RUN apt-get -qq update && apt-get -qq install -y --no-install-recommends -y \
        ros-noetic-tf \
        ros-noetic-vision-msgs \
        ros-noetic-pybind11-catkin \
        libgeographic-dev \
        && rm -rf /var/lib/apt/lists/*

#Install pre-commit hooks to /root/.cache/pre-commit/
RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add -
RUN add-apt-repository "deb http://apt.llvm.org/focal/ llvm-toolchain-focal-15 main"
RUN apt-get update -qq && apt-get -qq install -y --no-install-recommends \
                ruby shellcheck \
                clang-format-15 \
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
        && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 -b v2.4.0 $HDF5_REL.git \
        && mkdir HighFive/build \
        && cmake -DHIGHFIVE_EXAMPLES=OFF -B /HighFive/build -S /HighFive \
        && make -j"$(nproc)" -C /HighFive/build \
        && make install -C /HighFive/build \
        && rm -rf HighFive

####################################################
#Install flatbuffer
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
#Install gRPC
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

####################################################
#Install protobuf from github
####################################################
RUN curl -LO $PB_REL/download/v$PB_VER/protobuf-cpp-3.$PB_VER.zip \
        && unzip protobuf-cpp-3.$PB_VER.zip \
        && cd /protobuf-3.$PB_VER/ \
        && mkdir -p build \
        && cd build \
        && cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        && make -j"$(nproc)" \
        && make install \
        && cd ../.. \
        && rm protobuf-cpp-3.$PB_VER.zip \
        && rm -rf protobuf-$3.PB_VER

####################################################
#Install current googletest release
####################################################

RUN git clone https://github.com/google/googletest.git -b release-1.12.0 \
        && cd googletest \
        && mkdir -p build \
        && cd build \
        && cmake .. \
        && make -j"$(nproc)" \
        && make install \
        && cd ../../ \
        && rm -rf googletest

####################################################
#Install documentation dependencies
####################################################

#Install doxygen
RUN wget ${DOX_REL}/${DOX_VER}.tar.gz \
        && tar -xf ${DOX_VER}.tar.gz \
        && mkdir -p doxygen-${DOX_VER}/build \
        && cd doxygen-${DOX_VER}/build \
        && cmake -G "Unix Makefiles" .. \
        && make -j"$(nproc)" \
        && make install \
        && cd ../.. \
        && rm -rf doxygen-${DOX_VER} ${DOX_VER}.tar.gz

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
        && echo "source /opt/ros/noetic/setup.zsh" >> /root/.zshrc \
        && echo "[[ -e /seerep/devel/setup.bash ]] && source /seerep/devel/setup.bash" >> /root/.bashrc \
        && echo "[[ -e /seerep/devel/setup.zsh ]] && source /seerep/devel/setup.zsh" >> /root/.zshrc

RUN echo 'export PYTHONPATH="${PYTHONPATH}:/seerep/build/seerep_msgs:/seerep/build/seerep_com"' >> /root/.bashrc

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

RUN echo 'export PYTHONPATH="${PYTHONPATH}:/seerep/devel/lib:/seerep/devel/include/seerep_com/fbs/"' >> /home/docker/.bashrc

ENTRYPOINT [ "/bin/sh", "-c", "while sleep 1000; do :; done" ]
