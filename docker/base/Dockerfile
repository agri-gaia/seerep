# hadolint global ignore=DL3003
FROM ros:noetic

ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1
ENV DEBIAN_FRONTEND="noninteractive"
ENV HDF5_REL="https://github.com/BlueBrain/HighFive"
ENV HIGHFIVE_VER="2.4.0"
ENV FLATBUFFER_VER="2.0.0"
ENV GRPC_VER="1.35.0"
ENV PB_REL="https://github.com/protocolbuffers/protobuf/releases"
ENV PB_VER="21.12"
ENV GTEST_VER="1.12.0"
ENV DOX_REL="https://github.com/doxygen/doxygen/archive/refs/tags"
ENV DOX_VER="Release_1_9_3"
ENV CCACHE_VER="4.9.1"
ENV HADOLINT_VER="2.12.0"
ENV PIP3_VER="24.1.2"

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

#Install general required tools
RUN apt-get -qq update && apt-get -qq install -y --no-install-recommends -y \
        software-properties-common=0.99.9.12 \
        bash-completion=1:2.10-1ubuntu1 \
        autoconf=2.69-11.1 \
        automake=1:1.16.1-4ubuntu6 \
        wget=1.20.3-1ubuntu2.1 \
        curl=7.68.0-1ubuntu2.22 \
        cmake=3.16.3-1ubuntu1.20.04.1 \
        make=4.2.1-1.2 \
        unzip=6.0-25ubuntu1.2 \
        git=1:2.25.1-1ubuntu3.13 \
        gdb=9.2-0ubuntu1~20.04.2 \
        libcgal-dev=5.0.2-3 \
        sudo=1.8.31-1ubuntu1.5 \
        ssh=1:8.2p1-4ubuntu0.11 \
        vim=2:8.1.2269-1ubuntu5.23 \
        build-essential=12.8ubuntu1.1 \
        pkg-config=0.29.1-0ubuntu4 \
        libproj-dev=6.3.1-1 \
        python3-catkin-tools=0.9.2-1 \
        python3-osrf-pycommon=2.0.2-1 \
        ros-noetic-vision-msgs=0.0.2-1focal.20230620.191644 \
        ros-noetic-geographic-msgs=0.5.6-1focal.20230620.183241 \
        python3-pip=20.0.2-5ubuntu1.10 \
        flex=2.6.4-6.2 \
        bison=2:3.5.1+dfsg-1 \
        libjsoncpp-dev=1.7.4-3.1ubuntu2 \
        libcurl4-openssl-dev=7.68.0-1ubuntu2.22 \
        && rm -rf /var/lib/apt/lists/*


COPY requirements.dev.txt /tmp/requirements.dev.txt
COPY requirements.docs.txt /tmp/requirements.docs.txt

# Not sure why the path must be updated directly after upgrading pip.
# Setting it before via the Dockerfile ENV command does not work.

# Without updating the path, pip cannot find open3d versions > 0.13.0
# (likely due to the upgraded pip not getting found).

# Ignoring PyYAML is necessary because an upgrade of a package requires an uninstall of the old version.
# However, PyYAML is a distutil package, for which uninstall was deprecated with pip 10.0.0.
# With --ignore-installed, the uninstall is skipped and the new version is installd on top of the old one.
RUN pip3 install --no-cache-dir --upgrade pip==${PIP3_VER} \
        && PATH=/home/docker/.local/bin:${PATH} \
        && pip3 install -r /tmp/requirements.dev.txt --ignore-installed PyYAML --no-cache-dir \
        && pip3 install -r /tmp/requirements.docs.txt --ignore-installed PyYAML --no-cache-dir \
        && rm /tmp/requirements.dev.txt /tmp/requirements.docs.txt

#Install seerep dependencies from apt
RUN apt-get -qq update && apt-get -qq install -y --no-install-recommends -y \
        ros-noetic-tf=1.13.2-1focal.20231013.190129 \
        ros-noetic-vision-msgs=0.0.2-1focal.20230620.191644 \
        ros-noetic-pybind11-catkin=2.10.3-2focal.20230213.204608 \
        libgeographic-dev=1.50.1-1build1 \
        && rm -rf /var/lib/apt/lists/*

#Install pre-commit hooks to /root/.cache/pre-commit/
RUN wget --progress=dot:giga -O - https://apt.llvm.org/llvm-snapshot.gpg.key| apt-key add -
RUN add-apt-repository "deb http://apt.llvm.org/focal/ llvm-toolchain-focal-15 main"
RUN apt-get update -qq && apt-get -qq install -y --no-install-recommends \
                ruby=1:2.7+1 \
                shellcheck=0.7.0-2build2 \
                clang-format-15=1:15.0.7~++20231019083450+8dfdcc7b7bf6-1~exp1~20231019203526.112 \
                python3-catkin-lint=1.6.22-1~20.04 \
                && rm -rf /var/lib/apt/lists/*

#Install hadolint for pre-commit checks of Dockerfiles
RUN wget --progress=dot:giga -O /bin/hadolint https://github.com/hadolint/hadolint/releases/download/v${HADOLINT_VER}/hadolint-Linux-x86_64 \
        && chmod +x /bin/hadolint

####################################################
#Install CCACHE
####################################################
RUN wget --progress=dot:giga https://github.com/ccache/ccache/releases/download/v4.9.1/ccache-4.9.1.tar.gz \
        && tar -xf ccache-${CCACHE_VER}.tar.gz \
        && mkdir -p ccache-${CCACHE_VER}/build \
        && cd ccache-${CCACHE_VER}/build \
        && cmake -D CMAKE_BUILD_TYPE=Release .. \
        && make -j"$(nproc)" \
        && make install \
        && cd ../..  \
        && rm -rf ccache-${CCACHE_VER} ccache-${CCACHE_VER}.tar.gz

RUN ln -sf "$(which ccache)" /usr/local/bin/c++ && ln -sf "$(which ccache)" /usr/local/bin/cc

####################################################
#Install HighFive
####################################################
RUN apt-get -qq update && apt-get -qq install --no-install-recommends -y -o=Dpkg::Use-Pty=0 \
        libhdf5-dev=1.10.4+repack-11ubuntu1 \
        libboost-all-dev=1.71.0.0ubuntu2 \
        && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 -b v${HIGHFIVE_VER} $HDF5_REL.git \
        && mkdir HighFive/build \
        && cmake -DHIGHFIVE_EXAMPLES=OFF -B /HighFive/build -S /HighFive \
        && make -j"$(nproc)" -C /HighFive/build \
        && make install -C /HighFive/build \
        && rm -rf HighFive

####################################################
#Install Flatbuffers
####################################################
RUN wget --progress=dot:giga https://github.com/google/flatbuffers/archive/refs/tags/v2.0.0.tar.gz \
        && tar -xf v${FLATBUFFER_VER}.tar.gz \
        && mkdir flatbuffers-${FLATBUFFER_VER}/build \
        && cd flatbuffers-${FLATBUFFER_VER}/build \
        && cmake .. \
        && make -j"$(nproc)" -C /flatbuffers-${FLATBUFFER_VER}/build \
        && make install -C /flatbuffers-${FLATBUFFER_VER}/build \
        && cd ../.. \
        && rm -rf flatbuffers-${FLATBUFFER_VER} v${FLATBUFFER_VER}.tar.gz

####################################################
#Install gRPC
####################################################

#Clone the grpc repo
RUN git clone --recurse-submodules --depth 1 -b v${GRPC_VER} https://github.com/grpc/grpc \
        && mkdir -p grpc/cmake/build \
        && cd grpc/cmake/build \
        && cmake -DgRPC_INSTALL=ON -DgRPC_BUILD_TESTS=OFF ../.. \
        && make -j"$(nproc)" \
        && make install \
        && cd ../../.. \
        && rm -rf grpc

####################################################
#Install Protobuf
####################################################
RUN wget --progress=dot:giga $PB_REL/download/v$PB_VER/protobuf-cpp-3.$PB_VER.zip \
        && unzip protobuf-cpp-3.$PB_VER.zip \
        && cd /protobuf-3.$PB_VER/ \
        && mkdir -p build \
        && cd build \
        && cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        && make -j"$(nproc)" \
        && make install \
        && cd ../.. \
        && rm -rf protobuf-cpp-3.$PB_VER.zip protobuf-3.$PB_VER

####################################################
#Install GoogleTest
####################################################

RUN git clone https://github.com/google/googletest.git -b release-${GTEST_VER} \
        && cd googletest \
        && mkdir -p build \
        && cd build \
        && cmake .. \
        && make -j"$(nproc)" \
        && make install \
        && cd ../../ \
        && rm -rf googletest

####################################################
#Install Doxygen
####################################################

RUN wget --progress=dot:giga ${DOX_REL}/${DOX_VER}.tar.gz \
        && tar -xf ${DOX_VER}.tar.gz \
        && mkdir -p doxygen-${DOX_VER}/build \
        && cd doxygen-${DOX_VER}/build \
        && cmake -G "Unix Makefiles" .. \
        && make -j"$(nproc)" \
        && make install \
        && cd ../.. \
        && rm -rf doxygen-${DOX_VER} ${DOX_VER}.tar.gz

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

ENV PYTHONPATH="${PYTHONPATH}:/seerep/devel/lib:/seerep/devel/include/seerep_com/fbs/"

ENTRYPOINT [ "/bin/sh", "-c", "while sleep 1000; do :; done" ]
