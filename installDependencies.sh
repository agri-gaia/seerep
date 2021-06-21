#! /bin/bash

echo '#enable noninteractive installs'
export DEBIAN_FRONTEND="noninteractive"

echo '#Install other required tools'
apt-get -qq update && apt-get -qq install -y -o=Dpkg::Use-Pty=0 \
        autoconf \
        automake \
        libtool \
        curl \
        make \
        g++ \
        unzip \
        git \
        sudo \
        ssh \
        build-essential \
        pkg-config \
        python3-catkin-tools \
        python3-osrf-pycommon

echo '# Install pre-commit hooks to /root/.cache/pre-commit/'
apt-get update -qq \
    && apt-get install -y -qq --no-install-recommends git python3-pip ruby shellcheck clang-format-10 python3-catkin-lint \
    && rm -rf /var/lib/apt/lists/*
pip3 install pre-commit
cd /tmp/pre-commit \
    && git init \
    && pre-commit install-hooks \
    && rm -rf /tmp/pre-commit

cd /home/docker || exit 1


echo '####################################################'
echo '#Install cmake'
echo '####################################################'
apt-get update && apt-get install -qy apt-transport-https ca-certificates gnupg software-properties-common wget
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main'
apt-get update && apt-get install -qy kitware-archive-keyring && rm /etc/apt/trusted.gpg.d/kitware.gpg
apt-get update && apt-get install -qy cmake


echo '####################################################'
echo '#Install HighFive'
echo '####################################################'
apt-get -qq update && apt-get -qq install -y -o=Dpkg::Use-Pty=0 \
        libhdf5-dev \
        libhdf5-mpi-dev \
        libboost-all-dev \
        libeigen3-dev
HDF5_REL="https://github.com/BlueBrain/HighFive"
# HDF5_VER="2.2.2"
# curl -LO $HDF5_REL/archive/refs/tags/v$HDF5_VER.zip
# unzip v$HDF5_VER.zip
# mkdir HighFive-$HDF5_VER/build
# cd HighFive-$HDF5_VER/build
git clone --depth 1 $HDF5_REL.git
mkdir HighFive/build
cd HighFive/build || exit 1
cmake .. -DHIGHFIVE_EXAMPLES=OFF -DHIGHFIVE_PARALLEL_HDF5=ON -DHIGHFIVE_USE_EIGEN=ON
make -j"$(nproc)"
make install

echo '#remove HighFive Code'
cd ../..
# rm v$HDF5_VER.zip
# rm -rf HighFive-$HDF5_VER
rm -rf HighFive

echo '####################################################'
echo '#install protobuf via precompiled binaries from github'
echo '####################################################'
PB_REL="https://github.com/protocolbuffers/protobuf/releases"
PB_VER="3.15.6"
curl -LO $PB_REL/download/v$PB_VER/protobuf-cpp-$PB_VER.zip
unzip protobuf-cpp-$PB_VER.zip
cd protobuf-$PB_VER || exit 1
./configure --prefix=/usr

make -j"$(nproc)"
make check -j"$(nproc)"
make install
ldconfig # refresh shared library cache.

echo '#remove protobuf code'
cd .. || exit 1
rm protobuf-cpp-$PB_VER.zip
rm -rf protobuf-$PB_VER


echo '####################################################'
echo '#install gRPC'
echo '####################################################'

#non-local install dirs (in docker installed globally)
#create install dir and add to path
#export GRPC_INSTALL_DIR=/grpc
#mkdir ${GRPC_INSTALL_DIR}
#export PATH="$PATH:$GRPC_INSTALL_DIR/bin"

echo '#Clone the grpc repo'
git clone --recurse-submodules --depth 1 -b v1.35.0 https://github.com/grpc/grpc

echo '#Build and install gRPC and Protocol Buffers'
cd grpc || exit 1
mkdir -p cmake/build
pushd cmake/build || exit 1
cmake -DgRPC_INSTALL=ON \
    -DgRPC_BUILD_TESTS=OFF \
    ../..
make -j"$(nproc)"
make install
popd || exit 1

echo '#remove gRPC code'
cd .. || exit 1
rm -rf grpc

# remove apt lists so that they are not saved in the image layers
rm -rf /var/lib/apt/lists/*
