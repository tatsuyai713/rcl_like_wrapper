#!/bin/bash +x

VERSION=v1.66.1

GRPC_SOURCE_DIR=${PWD}/grpc

CMAKE_DIR=${GRPC_SOURCE_DIR}/cmake
GRPC_BUILD_DIR=${CMAKE_DIR}/build

INSTALL_DIR=/opt/grpc

BUILD_TYPE=Release
BUILD_PARALLEL=4

sudo apt-get update
sudo apt-get install ca-certificates gpg wget
test -f /usr/share/doc/kitware-archive-keyring/copyright ||
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null

echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ `lsb_release -cs` main" | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null

sudo apt update
sudo apt install -y libsystemd-dev cmake

# clone
rm -rf ${GRPC_SOURCE_DIR}
mkdir -p ${GRPC_SOURCE_DIR}
pushd ${GRPC_SOURCE_DIR}
git clone --recursive -b ${VERSION} https://github.com/grpc/grpc .
git submodule update --init --recursive
popd

sudo rm -rf /opt/grpc

# grpc
rm -rf ${GRPC_BUILD_DIR}
mkdir -p ${GRPC_BUILD_DIR}
pushd ${GRPC_BUILD_DIR}
cmake -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
      -DgRPC_BUILD_TESTS=OFF \
      -DgRPC_BUILD_CSHARP=OFF \
      -DgRPC_BUILD_GRPC_CSHARP_PLUGIN=OFF \
      -DgRPC_BUILD_GRPC_CPP_PLUGIN=ON \
      -DgRPC_BUILD_SHARED_LIBS=ON \
      -DgRPC_INSTALL=ON \
      -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
      -DBUILD_SHARED_LIBS=ON \
      -Dprotobuf_BUILD_SHARED_LIBS=ON \
      -DCMAKE_PREFIX_PATH=${INSTALL_DIR} \
      ${GRPC_SOURCE_DIR}

make -j${BUILD_PARALLEL}

sudo mkdir -p /opt/grpc
sudo make install
popd