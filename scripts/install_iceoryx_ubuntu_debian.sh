#!/bin/bash

iceoryx_version="2.0.6"

ICEORYX_WORK_DIR=./iceoryx_build

sudo apt update
sudo apt install -y gcc g++ cmake libacl1-dev libncurses5-dev pkg-config


sudo rm -rf ${ICEORYX_WORK_DIR}

mkdir ${ICEORYX_WORK_DIR}

cd ${ICEORYX_WORK_DIR}
wget https://github.com/eclipse-iceoryx/iceoryx/archive/refs/tags/v${iceoryx_version}.tar.gz
tar -xvf v${iceoryx_version}.tar.gz
cd iceoryx-${iceoryx_version}
WORKSPACE=$PWD

INSTALL_PATH=/opt/iceoryx
sudo rm -rf $INSTALL_PATH
sudo mkdir $INSTALL_PATH

cmake -Bbuild -Hiceoryx_meta -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH
cmake --build build 
sudo cmake --build build --target install

sed -i -e '/export PATH=$PATH:\/opt\/iceoryx\/bin/d' ~/.bashrc
echo 'export PATH=$PATH:/opt/iceoryx/bin' >> ~/.bashrc

if grep 'export LD_LIBRARY_PATH=/opt/iceoryx/lib:$LD_LIBRARY_PATH' ~/.bashrc >/dev/null; then
  echo "LD_LIBRARY_PATH libs are already added"
else
  echo 'export LD_LIBRARY_PATH=/opt/iceoryx/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
  source ~/.bashrc
fi
sudo ldconfig
