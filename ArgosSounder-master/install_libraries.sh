#!/bin/bash
HDF5_LIB=libhdf5-cpp-100
if [[ `lsb_release -rs` == "16.04" ]]
then
    HDF5_LIB=libhdf5-cpp-11
fi
sudo apt -y install python-scipy python3-scipy python-matplotlib python3-matplotlib libhdf5-dev $HDF5_LIB python-h5py python3-h5py

CMAKE_VERSION="$(cmake --version)"
if [[ "CMAKE_VERSION" < 3.8.0 ]]; then
    cd ~/Downloads
    wget https://cmake.org/files/v3.12/cmake-3.12.2-Linux-x86_64.sh
    sudo apt remove cmake
    chmod +x cmake-3.12.2-Linux-x86_64.sh
    sudo ./cmake-3.12.2-Linux-x86_64.sh # press Yes twice
    sudo mv cmake-3.12.2-Linux-x86_64 /opt/
    sudo ln -s /opt/cmake-3.12.2-Linux-x86_64/bin/* /usr/local/bin
fi
git clone https://github.com/nlohmann/json.git
cd json/
mkdir build/
cd build/
cmake ..
make -j
sudo make install

