#!/bin/bash
sudo apt update

# install GCC9
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install -y gcc-9 g++-9 build-essential
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 9

# install latest cmake
# sudo bash -c "$(wget -O - https://apt.kitware.com/kitware-archive.sh)"
# sudo apt install -y cmake cmake-qt-gui

# install latest clangd
sudo bash -c "$(wget -O - https://apt.llvm.org/llvm.sh)"
sudo update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-15 100

#install latest doxygen
sudo apt install -y doxygen