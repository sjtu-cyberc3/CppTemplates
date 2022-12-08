FROM ros:melodic 
# install basic tools & requirements
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y software-properties-common apt-utils \
                          bash-completion openssh-server wget curl vim tmux sudo \
                          libgtest-dev libgoogle-glog-dev libyaml-cpp-dev libeigen3-dev libfmt-dev
# install gcc
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test \
    && apt-get update \
    && apt-get install -y gcc-9 g++-9 gcc-11 g++-11 gdb build-essential ninja-build \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 9 
# install cmake
RUN wget https://apt.kitware.com/kitware-archive.sh \
    && bash kitware-archive.sh \
    && apt-get install -y cmake \
    && rm kitware-archive.sh
# install llvm
RUN wget https://apt.llvm.org/llvm.sh \
    && bash llvm.sh 15 all \
    && rm llvm.sh 
ENV PATH="/usr/lib/llvm-15/bin:${PATH}"
# build gtest
RUN cd /usr/src/gtest \
    && cmake -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --parallel $(nproc) \
    && cp build/libg* /usr/lib/ \
    && rm -rf build
# clean up
RUN apt-get autoremove -y \
    && apt-get clean
# create user
ARG USERNAME=cyber
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
USER $USERNAME
# setup ros enviroments
RUN echo 'source /opt/ros/melodic/setup.bash' >> /home/$USERNAME/.bashrc