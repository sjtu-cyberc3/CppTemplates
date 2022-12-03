FROM ros:melodic-ros-base-bionic
# Args for the non-root user
ARG USERNAME=cyber
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME
# Change repository to SJTU mirror
RUN sed -i 's/http:\/\/archive.ubuntu.com/http:\/\/mirror.sjtu.edu.cn/g' /etc/apt/sources.list
RUN sed -i 's/http:\/\/packages.ros.org/http:\/\/mirror.sjtu.edu.cn/g' /etc/apt/sources.list.d/ros1-latest.list
# Update and install general packages
RUN apt-get update && apt-get upgrade -y\
  && apt-get install -q -y openssh-client vim bash-completion tmux gdb wget
# Project related packages
RUN apt-get install -q -y libgoogle-glog-dev libeigen3-dev libyaml-cpp-dev
# Build and install gtest
WORKDIR /usr/src/gtest
RUN cmake .
RUN make
RUN mv libg* /usr/lib/
# External packages
CMD [ "/bin/bash" , "toolchains.sh"]
# Prepare for working dir
RUN chown -R $USERNAME /home/${USERNAME}
USER $USERNAME
RUN echo 'source /opt/ros/melodic/setup.bash' >> /home/$USERNAME/.bashrc
COPY cmake-tools-kits.json /home/${USERNAME}/.local/share/CMakeTools/
# Copy scripts to container
# COPY toolchains.sh /home/${USERNAME}/