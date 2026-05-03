FROM ultralytics/ultralytics:latest-jetson-jetpack6
SHELL ["/bin/bash", "-c"]
WORKDIR /quac

# install ros2 humble

RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN apt install -y software-properties-common
RUN add-apt-repository universe

RUN apt update && apt install curl -y
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb

RUN apt update
RUN apt upgrade -y
RUN DEBIAN_FRONTEND=noninteractive apt install -y tzdata && \
    ln -fs /usr/share/zoneinfo/Europe/Berlin /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata
RUN apt install -y ros-humble-ros-base
RUN apt install -y ros-dev-tools

# ros2 package dependencies

RUN apt install -y ros-humble-rmw-cyclonedds-cpp

# quac

WORKDIR /quac

COPY ./quac-interfaces /quac/src/quac-interfaces
RUN . /opt/ros/humble/setup.bash && colcon build

COPY ./quac_detection /quac/src/quac_detection
RUN . /opt/ros/humble/setup.bash && . /quac/install/setup.bash && colcon build --cmake-args -DBUILD_YOLO_DETECTION_SERVER=ON
