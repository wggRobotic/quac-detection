FROM ros:humble
SHELL ["/bin/bash", "-c"]
WORKDIR /quac

RUN apt update
RUN apt install libopencv-dev build-essential -y

# ros2 package dependencies

RUN apt install -y ros-humble-rmw-cyclonedds-cpp

# quac

WORKDIR /quac

COPY ./quac-interfaces /quac/src/quac-interfaces
RUN . /opt/ros/humble/setup.bash && colcon build

COPY ./quac_detection /quac/src/quac_detection
RUN . /opt/ros/humble/setup.bash && . /quac/install/setup.bash && colcon build --cmake-args -DBUILD_QRCODE_DETECTION_SERVER=ON