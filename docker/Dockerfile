FROM ros:jazzy-ros-base

ARG USER_ID
ARG GROUP_ID

ENV DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    build-essential \
    cmake \
    git \
    gosu \
    python3-colcon-common-extensions \
    libeigen3-dev \
    libpcl-dev \
    nano \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-ros-gz \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-pcl* \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-eigen \
    ros-jazzy-rviz2

RUN touch /var/lib/lastlog && \
    if getent passwd ${USER_ID}; then userdel -f $(getent passwd ${USER_ID} | cut -d: -f1); fi && \
    if getent group ${GROUP_ID}; then groupdel $(getent group ${GROUP_ID} | cut -d: -f1); fi || true

RUN groupadd -g ${GROUP_ID} user && \
    useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash user && \
    echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

WORKDIR /home/user/workspace

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo "source /home/user/workspace/install/setup.bash" >> /etc/bash.bashrc && \
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash'" >> /etc/bash.bashrc && \
    echo "alias cs='source install/setup.bash'" >> /etc/bash.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER user
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]