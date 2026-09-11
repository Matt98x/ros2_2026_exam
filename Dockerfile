FROM carms84/robeng:latest

USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install only the requested terminal emulator; do not upgrade existing packages.
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-upgrade --no-install-recommends terminator \
    && rm -rf /var/lib/apt/lists/*

COPY docker/terminator.config /usr/local/share/robot-programming/terminator.config
COPY docker/robot-terminals.bash /usr/local/bin/robot-terminals
COPY docker/robot-terminal-command.bash /usr/local/bin/robot-terminal-command
RUN chmod +x /usr/local/bin/robot-terminals /usr/local/bin/robot-terminal-command

COPY docker/ros-environment.bash /usr/local/share/robot-programming/ros-environment.bash
COPY docker/entrypoint.bash /usr/local/bin/robot-programming-entrypoint

RUN chmod +x /usr/local/bin/robot-programming-entrypoint \
    && sed -i '/^export DISPLAY=/d' /root/.bashrc \
    && echo 'source /usr/local/share/robot-programming/ros-environment.bash' >> /root/.bashrc \
    && echo 'source /usr/local/share/robot-programming/ros-environment.bash' >> /home/ubuntu/.bashrc \
    && ln -s /usr/local/share/robot-programming/ros-environment.bash /etc/profile.d/robot-programming.sh

ENV ROS_DISTRO=jazzy \
    ROS_WORKSPACE=/home/ubuntu/ros_ws \
    BASH_ENV=/usr/local/share/robot-programming/ros-environment.bash \
    GZ_SIM_RESOURCE_PATH=/home/ubuntu/gazebo_models

WORKDIR /home/ubuntu/ros_ws/src
ENTRYPOINT ["/usr/local/bin/robot-programming-entrypoint"]
CMD ["sleep", "infinity"]
