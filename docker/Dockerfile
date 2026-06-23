# Dev image for the OutdoorNav 2.x API examples.
# ROS 2 Jazzy + a colcon overlay with clearpath_msgs and clearpath_outdoornav
# message packages pre-built. Pin the refs via build-args for reproducibility.

FROM ghcr.io/nvanheyst/ros2-jazzy:ubuntu-headless-v0.1.0

ARG DEBIAN_FRONTEND=noninteractive
ARG CLEARPATH_MSGS_REF=main
ARG CLEARPATH_OUTDOORNAV_REF=eee7d9a

RUN apt-get update && apt-get install -y --no-install-recommends \
        git build-essential cmake \
        python3-pip python3-colcon-common-extensions \
        python3-empy python3-lark python3-numpy python3-transforms3d \
        ros-jazzy-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# Python deps used by row_generator_polygon.py
RUN pip install --no-cache-dir --break-system-packages shapely networkx pyproj

RUN mkdir -p /opt/ros2_overlay/src && cd /opt/ros2_overlay/src \
    && git clone https://github.com/clearpathrobotics/clearpath_msgs.git \
    && (cd clearpath_msgs && git checkout ${CLEARPATH_MSGS_REF}) \
    && git clone https://github.com/cpr-application/clearpath_outdoornav.git \
    && (cd clearpath_outdoornav && git checkout ${CLEARPATH_OUTDOORNAV_REF})

RUN /bin/bash -c 'source /opt/ros/jazzy/setup.bash && \
    cd /opt/ros2_overlay && \
    colcon build --merge-install --packages-up-to clearpath_outdoornav_api_lib \
    && rm -rf build log src'

# Interactive shells pick this up automatically.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc \
 && echo 'source /opt/ros2_overlay/install/setup.bash' >> /etc/bash.bashrc

# Entrypoint sources the same so non-interactive `docker compose run --rm dev
# python3 …` also has ROS on the path.
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

WORKDIR /examples
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]
