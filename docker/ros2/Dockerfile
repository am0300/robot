FROM ros:jazzy-ros-base-noble
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz-bridge python3-numpy \
    && rm -rf /var/lib/apt/lists/*
COPY ros2_app /opt/app
RUN chmod +x /opt/app/start_ros2.sh
CMD ["/opt/app/start_ros2.sh"]
