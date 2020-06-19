FROM ros:kinetic-ros-core

RUN apt-get update \ 
    && apt-get install -y ros-kinetic-tf ros-kinetic-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \ 
    && apt-get install -y libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

COPY ./ /root/horizon_highway_slam/

RUN echo "source /root/horizon_highway_slam/build/devel/setup.bash" >> \
    /root/.bashrc

