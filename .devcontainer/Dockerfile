FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   # openbox
   && apt-get -y install --no-install-recommends python3-pygame libopencv-dev \ 
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /workspaces/flyappy_autonomy_test_public/devel/setup.sh || echo LOL" >> ~/.bashrc