# Ubuntu 18.04 with nvidia-docker2 beta opengl support
FROM nvidia/opengl:1.0-glvnd-devel-ubuntu18.04

# install third-party packages for things like developemnt
RUN apt-get -qq update && apt-get --no-install-recommends -y install \
    git \
    gnupg \
    g++-8 \
    lsb-release \
    sudo \
    wget \
  && rm -rf /var/lib/apt/lists/*

# set up repositories
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
  && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

# install dependencies
COPY packages.apt .
COPY packages-bionic.apt .
RUN apt-get -qq update && apt-get --no-install-recommends -y install \
    $(sort -u $(find . -iname 'packages-bionic.apt' -o -iname 'packages.apt') | tr '\n' ' ') \
  && rm -rf /var/lib/apt/lists/*

# enable colcon for building packages (time zone needs to be enabled)
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' \
  && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN echo 'Etc/UTC' > /etc/timezone && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
  && apt-get -qq update && apt-get --no-install-recommends -y install \
    python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

# configure gcc8
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8 --slave /usr/bin/gcov gcov /usr/bin/gcov-8

# create a non-root user
# uses same user_id as user outside the container
# requires docker build argument user_id
ARG user_id
ENV USERNAME developer
RUN useradd -U --uid ${user_id} -ms /bin/bash $USERNAME \
 && echo "$USERNAME:$USERNAME" | chpasswd \
 && adduser $USERNAME sudo \
 && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

# commands below run as the developer user
USER $USERNAME

# create a colcon workspace that can build ign-gazebo from source
# (the ign-gazebo source code will be loaded into the src directory of this workspace)
RUN mkdir -p ~/ws/src

# start the container where the workspace is
WORKDIR /home/$USERNAME/ws

# start a bash shell by default
CMD [ "/bin/bash" ]
