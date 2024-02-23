

###########################################
# Base image
###########################################
FROM ubuntu:22.04 AS base
#FROM kasmweb/ubuntu-jammy-dind:1.15.0 AS base



ARG ROS_DISTRO=humble
ARG ORG_NAME=roboto-ai





ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN sudo add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-base \
    python3-argcomplete \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-rosdep \ 
    git \
  && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=${ROS_DISTRO}
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV DEBIAN_FRONTEND=



###########################################
#  Overlay image
###########################################
FROM base AS overlay



ARG ROS_DISTRO=humble
ARG BOT_NAME=questbot
ARG ORG_NAME=robotoai
ARG BUILD_USER=builder


ENV VIRTUAL_ENV=/opt/venv



ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]




# Update and install necessary packages
RUN apt-get update && \
    apt-get install -y python3 python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Install virtualenv
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install virtualenv

# # Create a virtual environment 
RUN virtualenv $VIRTUAL_ENV

ENV PATH=$PATH:${VIRTUAL_ENV}/bin


 
# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp



# Install colcon and rosdep
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && apt-get install -y python3-rosdep \
 && apt install -y python3-colcon-common-extensions 



RUN apt-get update && apt-get install -y make g++ 



RUN mkdir -p /$ORG_NAME/share/$BOT_NAME

RUN mkdir -p /opt/venv


RUN useradd -m $BUILD_USER

RUN chown -R $BUILD_USER:$BUILD_USER  /$ORG_NAME $VIRTUAL_ENV

RUN chmod -R o+rw $VIRTUAL_ENV

RUN chmod -R o+rwx /$ORG_NAME/share/$BOT_NAME

RUN echo $BUILD_USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$BUILD_USER\
  && chmod 0440 /etc/sudoers.d/$BUILD_USER 


USER $BUILD_USER

RUN sudo rosdep init 
RUN rosdep update 

RUN mkdir -p /home/${BUILD_USER}/workspace/src

WORKDIR /home/${BUILD_USER}/workspace/src

COPY ./src/.repos .

RUN vcs import < .repos 


WORKDIR /home/${BUILD_USER}/workspace/

RUN rosdep install --from-paths src --ignore-src -y 

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --install-base /${ORG_NAME}/share/$BOT_NAME/.

WORKDIR /

USER root

RUN deluser --remove-home ${BUILD_USER} 


###########################################
#  Developer image
###########################################
FROM overlay AS dev



ARG ROS_DISTRO

ENV ROS_DISTRO=$ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  openssh-client \
  python3-argcomplete \
  python3-pip \
  ros-dev-tools \
  ros-${ROS_DISTRO}-ament-* \
  vim \
  git-core \
  bash-completion \
  nano \
  && rm -rf /var/lib/apt/lists/*


ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1


SHELL [ "bash", "-c"]


RUN echo '#!/bin/bash' > /tmp/entrypoint.sh && \
    echo '' >> /tmp/entrypoint.sh && \
    echo '# Create the user and workspace directory' >> /tmp/entrypoint.sh && \
    echo 'groupadd --gid $USER_UID ${DEVELOPMENT_USERNAME}' >> /tmp/entrypoint.sh && \
    echo 'useradd -s /bin/bash --uid $USER_UID --gid $USER_UID -m ${DEVELOPMENT_USERNAME}' >> /tmp/entrypoint.sh && \
    echo 'echo "${DEVELOPMENT_USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${DEVELOPMENT_USERNAME}' >> /tmp/entrypoint.sh && \
    echo 'chmod 0440 /etc/sudoers.d/${DEVELOPMENT_USERNAME}' >> /tmp/entrypoint.sh && \
    echo 'mkdir -p /home/${DEVELOPMENT_USERNAME}/workspace' >> /tmp/entrypoint.sh && \
    echo 'chown -R ${DEVELOPMENT_USERNAME}:${DEVELOPMENT_USERNAME} /home/${DEVELOPMENT_USERNAME} ${VIRTUAL_ENV}' >> /tmp/entrypoint.sh && \
    echo 'chmod -R o+rwx /home/${DEVELOPMENT_USERNAME} ${VIRTUAL_ENV} ' >> /tmp/entrypoint.sh && \
    # echo 'echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/${DEVELOPMENT_USERNAME}/.bashrc' >> /tmp/entrypoint.sh \
    # echo 'echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/${DEVELOPMENT_USERNAME}/.bashrc' >> /tmp/entrypoint.sh \
    echo 'su ${DEVELOPMENT_USERNAME}' >> /tmp/entrypoint.sh 

ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1



RUN chmod +x /tmp/entrypoint.sh


ENTRYPOINT ["bash", "/tmp/entrypoint.sh" ]




###########################################
#  Deployment image
###########################################
FROM overlay AS deploy

ENV ROS_DISTRO=humble
ENV BOT_NAME=$BOT_NAME


RUN mkdir -p /$ORG_NAME/share/$BOT_NAME

RUN mkdir -p /opt/venv

ENV VIRTUAL_ENV=/opt/venv

RUN useradd -m service

RUN chown -R service:service /$ORG_NAME $VIRTUAL_ENV

RUN chmod -R o+r $VIRTUAL_ENV

RUN chmod -R o+x /$ORG_NAME/share/$BOT_NAME

RUN mkdir /$ORG_NAME/startup

ENV ORG_NAME=$ORG_NAME

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /$ORG_NAME/startup/start.sh && \
    echo 'source /${ORG_NAME}/share/${BOT_NAME}/setup.bash' >> /$ORG_NAME/startup/start.sh && \
    echo  'ros2 launch something something' >> /$ORG_NAME/startup/start.sh && \
    chmod +x /$ORG_NAME/startup/start.sh



WORKDIR /$ORG_NAME/startup

RUN chmod u+x start.sh

USER service

ENTRYPOINT [ "bash", "start.sh" ]


#  yeah moved ## older:should have move the colcon build in the install base to the github action so that it will be build there ...
#  one work pending that is the entrypoint which may be a bash script that launches dyno atman service. 
#   (all the other things will be taken care using the install.sh script)


# to do : have to decide and set the username for the deploy/release container, one who runs the startup service of our robot

# have to change the deploy to release





### 1) this is a dockerfile created for the purpose of aiding the development and deployment of the dyno atman

# i) it require the source code to be available using the dependencies.repo 
