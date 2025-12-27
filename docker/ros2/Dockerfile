FROM nvidia/cuda:13.1.0-runtime-ubuntu24.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set locale
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN << EOF 
	apt-get update 
	apt-get install -y locales
    locale-gen en_US en_US.UTF-8
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
EOF

WORKDIR /tmp/

# Install ROS2 Jazzy

RUN << EOF
	apt-get update
	
	# Install basic packages
	apt-get -y --quiet --no-install-recommends install software-properties-common curl gosu
	add-apt-repository universe

	# Install ros repository
	export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
	curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
	dpkg -i /tmp/ros2-apt-source.deb
	apt-get update

	# Install ros and ros tools (rviz2, rqt)
	apt-get -y --quiet --no-install-recommends install \
		ros-dev-tools \
		ros-jazzy-ros-base \
		ros-jazzy-rviz2 \
		ros-jazzy-rqt-gui \
		ros-jazzy-rqt-common-plugins

	apt-get -y autoremove
	apt-get clean autoclean
	rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
EOF

# Install python packages
RUN << EOF
	apt-get update
	apt-get -y --quiet --no-install-recommends install \
		python3 \
		python3-pip
	
	apt-get -y autoremove
	apt-get clean autoclean
	rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
EOF

COPY requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --break-system-packages -r /tmp/requirements.txt

# Install utils 
RUN << EOF
	apt-get update
	apt-get -y --quiet --no-install-recommends install \
		vim
	apt-get -y autoremove
	apt-get clean autoclean
	rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
EOF

# Setup entrypoint
WORKDIR /app/
COPY ./entrypoint.sh  /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Setup bashrc
COPY bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /etc/skel/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh", "bash"]
