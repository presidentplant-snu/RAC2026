FROM nvidia/cuda:13.1.0-runtime-ubuntu24.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set locale
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

WORKDIR /tmp/

RUN << EOF 
	apt-get update 
	apt-get install -y locales
    locale-gen en_US en_US.UTF-8
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
EOF

# Copied over from https://github.com/PX4/PX4-containers
# and https://github.com/PX4/PX4-Autopilot/blob/main/Tools/setup/ubuntu.sh

RUN << EOF
	apt-get update 
	# initial deps
	apt-get -y --quiet --no-install-recommends install \
		ca-certificates \
		gnupg \
		gosu \
		software-properties-common \
		sudo \
		wget \
		curl
 	# basic requirements
	apt-get -y --quiet --no-install-recommends install \
		astyle \
		build-essential \
		ccache \
		cmake \
		cppcheck \
		file \
		g++ \
		gcc \
		gdb \
		git \
		lcov \
		libssl-dev \
		libxml2-dev \
		libxml2-utils \
		make \
		ninja-build \
		python3 \
		python3-dev \
		python3-pip \
		python3-setuptools \
		python3-wheel \
		rsync \
		shellcheck \
		unzip \
		zip
 	# nuttx build tools
	apt-get -y --quiet --no-install-recommends install \
		autoconf \
		automake \
		bison \
		build-essential \
		bzip2 \
		file \
		flex \
		genromfs \
		gperf \
		libncurses-dev \
		libncurses6 \
		libncursesw6 \
		libtool \
		pkg-config \
		uncrustify \
		vim-common
	# cross compiling tools
	apt-get -y --quiet --no-install-recommends install \
		g++-multilib \
		gcc-arm-none-eabi \
		gcc-multilib 

	apt-get -y autoremove
	apt-get clean autoclean
	rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
EOF

COPY requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --break-system-packages -r /tmp/requirements.txt

# Setup Gazebo harmonic
# Copied over from https://gazebosim.org/docs/harmonic/install_ubuntu/ and above
RUN <<EOF
	curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
	echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
	apt-get update
	apt-get -y --quiet --no-install-recommends install gz-harmonic libunwind-dev cppzmq-dev \
	dmidecode \
	gstreamer1.0-plugins-bad \
	gstreamer1.0-plugins-base \
	gstreamer1.0-plugins-good \
	gstreamer1.0-plugins-ugly \
	gstreamer1.0-libav \
	libeigen3-dev \
	libgstreamer-plugins-base1.0-dev \
	libimage-exiftool-perl \
	libopencv-dev \
	libxml2-utils \
	pkg-config \
	protobuf-compiler

	apt-get -y autoremove
	apt-get clean autoclean
	rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
EOF

# Set up xRCE-DDS-Agent

RUN << EOF
	git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    sed -i '98s|2.12|2.13|' Micro-XRCE-DDS-Agent/CMakeLists.txt
    sed -i '99s|2.12.x|2.13.3|' Micro-XRCE-DDS-Agent/CMakeLists.txt 

    cd Micro-XRCE-DDS-Agent && \
	mkdir build && \
	cd build && \

    cmake .. && \
	make -j$(nproc) && \
	make install && \
	ldconfig /usr/local/lib/
EOF

# Setup entrypoint
WORKDIR /app/
COPY ./entrypoint.sh  /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Setup bashrc
RUN << EOF
cat << BASHRC >> /etc/skel/.bashrc
alias gazebo="PX4_SYS_AUTOSTART=4004 PX4_SIM_MODEL=standard_vtol PX4_GZ_WORLD=RAC_2025 ./build/px4_sitl_default/bin/px4"
alias dds="MicroXRCEAgent udp4 -p 8888"
BASHRC
EOF

ENTRYPOINT ["/usr/local/bin/entrypoint.sh", "bash"]
