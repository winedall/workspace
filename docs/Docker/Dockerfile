FROM ros:humble

  ENV DEBIAN_FRONTEND=noninteractive

  RUN apt-get update && apt-get install -y --no-install-recommends \
      systemd \
      systemd-sysv \
      dbus \
      sudo \
      openssh-server \
      iproute2 \
      iputils-ping \
      net-tools \
      udev \
      kmod \
      libcap2-bin \
      ca-certificates \
      curl \
      vim \
      less \
      procps \
      psmisc \
      lsof \
      python3-venv \
      python3-pip \
      python3-yaml \
      python3-numpy \
      libyaml-cpp-dev \
      libglfw3 \
      mesa-utils \
      libgl1-mesa-dri \
      libgl1-mesa-glx \
      libglu1-mesa \
      libx11-6 \
      libxrandr2 \
      libxinerama1 \
      libxcursor1 \
      portaudio19-dev \
      libportaudio2 \
      alsa-utils \
      evtest \
      xvfb \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-joy \
      ros-humble-cv-bridge \
      ros-humble-diagnostic-updater \
      && rm -rf /var/lib/apt/lists/*

  ENV LIBGL_ALWAYS_SOFTWARE=1

  RUN python3 -m pip install --no-cache-dir \
      onnxruntime \
      pyaudio \
      grpcio \
      grpcio-tools \
      piper-tts \
      numpy==1.26.4

  RUN mkdir -p /run/sshd \
      /etc/udev/rules.d \
      /etc/ld.so.conf.d \
      /root/.config \
      /root/.zl_robot

  RUN ssh-keygen -A \
      && sed -i 's/^#\?PermitRootLogin.*/PermitRootLogin yes/' /etc/ssh/sshd_config \
      && sed -i 's/^#\?PasswordAuthentication.*/PasswordAuthentication yes/' /etc/ssh/sshd_config \
      && sed -i 's/^#\?UsePAM.*/UsePAM yes/' /etc/ssh/sshd_config \
      && echo 'root:123456' | chpasswd \
      && systemctl enable ssh.service

  STOPSIGNAL SIGRTMIN+3

  CMD ["/sbin/init"]
