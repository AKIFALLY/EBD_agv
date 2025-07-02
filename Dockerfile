FROM osrf/ros:jazzy-desktop-full
LABEL Name=ros Version=0.0.1

# UTF8 中文顯示
RUN apt-get update && apt-get install -y locales \
    && locale-gen zh_TW.UTF-8 \
    && update-locale LANG=zh_TW.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# 確保所有指令在非互動模式下執行
ENV DEBIAN_FRONTEND=noninteractive

# 以下開始使用 bash (才可以用 source 指令)
SHELL ["/bin/bash", "-c"]

# 更新系統並安裝必要工具
RUN apt-get update && apt-get install -y \
    git \
    cargo \
    software-properties-common \
    python3-venv \
    && apt-add-repository universe \
    && apt-get update \
    && apt-get install -y clang \
    && rm -rf /var/lib/apt/lists/*

# 建立工作空間並下載 rmw_zenoh 原始碼
RUN mkdir -p ~/ws_rmw_zenoh/src && \
    cd ~/ws_rmw_zenoh/src && \
    git clone https://github.com/ros2/rmw_zenoh.git -b $ROS_DISTRO

# 安裝依賴項
WORKDIR /root/ws_rmw_zenoh

# 設定環境變數並建置專案
RUN apt-get update \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /var/lib/apt/lists/*

#zenoh安裝至/opt
WORKDIR /opt
RUN cp -r ~/ws_rmw_zenoh /opt \
    && rm -rf ~/ws_rmw_zenoh

# 安裝一些常用工具 ifconfig , ping , debugpy ,ssh server, gpiod, python3-libgpiod,rsync
RUN apt-get update && apt-get install -y \
    net-tools \
    iputils-ping \
    telnet \
    python3-debugpy \
    openssh-server \
    gpiod \
    python3-libgpiod \
    rsync \
    lsof \
    usbutils \
    ros-jazzy-joy-linux \
    && rm -rf /var/lib/apt/lists/*

# 建立 Python 虛擬環境
RUN python3 -m venv /opt/pyvenv_env

# 啟用虛擬環境並安裝 FastAPI,PyGame,postgresql 相關套件
RUN source /opt/pyvenv_env/bin/activate && \
    pip install --upgrade pip && \
    pip install fastapi uvicorn pydantic && \
    pip install pygame && \
    pip install sqlalchemy && \
    pip install psycopg2 && \
    pip install paho-mqtt && \
    pip install requests && \
    pip install networkx && \
    pip install sqlmodel && \
    pip install jinja2 && \
    pip install python-socketio && \
    pip install python-multipart

# 設定 Python 虛擬環境的 PYTHONPATH
RUN echo 'export PYTHONPATH=/opt/pyvenv_env/lib/python3.12/site-packages:$PYTHONPATH' >> ~/.bashrc


# 設定 ROS 環境與 rmw_zenoh 環境到 .bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ws_rmw_zenoh/install/setup.bash" >> ~/.bashrc

# 設定 自動 source 環境至 .bashrc
RUN echo "source /app/setup.bash" >> ~/.bashrc

# 設定 root 密碼為 36274806 , 開啟 root 登入 , 開啟密碼登入 , 開啟 2200 port
RUN sed -i 's/^#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -i 's/^#PasswordAuthentication no/PasswordAuthentication yes/' /etc/ssh/sshd_config && \
    sed -i 's/#Port 22/Port 2200/' /etc/ssh/sshd_config && \
    echo "root:36274806" | chpasswd

# 確保 SSH 服務可運行
RUN mkdir /var/run/sshd

# 產生 SSH 主機金鑰
RUN ssh-keygen -A

# 開放端口：2200（SSH）、1883（Mosquitto）9001（Mosquitto WebSocket）、1433（SQL Server）
#EXPOSE 2200 1883 9001 1433 
EXPOSE 2200 

WORKDIR /app

# 新增使用者 ct 並設定密碼，賦予 sudo 權限
RUN apt-get update && apt-get install -y sudo && \
    useradd -m -s /bin/bash ct && \
    echo "ct:36274806" | chpasswd && \
    usermod -aG sudo ct && \
    echo "ct ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers


# **容器啟動時自動執行**
ENTRYPOINT [ "/bin/bash","-c" ]
CMD ["/app/startup.agv.bash && tail -f /dev/null"]