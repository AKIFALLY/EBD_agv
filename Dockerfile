FROM osrf/ros:jazzy-desktop-full
LABEL Name=rosenv Version=0.0.1

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

# 安裝一些常用工具 ifconfig , ping 
RUN apt-get update && apt-get install -y \
    net-tools \
    iputils-ping \
    telnet \
    && rm -rf /var/lib/apt/lists/*

# 建立 Python 虛擬環境
RUN python3 -m venv /opt/fastapi_env

# 啟用虛擬環境並安裝 FastAPI 相關套件
RUN source /opt/fastapi_env/bin/activate && \
    pip install --upgrade pip && \
    pip install fastapi uvicorn pydantic

# 設定 Python 虛擬環境的 PYTHONPATH
RUN echo 'export PYTHONPATH=/opt/fastapi_env/lib/python3.12/site-packages:$PYTHONPATH' >> ~/.bashrc

# 設定 ROS 環境與 rmw_zenoh 環境到 .bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ws_rmw_zenoh/install/setup.bash" >> ~/.bashrc

# 設定 自動 source 環境至 .bashrc
RUN echo "source /app/setup.bash" >> ~/.bashrc

WORKDIR /

# 設定容器啟動時的環境變數（可選）
CMD ["/bin/bash"]
