#!/bin/bash
# /app/startup.bash

# 創建 AGV 環境標記檔案
echo "AGV_ENVIRONMENT=true" > /app/.agv_environment
echo "CONTAINER_TYPE=rosagv" >> /app/.agv_environment
echo "STARTUP_TIME=$(date)" >> /app/.agv_environment

# 🚗 統一設備身份識別
echo "🔍 開始統一設備身份識別..."
export CONTAINER_TYPE="agv"

if [ -f "/app/scripts/config_driven_device_detector.bash" ]; then
    # 執行配置驅動統一身份識別腳本
    source /app/scripts/config_driven_device_detector.bash
    if [ $? -eq 0 ]; then
        echo "✅ AGV 設備身份識別成功: $DEVICE_ID ($AGV_TYPE)"
        echo "📁 配置檔案: $DEVICE_CONFIG_FILE"
        echo "🚀 啟動套件: $AGV_LAUNCH_PACKAGE"
    else
        echo "⚠️ AGV 設備身份識別失敗，使用預設配置"
    fi
else
    echo "❌ 統一設備識別腳本不存在，使用預設配置"
    export DEVICE_ID="loader02"
    export AGV_ID="loader02"
    export AGV_TYPE="loader"
    export ROS_NAMESPACE="/loader02"
    export AGV_LAUNCH_PACKAGE="loader_agv"
    export AGV_LAUNCH_FILE="launch.py"
    export DEVICE_CONFIG_FILE="/app/config/agv/loader02_config.yaml"
fi

# 檢查是否安裝 Node.js (command -v 查詢是否存在指定的命令)
command -v node &> /dev/null
NODE_INSTALLED=$?

# 確認變數是否設定成功
echo "AGV Startup script is running..."
echo "ROS_DISTRO=$ROS_DISTRO"
echo "ZENOH_ROUTER_CONFIG_URI=$ZENOH_ROUTER_CONFIG_URI"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

source /opt/ros/$ROS_DISTRO/setup.bash
source /opt/ws_rmw_zenoh/install/setup.bash
#ros套件的interfaces source
source /app/keyence_plc_ws/install/setup.bash
source /app/plc_proxy_ws/install/setup.bash

#啟動時自動執行的腳本，可以在這裡定義各種函式，並在啟動時自動執行。
#啟動SSH
echo "🚀 啟動 SSH 服務..."
service ssh start
#啟動zenoh router
ZENOH_LOG_FILE="/tmp/zenoh_router.log"
ZENOH_PID_FILE="/tmp/zenoh_router.pid"
echo "🚀 啟動 Zenoh Router..."
nohup ros2 run rmw_zenoh_cpp rmw_zenohd > "$ZENOH_LOG_FILE" 2>&1 &
echo $! > "$ZENOH_PID_FILE"

#if [ $NODE_INSTALLED -eq 0 ]; then
#    # Node.js 服務 agvc.ui
#    AGVC_NODE_SCRIPT="/webui/src/server.js"
#    AGVC_LOG_FILE="/tmp/nodejs.log"
#    AGVC_PID_FILE="/tmp/node.pid"
#    echo "🚀 啟動第一個 Node.js 應用程式...agvc.ui"
#    cd /webui/
#    # 要過濾掉以 # 開頭的行
#    # 這樣可以避免將註解行也當作環境變數
#    env $(cat .env | grep -v '^#' | xargs) nohup node "$AGVC_NODE_SCRIPT" > "$AGVC_LOG_FILE" 2>&1 &
#    echo $! > "$AGVC_PID_FILE"
#    #nohup npm start > "$AGVC_LOG_FILE" 2>&1 &
#    #echo $! > "$AGVC_PID_FILE"
#
#    # Node.js 服務 op.ui
#    OPUI_NODE_SCRIPT="/opui/opui-server/src/server.js"
#    OPUI_LOG_FILE="/tmp/nodejs-opui.log"
#    OPUI_PID_FILE="/tmp/node-opui.pid"
#
#    if [ -f "$OPUI_NODE_SCRIPT" ]; then
#        echo "🚀 啟動第二個 Node.js 應用程式...op.ui"
#        cd /opui/opui-server/
#        # 要過濾掉以 # 開頭的行
#        # 這樣可以避免將註解行也當作環境變數
#        env $(cat .env | grep -v '^#' | xargs) nohup node "$OPUI_NODE_SCRIPT" > "$OPUI_LOG_FILE" 2>&1 &
#        echo $! > "$OPUI_PID_FILE"
#    else
#        echo "❌ 錯誤：$OPUI_NODE_SCRIPT 不存在，無法啟動 op.ui"
#        exit 1
#    fi
#fi

#檢查服務是否都已經啟動
# 檢查 SSH 是否已經運行
if pgrep -f "sshd" > /dev/null; then
    echo "✅ SSH 服務已經在運行中"
else
    echo "❌ SSH 服務 啟動失敗"
fi

# 檢查 Zenoh Router 是否已經運行
if [ -f "$ZENOH_PID_FILE" ] && pgrep -F "$ZENOH_PID_FILE" > /dev/null; then
    echo "✅ Zenoh Router 已經在運行中 (PID: $(cat $ZENOH_PID_FILE))"
else
    echo "❌ Zenoh Router 啟動失敗"
fi

#if [ $NODE_INSTALLED -eq 0 ]; then
#    # 循環檢查第一個 Node.js 服務 最多檢查 10 秒
#    START_TIME=$SECONDS
#    TIMEOUT=10
#    while [ $(($SECONDS - $START_TIME)) -lt $TIMEOUT ]; do
#        if pgrep -xaf "node $AGVC_NODE_SCRIPT" > /dev/null && [ -f "$AGVC_PID_FILE" ] && pgrep -F "$AGVC_PID_FILE" > /dev/null; then
#            echo "✅ 第一個 Node.js 應用程式(agvc.ui)已經在運行中 (PID: $(cat $AGVC_PID_FILE))"
#            break
#        else
#            echo "⏳ 等待第一個 Node.js 應用程式(agvc.ui)啟動... 已經等待 $(($SECONDS - $START_TIME)) 秒"
#        fi
#        sleep 1  # 每次檢查後等待 1 秒
#    done
#    # 如果超過 10 秒仍然沒有啟動成功，則顯示錯誤訊息
#    if [ $(($SECONDS - $START_TIME)) -ge $TIMEOUT ]; then
#        echo "❌ 第一個 Node.js 應用程式啟動失敗"
#    fi
#
#    # 循環檢查第二個 Node.js 服務
#    START_TIME=$SECONDS
#    while [ $(($SECONDS - $START_TIME)) -lt $TIMEOUT ]; do
#        if pgrep -xaf "node $OPUI_NODE_SCRIPT" > /dev/null && [ -f "$OPUI_PID_FILE" ] && pgrep -F "$OPUI_PID_FILE" > /dev/null; then
#            echo "✅ 第二個 Node.js 應用程式(op.ui)已經在運行中 (PID: $(cat $OPUI_PID_FILE))"
#            break
#        else
#            echo "⏳ 等待第二個 Node.js 應用程式(op.ui)啟動... 已經等待 $(($SECONDS - $START_TIME)) 秒"
#        fi
#        sleep 1  # 每次檢查後等待 1 秒
#    done
#    # 如果超過 10 秒仍然沒有啟動成功，則顯示錯誤訊息
#    if [ $(($SECONDS - $START_TIME)) -ge $TIMEOUT ]; then
#        echo "❌ 第二個 Node.js 應用程式啟動失敗"
#    fi
#else
#    echo "❌ Node.js 未安裝"
#fi

# -lt	less than	小於 <	[ "$a" -lt "$b" ]
# -le	less or equal	小於或等於 ≤	[ "$a" -le "$b" ]
# -eq	equal	等於 =	[ "$a" -eq "$b" ]
# -ne	not equal	不等於 ≠	[ "$a" -ne "$b" ]
# -gt	greater than	大於 >	[ "$a" -gt "$b" ]
# -ge	greater or equal	大於或等於 ≥	[ "$a" -ge "$b" ]

all_source() { 
    # 設定要載入的 workspace 路徑
    workspaces=(
        "/app/keyence_plc_ws/install"
        "/app/plc_proxy_ws/install"
        "/app/agv_cmd_service_ws/install"        
        "/app/joystick_ws/install"
        "/app/agv_ws/install"
        "/app/path_algorithm/install"
    )

    # 逐一檢查並 source
    for ws in "${workspaces[@]}"; do 
        if [ -d "$ws" ]; then 
            echo "Sourcing $ws/setup.bash"
            source "$ws/setup.bash"
        else
            echo "Warning: $ws 不存在，略過"
        fi
    done
}

    #啟動

all_source
export PYTHONPATH=/opt/pyvenv_env/lib/python3.12/site-packages:$PYTHONPATH
source /opt/pyvenv_env/bin/activate

    #啟動agv launch
AGV_LOG_FILE="/tmp/agv.log"
AGV_PID_FILE="/tmp/agv.pid"

#echo "🚀 啟動 agv launch..."
#nohup ros2 launch loader_agv launch.py > "$AGV_LOG_FILE" 2>&1 &
#echo $! > "$AGV_PID_FILE"
#
#    # 檢查 agv launch 是否已經運行
#if [ -f "$AGV_PID_FILE" ] && pgrep -F "$AGV_PID_FILE" > /dev/null; then
#    python3 -c "import sqlmodel; print(sqlmodel.__version__)"
#    python3 -c "import networkx; print(networkx.__version__)"
#
#    echo "✅ agv launch 已經在運行中 (PID: $(cat $AGV_PID_FILE))"
#else
#    echo "❌ agv launch 啟動失敗"
#fi

# =====================================
# 🖥️ AGVUI 服務自動啟動配置
# =====================================
AUTO_START_AGVUI=false  # 設定為 true 啟用自動啟動，false 停用
USE_ROS_LAUNCH=true    # 設定為 true 使用 ROS 2 Launch，false 使用原始方式

# 載入 setup.bash 以取得管理函數
source /app/setup.bash

if [ "$AUTO_START_AGVUI" = "true" ]; then
    # 等待系統資源就緒
    echo "⏳ 等待系統資源就緒..."
    sleep 5
    
    if [ "$USE_ROS_LAUNCH" = "true" ]; then
        echo "🚀 使用 ROS 2 Launch 啟動 AGVUI..."
        # 使用新的 ROS 2 Launch 方式
        if manage_web_agv_launch start; then
            echo "✅ AGVUI 服務啟動成功 (ROS 2 Launch)"
        else
            echo "⚠️ AGVUI 服務啟動失敗"
            echo "📝 請使用以下指令查看錯誤詳情："
            echo "   tail -f /tmp/web_agv_launch.log"
            echo "💡 容器仍在運行，您可以透過 SSH 連線進行診斷"
        fi
    else
        echo "🖥️ 使用傳統方式啟動 AGVUI..."
        # 使用原始方式（向後相容）
        if manage_agvui start; then
            echo "✅ AGVUI 服務啟動成功 (傳統方式)"
        else
            echo "⚠️ AGVUI 服務啟動失敗"
            echo "📝 請使用以下指令查看錯誤詳情："
            echo "   tail -f /tmp/agvui.log"
            echo "💡 容器仍在運行，您可以透過 SSH 連線進行診斷"
        fi
    fi
else
    echo "⏸️ AGVUI 自動啟動已停用 (AUTO_START_AGVUI=false)"
fi
