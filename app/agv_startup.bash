

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

all_source

source /opt/pyvenv_env/bin/activate

#啟動agv launch
AGV_LOG_FILE="/tmp/agv.log"
AGV_PID_FILE="/tmp/agv.pid"
echo "🚀 啟動 agv launch..."
nohup ros2 launch loader_agv launch.py > "$AGV_LOG_FILE" 2>&1 &
echo $! > "$AGV_PID_FILE"


# 檢查 agv launch 是否已經運行
if [ -f "$AGV_PID_FILE" ] && pgrep -F "$AGV_PID_FILE" > /dev/null; then
    python3 -c "import sqlmodel; print(sqlmodel.__version__)"
    python3 -c "import networkx; print(networkx.__version__)"


    echo "✅ agv launch 已經在運行中 (PID: $(cat $AGV_PID_FILE))"
else
    echo "❌ agv launch 啟動失敗"
fi