#!/bin/bash
# 如果是非交互式 shell，則跳過
[[ $- != *i* ]] && return

# 函式：檢查遠端主機是否可達
ping_all() { 
    # 定義要測試的 host
    local hosts=(
        "192.168.11.206"
        "192.168.11.152"
        "agvc.ui"
        "op.ui"
    )
    # 逐一測試
    for host in "${hosts[@]}"; do         
        if ping -c 1 "$host" &> /dev/null; then
            echo "✅ $host ping ok!"
        else
            echo "❌ 無法連接到遠端主機 $host"
            return 1
        fi
    done
}

# 測試所有列出的 ROS 2 workspace 套件
test_all() {
    # 定義要測試的 workspace 路徑
    local workspaces=(
        "/app/db_proxy_ws"
        "/app/rcs_ws"
    )

    # 逐一測試
    for ws in "${workspaces[@]}"; do 
        if [ -d "$ws" ]; then 
            echo "🔍 進入 workspace: $ws"
            cd "$ws" || continue

            # 建議先 source 環境
            if [ -f "install/setup.bash" ]; then
                source install/setup.bash
            fi

            # 執行 colcon test
            colcon test --event-handlers console_direct+
        else
            echo "⚠️ Warning: $ws 不存在，略過"
        fi
    done
}


# 函式：執行 colcon build 所有 _ws 資料夾
build_all() {
    BASE_DIR="/app/"
    
    # 查找所有 _ws 結尾的資料夾並執行 colcon build
    for dir in "$BASE_DIR"/*_ws; do
        if [ -d "$dir" ]; then
            echo "開始建置 $dir ..."
            # 進入資料夾並執行 colcon build
            cd "$dir" || continue
            colcon build
            if [ $? -eq 0 ]; then
                echo "$dir 建置成功！"
            else
                echo "$dir 建置失敗！"
            fi
            cd "$BASE_DIR" || continue
        fi
    done
}
# 函式：執行 colcon build --symlink-install所有 _ws 資料夾
build_all_symlink_install() {
    BASE_DIR="/app/"
    
    # 查找所有 _ws 結尾的資料夾並執行 colcon build
    for dir in "$BASE_DIR"/*_ws; do
        if [ -d "$dir" ]; then
            echo "開始建置 $dir ..."
            # 進入資料夾並執行 colcon build
            cd "$dir" || continue
            colcon build --symlink-install
            if [ $? -eq 0 ]; then
                echo "$dir 建置成功！"
            else
                echo "$dir 建置失敗！"
            fi
            cd "$BASE_DIR" || continue
        fi
    done
}

# 函式：執行 rm -rf 所有 _ws 資料夾 內的 build install log
clear_all() {
    BASE_DIR="/app/"
    
    # 查找所有 _ws 結尾的資料夾並執行 rm -rf
    for dir in "$BASE_DIR"/*_ws; do
        if [ -d "$dir" ]; then
            echo "開始清理 $dir ..."
            # 進入資料夾並執行 rm -rf
            cd "$dir" || continue
            rm -rf build/ install/ log/
            if [ $? -eq 0 ]; then
                echo "$dir 清理成功！"
            else
                echo "$dir 清理失敗！"
            fi
            cd "$BASE_DIR" || continue
        fi
    done
}



# 函式：同步檔案到遠端主機
app_upload() { 
    LOCAL_DIR="/app/"
    REMOTE_USER="root"  # 設定遠端使用者名稱
    REMOTE_HOST=$1  # 接收遠端主機 IP
    REMOTE_DIR="/app/"

    # 檢查遠端主機是否可達
    check_remote_host "$REMOTE_HOST"
    if [ $? -ne 0 ]; then
        echo "無法繼續上傳，遠端主機不可達。"
        return 1
    fi

    echo "執行 rsync 同步操作:rsync -avz --delete $LOCAL_DIR $REMOTE_USER@$REMOTE_HOST:$REMOTE_DIR"
    # 執行 rsync 同步操作
    rsync -avz --delete "$LOCAL_DIR" "$REMOTE_USER@$REMOTE_HOST:$REMOTE_DIR"
    if [ $? -eq 0 ]; then 
        echo "同步成功！" 
    else 
        echo "同步失敗！"
    fi
}

# 函式：從遠端主機下載檔案
app_download() { 
    LOCAL_DIR="/app/"
    REMOTE_USER="root"  # 設定遠端使用者名稱
    REMOTE_HOST=$1  # 接收遠端主機 IP
    REMOTE_DIR="/app/"

    # 檢查遠端主機是否可達
    check_remote_host "$REMOTE_HOST"
    if [ $? -ne 0 ]; then 
        echo "無法繼續下載，遠端主機不可達。"
        return 1
    fi

    echo "執行 rsync 同步操作:rsync -avz $REMOTE_USER@$REMOTE_HOST:$REMOTE_DIR $LOCAL_DIR"
    # 執行 rsync 同步操作
    rsync -avz "$REMOTE_USER@$REMOTE_HOST:$REMOTE_DIR" "$LOCAL_DIR"
    if [ $? -eq 0 ]; then 
        echo "下載成功！"
    else    
        echo "下載失敗！"
    fi
}

# 呼叫 app_upload 和 app_download 函式範例：
# app_upload 192.168.0.5
# app_download 192.168.0.5

# 定義 source_all 函式，會載入所有指定的 workspace 路徑
all_source() { 
    # 設定要載入的 workspace 路徑
    workspaces=(
        "/app/keyence_plc_ws/install"
        "/app/plc_proxy_ws/install"
        "/app/db_proxy_ws/install"
        "/app/ecs_ws/install"
        "/app/rcs_ws/install"
        "/app/agv_cmd_service_ws/install"
        "/app/launch_ws/install"
        
        "/app/joystick_ws/install"
        "/app/agv_ws/install"
        "/app/web_api_ws/install"

        "/app/kuka_fleet_ws/install"
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

# ===== SSH 控制函式 =====
manage_ssh() {
    case "$1" in
        start)
            if ! pgrep -f "sshd" > /dev/null; then
                echo "🚀 啟動 SSH 服務..."
                service ssh start
                echo "✅ SSH 服務已啟動"
            else
                echo "✅ SSH 服務已經在運行中"
            fi
            ;;
        stop)
            if pgrep -f "sshd" > /dev/null; then
                echo "⏳ 停止 SSH 服務..."
                service ssh stop
                echo "✅ SSH 服務已停止"
            else
                echo "❌ SSH 服務未運行"
            fi
            ;;
        restart)
            if pgrep -f "sshd" > /dev/null; then
                echo "🔄 重新啟動 SSH 服務..."
                service ssh restart
                echo "✅ SSH 服務已重新啟動"
            else
                echo "❌ SSH 服務未運行，無法重新啟動"
            fi
            ;;
        status)
            if pgrep -f "sshd" > /dev/null; then
                echo "✅ SSH 服務正在運行"
            else
                echo "❌ SSH 服務未運行"
            fi
            ;;
        *)
            echo "用法: manage_ssh {start|stop|restart|status}"
            return 1
            ;;
    esac
}


# ===== Zenoh Router 控制函式 =====
manage_zenoh() {
    ZENOH_LOG_FILE="/tmp/zenoh_router.log"
    ZENOH_PID_FILE="/tmp/zenoh_router.pid"

    case "$1" in
        start)
            if [ -f "$ZENOH_PID_FILE" ] && pgrep -F "$ZENOH_PID_FILE" > /dev/null; then
                echo "✅ Zenoh Router 已經在運行中 (PID: $(cat $ZENOH_PID_FILE))"
            else
                echo "🚀 啟動 Zenoh Router..."
                nohup ros2 run rmw_zenoh_cpp rmw_zenohd > "$ZENOH_LOG_FILE" 2>&1 &
                echo $! > "$ZENOH_PID_FILE"
                echo "✅ Zenoh Router 已啟動 (PID: $(cat $ZENOH_PID_FILE))"
            fi
            ;;

        stop)
            if [ -f "$ZENOH_PID_FILE" ]; then
                ZENOH_PID=$(cat "$ZENOH_PID_FILE")
                echo "⏳ 停止 Zenoh Router (PID: $ZENOH_PID)..."
                kill "$ZENOH_PID"
                sleep 2
                rm -f "$ZENOH_PID_FILE"
                echo "✅ Zenoh Router 已停止"
            else
                # 確保停止所有與 Zenoh Router 相關的進程
                echo "🚨 Zenoh Router 進程未找到，檢查端口佔用..."
                if pgrep -f "rmw_zenohd" > /dev/null; then
                    echo "⏳ 停止 Zenoh Router 進程..."
                    pkill -f "rmw_zenohd"
                    sleep 2
                    echo "✅ Zenoh Router 進程已停止"
                else
                    echo "❌ Zenoh Router 未運行"
                fi
            fi

            # 檢查端口 7447 是否仍被佔用，並強制釋放
            if lsof -i :7447 > /dev/null; then
                echo "🚨 端口 7447 仍然被占用，強制釋放..."
                # 查找佔用該端口的進程並強制終止
                lsof -i :7447 | awk 'NR>1 {print $2}' | xargs kill -9
                sleep 2
                echo "✅ 端口 7447 已強制釋放"
            else
                echo "✅ 端口 7447 沒有被占用"
            fi
            ;;

        restart)
            echo "🔄 重新啟動 Zenoh Router..."
            manage_zenoh stop
            sleep 2
            manage_zenoh start
            ;;

        status)
            if [ -f "$ZENOH_PID_FILE" ] && pgrep -F "$ZENOH_PID_FILE" > /dev/null; then
                echo "✅ Zenoh Router 正在運行 (PID: $(cat $ZENOH_PID_FILE))"
            else
                echo "❌ Zenoh Router 未運行"
            fi
            ;;

        *)
            echo "用法: manage_zenoh {start|stop|restart|status}"
            return 1
            ;;
    esac
}

# 設定 Zenoh 相關環境變數
export ZENOH_ROUTER_CONFIG_URI="/app/routerconfig.json5"
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"

# 確認環境變數設定
echo "✅ 設定 ZENOH_ROUTER_CONFIG_URI=$ZENOH_ROUTER_CONFIG_URI"
echo "✅ 設定 RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

# 自動啟動 Zenoh Router
manage_zenoh start
