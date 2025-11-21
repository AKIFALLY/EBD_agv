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
    # 移除默認值，要求必須通過設備識別或手動設置
    # export AGV_ID="loader02"  # 已禁用：為確保安全，不再提供默認值
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

# 🧹 啟動時自動清理臨時文件
# =============================================================================
echo "🧹 清理過期的臨時文件..."

# 清理 7 天前的 launch_params_* 目錄
LAUNCH_PARAMS_CLEANED=$(find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime +7 2>/dev/null | wc -l)
if [ "$LAUNCH_PARAMS_CLEANED" -gt 0 ]; then
    find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime +7 -exec rm -rf {} + 2>/dev/null
    echo "  ✅ 清理 $LAUNCH_PARAMS_CLEANED 個過期的 launch_params 目錄"
fi

# 清理孤立的 PID 文件 (進程不存在的 PID 文件)
# 正確處理多行 PID 文件（如 zenoh_router.pid 包含多個 PID）
for pid_file in /tmp/*.pid; do
    if [ -f "$pid_file" ]; then
        all_dead=true
        # 逐行讀取 PID 文件
        while read -r pid; do
            # 跳過空行
            [ -z "$pid" ] && continue
            # 如果任何一個 PID 還在運行，保留文件
            if kill -0 "$pid" 2>/dev/null; then
                all_dead=false
                break
            fi
        done < "$pid_file"

        # 只有當所有 PID 都不運行時才刪除文件
        if [ "$all_dead" = true ]; then
            rm -f "$pid_file"
            echo "  ✅ 清理孤立的 PID 文件: $(basename $pid_file)"
        fi
    fi
done

# 清理 7 天前的日誌文件（包含原始日誌和輪轉日誌）
OLD_LOGS=$(find /tmp -maxdepth 1 \( -name '*.log' -o -name '*.log.*' \) -type f -mtime +7 2>/dev/null | wc -l)
if [ "$OLD_LOGS" -gt 0 ]; then
    find /tmp -maxdepth 1 \( -name '*.log' -o -name '*.log.*' \) -type f -mtime +7 -delete 2>/dev/null
    echo "  ✅ 清理 $OLD_LOGS 個過期的日誌文件（含輪轉檔案）"
fi

echo "✅ 臨時文件清理完成"

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

# =====================================
# 🖥️ AGV 服務自動啟動配置
# =====================================
AUTO_START_WEB_AGV_LAUNCH=true    # Web AGV Launch 服務（端口 8003 - AGVUI 車載監控界面）
AUTO_START_AGV_LAUNCH=true       # AGV Launch 服務（本地 AGV 控制，默認關閉）

# 載入 setup.bash 以取得管理函數
source /app/setup.bash

# =============================================================================
# 📖 AGV Launch 使用說明
# =============================================================================
#
# 🔧 控制 AGV Launch 自動啟動:
#   - 啟用自動啟動: 將 AUTO_START_AGV_LAUNCH 設為 true
#   - 停用自動啟動: 將 AUTO_START_AGV_LAUNCH 設為 false（默認）
#
# 🚀 手動管理 AGV Launch:
#   在容器內執行以下指令:
#   manage_agv_launch start     # 啟動服務
#   manage_agv_launch stop      # 停止服務
#   manage_agv_launch restart   # 重啟服務
#   manage_agv_launch status    # 檢查狀態
#   manage_agv_launch logs      # 查看日誌
#
# =============================================================================

# =============================================================================
# 🔧 AGV Launch 啟動控制（默認關閉，需要時手動啟用）
# =============================================================================
if [ "$AUTO_START_AGV_LAUNCH" = "true" ]; then
    echo "🚗 啟動 AGV Launch 服務..."

    # 等待系統資源就緒
    echo "⏳ 等待系統資源就緒..."
    sleep 5

    # 捕獲錯誤但不退出，確保容器繼續運行
    if manage_agv_launch start; then
        echo "✅ AGV Launch 服務啟動成功"
    else
        echo "⚠️ AGV Launch 服務啟動失敗"
        echo "📝 請查看日誌: tail -f /tmp/agv_launch.log"
        echo "💡 容器仍在運行，您可以透過 SSH 連線進行診斷"
    fi
else
    echo "⏸️ AGV Launch 自動啟動已停用 (AUTO_START_AGV_LAUNCH=false)"
fi

echo ""  # 分隔線

# =============================================================================
# 🔧 Web AGV Launch 服務啟動控制（AGVUI 車載監控界面）
# =============================================================================
# 說明：這是唯一的自動啟動入口，統一管理 AGVUI 服務
# 向後兼容：用戶仍可手動執行 manage_agvui 指令（內部調用 manage_web_agv_launch）
# =============================================================================

if [ "$AUTO_START_WEB_AGV_LAUNCH" = "true" ]; then
    echo "🚀 啟動 Web AGV Launch 服務（AGVUI 車載監控界面）..."

    # 等待系統資源就緒
    echo "⏳ 等待系統資源就緒..."
    sleep 3

    # 捕獲錯誤但不退出，確保容器繼續運行
    if manage_web_agv_launch start; then
        echo "✅ Web AGV Launch 服務啟動成功"
        echo "📍 AGVUI 監控界面: http://$(hostname -I | awk '{print $1}'):8003"
    else
        echo "⚠️ Web AGV Launch 服務啟動失敗"
        echo "📝 請使用以下指令查看錯誤詳情："
        echo "   manage_web_agv_launch status"
        echo "   tail -f /tmp/web_agv_launch.log"
        echo "💡 容器仍在運行，您可以透過 SSH 連線進行診斷"
        # 不執行 exit，讓容器繼續運行
    fi
else
    echo "⏸️ Web AGV Launch 服務自動啟動已停用 (AUTO_START_WEB_AGV_LAUNCH=false)"
fi

# =============================================================================
# 🧹 啟動日誌輪換守護進程
# =============================================================================
echo "🧹 啟動日誌清理守護進程..."
if [ -f "/app/setup_modules/log-cleanup-daemon.bash" ]; then
    # 在背景啟動守護進程，輸出重定向到 /dev/null
    nohup bash /app/setup_modules/log-cleanup-daemon.bash > /dev/null 2>&1 &
    DAEMON_PID=$!

    # 等待 1 秒確保守護進程成功啟動
    sleep 1

    # 檢查守護進程是否仍在運行
    if kill -0 $DAEMON_PID 2>/dev/null; then
        echo "✅ 日誌清理守護進程已啟動 (PID: $DAEMON_PID)"
        echo "   📋 守護進程日誌: /tmp/log-cleanup-daemon.log"
        echo "   ⚙️ 輪轉策略: 每個檔案最大 10MB，保留 5 個版本"
        echo "   ⏰ 執行頻率: 每 6 小時自動檢查並輪轉"
        echo "   📝 查看日誌: tail -f /tmp/log-cleanup-daemon.log"
    else
        echo "⚠️ 日誌清理守護進程啟動後立即退出，請檢查日誌"
        echo "   查看錯誤: cat /tmp/log-cleanup-daemon.log"
    fi
else
    echo "⚠️ 日誌清理守護腳本不存在: /app/setup_modules/log-cleanup-daemon.bash"
    echo "   日誌清理功能未啟動，請手動管理日誌大小"
fi
echo ""

# =============================================================================
# 📖 Web AGV Launch (AGVUI) 使用說明
# =============================================================================
#
# 🎯 服務說明:
#   Web AGV Launch 提供 AGVUI 車載監控界面（端口 8003）
#   這是 AGV 容器中唯一的自動啟動入口，統一管理 AGVUI 服務
#
# 🔧 控制自動啟動:
#   - 啟用: 將 AUTO_START_WEB_AGV_LAUNCH 設為 true（默認）
#   - 停用: 將 AUTO_START_WEB_AGV_LAUNCH 設為 false
#
# 🚀 手動管理服務:
#   manage_web_agv_launch start     # 啟動服務
#   manage_web_agv_launch stop      # 停止服務
#   manage_web_agv_launch restart   # 重啟服務
#   manage_web_agv_launch status    # 檢查狀態
#   manage_web_agv_launch logs      # 查看日誌
#
#   或使用向後相容的別名:
#   manage_agvui start              # 內部調用 manage_web_agv_launch
#
# 📋 技術細節:
#   - 管理函數: manage_web_agv_launch (定義在 setup.bash)
#   - 向後相容別名: manage_agvui (內部調用 manage_web_agv_launch)
#   - 日誌檔案: /tmp/web_agv_launch.log
#   - PID 檔案: /tmp/web_agv_launch.pid
#   - 監控端口: 8003
#
# =============================================================================
