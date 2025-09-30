#!/bin/bash
# /app/startup.bash

# 創建 AGVC 環境標記檔案
echo "AGVC_ENVIRONMENT=true" > /app/.agvc_environment
echo "CONTAINER_TYPE=agvc_server" >> /app/.agvc_environment
echo "STARTUP_TIME=$(date)" >> /app/.agvc_environment

# 🖥️ 統一設備身份識別
echo "🔍 開始統一設備身份識別..."
export CONTAINER_TYPE="agvc"

if [ -f "/app/scripts/config_driven_device_detector.bash" ]; then
    # 執行配置驅動統一身份識別腳本
    source /app/scripts/config_driven_device_detector.bash
    if [ $? -eq 0 ]; then
        echo "✅ AGVC 設備身份識別成功: $DEVICE_ID ($AGVC_TYPE)"
        echo "📁 配置檔案: $DEVICE_CONFIG_FILE"
        echo "🎯 角色: $AGVC_ROLE"
        echo "🔧 工作空間: $AGVC_WORKSPACES"
    else
        echo "⚠️ AGVC 設備身份識別失敗，使用預設配置"
    fi
else
    echo "❌ 統一設備識別腳本不存在，使用預設配置"
    export DEVICE_ID="agvc01"
    export AGVC_ID="agvc01"
    export AGVC_TYPE="primary_controller"
    export AGVC_ROLE="primary"
    export ROS_NAMESPACE="/agvc01"
    export DEVICE_CONFIG_FILE="/app/config/agvc/agvc01_config.yaml"
    export AGVC_WORKSPACES="db_proxy_ws,web_api_ws,ecs_ws,rcs_ws,tafl_ws,tafl_wcs_ws"
fi

# 檢查是否安裝 Node.js (command -v 查詢是否存在指定的命令)
command -v node &> /dev/null
NODE_INSTALLED=$?

# 確認變數是否設定成功
echo "AGVC Startup script is running..."
echo "ROS_DISTRO=$ROS_DISTRO"
echo "ZENOH_ROUTER_CONFIG_URI=$ZENOH_ROUTER_CONFIG_URI"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

# 載入完整的 setup.bash 環境 (包含智能 all_source 和所有工具)
if [ -f "/app/setup.bash" ]; then
    echo "🔧 載入完整 setup.bash 環境..."
    source /app/setup.bash
    echo "✅ setup.bash 環境載入完成"
    
    # 明確載入 AGVC 工作空間 (包含 web_api_launch)
    echo "🔧 載入 AGVC 專用工作空間..."
    agvc_source > /dev/null 2>&1  # 靜默載入避免過多輸出
    echo "✅ AGVC 工作空間載入完成 (包含 web_api_launch)"
else
    echo "❌ setup.bash 不存在，使用基礎環境載入"
    source /opt/ros/$ROS_DISTRO/setup.bash
    source /opt/ws_rmw_zenoh/install/setup.bash
    #ros套件的interfaces source
    source /app/keyence_plc_ws/install/setup.bash
    source /app/plc_proxy_ws/install/setup.bash
fi

#啟動時自動執行的腳本，可以在這裡定義各種函式，並在啟動時自動執行。
#啟動SSH
echo "🚀 啟動 SSH 服務..."
service ssh start

# Zenoh Router 會在 source /app/setup.bash 時自動啟動 (setup.bash 第 1918 行)
# 使用統一的 manage_zenoh 函式管理，確保 PID 檔案格式一致
# 如需手動管理，可使用: manage_zenoh {start|stop|restart|status}

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

# 檢查 Zenoh Router 是否已經運行 (由 manage_zenoh 管理)
# 使用 manage_zenoh status 或直接檢查進程
if pgrep -f "rmw_zenohd" > /dev/null; then
    echo "✅ Zenoh Router 已經在運行中"
else
    echo "⚠️ Zenoh Router 尚未啟動 (將在載入 setup.bash 時自動啟動)"
fi

# =============================================================================
# 🔧 Web API Launch 啟動控制
# =============================================================================

# 設定自動啟動開關 (true=啟動, false=跳過)
AUTO_START_WEB_API_LAUNCH=true

# 根據開關決定是否啟動 Web API Launch
if [ "$AUTO_START_WEB_API_LAUNCH" = "true" ]; then
    echo "🌐 啟動 Web API Launch 服務群組..."
    # 捕獲錯誤但不退出，確保容器繼續運行
    if manage_web_api_launch start; then
        echo "✅ Web API Launch 服務群組啟動成功"
    else
        echo "⚠️ Web API Launch 服務群組啟動失敗"
        echo "📝 請使用以下指令查看錯誤詳情："
        echo "   tail -f /tmp/web_api_launch.log"
        echo "💡 容器仍在運行，您可以透過 SSH 連線進行診斷"
        # 不執行 exit，讓容器繼續運行
    fi
else
    echo "⏸️ Web API Launch 自動啟動已停用 (AUTO_START_WEB_API_LAUNCH=false)"
fi

# =============================================================================
# 📖 使用說明
# =============================================================================
#
# 🔧 控制 Web API Launch 自動啟動:
#   - 啟用自動啟動: 將 AUTO_START_WEB_API_LAUNCH 設為 true
#   - 停用自動啟動: 將 AUTO_START_WEB_API_LAUNCH 設為 false
#
# 🚀 手動管理 Web API Launch:
#   在容器內執行以下指令:
#   manage_web_api_launch start     # 啟動服務
#   manage_web_api_launch stop      # 停止服務
#   manage_web_api_launch restart   # 重啟服務
#   manage_web_api_launch status    # 檢查狀態
#
# 📋 函式說明:
#   manage_web_api_launch {start|stop|restart|status}
#   - 統一的服務管理介面 (定義在 setup.bash 中)
#   - 自動檢查重複啟動
#   - 完整的健康檢查和端口驗證
#   - 支援啟動、停止、重啟、狀態檢查等操作
#
# =============================================================================

echo "📦 工作空間載入完成，準備啟動 AGVC 專用服務..."
