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

# =============================================================================
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

# 載入完整的 setup.bash 環境 (包含all_source 和所有工具)
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

# ⚠️ SSH 和 Zenoh Router 的啟動已移至後續的 AUTO_START 控制區塊
# 請參考下方的「服務自動啟動配置」區段

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

# ⚠️ 服務啟動檢查已整合至 AUTO_START 控制區塊
# 每個服務啟動時會自動檢查狀態並回報結果

# =============================================================================
# 🔧 服務自動啟動配置 (所有服務默認啟用)
# =============================================================================

# 設定各服務的自動啟動開關 (true=啟動, false=跳過)
AUTO_START_SSH=true                    # SSH 服務（基礎設施）
AUTO_START_ZENOH=true                  # Zenoh Router（通訊層）
AUTO_START_AGVC_DATABASE_NODE=true     # AGVC 資料庫代理節點（資料層）
AUTO_START_PLC_SERVICE_AGVC=true       # PLC 服務（硬體接口層）
AUTO_START_ECS_CORE=true               # ECS 核心服務（設備控制層）
AUTO_START_RCS_CORE=true               # RCS 核心服務（機器人控制層）
AUTO_START_TAFL_WCS=false              # TAFL WCS 節點（流程控制層，已停用）
AUTO_START_KUKA_WCS=true               # KUKA WCS 節點（KUKA 任務自動化層）
AUTO_START_ROOM_TASK_BUILD=true        # 房間任務構建節點（任務構建層）
AUTO_START_TRANSFER_BOX_TASK_BUILD=true # 傳送箱任務構建節點（任務構建層）
AUTO_START_WEB_API_LAUNCH=true         # Web API Launch 服務群組（用戶界面層）

# =============================================================================
# 🚀 服務啟動流程
# =============================================================================
echo "⚙️ 開始啟動已啟用的 AGVC 服務..."
echo ""

# SSH 服務
if [ "$AUTO_START_SSH" = "true" ]; then
    echo "🔐 啟動 SSH 服務..."
    if manage_ssh start; then
        echo "✅ SSH 服務啟動成功"
    else
        echo "⚠️ SSH 服務啟動失敗"
    fi
    echo ""
fi

# Zenoh Router
if [ "$AUTO_START_ZENOH" = "true" ]; then
    echo "🌐 啟動 Zenoh Router..."
    if manage_zenoh start; then
        echo "✅ Zenoh Router 啟動成功"
    else
        echo "⚠️ Zenoh Router 啟動失敗"
    fi
    echo ""
fi

# AGVC Database Node
if [ "$AUTO_START_AGVC_DATABASE_NODE" = "true" ]; then
    echo "🗄️ 啟動 AGVC 資料庫代理節點..."
    if manage_agvc_database_node start; then
        echo "✅ 資料庫節點啟動成功"
    else
        echo "⚠️ 資料庫節點啟動失敗"
    fi
    echo ""
fi

# PLC Service
if [ "$AUTO_START_PLC_SERVICE_AGVC" = "true" ]; then
    echo "🔌 啟動 PLC 服務..."
    if manage_plc_service_agvc start; then
        echo "✅ PLC 服務啟動成功"
    else
        echo "⚠️ PLC 服務啟動失敗"
    fi
    echo ""
fi

# ECS Core
if [ "$AUTO_START_ECS_CORE" = "true" ]; then
    echo "🚪 啟動 ECS 核心服務..."
    if manage_ecs_core start; then
        echo "✅ ECS 核心啟動成功"
    else
        echo "⚠️ ECS 核心啟動失敗"
    fi
    echo ""
fi

# RCS Core
if [ "$AUTO_START_RCS_CORE" = "true" ]; then
    echo "🤖 啟動 RCS 核心服務..."
    if manage_rcs_core start; then
        echo "✅ RCS 核心啟動成功"
    else
        echo "⚠️ RCS 核心啟動失敗"
    fi
    echo ""
fi

# TAFL WCS
if [ "$AUTO_START_TAFL_WCS" = "true" ]; then
    echo "⚙️ 啟動 TAFL WCS 節點..."
    if manage_tafl_wcs start; then
        echo "✅ TAFL WCS 啟動成功"
    else
        echo "⚠️ TAFL WCS 啟動失敗"
    fi
    echo ""
fi

# KUKA WCS
if [ "$AUTO_START_KUKA_WCS" = "true" ]; then
    echo "🏭 啟動 KUKA WCS 節點..."
    if manage_kuka_wcs start; then
        echo "✅ KUKA WCS 啟動成功"
    else
        echo "⚠️ KUKA WCS 啟動失敗"
    fi
    echo ""
fi

# Room Task Build
if [ "$AUTO_START_ROOM_TASK_BUILD" = "true" ]; then
    echo "🏗️ 啟動房間任務構建節點..."
    if manage_room_task_build start; then
        echo "✅ 房間任務啟動成功"
    else
        echo "⚠️ 房間任務啟動失敗"
    fi
    echo ""
fi

# Transfer Box Task Build
if [ "$AUTO_START_TRANSFER_BOX_TASK_BUILD" = "true" ]; then
    echo "📦 啟動傳送箱任務構建節點..."
    if manage_transfer_box_task_build start; then
        echo "✅ 傳送箱任務啟動成功"
    else
        echo "⚠️ 傳送箱任務啟動失敗"
    fi
    echo ""
fi

# Web API Launch
if [ "$AUTO_START_WEB_API_LAUNCH" = "true" ]; then
    echo "🌐 啟動 Web API Launch 服務群組..."
    if manage_web_api_launch start; then
        echo "✅ Web API Launch 啟動成功"
    else
        echo "⚠️ Web API Launch 啟動失敗"
    fi
    echo ""
fi

# =============================================================================
# 🧹 啟動日誌清理守護進程
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
