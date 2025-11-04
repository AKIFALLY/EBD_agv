#!/bin/bash
# RosAGV Workspace Loader Module
# 包含工作空間載入、檔案同步和幫助文檔函數

# ============================================================================
# 遠端檔案同步函數
# ============================================================================

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

# ============================================================================
# 工作空間載入函數
# ============================================================================

# AGV 專用工作空間載入函數
agv_source() {
    echo "🚗 載入 AGV 車載系統專用工作空間..."

    # 確保 ROS 2 環境已載入
    if [ -z "$ROS_DISTRO" ]; then
        echo "⚠️ ROS 2 環境未載入，先載入基礎環境..."
        if [ -f "/opt/ros/jazzy/setup.bash" ]; then
            source /opt/ros/jazzy/setup.bash
        fi
        if [ -f "/opt/ws_rmw_zenoh/install/setup.bash" ]; then
            source /opt/ws_rmw_zenoh/install/setup.bash
        fi
    fi

    # AGV 車載系統專用工作空間 (按依賴順序排列)
    local agv_base_workspaces=(
        "/app/shared_constants_ws/install"  # 共享常數 (最優先)
        "/app/keyence_plc_ws/install"
        "/app/plc_proxy_ws/install"
        "/app/path_algorithm/install"
        "/app/db_proxy_ws/install"
    )

    local agv_app_workspaces=(
        "/app/agv_cmd_service_ws/install"
        "/app/joystick_ws/install"
        "/app/agv_ws/install"
        "/app/sensorpart_ws/install"
        "/app/uno_gpio_ws/install"
        "/app/web_api_ws/install"      # AGVUI 依賴（agv_ui_server）
        "/app/launch_ws/install"
    )

    # 載入 AGV 基礎工作空間
    echo "📦 載入 AGV 基礎工作空間..."
    for ws in "${agv_base_workspaces[@]}"; do
        if [ -d "$ws" ]; then
            echo "✅ Sourcing $(basename $(dirname $ws))"
            source "$ws/setup.bash"
        else
            echo "⚠️ Warning: $(basename $(dirname $ws)) 不存在，需要先建置"
        fi
    done

    # 載入 AGV 應用工作空間
    echo "🚀 載入 AGV 應用工作空間..."
    for ws in "${agv_app_workspaces[@]}"; do
        if [ -d "$ws" ]; then
            echo "✅ Sourcing $(basename $(dirname $ws))"
            source "$ws/setup.bash"
        else
            echo "⚠️ Warning: $(basename $(dirname $ws)) 不存在，需要先建置"
        fi
    done

    echo "✅ AGV 專用工作空間載入完成"
}

# AGVC 專用工作空間載入函數
agvc_source() {
    echo "🖥️ 載入 AGVC 管理系統專用工作空間..."

    # 確保 ROS 2 環境已載入
    if [ -z "$ROS_DISTRO" ]; then
        echo "⚠️ ROS 2 環境未載入，先載入基礎環境..."
        if [ -f "/opt/ros/jazzy/setup.bash" ]; then
            source /opt/ros/jazzy/setup.bash
        fi
        if [ -f "/opt/ws_rmw_zenoh/install/setup.bash" ]; then
            source /opt/ws_rmw_zenoh/install/setup.bash
        fi
    fi

    # AGVC 管理系統專用工作空間 (按依賴順序排列)
    local agvc_base_workspaces=(
        "/app/shared_constants_ws/install"  # 共享常數 (最優先)
        "/app/keyence_plc_ws/install"
        "/app/plc_proxy_ws/install"
        "/app/path_algorithm/install"
        "/app/agv_ws/install"
        "/app/db_proxy_ws/install"
    )

    local agvc_app_workspaces=(
        "/app/ecs_ws/install"
        "/app/rcs_ws/install"
        "/app/tafl_ws/install"       # TAFL parser and executor (新一代 WCS 基礎)
        "/app/tafl_wcs_ws/install"   # TAFL WCS integration (目前使用的 WCS 實作)
        "/app/web_api_ws/install"
        "/app/kuka_fleet_ws/install"
        "/app/launch_ws/install"
        "/app/wcs_ws/install"
    )

    # 載入 AGVC 基礎工作空間
    echo "📦 載入 AGVC 基礎工作空間..."
    for ws in "${agvc_base_workspaces[@]}"; do
        if [ -d "$ws" ]; then
            echo "✅ Sourcing $(basename $(dirname $ws))"
            source "$ws/setup.bash"
        else
            echo "⚠️ Warning: $(basename $(dirname $ws)) 不存在，需要先建置"
        fi
    done

    # 載入 AGVC 應用工作空間
    echo "🚀 載入 AGVC 應用工作空間..."
    for ws in "${agvc_app_workspaces[@]}"; do
        if [ -d "$ws" ]; then
            echo "✅ Sourcing $(basename $(dirname $ws))"
            source "$ws/setup.bash"
        else
            echo "⚠️ Warning: $(basename $(dirname $ws)) 不存在，需要先建置"
        fi
    done

    echo "✅ AGVC 專用工作空間載入完成"
}

# 自動工作空間載入函數 (根據環境自動選擇)
all_source() {
    echo "🔧 自動載入工作空間 (根據容器環境自動選擇)..."

    # 檢測當前環境並選擇對應的載入策略
    if [ "$CONTAINER_TYPE" = "agv" ]; then
        echo "🚗 檢測到 AGV 車載環境，載入 AGV 專用工作空間"
        agv_source
    elif [ "$CONTAINER_TYPE" = "agvc" ]; then
        echo "🖥️ 檢測到 AGVC 管理環境，載入 AGVC 專用工作空間"
        agvc_source
    elif is_agvc_environment; then
        echo "🖥️ 檢測到 AGVC 管理環境，載入 AGVC 專用工作空間"
        agvc_source
    else
        echo "🔄 無法確定環境類型，載入 AGV 工作空間 (預設)"
        agv_source
    fi
}

# ============================================================================
# 幫助和文檔函數
# ============================================================================

# 顯示幫助資訊
show_help() {
    log_header "RosAGV 開發環境 - 可用命令"

    echo -e "${CYAN}🔧 建置和測試:${NC}"
    echo "  build_all/ba           - 自動建置工作空間 (根據容器類型自動選擇)"
    echo "  build_agv              - 建置 AGV 車載系統專用工作空間"
    echo "  build_agvc             - 建置 AGVC 管理系統專用工作空間"
    echo "  build_all_workspaces   - 建置所有工作空間 (傳統方式)"
    echo "  build_all_smart/bas    - 依賴解析建置 (使用 colcon 依賴解析)"
    echo "  build_ws/build_single  - 建置指定的單一工作空間"
    echo "  test_all/ta            - 測試所有工作空間"
    echo "  test_ws/test_single    - 測試指定的單一工作空間"
    echo "  clean_all/ca           - 清理所有建置檔案"
    echo "  clean_ws/clean_single  - 清理指定的單一工作空間"
    echo "  all_source/sa/load_all - 自動載入工作空間環境 (根據容器類型自動選擇)"
    echo "  agv_source             - 載入 AGV 車載系統專用工作空間"
    echo "  agvc_source            - 載入 AGVC 管理系統專用工作空間"
    echo ""

    echo -e "${CYAN}📊 狀態和監控:${NC}"
    echo "  check_system_status/status - 檢查系統狀態"
    echo "  check_ros_env              - 檢查 ROS 2 環境"
    echo "  check_zenoh_status         - 檢查 Zenoh 狀態"
    echo "  check_status               - 智能狀態檢查（根據容器類型自動選擇）"
    if is_agvc_environment; then
        echo "  check_agvc_status          - 檢查 AGVC 系統狀態"
        echo "  manage_all_nodes status    - 查看所有 AGVC 節點狀態"
    else
        echo "  check_agv_status           - 檢查 AGV 容器狀態概覽"
        echo "  manage_agv_launch status   - 查看 AGV Launch 服務狀態"
    fi
    echo ""

    echo -e "${CYAN}⚙️  服務管理:${NC}"
    echo "  manage                 - 顯示所有服務狀態（整合列表）"
    echo "  manage_ssh <action>    - SSH 服務管理 (start|stop|restart|status)"
    echo "  manage_zenoh <action>  - Zenoh Router 管理 (start|stop|restart|status)"

    if is_agvc_environment; then
        echo "  manage_all_nodes <cmd>     - 統一管理所有 AGVC 節點 (start|stop|restart|status)"
        echo "  manage_web_api_launch <action> - Web API Launch 管理 (start|stop|restart|status)"
        echo "  manage_agvui <action>      - AGVUI 車載監控管理 (start|stop|restart|status|logs)"
        echo "  manage_tafl_wcs <action>   - TAFL WCS 節點管理 (start|stop|restart|status|logs)"
        echo "  manage_rcs_core <cmd>      - RCS 核心節點管理"
        echo "  manage_agvc_database_node  - AGVC 資料庫節點管理"
        echo "  manage_room_task_build     - Room Task Build 節點管理"
        echo "  start_db/stop_db           - 啟動/停止資料庫服務"
        echo "  start_ecs                  - 啟動 ECS 設備控制系統"
    else
        echo "  manage_agv_launch <action> - AGV Launch 服務管理 (start|stop|restart|status|logs)"
        echo "  local_agv/lagv <action>    - manage_agv_launch 的簡化別名"
        echo ""
        echo "  ${YELLOW}💡 AGVC 專用命令（在 AGV 容器不可用）:${NC}"
        echo "     manage_web_api_launch, manage_tafl_wcs, manage_agvui,"
        echo "     manage_agvc_database_node, manage_room_task_build"
    fi
    echo ""

    echo -e "${CYAN}🌐 網路和同步:${NC}"
    echo "  ping_all               - 檢查遠端主機連線"
    echo "  app_upload <IP>        - 同步檔案到遠端主機"
    echo "  app_download <IP>      - 從遠端主機下載檔案"
    echo ""

    echo -e "${CYAN}📚 幫助:${NC}"
    echo "  show_help/help         - 顯示此幫助資訊"
    echo ""

    echo -e "${YELLOW}💡 使用範例:${NC}"
    echo "  build_single agv_ws    # 建置 AGV 工作空間"
    echo "  test_single db_proxy_ws # 測試資料庫代理"
    echo "  clean_single rcs_ws    # 清理 RCS 工作空間"
    echo "  build1 web_api_ws      # 使用別名建置 Web API"
    echo ""
}
