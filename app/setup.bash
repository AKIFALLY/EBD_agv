#!/bin/bash
# RosAGV 專案環境設定腳本
# 支援互動式和非互動式 shell

# ============================================================================
# 顏色定義和日誌系統
# ============================================================================

# 顏色定義
export RED='\033[0;31m'
export GREEN='\033[0;32m'
export YELLOW='\033[1;33m'
export BLUE='\033[0;34m'
export PURPLE='\033[0;35m'
export CYAN='\033[0;36m'
export WHITE='\033[1;37m'
export NC='\033[0m' # No Color

# 日誌函數
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_debug() {
    [[ "${LOG_LEVEL:-INFO}" == "DEBUG" ]] && echo -e "${PURPLE}[DEBUG]${NC} $1"
}

log_header() {
    echo -e "${CYAN}╔══════════════════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║${NC} ${WHITE}$1${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════════════════════════════════╝${NC}"
}

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

# 測試所有工作空間
test_all() {
    log_info "開始測試所有工作空間..."

    # 定義要測試的 workspace 路徑
    local workspaces=(
        "/app/keyence_plc_ws"
        "/app/plc_proxy_ws"
        "/app/agv_cmd_service_ws"
        "/app/joystick_ws"
        "/app/agv_ws"
        "/app/db_proxy_ws"
        "/app/ecs_ws"
        "/app/rcs_ws"
        "/app/wcs_ws"
        "/app/web_api_ws"
        "/app/kuka_fleet_ws"
        "/app/launch_ws"
        "/app/sensorpart_ws"
    )

    local success_count=0
    local total_count=${#workspaces[@]}

    # 逐一測試
    for ws in "${workspaces[@]}"; do
        if [ -d "$ws" ]; then
            log_info "測試工作空間: $(basename $ws)"
            cd "$ws" || continue

            # 檢查是否已建置
            if [ ! -d "install" ]; then
                log_warning "工作空間未建置，跳過測試"
                continue
            fi

            # 載入環境
            if [ -f "install/setup.bash" ]; then
                source install/setup.bash
            fi

            # 執行測試
            if colcon test --event-handlers console_direct+; then
                log_success "$(basename $ws) 測試通過"
                ((success_count++))
            else
                log_error "$(basename $ws) 測試失敗"
            fi
        else
            log_warning "工作空間不存在: $ws"
        fi
    done

    log_info "測試完成: $success_count/$total_count 個工作空間通過測試"
}

# 測試單一工作空間
test_ws() {
    local workspace_name="$1"

    if [ -z "$workspace_name" ]; then
        log_error "請提供工作空間名稱"
        echo "用法: test_ws/test_single <workspace_name>"
        echo "範例: test_single agv_ws"
        return 1
    fi

    local workspace_path="/app/$workspace_name"

    if [ ! -d "$workspace_path" ]; then
        log_error "工作空間不存在: $workspace_path"
        return 1
    fi

    log_info "測試工作空間: $workspace_name"
    cd "$workspace_path" || return 1

    if [ ! -d "install" ]; then
        log_warning "工作空間未建置，先執行建置..."
        colcon build
    fi

    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    fi

    colcon test --event-handlers console_direct+
}

# 測試單一工作空間 (別名)
test_single() {
    test_ws "$@"
}


# 建置所有工作空間
build_all() {
    echo "🔧 開始建置所有工作空間..."

    BASE_DIR="/app/"
    local success_count=0
    local total_count=0

    # 查找所有 _ws 結尾的資料夾並執行 colcon build
    for dir in "$BASE_DIR"/*_ws; do
        if [ -d "$dir" ]; then
            ((total_count++))
            local ws_name=$(basename "$dir")
            echo "🔨 建置工作空間: $ws_name"

            # 進入資料夾並執行 colcon build
            cd "$dir" || continue

            if colcon build --event-handlers console_direct+; then
                echo "✅ $ws_name 建置成功"
                ((success_count++))
            else
                echo "❌ $ws_name 建置失敗"
            fi

            cd "$BASE_DIR" || continue
        fi
    done

    echo "📊 建置完成: $success_count/$total_count 個工作空間建置成功"
}

# 建置單一工作空間
build_ws() {
    local workspace_name="$1"

    if [ -z "$workspace_name" ]; then
        log_error "請提供工作空間名稱"
        echo "用法: build_ws/build_single <workspace_name>"
        echo "範例: build_single agv_ws"
        return 1
    fi

    local workspace_path="/app/$workspace_name"

    if [ ! -d "$workspace_path" ]; then
        log_error "工作空間不存在: $workspace_path"
        return 1
    fi

    log_info "建置工作空間: $workspace_name"
    cd "$workspace_path" || return 1

    if colcon build --event-handlers console_direct+; then
        log_success "$workspace_name 建置成功"

        # 自動載入環境
        if [ -f "install/setup.bash" ]; then
            source install/setup.bash
            log_success "環境已載入"
        fi

        return 0
    else
        log_error "$workspace_name 建置失敗"
        return 1
    fi
}

# 建置單一工作空間 (別名)
build_single() {
    build_ws "$@"
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

# 清理所有工作空間
clean_all() {
    echo "🧹 開始清理所有工作空間..."

    BASE_DIR="/app/"
    local success_count=0
    local total_count=0

    # 查找所有 _ws 結尾的資料夾並執行清理
    for dir in "$BASE_DIR"/*_ws; do
        if [ -d "$dir" ]; then
            ((total_count++))
            local ws_name=$(basename "$dir")
            echo "🧹 清理工作空間: $ws_name"

            # 進入資料夾並執行清理
            cd "$dir" || continue

            if rm -rf build/ install/ log/; then
                echo "✅ $ws_name 清理成功"
                ((success_count++))
            else
                echo "❌ $ws_name 清理失敗"
            fi

            cd "$BASE_DIR" || continue
        fi
    done

    echo "📊 清理完成: $success_count/$total_count 個工作空間清理成功"
}

# 清理單一工作空間
clean_ws() {
    local workspace_name="$1"

    if [ -z "$workspace_name" ]; then
        log_error "請提供工作空間名稱"
        echo "用法: clean_ws/clean_single <workspace_name>"
        echo "範例: clean_single agv_ws"
        return 1
    fi

    local workspace_path="/app/$workspace_name"

    if [ ! -d "$workspace_path" ]; then
        log_error "工作空間不存在: $workspace_path"
        return 1
    fi

    log_info "清理工作空間: $workspace_name"
    cd "$workspace_path" || return 1

    if rm -rf build/ install/ log/; then
        log_success "$workspace_name 清理成功"
        return 0
    else
        log_error "$workspace_name 清理失敗"
        return 1
    fi
}

# 清理單一工作空間 (別名)
clean_single() {
    clean_ws "$@"
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
    ## 自動載入設備識別與環境變數
    #if [ -f "/app/scripts/config_driven_device_detector.bash" ]; then
    #    source /app/scripts/config_driven_device_detector.bash
    #else
    #    echo "⚠️ 找不到設備識別腳本 /app/scripts/config_driven_device_detector.bash"
    #fi
    #echo "🔧 載入所有工作空間 (按依賴順序)..."

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

    # 設定要載入的 workspace 路徑 (按依賴順序排列)
    # 基礎工作空間 (被其他工作空間依賴)
    local base_workspaces=(
        "/app/keyence_plc_ws/install"
        "/app/plc_proxy_ws/install"
        "/app/path_algorithm/install"
        "/app/db_proxy_ws/install"
    )

    # 應用工作空間 (依賴基礎工作空間)
    local app_workspaces=(
        "/app/ecs_ws/install"
        "/app/agv_cmd_service_ws/install"
        "/app/joystick_ws/install"
        "/app/agv_ws/install"
        "/app/rcs_ws/install"
        "/app/wcs_ws/install"
        "/app/web_api_ws/install"
        "/app/kuka_fleet_ws/install"
        "/app/launch_ws/install"
        "/app/sensorpart_ws/install"
        "/app/uno_gpio_ws/install"
    )

    # 載入基礎工作空間
    echo "📦 載入基礎工作空間..."
    for ws in "${base_workspaces[@]}"; do
        if [ -d "$ws" ]; then
            echo "✅ Sourcing $(basename $(dirname $ws))"
            source "$ws/setup.bash"
        else
            echo "⚠️ Warning: $(basename $(dirname $ws)) 不存在，需要先建置"
        fi
    done

    # 載入應用工作空間
    echo "🚀 載入應用工作空間..."
    for ws in "${app_workspaces[@]}"; do
        if [ -d "$ws" ]; then
            echo "✅ Sourcing $(basename $(dirname $ws))"
            source "$ws/setup.bash"
        else
            echo "⚠️ Warning: $(basename $(dirname $ws)) 不存在，需要先建置"
        fi
    done

    echo "✅ 所有工作空間載入完成"
}

# ============================================================================
# 幫助和文檔函數
# ============================================================================

# 顯示幫助資訊
show_help() {
    log_header "RosAGV 開發環境 - 可用命令"

    echo -e "${CYAN}🔧 建置和測試:${NC}"
    echo "  build_all/ba           - 建置所有工作空間"
    echo "  build_ws/build_single  - 建置指定的單一工作空間"
    echo "  test_all/ta            - 測試所有工作空間"
    echo "  test_ws/test_single    - 測試指定的單一工作空間"
    echo "  clean_all/ca           - 清理所有建置檔案"
    echo "  clean_ws/clean_single  - 清理指定的單一工作空間"
    echo "  all_source/sa/load_all - 載入所有工作空間環境"
    echo ""

    echo -e "${CYAN}📊 狀態和監控:${NC}"
    echo "  check_system_status/status - 檢查系統狀態"
    echo "  check_ros_env              - 檢查 ROS 2 環境"
    echo "  check_zenoh_status         - 檢查 Zenoh 狀態"
    echo "  check_agvc_status          - 檢查 AGVC 系統狀態"
    echo ""

    echo -e "${CYAN}⚙️  服務管理:${NC}"
    echo "  manage_ssh <action>    - SSH 服務管理 (start|stop|restart|status)"
    echo "  manage_zenoh <action>  - Zenoh Router 管理 (start|stop|restart|status)"
    echo "  agvc_source            - 載入 AGVC 專用工作空間"
    if is_agvc_environment; then
        echo "  start_db/stop_db       - 啟動/停止資料庫服務 (僅 AGVC 環境)"
        echo "  start_ecs              - 啟動 ECS 設備控制系統 (僅 AGVC 環境)"
        echo "  check_agvc_status      - 檢查 AGVC 系統狀態 (僅 AGVC 環境)"
    else
        echo "  start_db/stop_db       - 啟動/停止資料庫服務 (僅限 AGVC 環境)"
        echo "  start_ecs              - 啟動 ECS 設備控制系統 (僅限 AGVC 環境)"
        echo "  check_agvc_status      - 檢查 AGVC 系統狀態 (僅限 AGVC 環境)"
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

# ===== ROS 2 環境載入 =====
# 載入 ROS 2 基礎環境
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✅ 載入 ROS 2 Jazzy 環境"
else
    echo "❌ ROS 2 Jazzy 環境不存在"
fi

# 載入 rmw_zenoh 環境
if [ -f "/opt/ws_rmw_zenoh/install/setup.bash" ]; then
    source /opt/ws_rmw_zenoh/install/setup.bash
    echo "✅ 載入 rmw_zenoh 環境"
else
    echo "❌ rmw_zenoh 環境不存在"
fi

# 設定虛擬環境 PYTHONPATH
if [ -d "/opt/pyvenv_env/lib/python3.12/site-packages" ]; then
    export PYTHONPATH="/opt/pyvenv_env/lib/python3.12/site-packages:$PYTHONPATH"
    echo "✅ 設定虛擬環境 PYTHONPATH"
else
    echo "⚠️ 虛擬環境路徑不存在"
fi

# 設定 Zenoh 相關環境變數
export ZENOH_ROUTER_CONFIG_URI="/app/routerconfig.json5"
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"

# 確認環境變數設定
echo "✅ 設定 ZENOH_ROUTER_CONFIG_URI=$ZENOH_ROUTER_CONFIG_URI"
echo "✅ 設定 RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

# ===== 狀態檢查函數 =====

# 檢查 ROS 2 環境
check_ros_env() {
    echo "🔍 檢查 ROS 2 環境..."

    # 檢查 ROS_DISTRO
    if [ -n "$ROS_DISTRO" ]; then
        echo "✅ ROS_DISTRO: $ROS_DISTRO"
    else
        echo "❌ ROS_DISTRO 未設定"
        return 1
    fi

    # 檢查 RMW_IMPLEMENTATION
    if [ "$RMW_IMPLEMENTATION" = "rmw_zenoh_cpp" ]; then
        echo "✅ RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
    else
        echo "⚠️ RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-未設定} (預期: rmw_zenoh_cpp)"
    fi

    # 檢查 ros2 指令
    if command -v ros2 > /dev/null; then
        echo "✅ ros2 指令可用"
    else
        echo "❌ ros2 指令不可用"
        return 1
    fi

    # 檢查 Python 虛擬環境
    if [ -n "$VIRTUAL_ENV" ]; then
        echo "✅ Python 虛擬環境: $VIRTUAL_ENV"
    else
        echo "⚠️ Python 虛擬環境未啟用"
    fi

    echo "✅ ROS 2 環境檢查完成"
}

# 檢查 Zenoh 狀態
check_zenoh_status() {
    echo "🔍 檢查 Zenoh Router 狀態..."

    # 檢查程序是否運行
    if pgrep -f rmw_zenohd > /dev/null; then
        local pid=$(pgrep -f rmw_zenohd)
        echo "✅ Zenoh Router 運行中 (PID: $pid)"
    else
        echo "❌ Zenoh Router 未運行"
        return 1
    fi

    # 檢查 PID 檔案
    if [ -f "/tmp/zenoh_router.pid" ]; then
        local stored_pid=$(cat /tmp/zenoh_router.pid)
        echo "✅ PID 檔案存在: $stored_pid"
    else
        echo "⚠️ PID 檔案不存在"
    fi

    # 檢查日誌檔案
    if [ -f "/tmp/zenoh_router.log" ]; then
        echo "✅ 日誌檔案存在"
        echo "📝 最近日誌:"
        tail -3 /tmp/zenoh_router.log | sed 's/^/   /'
    else
        echo "⚠️ 日誌檔案不存在"
    fi

    # 檢查配置檔案
    if [ -f "/app/routerconfig.json5" ]; then
        echo "✅ Zenoh 配置檔案存在"
    else
        echo "❌ Zenoh 配置檔案不存在"
        return 1
    fi

    # 檢查端口
    if netstat -tuln 2>/dev/null | grep ":7447 " > /dev/null; then
        echo "✅ Zenoh Router 端口 7447 已開啟"
    else
        echo "⚠️ Zenoh Router 端口 7447 未開啟"
    fi

    echo "✅ Zenoh 狀態檢查完成"
}

# 系統整體狀態檢查
check_system_status() {
    echo "🔍 系統整體狀態檢查..."
    echo "========================================"

    # 基礎服務檢查
    echo ""
    echo "=== 基礎服務 ==="

    # SSH 服務
    if pgrep -f sshd > /dev/null; then
        echo "✅ SSH 服務運行中"
    else
        echo "❌ SSH 服務未運行"
    fi

    # Zenoh Router
    check_zenoh_status

    # ROS 2 環境
    echo ""
    echo "=== ROS 2 環境 ==="
    check_ros_env

    # 工作空間狀態
    echo ""
    echo "=== 工作空間狀態 ==="
    local built_count=0
    local total_count=0

    for dir in /app/*_ws; do
        if [ -d "$dir" ]; then
            ((total_count++))
            local ws_name=$(basename "$dir")
            if [ -d "$dir/install" ]; then
                echo "✅ $ws_name (已建置)"
                ((built_count++))
            else
                echo "⚠️ $ws_name (未建置)"
            fi
        fi
    done

    echo ""
    echo "========================================"
    echo "📊 工作空間統計: $built_count/$total_count 已建置"
    echo "✅ 系統狀態檢查完成"
}

# 自動啟動 Zenoh Router
manage_zenoh start

# ===== 便捷別名 =====

# 為常用指令創建別名
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'
alias ..='cd ..'
alias ...='cd ../..'

# ROS 2 相關別名
alias rn='ros2 node list'
alias rt='ros2 topic list'
alias rs='ros2 service list'
alias ri='ros2 interface list'

# 工作空間管理別名
alias ba='build_all'
alias ta='test_all'
alias ca='clean_all'
alias sa='all_source'
alias load_all='all_source'  # 簡化的載入指令

# 單一工作空間操作別名
alias build1='build_single'
alias test1='test_single'
alias clean1='clean_single'

# 幫助和狀態別名
alias help='show_help'
alias status='check_system_status'

# 狀態檢查別名
alias status='check_system_status'
alias zenoh='check_zenoh_status'
alias rosenv='check_ros_env'

# 別名載入提示 (僅在互動式 shell 中顯示)
if [[ $- == *i* ]]; then
    log_debug "別名已載入: status, zenoh, rosenv, help, build1, test1, clean1"
fi

# ===== AGVC 專用函數 =====

# 檢測當前是否為 AGVC 環境
is_agvc_environment() {
    # 主要檢測方法：檢查 Docker Compose 設定的 CONTAINER_TYPE 環境變數
    # 這是最簡單、最可靠的檢測方式
    if [ "$CONTAINER_TYPE" = "agvc" ]; then
        return 0  # 是 AGVC 環境
    fi

    return 1  # 是 AGV 環境
}

# AGVC 專用工作空間載入函數
agvc_source() {
    echo "🔧 載入 AGVC 管理系統專用工作空間..."

    # AGVC 管理系統專用工作空間
    local agvc_workspaces=(
        "/app/db_proxy_ws/install"
        "/app/ecs_ws/install"
        "/app/rcs_ws/install"
        "/app/wcs_ws/install"
        "/app/web_api_ws/install"
        "/app/kuka_fleet_ws/install"
    )

    # 載入 AGVC 專用工作空間
    for ws in "${agvc_workspaces[@]}"; do
        if [ -d "$ws" ]; then
            echo "✅ Sourcing $ws/setup.bash"
            source "$ws/setup.bash"
        else
            echo "⚠️ Warning: $ws 不存在，需要先建置"
        fi
    done

    echo "✅ AGVC 專用工作空間載入完成"
}

# 啟動資料庫服務
start_db() {
    if ! is_agvc_environment; then
        echo "❌ 此功能僅適用於 AGVC 環境"
        return 1
    fi

    echo "🚀 啟動 PostgreSQL 資料庫服務..."

    # 檢查 PostgreSQL 是否已運行
    if pg_isready -h localhost -p 5432 >/dev/null 2>&1; then
        echo "✅ PostgreSQL 已經在運行中"
        return 0
    fi

    # 啟動 PostgreSQL 服務
    sudo service postgresql start

    # 等待服務啟動
    local timeout=30
    local count=0
    while [ $count -lt $timeout ]; do
        if pg_isready -h localhost -p 5432 >/dev/null 2>&1; then
            echo "✅ PostgreSQL 啟動成功"
            return 0
        fi
        echo "⏳ 等待 PostgreSQL 啟動... ($count/$timeout)"
        sleep 1
        ((count++))
    done

    echo "❌ PostgreSQL 啟動失敗"
    return 1
}

# 停止資料庫服務
stop_db() {
    if ! is_agvc_environment; then
        echo "❌ 此功能僅適用於 AGVC 環境"
        return 1
    fi

    echo "⏳ 停止 PostgreSQL 資料庫服務..."

    if pg_isready -h localhost -p 5432 >/dev/null 2>&1; then
        sudo service postgresql stop
        echo "✅ PostgreSQL 已停止"
    else
        echo "❌ PostgreSQL 未運行"
    fi
}

# 手動啟動 ECS 服務
start_ecs() {
    if ! is_agvc_environment; then
        echo "❌ 此功能僅適用於 AGVC 環境"
        return 1
    fi

    echo "🚀 啟動 ECS 設備控制系統..."

    # 檢查依賴是否可用
    if ! python3 -c "from plc_proxy.plc_client import PlcClient; from keyence_plc.keyence_plc_memory import PlcMemory" 2>/dev/null; then
        echo "❌ ECS 依賴不可用，請確保 plc_proxy_ws 和 keyence_plc_ws 已載入"
        return 1
    fi

    # 建置 ECS 工作空間（如果需要）
    if [ ! -d "/app/ecs_ws/install" ]; then
        echo "🔧 建置 ECS 工作空間..."
        cd /app/ecs_ws
        colcon build
        if [ $? -ne 0 ]; then
            echo "❌ ECS 建置失敗"
            return 1
        fi
    fi

    # 啟動 ECS 服務
    cd /app/ecs_ws
    source install/setup.bash

    # 檢查是否已在運行
    if [ -f "/tmp/ecs.pid" ] && ps -p $(cat /tmp/ecs.pid) > /dev/null; then
        echo "✅ ECS 服務已經在運行中"
        return 0
    fi

    # 啟動 ECS 核心節點
    echo "🚀 啟動 ECS 核心節點..."
    nohup ros2 run ecs ecs_core --ros-args -p db_url_agvc:="postgresql+psycopg2://agvc:password@localhost/agvc" > /tmp/ecs.log 2>&1 &
    echo $! > /tmp/ecs.pid

    # 等待啟動
    sleep 3
    if ps -p $(cat /tmp/ecs.pid) > /dev/null; then
        echo "✅ ECS 服務啟動成功 (PID: $(cat /tmp/ecs.pid))"
    else
        echo "❌ ECS 服務啟動失敗，檢查日誌: tail -f /tmp/ecs.log"
        return 1
    fi
}

# 檢查 AGVC 系統狀態
check_agvc_status() {
    if ! is_agvc_environment; then
        echo "❌ 此功能僅適用於 AGVC 環境"
        return 1
    fi

    echo "🔍 檢查 AGVC 管理系統狀態..."

    # 檢查 PostgreSQL
    if pg_isready -h localhost -p 5432 >/dev/null 2>&1; then
        echo "✅ PostgreSQL 運行中"
    else
        echo "❌ PostgreSQL 未運行"
    fi

    # 檢查 ECS 服務
    if [ -f "/tmp/ecs.pid" ] && ps -p $(cat "/tmp/ecs.pid") > /dev/null; then
        echo "✅ ECS 服務運行中 (PID: $(cat /tmp/ecs.pid))"
    else
        echo "❌ ECS 服務未運行"
    fi

    # 檢查 AGVC 專用工作空間
    echo "=== AGVC 工作空間狀態 ==="
    local agvc_workspaces=("db_proxy_ws" "ecs_ws" "rcs_ws" "wcs_ws" "web_api_ws" "kuka_fleet_ws")
    for ws in "${agvc_workspaces[@]}"; do
        if [ -d "/app/$ws/install" ]; then
            echo "✅ $ws 已建置"
        else
            echo "❌ $ws 未建置"
        fi
    done
}

# ===== 初始化完成訊息 =====

# 檢測環境並顯示對應訊息
if is_agvc_environment; then
    echo "🖥️ RosAGV AGVC 管理系統環境已載入"
    echo "📊 資料庫支援：PostgreSQL"
else
    echo "🚗 RosAGV AGV 車載系統環境已載入"
fi

echo "🔧 通用指令："
echo "  build_all/ba         - 建置所有工作空間"
echo "  build_ws <name>      - 建置指定工作空間"
echo "  test_all/ta          - 測試所有工作空間"
echo "  test_ws <name>       - 測試指定工作空間"
echo "  clean_all/ca         - 清理所有工作空間"
echo "  clean_ws <name>      - 清理指定工作空間"
echo "  all_source/sa        - 載入所有工作空間"
echo "  check_system_status/status - 檢查系統狀態"
echo "  check_zenoh_status/zenoh   - 檢查 Zenoh 狀態"
echo "  check_ros_env/rosenv       - 檢查 ROS 2 環境"
echo "  manage_zenoh <cmd>         - 管理 Zenoh Router"
echo "  manage_ssh <cmd>           - 管理 SSH 服務"

if is_agvc_environment; then
    echo ""
    echo "🖥️ AGVC 專用指令："
    echo "  agvc_source          - 載入 AGVC 專用工作空間"
    echo "  start_db/stop_db     - 啟動/停止資料庫"
    echo "  start_ecs            - 啟動 ECS 設備控制系統"
    echo "  check_agvc_status    - 檢查 AGVC 系統狀態"
fi

# ===== 統一設備身份管理函數 =====

# 統一設備身份檢查
check_device_identity() {
    echo "╔══════════════════════════════════════════════════════════════════════════════╗"
    echo "║ 統一設備身份資訊"
    echo "╚══════════════════════════════════════════════════════════════════════════════╝"

    if [ -f "/app/.device_identity" ]; then
        source /app/.device_identity
        log_success "設備身份已載入: $DEVICE_ID ($CONTAINER_TYPE)"

        echo "🔧 統一設備資訊:"
        echo "  設備 ID: ${DEVICE_ID:-未設定}"
        echo "  容器類型: ${CONTAINER_TYPE:-未設定}"
        echo "  主要 MAC: ${PRIMARY_MAC:-未設定}"
        echo "  識別時間: ${IDENTIFICATION_TIME:-未知}"
        echo "  識別狀態: $([ "$IDENTIFICATION_SUCCESS" = "0" ] && echo "✅ 成功" || echo "❌ 失敗")"
        echo "  識別方法: ${IDENTIFICATION_METHOD:-未知}"

        # 根據容器類型顯示專屬資訊
        case "$CONTAINER_TYPE" in
            "agv")
                if [ -f "/app/.agv_identity" ]; then
                    source /app/.agv_identity
                    echo ""
                    echo "🚗 AGV 專屬資訊:"
                    echo "  AGV ID: ${AGV_ID:-未設定}"
                    echo "  AGV 類型: ${AGV_TYPE:-未設定}"
                    echo "  ROS 命名空間: ${ROS_NAMESPACE:-未設定}"
                    echo "  啟動套件: ${AGV_LAUNCH_PACKAGE:-未設定}"
                    echo "  配置檔案: ${DEVICE_CONFIG_FILE:-未設定}"
                fi
                ;;
            "agvc")
                if [ -f "/app/.agvc_identity" ]; then
                    source /app/.agvc_identity
                    echo ""
                    echo "🖥️ AGVC 專屬資訊:"
                    echo "  AGVC ID: ${AGVC_ID:-未設定}"
                    echo "  AGVC 類型: ${AGVC_TYPE:-未設定}"
                    echo "  AGVC 角色: ${AGVC_ROLE:-未設定}"
                    echo "  ROS 命名空間: ${ROS_NAMESPACE:-未設定}"
                    echo "  配置檔案: ${DEVICE_CONFIG_FILE:-未設定}"
                    echo "  工作空間: ${AGVC_WORKSPACES:-未設定}"
                fi
                ;;
        esac
    else
        log_warning "統一設備身份檔案不存在，請執行身份識別"
        echo "💡 執行 identify_device_manual 進行手動識別"
    fi
}

# 手動觸發統一設備身份識別
identify_device_manual() {
    log_info "手動觸發統一設備身份識別..."
    if [ -f "/app/scripts/config_driven_device_detector.bash" ]; then
        export DEVICE_DEBUG=true
        source /app/scripts/config_driven_device_detector.bash
        log_success "統一設備身份識別完成"
        check_device_identity
    else
        log_error "統一設備識別腳本不存在"
    fi
}

# 顯示設備 MAC 地址資訊和管理建議
show_device_mac_info() {
    local verbose_mode=false
    local update_config=false
    local generate_compose=false

    # 解析參數
    while [[ $# -gt 0 ]]; do
        case $1 in
            --verbose|-v)
                verbose_mode=true
                shift
                ;;
            --update-config|-u)
                update_config=true
                shift
                ;;
            --generate-compose|-g)
                generate_compose=true
                shift
                ;;
            --help|-h)
                echo "用法: show_device_mac_info [選項]"
                echo "選項:"
                echo "  --verbose, -v        顯示詳細的網路介面資訊"
                echo "  --update-config, -u  自動更新配置檔案中的 MAC 地址"
                echo "  --generate-compose, -g 生成固定 MAC 地址的 Docker Compose 配置"
                echo "  --help, -h          顯示此幫助資訊"
                return 0
                ;;
            *)
                log_warning "未知參數: $1"
                shift
                ;;
        esac
    done

    echo "╔══════════════════════════════════════════════════════════════════════════════╗"
    echo "║ 設備 MAC 地址資訊"
    echo "╚══════════════════════════════════════════════════════════════════════════════╝"

    # 讀取設備身份資訊
    local device_id=""
    local device_type=""
    local container_type=""
    local identification_method=""
    local network_mode=""

    if [ -f "/app/.device_identity" ]; then
        source /app/.device_identity
        device_id="$DEVICE_ID"
        container_type="$CONTAINER_TYPE"
        identification_method="$IDENTIFICATION_METHOD"
    else
        log_warning "設備身份檔案不存在，請先執行設備識別"
        device_id="未知"
        container_type="未知"
        identification_method="未執行"
    fi

    # 根據容器類型讀取專屬身份資訊
    case "$container_type" in
        "agv")
            if [ -f "/app/.agv_identity" ]; then
                source /app/.agv_identity
                device_type="$AGV_TYPE"
            fi
            ;;
        "agvc")
            if [ -f "/app/.agvc_identity" ]; then
                source /app/.agvc_identity
                device_type="$AGVC_TYPE"
            fi
            ;;
    esac

    # 檢測網路模式
    _detect_network_mode
    network_mode="$_NETWORK_MODE"

    echo "🔧 當前設備資訊:"
    echo "  設備 ID: ${device_id:-未知}"
    echo "  設備類型: ${device_type:-未知}"
    echo "  容器類型: ${container_type:-未知}"
    echo "  識別方法: ${identification_method:-未知}"
    echo "  網路模式: $network_mode"
    echo ""

    # 獲取實際網路介面 MAC 地址
    _get_actual_mac_addresses "$verbose_mode"

    # 獲取配置檔案中的 MAC 地址
    _get_config_mac_addresses "$device_id" "$container_type"

    # 顯示識別狀態和建議
    _show_identification_status_and_recommendations "$device_id" "$container_type" "$network_mode"

    # 執行額外功能
    if [ "$update_config" = true ]; then
        _update_config_mac_addresses "$device_id" "$container_type"
    fi

    if [ "$generate_compose" = true ]; then
        _generate_compose_config "$device_id" "$container_type"
    fi
}

# MAC 地址資訊顯示函數的簡化別名
mac_info() {
    show_device_mac_info "$@"
}

# 檢測容器網路模式
_detect_network_mode() {
    _NETWORK_MODE="unknown"

    # 檢查是否在容器內
    if [ ! -f "/.dockerenv" ]; then
        _NETWORK_MODE="host (非容器環境)"
        return 0
    fi

    # 檢查網路命名空間
    local host_net_ns=""
    local container_net_ns=""

    # 嘗試獲取宿主機網路命名空間 ID
    if [ -f "/proc/1/ns/net" ]; then
        container_net_ns=$(readlink /proc/1/ns/net 2>/dev/null)
    fi

    # 檢查是否有 host 網路模式的特徵
    # host 模式下容器會看到宿主機的所有網路介面
    local interface_count=$(ls /sys/class/net/ 2>/dev/null | wc -l)
    local docker_interfaces=$(ls /sys/class/net/ 2>/dev/null | grep -E "^(docker|br-|veth)" | wc -l)

    # 檢查是否存在典型的宿主機介面
    if ls /sys/class/net/ 2>/dev/null | grep -qE "^(enp|eth0|wlan)"; then
        # 如果有物理網路介面，可能是 host 模式
        if [ "$interface_count" -gt 3 ] || [ "$docker_interfaces" -gt 0 ]; then
            _NETWORK_MODE="host (使用宿主機網路)"
        else
            _NETWORK_MODE="bridge (容器獨立網路)"
        fi
    else
        # 只有容器內的虛擬介面
        _NETWORK_MODE="bridge (容器獨立網路)"
    fi

    # 檢查是否有 Docker 分配的 MAC 地址模式
    for interface in $(ls /sys/class/net/ 2>/dev/null); do
        if [ -f "/sys/class/net/$interface/address" ]; then
            local mac=$(cat "/sys/class/net/$interface/address" 2>/dev/null | tr '[:lower:]' '[:upper:]')
            # Docker 預設 MAC 地址模式：02:42:xx:xx:xx:xx
            if [[ "$mac" =~ ^02:42: ]]; then
                _NETWORK_MODE="bridge (容器獨立網路，動態 MAC)"
                break
            fi
        fi
    done
}

# 獲取實際網路介面 MAC 地址
_get_actual_mac_addresses() {
    local verbose_mode="$1"
    local primary_mac=""
    local primary_interface=""

    echo "📡 實際網路介面 MAC 地址:"

    # 按優先級順序檢查網路介面
    local interface_priority=("enp4s0" "eth0")
    local found_primary=false

    # 首先檢查高優先級介面
    for interface in "${interface_priority[@]}"; do
        if [ -f "/sys/class/net/$interface/address" ]; then
            local mac=$(cat "/sys/class/net/$interface/address" 2>/dev/null | tr '[:lower:]' '[:upper:]')
            if [ -n "$mac" ] && [ "$mac" != "00:00:00:00:00:00" ]; then
                primary_mac="$mac"
                primary_interface="$interface"
                found_primary=true
                echo "  ✅ $interface: $mac (主要識別 MAC)"
                break
            fi
        fi
    done

    # 如果沒找到高優先級介面，檢查 enx* 介面
    if [ "$found_primary" = false ]; then
        for interface in $(ls /sys/class/net/ 2>/dev/null | grep "^enx" | sort); do
            if [ -f "/sys/class/net/$interface/address" ]; then
                local mac=$(cat "/sys/class/net/$interface/address" 2>/dev/null | tr '[:lower:]' '[:upper:]')
                if [ -n "$mac" ] && [ "$mac" != "00:00:00:00:00:00" ]; then
                    primary_mac="$mac"
                    primary_interface="$interface"
                    found_primary=true
                    echo "  ✅ $interface: $mac (主要識別 MAC - USB 網路)"
                    break
                fi
            fi
        done
    fi

    # 如果還沒找到，檢查其他非虛擬介面
    if [ "$found_primary" = false ]; then
        for interface in $(ls /sys/class/net/ 2>/dev/null); do
            case "$interface" in
                lo|docker0|br-*|veth*) continue ;;
                *)
                    if [ -f "/sys/class/net/$interface/address" ]; then
                        local mac=$(cat "/sys/class/net/$interface/address" 2>/dev/null | tr '[:lower:]' '[:upper:]')
                        if [ -n "$mac" ] && [ "$mac" != "00:00:00:00:00:00" ]; then
                            primary_mac="$mac"
                            primary_interface="$interface"
                            found_primary=true
                            # 檢查是否為 Docker 動態分配的 MAC
                            if [[ "$mac" =~ ^02:42: ]]; then
                                echo "  ⚠️ $interface: $mac (主要識別 MAC - 動態分配，每次重啟會變更)"
                            else
                                echo "  ✅ $interface: $mac (主要識別 MAC)"
                            fi
                            break
                        fi
                    fi
                    ;;
            esac
        done
    fi

    # 顯示其他網路介面（詳細模式或非主要介面）
    if [ "$verbose_mode" = true ] || [ "$found_primary" = false ]; then
        for interface in $(ls /sys/class/net/ 2>/dev/null); do
            if [ "$interface" != "$primary_interface" ]; then
                case "$interface" in
                    lo|docker0|br-*|veth*)
                        if [ "$verbose_mode" = true ]; then
                            echo "  🚫 $interface: (虛擬介面，已排除)"
                        fi
                        ;;
                    *)
                        if [ -f "/sys/class/net/$interface/address" ]; then
                            local mac=$(cat "/sys/class/net/$interface/address" 2>/dev/null | tr '[:lower:]' '[:upper:]')
                            if [ -n "$mac" ] && [ "$mac" != "00:00:00:00:00:00" ]; then
                                case "$interface" in
                                    wlan*) echo "  📱 $interface: $mac (無線網路)" ;;
                                    *) echo "  📡 $interface: $mac" ;;
                                esac
                            fi
                        fi
                        ;;
                esac
            fi
        done
    fi

    # 顯示排除的虛擬介面摘要
    local excluded_interfaces=$(ls /sys/class/net/ 2>/dev/null | grep -E "^(lo|docker0|br-|veth)" | tr '\n' ', ' | sed 's/,$//')
    if [ -n "$excluded_interfaces" ] && [ "$verbose_mode" = false ]; then
        echo "  🚫 已排除: $excluded_interfaces"
    fi

    # 設定全域變數供其他函數使用
    _PRIMARY_MAC="$primary_mac"
    _PRIMARY_INTERFACE="$primary_interface"

    if [ "$found_primary" = false ]; then
        echo "  ❌ 未找到有效的主要 MAC 地址"
    fi

    echo ""
}

# 獲取配置檔案中的 MAC 地址
_get_config_mac_addresses() {
    local device_id="$1"
    local container_type="$2"
    local config_file="/app/config/hardware_mapping.yaml"

    echo "📋 配置檔案中的 MAC 地址:"

    if [ ! -f "$config_file" ]; then
        echo "  ❌ 配置檔案不存在: $config_file"
        _CONFIG_MAC_ADDRESSES=()
        return 1
    fi

    if [ -z "$device_id" ] || [ "$device_id" = "未知" ]; then
        echo "  ⚠️ 設備 ID 未知，無法讀取配置"
        _CONFIG_MAC_ADDRESSES=()
        return 1
    fi

    # 使用 Python 解析 YAML 並獲取 MAC 地址列表
    local mac_addresses_result=$(python3 -c "
import yaml
import sys
try:
    with open('$config_file', 'r') as f:
        config = yaml.safe_load(f)

    devices_key = '${container_type}_devices'
    if devices_key not in config:
        print('ERROR: devices_key_not_found', file=sys.stderr)
        sys.exit(1)

    if '$device_id' not in config[devices_key]:
        print('ERROR: device_not_found', file=sys.stderr)
        sys.exit(1)

    device_config = config[devices_key]['$device_id']
    if 'mac_addresses' not in device_config:
        print('ERROR: mac_addresses_not_found', file=sys.stderr)
        sys.exit(1)

    mac_addresses = device_config['mac_addresses']
    if isinstance(mac_addresses, list):
        for mac in mac_addresses:
            print(mac.upper())
    else:
        print('ERROR: mac_addresses_not_list', file=sys.stderr)
        sys.exit(1)

except Exception as e:
    print(f'ERROR: {e}', file=sys.stderr)
    sys.exit(1)
" 2>/dev/null)

    if [ $? -ne 0 ] || [ -z "$mac_addresses_result" ]; then
        echo "  ⚠️ 無法從配置檔案讀取 MAC 地址"
        echo "    設備 ID: $device_id"
        echo "    容器類型: $container_type"
        _CONFIG_MAC_ADDRESSES=()
        return 1
    fi

    # 將結果轉換為陣列
    _CONFIG_MAC_ADDRESSES=()
    while IFS= read -r mac; do
        if [ -n "$mac" ]; then
            _CONFIG_MAC_ADDRESSES+=("$mac")
        fi
    done <<< "$mac_addresses_result"

    # 顯示配置檔案中的 MAC 地址並與實際 MAC 比對
    local primary_mac="${_PRIMARY_MAC:-}"
    local match_found=false

    for config_mac in "${_CONFIG_MAC_ADDRESSES[@]}"; do
        if [ -n "$primary_mac" ] && [ "$config_mac" = "$primary_mac" ]; then
            echo "  ✅ $config_mac (匹配)"
            match_found=true
        else
            echo "  ❌ $config_mac (不匹配)"
        fi
    done

    if [ ${#_CONFIG_MAC_ADDRESSES[@]} -eq 0 ]; then
        echo "  ⚠️ 配置檔案中未找到 MAC 地址"
    fi

    echo ""

    # 設定匹配狀態供其他函數使用
    _MAC_MATCH_STATUS="$match_found"
}

# 顯示識別狀態和建議
_show_identification_status_and_recommendations() {
    local device_id="$1"
    local container_type="$2"
    local network_mode="$3"

    echo "🎯 識別狀態:"

    # 根據 MAC 地址匹配狀態顯示識別狀態
    if [ "$_MAC_MATCH_STATUS" = true ]; then
        echo "  ✅ MAC 地址匹配，設備識別正常"
        echo "  💡 建議: 配置檔案已包含正確的 MAC 地址"
    else
        echo "  ❌ MAC 地址不匹配，使用預設降級識別"

        # 根據網路模式提供不同的建議
        case "$network_mode" in
            *"bridge"*"動態 MAC"*)
                echo ""
                echo "⚠️ ${container_type^^} 容器 MAC 地址問題:"
                echo "  問題: Bridge 網路模式下，容器 MAC 地址每次 compose up 都會變更"
                echo ""
                echo "🔧 解決方案建議:"
                echo "  1. 固定容器 MAC 地址 (推薦):"
                echo "     在 docker-compose.${container_type}.yml 中添加:"
                echo "     services:"
                echo "       ${container_type}_server:"
                echo "         networks:"
                echo "           ${container_type}_network:"
                if [ ${#_CONFIG_MAC_ADDRESSES[@]} -gt 0 ]; then
                    echo "             mac_address: \"${_CONFIG_MAC_ADDRESSES[0]}\""
                else
                    echo "             mac_address: \"02:42:AC:14:00:10\"  # 請替換為實際 MAC"
                fi
                echo ""
                echo "  2. 使用 host 網路模式:"
                echo "     network_mode: \"host\""
                echo "     (注意: 會與宿主機共享網路，可能有端口衝突)"
                echo ""
                echo "  3. 更新配置檔案 MAC 地址:"
                echo "     執行: show_device_mac_info --update-config"
                ;;
            *"host"*)
                echo ""
                echo "💡 建議操作:"
                echo "  1. 更新配置檔案中的 MAC 地址:"
                if [ -n "$_PRIMARY_MAC" ]; then
                    echo "     將 $_PRIMARY_MAC 添加到 hardware_mapping.yaml"
                fi
                echo "  2. 執行自動更新:"
                echo "     show_device_mac_info --update-config"
                ;;
            *)
                echo ""
                echo "💡 建議檢查網路配置和 MAC 地址設定"
                ;;
        esac
    fi

    echo ""
}

# 自動更新配置檔案中的 MAC 地址
_update_config_mac_addresses() {
    local device_id="$1"
    local container_type="$2"
    local config_file="/app/config/hardware_mapping.yaml"

    echo "🔧 自動更新配置檔案 MAC 地址:"

    if [ -z "$_PRIMARY_MAC" ]; then
        log_error "無法獲取主要 MAC 地址，更新失敗"
        return 1
    fi

    if [ ! -f "$config_file" ]; then
        log_error "配置檔案不存在: $config_file"
        return 1
    fi

    # 檢查 MAC 地址是否已存在於配置中
    for config_mac in "${_CONFIG_MAC_ADDRESSES[@]}"; do
        if [ "$config_mac" = "$_PRIMARY_MAC" ]; then
            log_info "MAC 地址 $_PRIMARY_MAC 已存在於配置檔案中"
            return 0
        fi
    done

    # 備份原始配置檔案
    local backup_file="${config_file}.backup.$(date +%Y%m%d_%H%M%S)"
    cp "$config_file" "$backup_file"
    log_info "已備份原始配置檔案: $backup_file"

    # 使用 Python 更新配置檔案
    python3 -c "
import yaml
import sys
from datetime import datetime

try:
    # 讀取配置檔案
    with open('$config_file', 'r') as f:
        config = yaml.safe_load(f)

    devices_key = '${container_type}_devices'
    if devices_key not in config or '$device_id' not in config[devices_key]:
        print('ERROR: 設備配置不存在', file=sys.stderr)
        sys.exit(1)

    device_config = config[devices_key]['$device_id']

    # 確保 mac_addresses 是列表
    if 'mac_addresses' not in device_config:
        device_config['mac_addresses'] = []
    elif not isinstance(device_config['mac_addresses'], list):
        device_config['mac_addresses'] = [device_config['mac_addresses']]

    # 添加新的 MAC 地址（如果不存在）
    new_mac = '$_PRIMARY_MAC'
    if new_mac not in [mac.upper() for mac in device_config['mac_addresses']]:
        device_config['mac_addresses'].append(new_mac)
        print(f'已添加 MAC 地址: {new_mac}')

    # 添加更新註釋
    if 'description' in device_config:
        device_config['description'] += f' (MAC 更新: {datetime.now().strftime(\"%Y-%m-%d %H:%M\")})'

    # 寫回配置檔案
    with open('$config_file', 'w') as f:
        yaml.dump(config, f, default_flow_style=False, allow_unicode=True, indent=2)

    print('配置檔案更新成功')

except Exception as e:
    print(f'ERROR: {e}', file=sys.stderr)
    sys.exit(1)
"

    if [ $? -eq 0 ]; then
        log_success "配置檔案更新完成"
        log_info "新的 MAC 地址 $_PRIMARY_MAC 已添加到設備 $device_id 的配置中"
    else
        log_error "配置檔案更新失敗"
        return 1
    fi
}

# 生成固定 MAC 地址的 Docker Compose 配置
_generate_compose_config() {
    local device_id="$1"
    local container_type="$2"

    echo "🐳 生成 Docker Compose 配置:"

    if [ "$container_type" != "agvc" ]; then
        log_info "AGV 容器通常使用 host 網路模式，無需固定 MAC 地址"
        return 0
    fi

    local mac_address="${_PRIMARY_MAC:-02:42:AC:14:00:10}"
    if [ ${#_CONFIG_MAC_ADDRESSES[@]} -gt 0 ]; then
        mac_address="${_CONFIG_MAC_ADDRESSES[0]}"
    fi

    local compose_config_file="/tmp/docker-compose.agvc.mac-fixed.yml"

    cat > "$compose_config_file" << EOF
# Docker Compose 配置 - 固定 AGVC 容器 MAC 地址
# 生成時間: $(date)
# 設備 ID: $device_id
# MAC 地址: $mac_address

version: '3.8'

services:
  agvc_server:
    image: yazelin/agvc:latest
    container_name: agvc_server
    restart: unless-stopped

    # 固定 MAC 地址配置
    networks:
      agvc_network:
        mac_address: "$mac_address"

    # 端口映射
    ports:
      - "2200:2200"
      - "3000-3001:3000-3001"
      - "5173:5173"
      - "7447:7447"
      - "8000-8002:8000-8002"

    # 卷掛載
    volumes:
      - ./app:/app
      - ./data:/data
      - ./logs:/logs

    # 環境變數
    environment:
      - CONTAINER_TYPE=agvc
      - MANUAL_DEVICE_ID=$device_id

    # 啟動命令
    command: /bin/bash -c '/app/startup.agvc.bash && tail -f /dev/null'

# 自定義網路配置
networks:
  agvc_network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.20.0.0/16
          gateway: 172.20.0.1

EOF

    log_success "Docker Compose 配置已生成: $compose_config_file"
    echo ""
    echo "📋 使用方式:"
    echo "  1. 停止當前 AGVC 容器:"
    echo "     docker compose -f docker-compose.agvc.yml down"
    echo ""
    echo "  2. 使用新配置啟動:"
    echo "     docker compose -f $compose_config_file up -d"
    echo ""
    echo "  3. 或者將配置合併到現有的 docker-compose.agvc.yml 中"
    echo ""
    echo "⚠️ 注意事項:"
    echo "  - 確保 MAC 地址 $mac_address 在網路中是唯一的"
    echo "  - 固定 MAC 地址後，設備識別將更加穩定"
    echo "  - 建議在生產環境部署前進行測試"
}

# 顯示設備配置資訊
show_device_config() {
    echo "╔══════════════════════════════════════════════════════════════════════════════╗"
    echo "║ 設備配置資訊"
    echo "╚══════════════════════════════════════════════════════════════════════════════╝"

    if [ -f "/app/.device_identity" ]; then
        source /app/.device_identity

        echo "📁 配置檔案檢查:"
        if [ -n "$DEVICE_CONFIG_FILE" ] && [ -f "$DEVICE_CONFIG_FILE" ]; then
            echo "  ✅ 配置檔案存在: $DEVICE_CONFIG_FILE"
            echo "  📊 檔案大小: $(du -h "$DEVICE_CONFIG_FILE" | cut -f1)"
            echo "  🕒 修改時間: $(stat -c %y "$DEVICE_CONFIG_FILE" 2>/dev/null || echo "無法獲取")"
        else
            echo "  ❌ 配置檔案不存在: ${DEVICE_CONFIG_FILE:-未設定}"
        fi

        echo ""
        echo "🗂️ 硬體映射檔案檢查:"
        if [ -f "/app/config/hardware_mapping.yaml" ]; then
            echo "  ✅ 硬體映射檔案存在"
            echo "  📊 檔案大小: $(du -h "/app/config/hardware_mapping.yaml" | cut -f1)"
        else
            echo "  ❌ 硬體映射檔案不存在"
        fi

        echo ""
        echo "📋 日誌檔案檢查:"
        for log_file in "/tmp/device_identification.log" "/tmp/device_hardware_info.log"; do
            if [ -f "$log_file" ]; then
                echo "  ✅ $(basename "$log_file"): $(du -h "$log_file" | cut -f1)"
            else
                echo "  ❌ $(basename "$log_file"): 不存在"
            fi
        done
    else
        log_warning "請先執行設備身份識別"
    fi
}

echo ""
echo "🔧 統一設備身份管理指令："
echo "  check_device_identity    - 檢查設備身份資訊"
echo "  identify_device_manual   - 手動觸發設備身份識別"
echo "  show_device_config       - 顯示設備配置資訊"
echo "  show_device_mac_info     - 顯示設備 MAC 地址資訊和管理建議"
echo "  mac_info                 - show_device_mac_info 的簡化別名"

echo ""
