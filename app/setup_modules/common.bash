#!/bin/bash
# RosAGV Common Module
# 包含顏色定義、日誌系統和基礎工具函數

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

# ============================================================================
# 基礎工具函數
# ============================================================================

# ============================================================================
# 日誌輪轉函數 - Log Rotation
# ============================================================================
# 功能：自動輪轉過大的日誌檔案，防止磁碟空間耗盡
# 用法：rotate_log_file <log_file> [max_size_mb] [keep_count]
# 參數：
#   log_file     - 日誌檔案路徑（必要）
#   max_size_mb  - 觸發輪轉的檔案大小（MB），預設 10
#   keep_count   - 保留的輪轉份數，預設 5
# 範例：
#   rotate_log_file "/tmp/web_api.log" 10 5
#   → 當 web_api.log 超過 10MB 時，輪轉為 web_api.log.1
#   → 保留最近 5 個版本：.log.1 ~ .log.5
# ============================================================================
rotate_log_file() {
    local log_file=$1
    local max_size_mb=${2:-10}  # 預設 10 MB
    local keep_count=${3:-5}    # 預設保留 5 個舊檔案

    # 參數驗證
    if [ -z "$log_file" ]; then
        log_error "rotate_log_file: 缺少日誌檔案路徑參數"
        return 1
    fi

    # 檢查檔案是否存在
    if [ ! -f "$log_file" ]; then
        # 檔案不存在不算錯誤，直接返回
        return 0
    fi

    # 檢查檔案大小（使用 stat 指令，兼容 Linux）
    # Linux 使用 stat -c%s，macOS 使用 stat -f%z
    local file_size
    if stat -c%s "$log_file" &>/dev/null; then
        file_size=$(stat -c%s "$log_file" 2>/dev/null)
    else
        file_size=$(stat -f%z "$log_file" 2>/dev/null)
    fi

    # 檢查 stat 是否成功
    if [ -z "$file_size" ]; then
        log_error "rotate_log_file: 無法取得檔案大小: $log_file"
        return 1
    fi

    # 轉換為 MB
    local size_mb=$((file_size / 1024 / 1024))

    # 如果檔案小於閾值，不需要輪轉
    if [ $size_mb -lt $max_size_mb ]; then
        log_debug "rotate_log_file: $log_file (${size_mb}MB) 小於閾值 ${max_size_mb}MB，跳過輪轉"
        return 0
    fi

    # 執行輪轉
    log_info "🔄 輪轉日誌: $(basename $log_file) (${size_mb}MB > ${max_size_mb}MB)"

    # 刪除最舊的檔案（如 .log.5）
    if [ -f "${log_file}.${keep_count}" ]; then
        rm -f "${log_file}.${keep_count}"
        log_debug "刪除最舊的輪轉檔案: ${log_file}.${keep_count}"
    fi

    # 輪轉現有檔案：.log.4 → .log.5, .log.3 → .log.4, ...
    for i in $(seq $((keep_count - 1)) -1 1); do
        if [ -f "${log_file}.${i}" ]; then
            mv "${log_file}.${i}" "${log_file}.$((i + 1))"
            log_debug "輪轉: ${log_file}.${i} → ${log_file}.$((i + 1))"
        fi
    done

    # 輪轉當前檔案：.log → .log.1
    mv "$log_file" "${log_file}.1"

    # 建立新的空日誌檔案（保持原有權限）
    touch "$log_file"

    log_success "✅ 日誌輪轉完成: $(basename $log_file) → $(basename ${log_file}).1"
    return 0
}

# 函式：檢查遠端主機是否可達
ping_all() {
    # 定義要測試的 host
    local hosts=(
        "192.168.10.3"
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
        "/app/shared_constants_ws"
        "/app/keyence_plc_ws"
        "/app/plc_proxy_ws"
        "/app/agv_cmd_service_ws"
        "/app/joystick_ws"
        "/app/agv_ws"
        "/app/db_proxy_ws"
        "/app/ecs_ws"
        "/app/rcs_ws"
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

# ============================================================================
# 環境檢測函數
# ============================================================================

# 檢測當前是否為 AGVC 環境
is_agvc_environment() {
    # 主要檢測方法：檢查 Docker Compose 設定的 CONTAINER_TYPE 環境變數
    # 這是最簡單、最可靠的檢測方式
    if [ "$CONTAINER_TYPE" = "agvc" ]; then
        return 0  # 是 AGVC 環境
    fi

    return 1  # 是 AGV 環境
}

is_agv_environment() {
    # 主要檢測方法：檢查 Docker Compose 設定的 CONTAINER_TYPE 環境變數
    # 這是最簡單、最可靠的檢測方式
    if [ "$CONTAINER_TYPE" = "agv" ]; then
        return 0  # 是 AGV 環境
    fi

    return 1  # 是 AGVC 環境
}

# ============================================================================
# 節點啟動驗證輔助函數
# ============================================================================
# 這些函數提供統一的節點啟動驗證邏輯，支援智能重試和進度顯示
# 用於替換 node-management.bash 中的固定 sleep + 單次檢查模式
# ============================================================================

# 驗證 ROS 2 節點是否成功啟動並註冊到網路
# 使用場景：ros2 run 啟動的節點，需要確認 ROS 2 網路註冊
# 參數：
#   $1: node_name - 完整節點名稱（例如："/agvc/plc_service" 或 "tafl_wcs_node"）
#   $2: timeout - 超時時間（秒），預設 10 秒
#   $3: show_progress - 是否顯示等待進度，預設 true
# 返回值：
#   0 - 節點成功啟動並註冊
#   1 - 超時失敗
# 範例：
#   if verify_ros2_node_startup "/agvc/plc_service" 10; then
#       echo "✅ Node started successfully"
#   fi
verify_ros2_node_startup() {
    local node_name="$1"
    local timeout="${2:-10}"
    local show_progress="${3:-true}"

    if [ -z "$node_name" ]; then
        log_error "verify_ros2_node_startup: node_name 參數為空"
        return 1
    fi

    local waited=0
    local node_found=false

    [ "$show_progress" = "true" ] && echo "⏳ 等待節點啟動並註冊到 ROS 2 網路（節點：$node_name，最多等待 ${timeout} 秒）..."

    while [ $waited -lt $timeout ]; do
        sleep 1
        waited=$((waited + 1))

        # 使用 ros2 node list 檢查節點是否已註冊
        if ros2 node list 2>/dev/null | grep -q "$node_name"; then
            node_found=true
            break
        fi

        [ "$show_progress" = "true" ] && echo -n "."
    done

    [ "$show_progress" = "true" ] && echo ""

    if [ "$node_found" = true ]; then
        [ "$show_progress" = "true" ] && echo "✅ 節點驗證成功（等待時間：${waited} 秒）"
        return 0
    else
        [ "$show_progress" = "true" ] && echo "❌ 節點驗證超時（${timeout} 秒內未檢測到節點）"
        return 1
    fi
}

# 驗證進程是否成功啟動
# 使用場景：launch 檔案或原生進程，使用進程名稱模式匹配
# 參數：
#   $1: process_pattern - pgrep 搜尋模式（例如："rcs_launch.py" 或 "agv_ui_server"）
#   $2: timeout - 超時時間（秒），預設 10 秒
#   $3: show_progress - 是否顯示等待進度，預設 true
# 返回值：
#   0 - 進程成功啟動
#   1 - 超時失敗
# 範例：
#   if verify_process_startup "rcs_launch.py" 10; then
#       echo "✅ Process started successfully"
#   fi
verify_process_startup() {
    local process_pattern="$1"
    local timeout="${2:-10}"
    local show_progress="${3:-true}"

    if [ -z "$process_pattern" ]; then
        log_error "verify_process_startup: process_pattern 參數為空"
        return 1
    fi

    local waited=0
    local process_found=false

    [ "$show_progress" = "true" ] && echo "⏳ 等待進程啟動（模式：$process_pattern，最多等待 ${timeout} 秒）..."

    while [ $waited -lt $timeout ]; do
        sleep 1
        waited=$((waited + 1))

        # 使用 pgrep 檢查進程是否存在
        if pgrep -f "$process_pattern" > /dev/null 2>&1; then
            process_found=true
            break
        fi

        [ "$show_progress" = "true" ] && echo -n "."
    done

    [ "$show_progress" = "true" ] && echo ""

    if [ "$process_found" = true ]; then
        local pids=$(pgrep -f "$process_pattern" | tr '\n' ' ')
        [ "$show_progress" = "true" ] && echo "✅ 進程驗證成功（PIDs: $pids，等待時間：${waited} 秒）"
        return 0
    else
        [ "$show_progress" = "true" ] && echo "❌ 進程驗證超時（${timeout} 秒內未檢測到進程）"
        return 1
    fi
}

# 組合驗證：同時檢查進程存活 + ROS 2 節點註冊或進程模式
# 使用場景：需要多層驗證的複雜節點（進程存在 + 功能正常）
# 參數：
#   $1: parent_pid - 主進程 PID（必要，用於檢測進程是否意外終止）
#   $2: node_name - ROS 2 節點名稱（可選，若提供則驗證 ROS 2 註冊）
#   $3: process_pattern - pgrep 搜尋模式（可選，若提供則驗證進程存在）
#   $4: timeout - 超時時間（秒），預設 10 秒
# 返回值：
#   0 - 所有檢查通過
#   1 - 驗證超時
#   2 - 主進程意外終止
# 範例：
#   if verify_node_startup_combined "$PARENT_PID" "/agvc/node_name" "" 15; then
#       echo "✅ Fully verified"
#   fi
verify_node_startup_combined() {
    local parent_pid="$1"
    local node_name="$2"
    local process_pattern="$3"
    local timeout="${4:-10}"

    if [ -z "$parent_pid" ]; then
        log_error "verify_node_startup_combined: parent_pid 參數為空"
        return 1
    fi

    local waited=0
    local verification_passed=false

    echo "⏳ 組合驗證節點啟動（PID: $parent_pid，最多等待 ${timeout} 秒）..."

    while [ $waited -lt $timeout ]; do
        sleep 1
        waited=$((waited + 1))

        # 首先檢查主進程是否還活著
        if ! kill -0 "$parent_pid" 2>/dev/null; then
            echo "❌ 主進程意外終止（PID: $parent_pid）"
            return 2
        fi

        # 檢查 ROS 2 節點（如果指定）
        if [ -n "$node_name" ]; then
            if ros2 node list 2>/dev/null | grep -q "$node_name"; then
                verification_passed=true
                echo ""
                echo "✅ ROS 2 節點驗證成功（節點：$node_name，等待時間：${waited} 秒）"
                break
            fi
        fi

        # 檢查進程模式（如果指定）
        if [ -n "$process_pattern" ]; then
            if pgrep -f "$process_pattern" > /dev/null 2>&1; then
                verification_passed=true
                local pids=$(pgrep -f "$process_pattern" | tr '\n' ' ')
                echo ""
                echo "✅ 進程驗證成功（模式：$process_pattern，PIDs: $pids，等待時間：${waited} 秒）"
                break
            fi
        fi

        echo -n "."
    done

    if [ "$verification_passed" = false ]; then
        echo ""
        echo "❌ 組合驗證超時（${timeout} 秒）"
        return 1
    fi

    return 0
}
