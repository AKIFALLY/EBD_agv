#!/bin/bash
# RosAGV 智能建置輔助工具
# 版本: 1.0
# 說明: 提供智能化的工作空間建置管理，支援並行建置、錯誤診斷和增量建置

set -e  # 遇到錯誤立即退出

# ============================================================================
# 常數定義
# ============================================================================

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# 路徑定義
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
APP_DIR="$PROJECT_ROOT/app"

# 工作空間分類定義
declare -A WORKSPACE_CATEGORIES=(
    ["base"]="keyence_plc_ws plc_proxy_ws path_algorithm"
    ["agv"]="agv_cmd_service_ws joystick_ws agv_ws sensorpart_ws uno_gpio_ws"
    ["agvc"]="db_proxy_ws ecs_ws rcs_ws wcs_ws ai_wcs_ws web_api_ws kuka_fleet_ws"
    ["common"]="launch_ws"
)

# 建置配置
declare -A BUILD_PROFILES=(
    ["fast"]="並行建置，跳過測試"
    ["full"]="完整建置，包含測試"  
    ["incremental"]="增量建置，只建置修改的工作空間"
    ["debug"]="除錯建置，包含詳細日誌"
    ["release"]="發布建置，最佳化編譯"
)

# 預設設定
DEFAULT_JOBS=4
DEFAULT_TIMEOUT=300
BUILD_LOG_DIR="/tmp/rosagv_build_logs"

# ============================================================================
# 工具函數
# ============================================================================

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

log_build() {
    echo -e "${PURPLE}[BUILD]${NC} $1"
}

show_header() {
    echo -e "${CYAN}🔨 RosAGV 智能建置輔助工具${NC}"
    echo -e "${CYAN}=============================${NC}"
    echo ""
}

show_usage() {
    echo "使用方式:"
    echo "  $0 [profile] [options]"
    echo ""
    echo "建置配置檔:"
    for profile in "${!BUILD_PROFILES[@]}"; do
        echo "  $profile                  # ${BUILD_PROFILES[$profile]}"
    done
    echo ""
    echo "選項:"
    echo "  --workspace <WS>         # 指定單一工作空間"
    echo "  --workspaces <WSs>       # 指定多個工作空間，逗號分隔"
    echo "  --category <CAT>         # 指定工作空間類別: base,agv,agvc,common"
    echo "  --jobs <N>               # 並行建置任務數 (預設: $DEFAULT_JOBS)"
    echo "  --timeout <SEC>          # 建置超時時間 (預設: ${DEFAULT_TIMEOUT}s)"
    echo "  --clean                  # 建置前先清理"
    echo "  --symlink                # 使用 symlink-install"
    echo "  --continue-on-error      # 錯誤時繼續建置其他工作空間"
    echo "  --output <FORMAT>        # 輸出格式: table, json, summary"
    echo "  --log-level <LEVEL>      # 日誌層級: quiet, normal, verbose"
    echo "  --save-logs <DIR>        # 儲存建置日誌到指定目錄"
    echo "  --report                 # 生成建置報告"
    echo "  -h, --help              # 顯示此幫助資訊"
    echo ""
    echo "範例:"
    echo "  $0 fast                              # 快速並行建置"
    echo "  $0 full --category agv               # 完整建置 AGV 相關工作空間"
    echo "  $0 incremental --jobs 8              # 8 核心增量建置"
    echo "  $0 debug --workspace agv_ws          # 除錯建置單一工作空間"
    echo "  $0 release --clean --report          # 發布建置並生成報告"
}

# ============================================================================
# 核心建置函數
# ============================================================================

check_dependencies() {
    local missing_deps=()
    
    # 檢查必要工具
    for tool in colcon timeout; do
        if ! command -v "$tool" &> /dev/null; then
            missing_deps+=("$tool")
        fi
    done
    
    # 檢查是否在容器環境中
    if [ -z "$CONTAINER_TYPE" ]; then
        log_warning "未檢測到容器環境變數 CONTAINER_TYPE"
        log_info "建議在 AGV 或 AGVC 容器中執行此工具"
    fi
    
    if [ ${#missing_deps[@]} -gt 0 ]; then
        log_error "缺少必要工具: ${missing_deps[*]}"
        log_info "請確認在正確的 ROS 2 環境中執行"
        return 1
    fi
    
    return 0
}

get_all_workspaces() {
    local workspaces=()
    
    # 掃描 app 目錄下的所有 _ws 目錄
    for dir in "$APP_DIR"/*_ws; do
        if [ -d "$dir" ]; then
            workspaces+=($(basename "$dir"))
        fi
    done
    
    # 添加 path_algorithm (不以 _ws 結尾但也是工作空間)
    if [ -d "$APP_DIR/path_algorithm" ]; then
        workspaces+=("path_algorithm")
    fi
    
    echo "${workspaces[@]}"
}

get_workspaces_by_category() {
    local category="$1"
    local result=()
    
    if [ -n "${WORKSPACE_CATEGORIES[$category]}" ]; then
        read -ra result <<< "${WORKSPACE_CATEGORIES[$category]}"
    else
        log_error "未知的工作空間類別: $category"
        return 1
    fi
    
    echo "${result[@]}"
}

check_workspace_modified() {
    local workspace="$1"
    local workspace_path="$APP_DIR/$workspace"
    
    if [ ! -d "$workspace_path" ]; then
        return 1
    fi
    
    # 檢查 src 目錄的修改時間
    local src_dir="$workspace_path/src"
    local install_dir="$workspace_path/install"
    
    if [ ! -d "$install_dir" ]; then
        # 如果 install 目錄不存在，則需要建置
        return 0
    fi
    
    if [ ! -d "$src_dir" ]; then
        # 如果沒有 src 目錄，跳過
        return 1
    fi
    
    # 比較 src 和 install 的修改時間
    local src_newer=$(find "$src_dir" -newer "$install_dir" -type f | head -1)
    
    if [ -n "$src_newer" ]; then
        return 0  # 需要重新建置
    else
        return 1  # 不需要建置
    fi
}

estimate_build_time() {
    local workspace="$1"
    
    # 根據工作空間大小和複雜度估算建置時間 (秒)
    case "$workspace" in
        "agv_ws"|"web_api_ws")
            echo 120  # 2分鐘
            ;;
        "rcs_ws"|"wcs_ws"|"ecs_ws")
            echo 90   # 1.5分鐘
            ;;
        "db_proxy_ws"|"keyence_plc_ws")
            echo 60   # 1分鐘
            ;;
        *)
            echo 45   # 45秒
            ;;
    esac
}

build_workspace() {
    local workspace="$1"
    local profile="$2"
    local jobs="$3"
    local timeout="$4"
    local use_symlink="$5"
    local log_level="$6"
    local continue_on_error="$7"
    
    local workspace_path="$APP_DIR/$workspace"
    
    if [ ! -d "$workspace_path" ]; then
        log_error "工作空間不存在: $workspace"
        return 1
    fi
    
    log_build "開始建置工作空間: $workspace"
    
    # 切換到工作空間目錄
    cd "$workspace_path" || return 1
    
    # 建置參數配置
    local build_args=()
    build_args+=("--event-handlers" "console_direct+")
    
    # 設定並行任務數
    if [ "$jobs" -gt 1 ]; then
        build_args+=("--parallel-workers" "$jobs")
    fi
    
    # 設定 symlink 安裝
    if [ "$use_symlink" = "true" ]; then
        build_args+=("--symlink-install")
    fi
    
    # 設定建置類型
    case "$profile" in
        "debug")
            build_args+=("--cmake-args" "-DCMAKE_BUILD_TYPE=Debug")
            ;;
        "release")
            build_args+=("--cmake-args" "-DCMAKE_BUILD_TYPE=Release")
            ;;
    esac
    
    # 設定日誌層級
    case "$log_level" in
        "quiet")
            build_args+=("--event-handlers" "console_cohesion+")
            ;;
        "verbose")
            build_args+=("--event-handlers" "console_direct+" "--cmake-args" "-DCMAKE_VERBOSE_MAKEFILE=ON")
            ;;
    esac
    
    # 如果錯誤時繼續
    if [ "$continue_on_error" = "true" ]; then
        build_args+=("--continue-on-error")
    fi
    
    # 執行建置
    local start_time=$(date +%s)
    local build_success=false
    
    log_info "執行指令: colcon build ${build_args[*]}"
    
    if timeout "$timeout" colcon build "${build_args[@]}"; then
        build_success=true
        local end_time=$(date +%s)
        local build_time=$((end_time - start_time))
        log_success "$workspace 建置成功 (耗時: ${build_time}s)"
    else
        local end_time=$(date +%s)
        local build_time=$((end_time - start_time))
        log_error "$workspace 建置失敗 (耗時: ${build_time}s)"
        
        # 分析建置錯誤
        analyze_build_error "$workspace"
    fi
    
    # 返回結果
    if [ "$build_success" = true ]; then
        return 0
    else
        return 1
    fi
}

analyze_build_error() {
    local workspace="$1"
    local log_dir="$APP_DIR/$workspace/log"
    
    if [ ! -d "$log_dir" ]; then
        log_warning "找不到建置日誌目錄: $log_dir"
        return
    fi
    
    log_info "分析建置錯誤..."
    
    # 查找最新的建置日誌
    local latest_log=$(find "$log_dir" -name "latest_build" -type l)
    if [ -n "$latest_log" ]; then
        local actual_log_dir=$(readlink -f "$latest_log")
        
        # 分析常見錯誤模式
        local error_patterns=(
            "error:"
            "Error"  
            "FAILED"
            "No such file"
            "Permission denied"
            "undefined reference"
            "fatal error"
        )
        
        echo -e "${YELLOW}常見錯誤檢查:${NC}"
        for pattern in "${error_patterns[@]}"; do
            local error_count=$(rg -r "$pattern" "$actual_log_dir" 2>/dev/null | wc -l)
            if [ "$error_count" -gt 0 ]; then
                echo -e "  ${RED}$pattern: $error_count 次${NC}"
                
                # 顯示第一個錯誤的上下文
                local first_error=$(rg -r "$pattern" "$actual_log_dir" 2>/dev/null | head -1)
                if [ -n "$first_error" ]; then
                    echo -e "    ${BLUE}範例: $first_error${NC}"
                fi
            fi
        done
        
        # 提供修復建議
        provide_fix_suggestions "$workspace" "$actual_log_dir"
    fi
}

provide_fix_suggestions() {
    local workspace="$1"
    local log_dir="$2"
    
    echo -e "${CYAN}修復建議:${NC}"
    
    # 檢查依賴問題
    if grep -q "could not find" "$log_dir"/* 2>/dev/null; then
        echo "  1. 檢查套件依賴是否正確安裝"
        echo "     rosdep install --from-paths src --ignore-src -r -y"
    fi
    
    # 檢查權限問題
    if grep -q "Permission denied" "$log_dir"/* 2>/dev/null; then
        echo "  2. 檢查檔案權限"
        echo "     sudo chown -R \$USER:\$USER $APP_DIR/$workspace"
    fi
    
    # 檢查磁碟空間
    local available_space=$(df "$APP_DIR" | awk 'NR==2 {print $4}')
    if [ "$available_space" -lt 1048576 ]; then  # 小於 1GB
        echo "  3. 磁碟空間不足，建議清理"
        echo "     df -h $APP_DIR"
    fi
    
    # 工作空間專屬建議
    case "$workspace" in
        "agv_ws"|"*_agv")
            echo "  4. AGV 工作空間建議:"
            echo "     - 確認 agv_interfaces 先建置完成"
            echo "     - 檢查 Python 路徑設定"
            ;;
        "web_api_ws")
            echo "  4. Web API 建議:"
            echo "     - 確認 FastAPI 和相關套件已安裝"
            echo "     - 檢查 Node.js 前端建置"
            ;;
        "db_proxy_ws")
            echo "  4. 資料庫代理建議:"
            echo "     - 確認 PostgreSQL 開發套件已安裝"
            echo "     - 檢查資料庫連接設定"
            ;;
    esac
}

# ============================================================================
# 建置配置檔實現
# ============================================================================

run_fast_build() {
    local workspaces=("$@")
    local jobs="${BUILD_JOBS:-$DEFAULT_JOBS}"
    local continue_on_error="true"
    
    log_info "執行快速並行建置 ($jobs 個並行任務)"
    
    local success_count=0
    local total_count=${#workspaces[@]}
    
    for workspace in "${workspaces[@]}"; do
        if build_workspace "$workspace" "fast" "$jobs" "$DEFAULT_TIMEOUT" "false" "normal" "$continue_on_error"; then
            ((success_count++))
        fi
    done
    
    log_info "快速建置完成: $success_count/$total_count 個工作空間建置成功"
    
    if [ $success_count -eq $total_count ]; then
        return 0
    else
        return 1
    fi
}

run_full_build() {
    local workspaces=("$@")
    local jobs="${BUILD_JOBS:-2}"  # 完整建置使用較少並行任務
    
    log_info "執行完整建置 (包含測試)"
    
    local success_count=0
    local total_count=${#workspaces[@]}
    
    for workspace in "${workspaces[@]}"; do
        log_build "完整建置: $workspace"
        
        # 先建置
        if build_workspace "$workspace" "full" "$jobs" "$DEFAULT_TIMEOUT" "false" "verbose" "false"; then
            # 再執行測試
            log_info "執行測試: $workspace"
            cd "$APP_DIR/$workspace" || continue
            
            if timeout 120 colcon test --event-handlers console_direct+; then
                log_success "$workspace 測試通過"
                ((success_count++))
            else
                log_error "$workspace 測試失敗"
            fi
        fi
    done
    
    log_info "完整建置完成: $success_count/$total_count 個工作空間通過測試"
    
    if [ $success_count -eq $total_count ]; then
        return 0
    else
        return 1
    fi
}

run_incremental_build() {
    local workspaces=("$@")
    
    log_info "執行增量建置 (只建置修改的工作空間)"
    
    local modified_workspaces=()
    
    # 檢查哪些工作空間需要重新建置
    for workspace in "${workspaces[@]}"; do
        if check_workspace_modified "$workspace"; then
            modified_workspaces+=("$workspace")
            log_info "檢測到修改: $workspace"
        else
            log_info "跳過未修改: $workspace"
        fi
    done
    
    if [ ${#modified_workspaces[@]} -eq 0 ]; then
        log_success "所有工作空間都是最新的，無需建置"
        return 0
    fi
    
    log_info "需要建置 ${#modified_workspaces[@]} 個工作空間: ${modified_workspaces[*]}"
    
    # 執行快速建置
    run_fast_build "${modified_workspaces[@]}"
}

run_debug_build() {
    local workspaces=("$@")
    
    log_info "執行除錯建置 (包含詳細日誌)"
    
    for workspace in "${workspaces[@]}"; do
        log_build "除錯建置: $workspace"
        
        if ! build_workspace "$workspace" "debug" 1 "$DEFAULT_TIMEOUT" "true" "verbose" "false"; then
            log_error "除錯建置失敗，停止後續建置"
            return 1
        fi
    done
    
    log_success "除錯建置完成"
}

run_release_build() {
    local workspaces=("$@")
    local clean_first="$1"
    
    log_info "執行發布建置 (最佳化編譯)"
    
    if [ "$clean_first" = "true" ]; then
        log_info "清理所有工作空間..."
        for workspace in "${workspaces[@]}"; do
            cd "$APP_DIR/$workspace" || continue
            rm -rf build/ install/ log/ 2>/dev/null || true
            log_info "已清理: $workspace"
        done
    fi
    
    local success_count=0
    local total_count=${#workspaces[@]}
    
    for workspace in "${workspaces[@]}"; do
        log_build "發布建置: $workspace"
        
        if build_workspace "$workspace" "release" 2 "$DEFAULT_TIMEOUT" "false" "normal" "false"; then
            ((success_count++))
        else
            log_error "發布建置失敗: $workspace"
            return 1
        fi
    done
    
    log_success "發布建置完成: $success_count/$total_count 個工作空間建置成功"
}

# ============================================================================
# 報告和統計功能
# ============================================================================

generate_build_report() {
    local workspaces=("$@")
    local report_file="/tmp/rosagv_build_report_$(date +%Y%m%d_%H%M%S).txt"
    
    log_info "生成建置報告..."
    
    {
        echo "RosAGV 建置報告"
        echo "生成時間: $(date)"
        echo "========================================"
        echo ""
        
        echo "=== 系統環境 ==="
        echo "容器類型: ${CONTAINER_TYPE:-未設定}"
        echo "Python 版本: $(python3 --version 2>/dev/null || echo '未安裝')"
        echo "ROS 2 版本: ${ROS_DISTRO:-未設定}"
        echo "Colcon 版本: $(colcon --version 2>/dev/null || echo '未安裝')"
        echo ""
        
        echo "=== 工作空間狀態 ==="
        for workspace in "${workspaces[@]}"; do
            local workspace_path="$APP_DIR/$workspace"
            
            if [ -d "$workspace_path" ]; then
                echo "📁 $workspace"
                
                # 檢查建置狀態
                if [ -d "$workspace_path/install" ]; then
                    local install_time=$(stat -c %Y "$workspace_path/install" 2>/dev/null || echo "0")
                    local install_date=$(date -d "@$install_time" 2>/dev/null || echo "未知")
                    echo "  ✅ 已建置 (時間: $install_date)"
                    
                    # 檢查套件數量
                    local pkg_count=$(find "$workspace_path/install" -name "package.xml" | wc -l)
                    echo "  📦 套件數量: $pkg_count"
                else
                    echo "  ❌ 未建置"
                fi
                
                # 檢查最近錯誤
                if [ -d "$workspace_path/log" ]; then
                    local error_count=$(find "$workspace_path/log" -name "*.log" -exec grep -l "error\|Error\|FAILED" {} \; 2>/dev/null | wc -l)
                    if [ "$error_count" -gt 0 ]; then
                        echo "  ⚠️  發現 $error_count 個錯誤日誌"
                    fi
                fi
                
                echo ""
            fi
        done
        
        echo "=== 建置建議 ==="
        echo "1. 定期清理建置暫存檔案以節省空間"
        echo "2. 使用增量建置加速開發流程"
        echo "3. 大型變更後執行完整建置和測試"
        echo "4. 使用並行建置提高效率 (建議 2-4 個並行任務)"
        echo ""
        
        echo "========================================"
        echo "報告生成完成: $(date)"
        
    } | tee "$report_file"
    
    log_success "建置報告已生成: $report_file"
}

# ============================================================================
# 主程式
# ============================================================================

main() {
    # 先檢查是否為 help 模式
    if [[ "$1" =~ ^(-h|--help)$ ]]; then
        show_header
        show_usage
        exit 0
    fi
    
    # 檢查依賴
    if ! check_dependencies; then
        exit 1
    fi
    
    # 解析參數
    local build_profile="fast"
    local target_workspaces=()
    local category=""
    local jobs="$DEFAULT_JOBS"
    local timeout="$DEFAULT_TIMEOUT"
    local clean_first="false"
    local use_symlink="false"
    local continue_on_error="false"
    local output_format="table"
    local log_level="normal"
    local save_logs=""
    local generate_report="false"
    
    # 檢查第一個參數是否為建置配置檔
    if [[ "$1" =~ ^(fast|full|incremental|debug|release)$ ]]; then
        build_profile="$1"
        shift
    fi
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --workspace)
                target_workspaces+=("$2")
                shift 2
                ;;
            --workspaces)
                IFS=',' read -ra ws_list <<< "$2"
                target_workspaces+=("${ws_list[@]}")
                shift 2
                ;;
            --category)
                category="$2"
                shift 2
                ;;
            --jobs)
                jobs="$2"
                BUILD_JOBS="$jobs"
                shift 2
                ;;
            --timeout)
                timeout="$2"
                shift 2
                ;;
            --clean)
                clean_first="true"
                shift
                ;;
            --symlink)
                use_symlink="true"
                shift
                ;;
            --continue-on-error)
                continue_on_error="true"
                shift
                ;;
            --output)
                output_format="$2"
                shift 2
                ;;
            --log-level)
                log_level="$2"
                shift 2
                ;;
            --save-logs)
                save_logs="$2"
                shift 2
                ;;
            --report)
                generate_report="true"
                shift
                ;;
            -h|--help)
                show_header
                show_usage
                exit 0
                ;;
            *)
                log_error "未知參數: $1"
                show_usage
                exit 1
                ;;
        esac
    done
    
    # 確定要建置的工作空間
    if [ ${#target_workspaces[@]} -eq 0 ]; then
        if [ -n "$category" ]; then
            target_workspaces=($(get_workspaces_by_category "$category"))
        else
            target_workspaces=($(get_all_workspaces))
        fi
    fi
    
    if [ ${#target_workspaces[@]} -eq 0 ]; then
        log_error "沒有找到要建置的工作空間"
        exit 1
    fi
    
    # 創建日誌目錄
    if [ -n "$save_logs" ]; then
        mkdir -p "$save_logs"
        BUILD_LOG_DIR="$save_logs"
    fi
    
    # 顯示標題
    show_header
    log_info "建置配置檔: ${BUILD_PROFILES[$build_profile]}"
    log_info "目標工作空間: ${target_workspaces[*]}"
    log_info "並行任務數: $jobs"
    echo ""
    
    # 執行對應的建置配置檔
    case $build_profile in
        fast)
            run_fast_build "${target_workspaces[@]}"
            ;;
        full)
            run_full_build "${target_workspaces[@]}"
            ;;
        incremental)
            run_incremental_build "${target_workspaces[@]}"
            ;;
        debug)
            run_debug_build "${target_workspaces[@]}"
            ;;
        release)
            run_release_build "${target_workspaces[@]}" "$clean_first"
            ;;
        *)
            log_error "未知的建置配置檔: $build_profile"
            exit 1
            ;;
    esac
    
    local build_result=$?
    
    # 生成報告
    if [ "$generate_report" = "true" ]; then
        generate_build_report "${target_workspaces[@]}"
    fi
    
    # 返回建置結果
    if [ $build_result -eq 0 ]; then
        log_success "建置任務完成！"
    else
        log_error "建置任務失敗！"
    fi
    
    exit $build_result
}

# 如果直接執行此腳本
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi