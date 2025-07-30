#!/bin/bash
# RosAGV 測試執行和報告工具
# 版本: 1.0
# 說明: 自動化測試執行、結果分析和報告生成，支援 ROS 2 測試框架

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

# 測試類型定義
declare -A TEST_TYPES=(
    ["unit"]="單元測試"
    ["integration"]="整合測試"
    ["system"]="系統測試"
    ["performance"]="性能測試"
    ["regression"]="迴歸測試"
)

# 測試框架檢測
declare -A TEST_FRAMEWORKS=(
    ["pytest"]="Python pytest 框架"
    ["unittest"]="Python unittest 框架"
    ["gtest"]="Google Test C++ 框架"
    ["colcon"]="ROS 2 colcon 測試框架"
)

# 報告格式
declare -A REPORT_FORMATS=(
    ["console"]="控制台輸出"
    ["html"]="HTML 格式報告"
    ["xml"]="XML 格式報告"
    ["json"]="JSON 格式報告"
    ["junit"]="JUnit XML 格式"
)

# 預設設定
DEFAULT_TIMEOUT=300
DEFAULT_RETRIES=1
REPORT_DIR="/tmp/rosagv_test_reports"

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

log_test() {
    echo -e "${PURPLE}[TEST]${NC} $1"
}

show_header() {
    echo -e "${CYAN}🧪 RosAGV 測試執行和報告工具${NC}"
    echo -e "${CYAN}===============================${NC}"
    echo ""
}

show_usage() {
    echo "使用方式:"
    echo "  $0 [test_type] [options]"
    echo ""
    echo "測試類型:"
    for type in "${!TEST_TYPES[@]}"; do
        echo "  $type                     # ${TEST_TYPES[$type]}"
    done
    echo ""
    echo "選項:"
    echo "  --workspace <WS>         # 指定單一工作空間"
    echo "  --workspaces <WSs>       # 指定多個工作空間，逗號分隔"
    echo "  --package <PKG>          # 指定單一套件"
    echo "  --packages <PKGs>        # 指定多個套件，逗號分隔"
    echo "  --test-name <NAME>       # 指定特定測試名稱或模式"
    echo "  --framework <FW>         # 指定測試框架: pytest, unittest, gtest, colcon"
    echo "  --timeout <SEC>          # 測試超時時間 (預設: ${DEFAULT_TIMEOUT}s)"
    echo "  --retries <N>            # 失敗重試次數 (預設: $DEFAULT_RETRIES)"
    echo "  --parallel               # 並行執行測試"
    echo "  --coverage               # 生成覆蓋率報告"
    echo "  --output <FORMAT>        # 報告格式: console, html, xml, json, junit"
    echo "  --report-dir <DIR>       # 報告輸出目錄"
    echo "  --verbose                # 顯示詳細輸出"
    echo "  --dry-run                # 只顯示要執行的測試，不實際執行"
    echo "  --continue-on-failure    # 測試失敗時繼續執行其他測試"
    echo "  --filter <PATTERN>       # 過濾測試案例"
    echo "  --baseline <DIR>         # 與基準測試結果比較"
    echo "  -h, --help              # 顯示此幫助資訊"
    echo ""
    echo "範例:"
    echo "  $0 unit --workspace agv_ws               # 執行 AGV 工作空間單元測試"
    echo "  $0 integration --parallel --coverage     # 並行執行整合測試並生成覆蓋率"
    echo "  $0 system --output html --report-dir ./reports"
    echo "  $0 regression --baseline ./last_results  # 迴歸測試與基準比較"
    echo "  $0 performance --filter '*performance*'  # 執行性能相關測試"
}

# ============================================================================
# 核心測試函數
# ============================================================================

check_dependencies() {
    local missing_deps=()
    local optional_deps=()
    
    # 檢查必要工具
    if ! command -v colcon &> /dev/null; then
        missing_deps+=("colcon")
    fi
    
    # 檢查可選工具
    if ! command -v pytest &> /dev/null; then
        optional_deps+=("pytest")
    fi
    
    if ! command -v coverage &> /dev/null; then
        optional_deps+=("coverage")
    fi
    
    if [ ${#missing_deps[@]} -gt 0 ]; then
        log_error "缺少必要工具: ${missing_deps[*]}"
        return 1
    fi
    
    if [ ${#optional_deps[@]} -gt 0 ]; then
        log_warning "缺少可選工具: ${optional_deps[*]}"
        log_info "某些功能可能無法使用"
    fi
    
    return 0
}

discover_workspaces() {
    local workspaces=()
    
    for dir in "$APP_DIR"/*_ws; do
        if [ -d "$dir" ]; then
            workspaces+=($(basename "$dir"))
        fi
    done
    
    # 添加 path_algorithm
    if [ -d "$APP_DIR/path_algorithm" ]; then
        workspaces+=("path_algorithm")
    fi
    
    echo "${workspaces[@]}"
}

discover_test_files() {
    local workspace="$1"
    local workspace_path="$APP_DIR/$workspace"
    
    if [ ! -d "$workspace_path" ]; then
        return 1
    fi
    
    local test_files=()
    
    # 查找 Python 測試檔案
    while IFS= read -r -d '' file; do
        test_files+=("$file")
    done < <(find "$workspace_path" -name "test_*.py" -o -name "*_test.py" -print0 2>/dev/null)
    
    # 查找 C++ 測試檔案
    while IFS= read -r -d '' file; do
        test_files+=("$file")
    done < <(find "$workspace_path" -name "*_test.cpp" -o -name "test_*.cpp" -print0 2>/dev/null)
    
    echo "${test_files[@]}"
}

detect_test_framework() {
    local workspace="$1"
    local workspace_path="$APP_DIR/$workspace"
    
    # 檢查是否有 pytest.ini 或 pytest 相關配置
    if [ -f "$workspace_path/pytest.ini" ] || [ -f "$workspace_path/pyproject.toml" ]; then
        echo "pytest"
        return
    fi
    
    # 檢查是否有 CMakeLists.txt (可能使用 gtest)
    if find "$workspace_path" -name "CMakeLists.txt" -exec grep -l "gtest\|GTest" {} \; 2>/dev/null | head -1 >/dev/null; then
        echo "gtest"
        return
    fi
    
    # 檢查 Python 測試檔案中的 import
    local python_tests=$(find "$workspace_path" -name "test_*.py" -o -name "*_test.py" 2>/dev/null | head -1)
    if [ -n "$python_tests" ]; then
        if grep -q "import pytest" "$python_tests" 2>/dev/null; then
            echo "pytest"
            return
        elif grep -q "import unittest" "$python_tests" 2>/dev/null; then
            echo "unittest"
            return
        fi
    fi
    
    # 預設使用 colcon
    echo "colcon"
}

run_colcon_tests() {
    local workspace="$1"
    local packages=("${@:2}")
    local timeout="$3"
    local parallel="$4"
    local verbose="$5"
    local coverage="$6"
    
    local workspace_path="$APP_DIR/$workspace"
    
    if [ ! -d "$workspace_path" ]; then
        log_error "工作空間不存在: $workspace"
        return 1
    fi
    
    cd "$workspace_path" || return 1
    
    # 確保工作空間已建置
    if [ ! -d "install" ]; then
        log_warning "工作空間未建置，先執行建置..."
        colcon build --event-handlers console_direct+
    fi
    
    # 載入環境
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    fi
    
    # 建構測試指令
    local test_args=()
    test_args+=("test")
    test_args+=("--event-handlers" "console_direct+")
    
    # 設定超時
    test_args+=("--pytest-args" "--timeout=$timeout")
    
    # 設定並行執行
    if [ "$parallel" = "true" ]; then
        test_args+=("--parallel-workers" "$(nproc)")
    fi
    
    # 設定詳細輸出
    if [ "$verbose" = "true" ]; then
        test_args+=("--pytest-args" "-v")
    fi
    
    # 設定覆蓋率
    if [ "$coverage" = "true" ]; then
        test_args+=("--pytest-args" "--cov=.")
    fi
    
    # 指定套件
    if [ ${#packages[@]} -gt 0 ]; then
        for pkg in "${packages[@]}"; do
            test_args+=("--packages-select" "$pkg")
        done
    fi
    
    log_test "執行 colcon 測試: ${test_args[*]}"
    
    # 執行測試
    if timeout "$timeout" colcon "${test_args[@]}"; then
        log_success "colcon 測試完成"
        return 0
    else
        log_error "colcon 測試失敗"
        return 1
    fi
}

run_pytest_tests() {
    local workspace="$1" 
    local test_pattern="$2"
    local timeout="$3"
    local parallel="$4"
    local verbose="$5"
    local coverage="$6"
    local output_format="$7"
    local report_dir="$8"
    
    local workspace_path="$APP_DIR/$workspace"
    
    if [ ! -d "$workspace_path" ]; then
        log_error "工作空間不存在: $workspace"
        return 1
    fi
    
    cd "$workspace_path" || return 1
    
    # 建構 pytest 指令
    local pytest_args=()
    
    # 設定測試路徑或模式
    if [ -n "$test_pattern" ]; then
        pytest_args+=("-k" "$test_pattern")
    fi
    
    # 設定超時
    pytest_args+=("--timeout=$timeout")
    
    # 設定並行執行
    if [ "$parallel" = "true" ] && command -v pytest-xdist &> /dev/null; then
        pytest_args+=("-n" "auto")
    fi
    
    # 設定詳細輸出
    if [ "$verbose" = "true" ]; then
        pytest_args+=("-v")
    fi
    
    # 設定覆蓋率
    if [ "$coverage" = "true" ]; then
        pytest_args+=("--cov=.")
        pytest_args+=("--cov-report=html:$report_dir/coverage")
        pytest_args+=("--cov-report=xml:$report_dir/coverage.xml")
    fi
    
    # 設定輸出格式
    case "$output_format" in
        "html")
            pytest_args+=("--html=$report_dir/report.html")
            ;;
        "xml"|"junit")
            pytest_args+=("--junit-xml=$report_dir/results.xml")
            ;;
        "json")
            pytest_args+=("--json-report" "--json-report-file=$report_dir/results.json")
            ;;
    esac
    
    log_test "執行 pytest: ${pytest_args[*]}"
    
    # 執行測試
    if timeout "$timeout" pytest "${pytest_args[@]}"; then
        log_success "pytest 測試完成"
        return 0
    else
        log_error "pytest 測試失敗"
        return 1
    fi
}

run_performance_tests() {
    local workspaces=("$@")
    
    log_test "執行性能測試..."
    
    for workspace in "${workspaces[@]}"; do
        log_info "性能測試工作空間: $workspace"
        
        # 查找性能測試檔案
        local perf_tests=$(find "$APP_DIR/$workspace" -name "*performance*" -name "*.py" 2>/dev/null)
        
        if [ -z "$perf_tests" ]; then
            log_warning "$workspace 沒有找到性能測試"
            continue
        fi
        
        # 執行性能測試
        for test_file in $perf_tests; do
            log_test "執行性能測試: $(basename "$test_file")"
            
            cd "$(dirname "$test_file")" || continue
            
            # 使用 time 指令測量執行時間
            if command -v python3 &> /dev/null; then
                local test_output=$(/usr/bin/time -f "Time: %e seconds, Memory: %M KB" python3 "$(basename "$test_file")" 2>&1)
                echo "$test_output"
                
                # 提取性能指標
                local exec_time=$(echo "$test_output" | grep "Time:" | sed 's/.*Time: \([0-9.]*\) seconds.*/\1/')
                local memory_usage=$(echo "$test_output" | grep "Memory:" | sed 's/.*Memory: \([0-9]*\) KB.*/\1/')
                
                log_info "執行時間: ${exec_time}s, 記憶體使用: ${memory_usage}KB"
            fi
        done
    done
}

analyze_test_results() {
    local workspace="$1"
    local report_dir="$2"
    
    log_info "分析測試結果..."
    
    local workspace_path="$APP_DIR/$workspace"
    local log_dir="$workspace_path/log"
    
    # 分析 colcon test 結果
    if [ -d "$log_dir" ]; then
        local latest_test=$(find "$log_dir" -name "latest_test" -type l 2>/dev/null)
        if [ -n "$latest_test" ]; then
            local actual_log_dir=$(readlink -f "$latest_test")
            
            # 統計測試結果
            local total_tests=0
            local passed_tests=0
            local failed_tests=0
            local skipped_tests=0
            
            # 查找測試結果檔案
            local result_files=$(find "$actual_log_dir" -name "*.xml" -o -name "*.json" 2>/dev/null)
            
            if [ -n "$result_files" ]; then
                for result_file in $result_files; do
                    if [[ "$result_file" == *.xml ]]; then
                        # 解析 XML 結果
                        if command -v xmllint &> /dev/null; then
                            local xml_tests=$(xmllint --xpath "count(//testcase)" "$result_file" 2>/dev/null || echo "0")
                            local xml_failures=$(xmllint --xpath "count(//failure)" "$result_file" 2>/dev/null || echo "0")
                            local xml_errors=$(xmllint --xpath "count(//error)" "$result_file" 2>/dev/null || echo "0")
                            local xml_skipped=$(xmllint --xpath "count(//skipped)" "$result_file" 2>/dev/null || echo "0")
                            
                            total_tests=$((total_tests + xml_tests))
                            failed_tests=$((failed_tests + xml_failures + xml_errors))
                            skipped_tests=$((skipped_tests + xml_skipped))
                        fi
                    fi
                done
                
                passed_tests=$((total_tests - failed_tests - skipped_tests))
                
                # 輸出統計結果
                echo -e "${CYAN}測試統計:${NC}"
                echo -e "  總測試數: ${BLUE}$total_tests${NC}"
                echo -e "  通過: ${GREEN}$passed_tests${NC}"
                echo -e "  失敗: ${RED}$failed_tests${NC}"
                echo -e "  跳過: ${YELLOW}$skipped_tests${NC}"
                
                if [ $total_tests -gt 0 ]; then
                    local pass_rate=$((passed_tests * 100 / total_tests))
                    echo -e "  成功率: ${GREEN}${pass_rate}%${NC}"
                fi
            fi
        fi
    fi
    
    # 分析失敗的測試
    if [ $failed_tests -gt 0 ]; then
        echo -e "${YELLOW}失敗測試分析:${NC}"
        
        # 查找錯誤模式
        local error_patterns=(
            "AssertionError"
            "TimeoutError"
            "ConnectionError"
            "ImportError"
            "AttributeError"
        )
        
        for pattern in "${error_patterns[@]}"; do
            local pattern_count=$(rg -r "$pattern" "$actual_log_dir" 2>/dev/null | wc -l)
            if [ $pattern_count -gt 0 ]; then
                echo -e "  ${RED}$pattern: $pattern_count 次${NC}"
            fi
        done
    fi
}

generate_test_report() {
    local workspaces=("$@")
    local report_dir="$1"; shift
    local output_format="$1"; shift  
    local workspaces=("$@")
    
    mkdir -p "$report_dir"
    
    local report_file="$report_dir/test_report_$(date +%Y%m%d_%H%M%S)"
    
    case "$output_format" in
        "html")
            report_file="${report_file}.html"
            generate_html_report "$report_file" "${workspaces[@]}"
            ;;
        "json")
            report_file="${report_file}.json"
            generate_json_report "$report_file" "${workspaces[@]}"
            ;;
        *)
            report_file="${report_file}.txt"
            generate_text_report "$report_file" "${workspaces[@]}"
            ;;
    esac
    
    log_success "測試報告已生成: $report_file"
}

generate_text_report() {
    local report_file="$1"
    shift
    local workspaces=("$@")
    
    {
        echo "RosAGV 測試執行報告"
        echo "生成時間: $(date)"
        echo "========================================"
        echo ""
        
        echo "=== 測試環境 ==="
        echo "容器類型: ${CONTAINER_TYPE:-未設定}"
        echo "Python 版本: $(python3 --version 2>/dev/null || echo '未安裝')"
        echo "pytest 版本: $(pytest --version 2>/dev/null | head -1 || echo '未安裝')"
        echo ""
        
        echo "=== 工作空間測試結果 ==="
        for workspace in "${workspaces[@]}"; do
            echo "📁 $workspace"
            
            # 檢查測試檔案
            local test_files=($(discover_test_files "$workspace"))
            echo "  測試檔案數量: ${#test_files[@]}"
            
            # 檢查測試框架
            local framework=$(detect_test_framework "$workspace")
            echo "  檢測到的測試框架: $framework"
            
            # 分析測試結果
            analyze_test_results "$workspace" "$report_file" || true
            
            echo ""
        done
        
        echo "=== 測試建議 ==="
        echo "1. 定期執行完整測試套件確保程式品質"
        echo "2. 新功能開發時先寫測試案例 (TDD)"
        echo "3. 維持測試覆蓋率在 80% 以上"
        echo "4. 使用並行測試提高執行效率"
        echo ""
        
        echo "========================================"
        echo "報告生成完成: $(date)"
        
    } | tee "$report_file"
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
    local test_type="unit"
    local target_workspaces=()
    local target_packages=()
    local test_name=""
    local framework=""
    local timeout="$DEFAULT_TIMEOUT"
    local retries="$DEFAULT_RETRIES"
    local parallel="false"
    local coverage="false"
    local output_format="console"
    local report_dir="$REPORT_DIR"
    local verbose="false"
    local dry_run="false"
    local continue_on_failure="false"
    local filter_pattern=""
    local baseline_dir=""
    
    # 檢查第一個參數是否為測試類型
    if [[ "$1" =~ ^(unit|integration|system|performance|regression)$ ]]; then
        test_type="$1"
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
            --package)
                target_packages+=("$2")
                shift 2
                ;;
            --packages)
                IFS=',' read -ra pkg_list <<< "$2"
                target_packages+=("${pkg_list[@]}")
                shift 2
                ;;
            --test-name)
                test_name="$2"
                shift 2
                ;;
            --framework)
                framework="$2"
                shift 2
                ;;
            --timeout)
                timeout="$2"
                shift 2
                ;;
            --retries)
                retries="$2"
                shift 2
                ;;
            --parallel)
                parallel="true"
                shift
                ;;
            --coverage)
                coverage="true"
                shift
                ;;
            --output)
                output_format="$2"
                shift 2
                ;;
            --report-dir)
                report_dir="$2"
                shift 2
                ;;
            --verbose)
                verbose="true"
                shift
                ;;
            --dry-run)
                dry_run="true"
                shift
                ;;
            --continue-on-failure)
                continue_on_failure="true"
                shift
                ;;
            --filter)
                filter_pattern="$2"
                shift 2
                ;;
            --baseline)
                baseline_dir="$2"
                shift 2
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
    
    # 如果沒有指定工作空間，自動發現
    if [ ${#target_workspaces[@]} -eq 0 ]; then
        target_workspaces=($(discover_workspaces))
    fi
    
    if [ ${#target_workspaces[@]} -eq 0 ]; then
        log_error "沒有找到要測試的工作空間"
        exit 1
    fi
    
    # 創建報告目錄
    mkdir -p "$report_dir"
    
    # 顯示標題
    show_header
    log_info "測試類型: ${TEST_TYPES[$test_type]}"
    log_info "目標工作空間: ${target_workspaces[*]}"
    if [ ${#target_packages[@]} -gt 0 ]; then
        log_info "目標套件: ${target_packages[*]}"
    fi
    echo ""
    
    # Dry run 模式
    if [ "$dry_run" = "true" ]; then
        log_info "Dry run 模式 - 顯示要執行的測試："
        for workspace in "${target_workspaces[@]}"; do
            local test_files=($(discover_test_files "$workspace"))
            local detected_framework=$(detect_test_framework "$workspace")
            
            echo "📁 $workspace"
            echo "  框架: $detected_framework"
            echo "  測試檔案: ${#test_files[@]} 個"
            for test_file in "${test_files[@]}"; do
                echo "    - $(basename "$test_file")"
            done
            echo ""
        done
        exit 0
    fi
    
    # 執行測試
    local overall_success=true
    
    for workspace in "${target_workspaces[@]}"; do
        log_test "測試工作空間: $workspace"
        
        # 檢測測試框架
        local workspace_framework="$framework"
        if [ -z "$workspace_framework" ]; then
            workspace_framework=$(detect_test_framework "$workspace")
        fi
        
        log_info "使用測試框架: $workspace_framework"
        
        # 根據測試類型和框架執行測試
        local test_success=false
        
        case "$test_type" in
            "performance")
                run_performance_tests "$workspace"
                test_success=$?
                ;;
            *)
                case "$workspace_framework" in
                    "pytest")
                        run_pytest_tests "$workspace" "$filter_pattern" "$timeout" "$parallel" "$verbose" "$coverage" "$output_format" "$report_dir"
                        test_success=$?
                        ;;
                    "colcon"|*)
                        run_colcon_tests "$workspace" "${target_packages[@]}" "$timeout" "$parallel" "$verbose" "$coverage"
                        test_success=$?
                        ;;
                esac
                ;;
        esac
        
        if [ $test_success -eq 0 ]; then
            log_success "$workspace 測試通過"
        else
            log_error "$workspace 測試失敗"
            overall_success=false
            
            if [ "$continue_on_failure" != "true" ]; then
                break
            fi
        fi
        
        echo ""
    done
    
    # 生成報告
    generate_test_report "$report_dir" "$output_format" "${target_workspaces[@]}"
    
    # 返回結果
    if [ "$overall_success" = true ]; then
        log_success "所有測試完成！"
        exit 0
    else
        log_error "部分測試失敗！"
        exit 1
    fi
}

# 如果直接執行此腳本
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi