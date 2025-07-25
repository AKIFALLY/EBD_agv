#!/bin/bash
# RosAGV 代碼分析和檢查工具
# 版本: 1.0
# 說明: Python 代碼品質檢查、ROS 2 最佳實踐驗證、代碼風格統一檢查和安全漏洞掃描

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

# 分析類型定義
declare -A ANALYSIS_TYPES=(
    ["style"]="代碼風格檢查"
    ["quality"]="代碼品質分析"
    ["security"]="安全漏洞掃描"
    ["ros2"]="ROS 2 最佳實踐"
    ["complexity"]="代碼複雜度分析"
    ["imports"]="依賴關係分析"
    ["documentation"]="文檔完整性檢查"
    ["performance"]="性能問題檢測"
)

# 嚴重程度定義
declare -A SEVERITY_LEVELS=(
    ["critical"]="嚴重"
    ["error"]="錯誤"
    ["warning"]="警告"
    ["info"]="資訊"
    ["style"]="風格"
)

# 工具配置
declare -A ANALYSIS_TOOLS=(
    ["flake8"]="Python 代碼風格檢查"
    ["pylint"]="Python 代碼品質分析"
    ["mypy"]="Python 類型檢查"
    ["bandit"]="Python 安全漏洞掃描"
    ["black"]="Python 代碼格式化檢查"
    ["isort"]="Python import 排序檢查"
    ["radon"]="Python 代碼複雜度分析"
    ["safety"]="Python 套件安全檢查"
)

# 預設設定
DEFAULT_MAX_LINE_LENGTH=88
DEFAULT_COMPLEXITY_THRESHOLD=10
REPORT_DIR="/tmp/rosagv_code_analysis"

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

log_analysis() {
    echo -e "${PURPLE}[ANALYSIS]${NC} $1"
}

show_header() {
    echo -e "${CYAN}🔍 RosAGV 代碼分析和檢查工具${NC}"
    echo -e "${CYAN}=============================${NC}"
    echo ""
}

show_usage() {
    echo "使用方式:"
    echo "  $0 [analysis_type] [options]"
    echo ""
    echo "分析類型:"
    for type in "${!ANALYSIS_TYPES[@]}"; do
        echo "  $type                     # ${ANALYSIS_TYPES[$type]}"
    done
    echo ""
    echo "選項:"
    echo "  --workspace <WS>         # 指定單一工作空間"
    echo "  --workspaces <WSs>       # 指定多個工作空間，逗號分隔"
    echo "  --file <FILE>            # 指定單一檔案"
    echo "  --files <FILES>          # 指定多個檔案，逗號分隔"
    echo "  --exclude <PATTERN>      # 排除檔案模式"
    echo "  --include <PATTERN>      # 僅包含檔案模式"
    echo "  --severity <LEVEL>       # 最低嚴重程度: critical, error, warning, info, style"
    echo "  --max-line-length <N>    # 最大行長度 (預設: $DEFAULT_MAX_LINE_LENGTH)"
    echo "  --complexity-max <N>     # 最大複雜度 (預設: $DEFAULT_COMPLEXITY_THRESHOLD)"
    echo "  --output <FORMAT>        # 輸出格式: console, json, html, csv"
    echo "  --report-dir <DIR>       # 報告輸出目錄"
    echo "  --fix                    # 自動修復可修復的問題"
    echo "  --config <FILE>          # 使用自定義配置檔案"
    echo "  --verbose                # 顯示詳細輸出"
    echo "  --dry-run                # 只檢查，不執行實際分析"
    echo "  -h, --help              # 顯示此幫助資訊"
    echo ""
    echo "範例:"
    echo "  $0 style --workspace agv_ws              # 檢查 AGV 工作空間代碼風格"
    echo "  $0 quality --workspaces agv_ws,web_api_ws # 分析多個工作空間代碼品質"
    echo "  $0 security --severity error             # 安全掃描，只顯示錯誤級別以上"
    echo "  $0 ros2 --file src/agv_node.py --fix     # ROS 2 檢查並自動修復"
    echo "  $0 complexity --output html --report-dir ./reports"
}

# ============================================================================
# 核心分析函數
# ============================================================================

check_dependencies() {
    # 先檢查是否為 help 模式，如果是就直接返回成功
    for arg in "$@"; do
        if [[ "$arg" =~ ^(-h|--help)$ ]]; then
            return 0
        fi
    done
    
    local missing_tools=()
    local optional_tools=()
    
    # 檢查 Python 環境
    if ! command -v python3 &> /dev/null; then
        missing_tools+=("python3")
    fi
    
    # 檢查基本工具
    for tool in find grep sed awk; do
        if ! command -v "$tool" &> /dev/null; then
            missing_tools+=("$tool")
        fi
    done
    
    # 檢查 Python 分析工具
    for tool in "${!ANALYSIS_TOOLS[@]}"; do
        if ! python3 -c "import $tool" &> /dev/null; then
            optional_tools+=("$tool")
        fi
    done
    
    if [ ${#missing_tools[@]} -gt 0 ]; then
        log_error "缺少必要工具: ${missing_tools[*]}"
        return 1
    fi
    
    if [ ${#optional_tools[@]} -gt 0 ]; then
        log_warning "建議安裝分析工具以獲得完整功能:"
        for tool in "${optional_tools[@]}"; do
            echo "  pip3 install $tool"
        done
        echo ""
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

find_python_files() {
    local search_paths=("$@")
    local python_files=()
    
    for path in "${search_paths[@]}"; do
        if [ -f "$path" ]; then
            # 檢查是否為 Python 檔案
            if [[ "$path" == *.py ]] || head -1 "$path" 2>/dev/null | grep -q "#!/.*python"; then
                python_files+=("$path")
            fi
        elif [ -d "$path" ]; then
            # 遞迴查找 Python 檔案
            while IFS= read -r -d '' file; do
                python_files+=("$file")
            done < <(find "$path" -name "*.py" -type f -print0 2>/dev/null)
        fi
    done
    
    echo "${python_files[@]}"
}

analyze_code_style() {
    local files=("$@")
    local max_line_length="$1"; shift
    local files=("$@")
    
    log_analysis "執行代碼風格檢查..."
    
    local issues=()
    local total_files=${#files[@]}
    local processed_files=0
    
    for file in "${files[@]}"; do
        processed_files=$((processed_files + 1))
        log_info "處理檔案 ($processed_files/$total_files): $(basename "$file")"
        
        # Flake8 檢查
        if command -v flake8 &> /dev/null; then
            local flake8_result
            flake8_result=$(flake8 --max-line-length="$max_line_length" --select=E,W,F --format='%(path)s:%(row)d:%(col)d: %(code)s %(text)s' "$file" 2>/dev/null || true)
            
            if [ -n "$flake8_result" ]; then
                while IFS= read -r line; do
                    if [ -n "$line" ]; then
                        issues+=("style:warning:flake8:$line")
                    fi
                done <<< "$flake8_result"
            fi
        fi
        
        # Black 格式檢查
        if command -v black &> /dev/null; then
            if ! black --check --quiet "$file" 2>/dev/null; then
                issues+=("style:info:black:$file:代碼格式不符合 Black 標準")
            fi
        fi
        
        # isort import 排序檢查
        if command -v isort &> /dev/null; then
            if ! isort --check-only --quiet "$file" 2>/dev/null; then
                issues+=("style:info:isort:$file:import 語句排序不正確")
            fi
        fi
        
        # 自定義風格檢查
        check_custom_style_rules "$file" issues
    done
    
    echo "${issues[@]}"
}

check_custom_style_rules() {
    local file="$1"
    local -n issues_ref=$2
    
    local line_num=0
    while IFS= read -r line; do
        line_num=$((line_num + 1))
        
        # 檢查 TODO/FIXME 註釋
        if [[ "$line" =~ (TODO|FIXME|XXX|HACK) ]]; then
            issues_ref+=("style:info:custom:$file:$line_num:發現待辦事項註釋")
        fi
        
        # 檢查過長的行 (已由 flake8 處理，這裡作為備用)
        if [ ${#line} -gt $DEFAULT_MAX_LINE_LENGTH ]; then
            issues_ref+=("style:warning:custom:$file:$line_num:行過長 (${#line} 字符)")
        fi
        
        # 檢查硬編碼路徑
        if [[ "$line" =~ /home/|/tmp/|C:\\ ]]; then
            issues_ref+=("style:warning:custom:$file:$line_num:可能包含硬編碼路徑")
        fi
        
        # 檢查 print 語句 (在生產代碼中)
        if [[ "$line" =~ ^[[:space:]]*print\( ]] && [[ "$file" != *test* ]]; then
            issues_ref+=("style:info:custom:$file:$line_num:生產代碼中使用 print 語句")
        fi
        
    done < "$file"
}

analyze_code_quality() {
    local files=("$@")
    
    log_analysis "執行代碼品質分析..."
    
    local issues=()
    local total_files=${#files[@]}
    local processed_files=0
    
    for file in "${files[@]}"; do
        processed_files=$((processed_files + 1))
        log_info "處理檔案 ($processed_files/$total_files): $(basename "$file")"
        
        # Pylint 分析
        if command -v pylint &> /dev/null; then
            local pylint_result
            pylint_result=$(pylint --output-format=text --score=no "$file" 2>/dev/null || true)
            
            while IFS= read -r line; do
                if [[ "$line" =~ ^[^:]+:[0-9]+:[0-9]+:[[:space:]]*(C|R|W|E|F)[0-9]+: ]]; then
                    local severity="warning"
                    if [[ "$line" =~ E[0-9]+: ]]; then
                        severity="error"
                    elif [[ "$line" =~ F[0-9]+: ]]; then
                        severity="critical"
                    elif [[ "$line" =~ C[0-9]+: ]]; then
                        severity="style"
                    fi
                    issues+=("quality:$severity:pylint:$line")
                fi
            done <<< "$pylint_result"
        fi
        
        # MyPy 類型檢查
        if command -v mypy &> /dev/null; then
            local mypy_result
            mypy_result=$(mypy --no-error-summary "$file" 2>/dev/null || true)
            
            while IFS= read -r line; do
                if [ -n "$line" ] && [[ "$line" =~ ^[^:]+:[0-9]+: ]]; then
                    local severity="warning"
                    if [[ "$line" =~ error: ]]; then
                        severity="error"
                    fi
                    issues+=("quality:$severity:mypy:$line")
                fi
            done <<< "$mypy_result"
        fi
        
        # 自定義品質檢查
        check_custom_quality_rules "$file" issues
    done
    
    echo "${issues[@]}"
}

check_custom_quality_rules() {
    local file="$1"
    local -n issues_ref=$2
    
    # 檢查函數複雜度 (簡單版本)
    local function_lines=0
    local in_function=false
    local function_name=""
    local line_num=0
    
    while IFS= read -r line; do
        line_num=$((line_num + 1))
        
        # 檢查函數定義
        if [[ "$line" =~ ^[[:space:]]*def[[:space:]]+([a-zA-Z_][a-zA-Z0-9_]*) ]]; then
            if [ "$in_function" = true ] && [ $function_lines -gt 50 ]; then
                issues_ref+=("quality:warning:custom:$file:$line_num:函數 $function_name 過長 ($function_lines 行)")
            fi
            
            function_name="${BASH_REMATCH[1]}"
            in_function=true
            function_lines=1
        elif [ "$in_function" = true ]; then
            if [[ "$line" =~ ^[[:space:]]*def[[:space:]] ]] || [[ "$line" =~ ^[[:space:]]*class[[:space:]] ]] || [[ "$line" =~ ^[^[:space:]] ]]; then
                if [ $function_lines -gt 50 ]; then
                    issues_ref+=("quality:warning:custom:$file:$line_num:函數 $function_name 過長 ($function_lines 行)")
                fi
                in_function=false
            else
                function_lines=$((function_lines + 1))
            fi
        fi
        
        # 檢查異常處理
        if [[ "$line" =~ ^[[:space:]]*except: ]] || [[ "$line" =~ ^[[:space:]]*except[[:space:]]+Exception: ]]; then
            issues_ref+=("quality:warning:custom:$file:$line_num:捕獲過於寬泛的異常")
        fi
        
        # 檢查空的 except 區塊
        if [[ "$line" =~ ^[[:space:]]*except.*:[[:space:]]*$ ]]; then
            local next_line_num=$((line_num + 1))
            local next_line=$(sed -n "${next_line_num}p" "$file")
            if [[ "$next_line" =~ ^[[:space:]]*pass[[:space:]]*$ ]] || [ -z "$next_line" ]; then
                issues_ref+=("quality:error:custom:$file:$line_num:空的異常處理區塊")
            fi
        fi
        
    done < "$file"
}

analyze_security() {
    local files=("$@")
    
    log_analysis "執行安全漏洞掃描..."
    
    local issues=()
    local total_files=${#files[@]}
    local processed_files=0
    
    for file in "${files[@]}"; do
        processed_files=$((processed_files + 1))
        log_info "處理檔案 ($processed_files/$total_files): $(basename "$file")"
        
        # Bandit 安全掃描
        if command -v bandit &> /dev/null; then
            local bandit_result
            bandit_result=$(bandit -f json "$file" 2>/dev/null || true)
            
            if [ -n "$bandit_result" ] && [ "$bandit_result" != "null" ]; then
                # 解析 JSON 結果 (簡化版本)
                local bandit_lines
                bandit_lines=$(echo "$bandit_result" | grep -o '"line_number": [0-9]*' | sed 's/"line_number": //' || true)
                
                if [ -n "$bandit_lines" ]; then
                    while IFS= read -r line_num; do
                        if [ -n "$line_num" ]; then
                            issues+=("security:error:bandit:$file:$line_num:潛在安全問題")
                        fi
                    done <<< "$bandit_lines"
                fi
            fi
        fi
        
        # 自定義安全檢查
        check_custom_security_rules "$file" issues
    done
    
    echo "${issues[@]}"
}

check_custom_security_rules() {
    local file="$1"
    local -n issues_ref=$2
    
    local line_num=0
    while IFS= read -r line; do
        line_num=$((line_num + 1))
        
        # 檢查硬編碼密碼
        if [[ "$line" =~ (password|passwd|pwd)[[:space:]]*=[[:space:]]*[\"\'][^\"\']+ ]]; then
            issues_ref+=("security:critical:custom:$file:$line_num:可能包含硬編碼密碼")
        fi
        
        # 檢查 SQL 注入風險
        if [[ "$line" =~ execute\(.*%.*\) ]] || [[ "$line" =~ \.format\(.*\) ]]; then
            if [[ "$line" =~ (SELECT|INSERT|UPDATE|DELETE) ]]; then
                issues_ref+=("security:error:custom:$file:$line_num:潛在 SQL 注入風險")
            fi
        fi
        
        # 檢查不安全的隨機數
        if [[ "$line" =~ random\.random|random\.randint ]]; then
            issues_ref+=("security:warning:custom:$file:$line_num:使用不安全的隨機數生成器")
        fi
        
        # 檢查 shell 命令注入
        if [[ "$line" =~ os\.system|subprocess\.call.*shell=True ]]; then
            issues_ref+=("security:error:custom:$file:$line_num:潛在命令注入風險")
        fi
        
    done < "$file"
}

analyze_ros2_practices() {
    local files=("$@")
    
    log_analysis "執行 ROS 2 最佳實踐檢查..."
    
    local issues=()
    local total_files=${#files[@]}
    local processed_files=0
    
    for file in "${files[@]}"; do
        processed_files=$((processed_files + 1))
        log_info "處理檔案 ($processed_files/$total_files): $(basename "$file")"
        
        check_ros2_best_practices "$file" issues
    done
    
    echo "${issues[@]}"
}

check_ros2_best_practices() {
    local file="$1"
    local -n issues_ref=$2
    
    local line_num=0
    local has_node_class=false
    local has_lifecycle=false
    local has_parameter_declaration=false
    
    while IFS= read -r line; do
        line_num=$((line_num + 1))
        
        # 檢查是否使用 Node 基類
        if [[ "$line" =~ class[[:space:]]+.*Node\)|rclpy\.node\.Node ]]; then
            has_node_class=true
        fi
        
        # 檢查生命週期管理
        if [[ "$line" =~ lifecycle|LifecycleNode ]]; then
            has_lifecycle=true
        fi
        
        # 檢查參數聲明
        if [[ "$line" =~ declare_parameter ]]; then
            has_parameter_declaration=true
        fi
        
        # 檢查不良實踐
        if [[ "$line" =~ rclpy\.spin_once.*timeout=0 ]]; then
            issues_ref+=("ros2:warning:custom:$file:$line_num:使用 spin_once 無超時可能造成 CPU 占用")
        fi
        
        if [[ "$line" =~ \.get_logger\(\)\.info\(.*\%.*\) ]]; then
            issues_ref+=("ros2:info:custom:$file:$line_num:建議使用 f-string 而非 % 格式化")
        fi
        
        # 檢查 QoS 設定
        if [[ "$line" =~ create_subscription|create_publisher ]] && ! [[ "$line" =~ qos_profile ]]; then
            issues_ref+=("ros2:info:custom:$file:$line_num:建議明確設定 QoS profile")
        fi
        
        # 檢查錯誤處理
        if [[ "$line" =~ rclpy\.spin ]] && ! grep -q "except" "$file"; then
            issues_ref+=("ros2:warning:custom:$file:$line_num:缺少 ROS 2 spin 的異常處理")
        fi
        
    done < "$file"
    
    # 檢查整體結構
    if grep -q "class.*Node" "$file" && ! $has_parameter_declaration; then
        issues_ref+=("ros2:info:custom:$file:0:ROS 2 節點建議聲明參數")
    fi
}

analyze_complexity() {
    local files=("$@")
    local max_complexity="$1"; shift
    local files=("$@")
    
    log_analysis "執行代碼複雜度分析..."
    
    local issues=()
    local total_files=${#files[@]}
    local processed_files=0
    
    for file in "${files[@]}"; do
        processed_files=$((processed_files + 1))
        log_info "處理檔案 ($processed_files/$total_files): $(basename "$file")"
        
        # Radon 複雜度分析
        if command -v radon &> /dev/null; then
            local radon_result
            radon_result=$(radon cc "$file" -s -n "$max_complexity" 2>/dev/null || true)
            
            while IFS= read -r line; do
                if [[ "$line" =~ ^[[:space:]]*[A-Z][[:space:]].*\([0-9]+\) ]]; then
                    local complexity=$(echo "$line" | grep -o '([0-9]*)' | tr -d '()')
                    local function_name=$(echo "$line" | awk '{print $2}')
                    
                    local severity="warning"
                    if [ "$complexity" -gt $((max_complexity * 2)) ]; then
                        severity="error"
                    fi
                    
                    issues+=("complexity:$severity:radon:$file:函數 $function_name 複雜度過高 ($complexity)")
                fi
            done <<< "$radon_result"
        fi
        
        # 自定義複雜度檢查
        check_custom_complexity_rules "$file" "$max_complexity" issues
    done
    
    echo "${issues[@]}"
}

check_custom_complexity_rules() {
    local file="$1"
    local max_complexity="$2"
    local -n issues_ref=$3
    
    local line_num=0
    local function_name=""
    local nested_level=0
    local max_nested=0
    
    while IFS= read -r line; do
        line_num=$((line_num + 1))
        
        # 檢查函數定義
        if [[ "$line" =~ ^[[:space:]]*def[[:space:]]+([a-zA-Z_][a-zA-Z0-9_]*) ]]; then
            function_name="${BASH_REMATCH[1]}"
            nested_level=0
            max_nested=0
        fi
        
        # 計算嵌套層級
        local current_nested=0
        if [[ "$line" =~ ^[[:space:]]*(if|for|while|try|with) ]]; then
            current_nested=$(( (${#line} - ${#line##*( )}) / 4 ))
            if [ $current_nested -gt $max_nested ]; then
                max_nested=$current_nested
            fi
        fi
        
        # 檢查過深的嵌套
        if [ $max_nested -gt 4 ]; then
            issues_ref+=("complexity:warning:custom:$file:$line_num:函數 $function_name 嵌套層級過深 ($max_nested)")
        fi
        
        # 檢查過多的參數
        if [[ "$line" =~ ^[[:space:]]*def[[:space:]] ]]; then
            local param_count=$(echo "$line" | grep -o ',' | wc -l)
            param_count=$((param_count + 1))
            if [ "$param_count" -gt 5 ]; then
                issues_ref+=("complexity:warning:custom:$file:$line_num:函數參數過多 ($param_count)")
            fi
        fi
        
    done < "$file"
}

format_analysis_results() {
    local issues=("$@")
    local output_format="$1"; shift
    local issues=("$@")
    
    if [ ${#issues[@]} -eq 0 ]; then
        if [ "$output_format" = "json" ]; then
            echo '{"issues": [], "summary": {"total": 0}}'
        else
            log_success "沒有發現問題！"
        fi
        return
    fi
    
    case "$output_format" in
        "json")
            format_json_output "${issues[@]}"
            ;;
        "html")
            format_html_output "${issues[@]}"
            ;;
        "csv")
            format_csv_output "${issues[@]}"
            ;;
        *)
            format_console_output "${issues[@]}"
            ;;
    esac
}

format_console_output() {
    local issues=("$@")
    
    # 檢查是否有問題要處理
    if [ ${#issues[@]} -eq 0 ]; then
        log_success "沒有發現問題！"
        return 0
    fi
    
    # 按嚴重程度分組
    declare -A severity_counts
    declare -A severity_issues
    
    for issue in "${issues[@]}"; do
        if [ -n "$issue" ]; then
            local severity=$(echo "$issue" | cut -d':' -f2)
            if [ -n "$severity" ]; then
                severity_counts[$severity]=$((${severity_counts[$severity]:-0} + 1))
                severity_issues[$severity]+="$issue"$'\n'
            fi
        fi
    done
    
    echo -e "${CYAN}代碼分析結果摘要${NC}"
    echo "========================="
    echo ""
    
    # 顯示統計
    local total_issues=${#issues[@]}
    echo -e "總問題數: ${BLUE}$total_issues${NC}"
    echo ""
    
    for severity in critical error warning info style; do
        local count=${severity_counts[$severity]:-0}
        if [ $count -gt 0 ]; then
            local color="$BLUE"
            case $severity in
                critical) color="$RED" ;;
                error) color="$RED" ;;
                warning) color="$YELLOW" ;;
                info) color="$BLUE" ;;
                style) color="$PURPLE" ;;
            esac
            
            echo -e "${color}${SEVERITY_LEVELS[$severity]}: $count${NC}"
        fi
    done
    
    echo ""
    echo -e "${CYAN}詳細問題列表${NC}"
    echo "==================="
    echo ""
    
    # 按嚴重程度顯示問題
    for severity in critical error warning info style; do
        local issues_text="${severity_issues[$severity]}"
        if [ -n "$issues_text" ]; then
            echo -e "${PURPLE}${SEVERITY_LEVELS[$severity]} 級別問題:${NC}"
            echo ""
            
            while IFS= read -r issue; do
                if [ -n "$issue" ]; then
                    local file_info=$(echo "$issue" | cut -d':' -f4-)
                    local tool=$(echo "$issue" | cut -d':' -f3)
                    echo -e "  📁 $file_info ${BLUE}[$tool]${NC}"
                fi
            done <<< "$issues_text"
            echo ""
        fi
    done
}

format_json_output() {
    local issues=("$@")
    
    echo "{"
    echo '  "timestamp": "'$(date -Iseconds)'",'
    echo '  "total_issues": '${#issues[@]}','
    echo '  "issues": ['
    
    local first=true
    for issue in "${issues[@]}"; do
        if [ "$first" = true ]; then
            first=false
        else
            echo ","
        fi
        
        local type=$(echo "$issue" | cut -d':' -f1)
        local severity=$(echo "$issue" | cut -d':' -f2)
        local tool=$(echo "$issue" | cut -d':' -f3)
        local description=$(echo "$issue" | cut -d':' -f4- | sed 's/"/\\"/g')
        
        echo -n '    {'
        echo -n '"type": "'$type'", '
        echo -n '"severity": "'$severity'", '
        echo -n '"tool": "'$tool'", '
        echo -n '"description": "'$description'"'
        echo -n '}'
    done
    
    echo ""
    echo "  ],"
    
    # 統計摘要
    declare -A counts
    for issue in "${issues[@]}"; do
        local severity=$(echo "$issue" | cut -d':' -f2)
        counts[$severity]=$((${counts[$severity]:-0} + 1))
    done
    
    echo '  "summary": {'
    echo '    "critical": '${counts[critical]:-0}','
    echo '    "error": '${counts[error]:-0}','
    echo '    "warning": '${counts[warning]:-0}','
    echo '    "info": '${counts[info]:-0}','
    echo '    "style": '${counts[style]:-0}
    echo '  }'
    echo "}"
}

generate_analysis_report() {
    local workspaces=("$@")
    local report_dir="$1"; shift
    local workspaces=("$@")
    
    mkdir -p "$report_dir"
    
    local report_file="$report_dir/code_analysis_report_$(date +%Y%m%d_%H%M%S).html"
    
    log_info "生成分析報告..."
    
    {
        echo "<!DOCTYPE html>"
        echo "<html><head><title>RosAGV 代碼分析報告</title>"
        echo "<style>"
        echo "body { font-family: Arial, sans-serif; margin: 20px; }"
        echo ".header { background: #f0f0f0; padding: 10px; border-radius: 5px; }"
        echo ".critical { color: #d32f2f; }"
        echo ".error { color: #f57c00; }"
        echo ".warning { color: #fbc02d; }"
        echo ".info { color: #1976d2; }"
        echo ".style { color: #7b1fa2; }"
        echo "table { border-collapse: collapse; width: 100%; }"
        echo "th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }"
        echo "th { background-color: #f2f2f2; }"
        echo "</style>"
        echo "</head><body>"
        
        echo "<div class='header'>"
        echo "<h1>RosAGV 代碼分析報告</h1>"
        echo "<p>生成時間: $(date)</p>"
        echo "<p>分析工作空間: ${workspaces[*]}</p>"
        echo "</div>"
        
        echo "<h2>分析摘要</h2>"
        echo "<p>此報告包含代碼品質、風格、安全性和 ROS 2 最佳實踐的分析結果。</p>"
        
        echo "<h2>建議改善</h2>"
        echo "<ul>"
        echo "<li>優先處理 critical 和 error 級別的問題</li>"
        echo "<li>定期執行代碼分析確保品質</li>"
        echo "<li>使用自動格式化工具統一代碼風格</li>"
        echo "<li>加強安全性檢查和測試覆蓋率</li>"
        echo "</ul>"
        
        echo "</body></html>"
        
    } > "$report_file"
    
    log_success "分析報告已生成: $report_file"
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
    if ! check_dependencies "$@"; then
        exit 1
    fi
    
    # 解析參數
    local analysis_type="style"
    local target_workspaces=()
    local target_files=()
    local exclude_pattern=""
    local include_pattern="*.py"
    local min_severity="info"
    local max_line_length="$DEFAULT_MAX_LINE_LENGTH"
    local max_complexity="$DEFAULT_COMPLEXITY_THRESHOLD"
    local output_format="console"
    local report_dir="$REPORT_DIR"
    local auto_fix="false"
    local config_file=""
    local verbose="false"
    local dry_run="false"
    
    # 檢查第一個參數是否為分析類型
    if [[ "$1" =~ ^(style|quality|security|ros2|complexity|imports|documentation|performance)$ ]]; then
        analysis_type="$1"
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
            --file)
                target_files+=("$2")
                shift 2
                ;;
            --files)
                IFS=',' read -ra file_list <<< "$2"
                target_files+=("${file_list[@]}")
                shift 2
                ;;
            --exclude)
                exclude_pattern="$2"
                shift 2
                ;;
            --include)
                include_pattern="$2"
                shift 2
                ;;
            --severity)
                min_severity="$2"
                shift 2
                ;;
            --max-line-length)
                max_line_length="$2"
                shift 2
                ;;
            --complexity-max)
                max_complexity="$2"
                shift 2
                ;;
            --output)
                output_format="$2"
                shift 2
                ;;
            --report-dir)
                report_dir="$2"
                shift 2
                ;;
            --fix)
                auto_fix="true"
                shift
                ;;
            --config)
                config_file="$2"
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
    
    # 如果沒有指定工作空間或檔案，自動發現工作空間
    if [ ${#target_workspaces[@]} -eq 0 ] && [ ${#target_files[@]} -eq 0 ]; then
        target_workspaces=($(discover_workspaces))
    fi
    
    # 收集要分析的檔案
    local all_files=()
    
    # 從工作空間收集檔案
    for workspace in "${target_workspaces[@]}"; do
        local workspace_path="$APP_DIR/$workspace"
        if [ -d "$workspace_path" ]; then
            local workspace_files=($(find_python_files "$workspace_path"))
            all_files+=("${workspace_files[@]}")
        else
            log_warning "工作空間不存在: $workspace"
        fi
    done
    
    # 添加指定的檔案
    for file in "${target_files[@]}"; do
        if [ -f "$file" ]; then
            all_files+=("$file")
        else
            log_warning "檔案不存在: $file"
        fi
    done
    
    if [ ${#all_files[@]} -eq 0 ]; then
        log_error "沒有找到要分析的檔案"
        exit 1
    fi
    
    # 創建報告目錄
    mkdir -p "$report_dir"
    
    # 顯示標題
    show_header
    log_info "分析類型: ${ANALYSIS_TYPES[$analysis_type]}"
    log_info "檔案數量: ${#all_files[@]}"
    log_info "最小嚴重程度: $min_severity"
    echo ""
    
    # Dry run 模式
    if [ "$dry_run" = "true" ]; then
        log_info "Dry run 模式 - 顯示要分析的檔案："
        for file in "${all_files[@]}"; do
            echo "  - $file"
        done
        exit 0
    fi
    
    # 執行對應的分析
    local analysis_results=()
    
    case $analysis_type in
        style)
            analysis_results=($(analyze_code_style "$max_line_length" "${all_files[@]}"))
            ;;
        quality)
            analysis_results=($(analyze_code_quality "${all_files[@]}"))
            ;;
        security)
            analysis_results=($(analyze_security "${all_files[@]}"))
            ;;
        ros2)
            analysis_results=($(analyze_ros2_practices "${all_files[@]}"))
            ;;
        complexity)
            analysis_results=($(analyze_complexity "$max_complexity" "${all_files[@]}"))
            ;;
        *)
            log_error "分析類型 '$analysis_type' 尚未實現"
            exit 1
            ;;
    esac
    
    # 格式化和輸出結果
    format_analysis_results "$output_format" "${analysis_results[@]}"
    
    # 生成報告
    if [ "$output_format" = "html" ]; then
        generate_analysis_report "$report_dir" "${target_workspaces[@]}"
    fi
    
    # 返回結果
    if [ ${#analysis_results[@]} -eq 0 ]; then
        log_success "代碼分析完成，未發現問題！"
        exit 0
    else
        log_warning "代碼分析完成，發現 ${#analysis_results[@]} 個問題"
        exit 1
    fi
}

# 如果直接執行此腳本
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi