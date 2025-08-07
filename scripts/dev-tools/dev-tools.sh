#!/bin/bash
# RosAGV 統一開發工具集
# 版本: 1.0
# 說明: 整合所有開發工具的統一介面，提供便捷的開發工作流管理

# 不使用 set -e，改用手動錯誤處理，避免關閉終端

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

# 工具定義
declare -A DEV_TOOLS=(
    ["build-helper"]="智能建置輔助工具"
    ["test-runner"]="測試執行和報告工具"
    ["code-analyzer"]="代碼分析和檢查工具"
    ["deploy-helper"]="部署輔助工具"
)

# 工作流定義
declare -A WORKFLOWS=(
    ["dev-setup"]="開發環境設置"
    ["code-check"]="代碼品質檢查"
    ["build-test"]="建置和測試"
    ["deploy-dev"]="開發環境部署"
    ["deploy-prod"]="生產環境部署"
    ["full-ci"]="完整 CI/CD 流程"
)

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

log_workflow() {
    echo -e "${PURPLE}[WORKFLOW]${NC} $1"
}

show_header() {
    echo -e "${CYAN}🛠️  RosAGV 統一開發工具集${NC}"
    echo -e "${CYAN}=========================${NC}"
    echo ""
}

show_usage() {
    echo "使用方式:"
    echo "  $0 [command] [options]"
    echo ""
    echo "工具命令:"
    for tool in "${!DEV_TOOLS[@]}"; do
        echo "  $tool                    # ${DEV_TOOLS[$tool]}"
    done
    echo ""
    echo "工作流命令:"
    for workflow in "${!WORKFLOWS[@]}"; do
        echo "  $workflow                # ${WORKFLOWS[$workflow]}"
    done
    echo ""
    echo "管理命令:"
    echo "  list                     # 列出所有可用工具"
    echo "  status                   # 顯示開發環境狀態"
    echo "  setup                    # 初始化開發環境"
    echo "  doctor                   # 診斷開發環境問題"
    echo "  clean                    # 清理開發檔案"
    echo ""
    echo "選項:"
    echo "  --workspace <WS>         # 指定工作空間"
    echo "  --components <LIST>      # 指定組件，逗號分隔"
    echo "  --profile <PROFILE>      # 指定建置配置"
    echo "  --mode <MODE>            # 指定模式 (development, production)"
    echo "  --parallel <N>           # 並行執行數量"
    echo "  --verbose                # 顯示詳細輸出"
    echo "  --dry-run                # 只檢查，不執行"
    echo "  -h, --help              # 顯示此幫助資訊"
    echo ""
    echo "範例:"
    echo "  $0 build-helper fast --workspace agv_ws    # 快速建置 AGV 工作空間"
    echo "  $0 code-check                              # 執行代碼品質檢查工作流"
    echo "  $0 full-ci --mode production               # 執行完整 CI/CD 流程"
    echo "  $0 status                                  # 檢查開發環境狀態"
    echo "  $0 doctor                                  # 診斷環境問題"
}

# ============================================================================
# 工具檢查和載入函數
# ============================================================================

check_dev_tools() {
    local missing_tools=()
    local available_tools=()
    
    for tool in "${!DEV_TOOLS[@]}"; do
        local tool_path="$SCRIPT_DIR/${tool}.sh"
        if [ -f "$tool_path" ] && [ -x "$tool_path" ]; then
            available_tools+=("$tool")
        else
            missing_tools+=("$tool")
        fi
    done
    
    if [ ${#missing_tools[@]} -gt 0 ]; then
        log_warning "缺少工具: ${missing_tools[*]}"
        return 1
    fi
    
    return 0
}

load_dev_tool_functions() {
    # 載入所有工具的函數 (如果它們支援 source)
    for tool in "${!DEV_TOOLS[@]}"; do
        local tool_path="$SCRIPT_DIR/${tool}.sh"
        if [ -f "$tool_path" ]; then
            # 嘗試 source 工具腳本來獲得函數
            source "$tool_path" 2>/dev/null || true
        fi
    done
}

# ============================================================================
# 工作流函數
# ============================================================================

workflow_dev_setup() {
    log_workflow "執行開發環境設置工作流..."
    
    local steps=(
        "檢查系統依賴"
        "驗證 Docker 環境"
        "檢查配置檔案"
        "建置基礎工作空間"
        "啟動基礎服務"
    )
    
    echo ""
    log_info "工作流步驟："
    for i in "${!steps[@]}"; do
        echo -e "  $((i+1)). ${steps[i]}"
    done
    echo ""
    
    # 1. 檢查系統依賴
    log_info "步驟 1/5: 檢查系統依賴"
    if command -v docker &> /dev/null && command -v docker-compose &> /dev/null; then
        log_success "Docker 環境正常"
    else
        log_error "Docker 環境缺失"
        return 1
    fi
    
    # 2. 驗證 Docker 環境
    log_info "步驟 2/5: 驗證 Docker 環境"
    if docker ps &> /dev/null; then
        log_success "Docker 服務運行正常"
    else
        log_error "Docker 服務未運行"
        return 1
    fi
    
    # 3. 檢查配置檔案
    log_info "步驟 3/5: 檢查配置檔案"
    "$SCRIPT_DIR/deploy-helper.sh" validate
    
    # 4. 建置基礎工作空間
    log_info "步驟 4/5: 建置基礎工作空間"
    "$SCRIPT_DIR/build-helper.sh" fast --workspaces keyence_plc_ws,plc_proxy_ws,path_algorithm
    
    # 5. 啟動基礎服務
    log_info "步驟 5/5: 啟動基礎服務"
    "$SCRIPT_DIR/deploy-helper.sh" deploy docker --components zenoh,database
    
    log_success "開發環境設置完成！"
}

workflow_code_check() {
    log_workflow "執行代碼品質檢查工作流..."
    
    local analysis_types=("style" "quality" "security" "ros2")
    local overall_result=0
    
    echo ""
    log_info "將執行以下分析："
    for type in "${analysis_types[@]}"; do
        echo -e "  • $type"
    done
    echo ""
    
    for type in "${analysis_types[@]}"; do
        log_info "執行 $type 分析..."
        
        if "$SCRIPT_DIR/code-analyzer.sh" "$type" --severity warning; then
            log_success "$type 分析通過"
        else
            log_warning "$type 分析發現問題"
            overall_result=1
        fi
        echo ""
    done
    
    if [ $overall_result -eq 0 ]; then
        log_success "代碼品質檢查全部通過！"
    else
        log_warning "代碼品質檢查發現問題，請檢查詳細報告"
    fi
    
    return $overall_result
}

workflow_build_test() {
    log_workflow "執行建置和測試工作流..."
    
    local components=("$@")
    
    # 如果沒有指定組件，使用全部
    if [ ${#components[@]} -eq 0 ]; then
        components=("all")
    fi
    
    echo ""
    log_info "建置和測試組件: ${components[*]}"
    echo ""
    
    # 1. 清理舊的建置
    log_info "步驟 1/4: 清理舊的建置檔案"
    "$SCRIPT_DIR/build-helper.sh" clean --workspaces "${components[@]}"
    
    # 2. 執行建置
    log_info "步驟 2/4: 執行建置"
    if "$SCRIPT_DIR/build-helper.sh" full --workspaces "${components[@]}"; then
        log_success "建置成功"
    else
        log_error "建置失敗"
        return 1
    fi
    
    # 3. 執行測試
    log_info "步驟 3/4: 執行測試"
    if "$SCRIPT_DIR/test-runner.sh" unit --workspaces "${components[@]}"; then
        log_success "單元測試通過"
    else
        log_warning "單元測試有失敗項目"
    fi
    
    # 4. 生成報告
    log_info "步驟 4/4: 生成測試報告"
    "$SCRIPT_DIR/test-runner.sh" report --output html
    
    log_success "建置和測試工作流完成！"
}

workflow_deploy_dev() {
    log_workflow "執行開發環境部署工作流..."
    
    # 1. 預檢查
    log_info "步驟 1/4: 執行部署前檢查"
    "$SCRIPT_DIR/deploy-helper.sh" pre-check
    
    # 2. 備份當前配置
    log_info "步驟 2/4: 備份當前配置"
    local backup_id=$("$SCRIPT_DIR/deploy-helper.sh" backup)
    log_info "備份 ID: $backup_id"
    
    # 3. 執行部署
    log_info "步驟 3/4: 執行開發環境部署"
    if "$SCRIPT_DIR/deploy-helper.sh" deploy development; then
        log_success "部署成功"
    else
        log_error "部署失敗，嘗試回滾..."
        "$SCRIPT_DIR/deploy-helper.sh" restore "$backup_id"
        return 1
    fi
    
    # 4. 驗證部署
    log_info "步驟 4/4: 驗證部署結果"
    "$SCRIPT_DIR/deploy-helper.sh" status
    
    log_success "開發環境部署工作流完成！"
}

workflow_deploy_prod() {
    log_workflow "執行生產環境部署工作流..."
    
    log_warning "生產環境部署需要額外確認"
    read -p "確定要部署到生產環境嗎? (yes/no): " confirm
    
    if [ "$confirm" != "yes" ]; then
        log_info "部署已取消"
        return 0
    fi
    
    # 1. 完整檢查
    log_info "步驟 1/6: 執行完整檢查"
    if ! workflow_code_check; then
        log_error "代碼檢查未通過，無法部署到生產環境"
        return 1
    fi
    
    # 2. 完整測試
    log_info "步驟 2/6: 執行完整測試"
    if ! "$SCRIPT_DIR/test-runner.sh" all --coverage; then
        log_error "測試未通過，無法部署到生產環境"
        return 1
    fi
    
    # 3. 建置生產版本
    log_info "步驟 3/6: 建置生產版本"
    "$SCRIPT_DIR/build-helper.sh" release
    
    # 4. 備份當前配置
    log_info "步驟 4/6: 備份當前配置"
    local backup_id=$("$SCRIPT_DIR/deploy-helper.sh" backup)
    
    # 5. 執行生產部署
    log_info "步驟 5/6: 執行生產環境部署"
    if "$SCRIPT_DIR/deploy-helper.sh" deploy production; then
        log_success "生產部署成功"
    else
        log_error "生產部署失敗，執行回滾..."
        "$SCRIPT_DIR/deploy-helper.sh" restore "$backup_id"
        return 1
    fi
    
    # 6. 驗證部署
    log_info "步驟 6/6: 驗證生產部署"
    "$SCRIPT_DIR/deploy-helper.sh" status
    
    log_success "生產環境部署工作流完成！"
}

workflow_full_ci() {
    log_workflow "執行完整 CI/CD 流程..."
    
    local mode="${1:-development}"
    
    echo ""
    log_info "CI/CD 流程模式: $mode"
    echo ""
    
    local steps=(
        "代碼品質檢查"
        "建置和測試"
        "部署到 $mode 環境"
        "部署驗證"
        "清理作業"
    )
    
    log_info "CI/CD 流程步驟："
    for i in "${!steps[@]}"; do
        echo -e "  $((i+1)). ${steps[i]}"
    done
    echo ""
    
    local overall_result=0
    
    # 1. 代碼品質檢查
    log_info "步驟 1/5: 代碼品質檢查"
    if ! workflow_code_check; then
        if [ "$mode" = "production" ]; then
            log_error "生產環境部署要求代碼品質檢查通過"
            return 1
        else
            log_warning "代碼品質檢查有問題，但繼續開發環境流程"
            overall_result=1
        fi
    fi
    
    # 2. 建置和測試
    log_info "步驟 2/5: 建置和測試"
    if ! workflow_build_test; then
        log_error "建置和測試失敗"
        return 1
    fi
    
    # 3. 部署
    log_info "步驟 3/5: 部署到 $mode 環境"
    case "$mode" in
        "development"|"dev")
            workflow_deploy_dev
            ;;
        "production"|"prod")
            workflow_deploy_prod
            ;;
        *)
            log_error "不支援的部署模式: $mode"
            return 1
            ;;
    esac
    
    # 4. 部署驗證
    log_info "步驟 4/5: 部署驗證"
    "$SCRIPT_DIR/deploy-helper.sh" status
    
    # 5. 清理作業
    log_info "步驟 5/5: 清理作業"
    "$SCRIPT_DIR/deploy-helper.sh" cleanup
    
    if [ $overall_result -eq 0 ]; then
        log_success "完整 CI/CD 流程成功完成！"
    else
        log_warning "CI/CD 流程完成，但存在一些問題"
    fi
    
    return $overall_result
}

# ============================================================================
# 管理命令函數
# ============================================================================

cmd_list() {
    show_header
    
    echo -e "${CYAN}可用的開發工具${NC}"
    echo "=================="
    echo ""
    
    for tool in "${!DEV_TOOLS[@]}"; do
        local tool_path="$SCRIPT_DIR/${tool}.sh"
        if [ -f "$tool_path" ] && [ -x "$tool_path" ]; then
            echo -e "  ${GREEN}✓${NC} $tool - ${DEV_TOOLS[$tool]}"
        else
            echo -e "  ${RED}✗${NC} $tool - ${DEV_TOOLS[$tool]} (缺失)"
        fi
    done
    
    echo ""
    echo -e "${CYAN}可用的工作流${NC}"
    echo "================"
    echo ""
    
    for workflow in "${!WORKFLOWS[@]}"; do
        echo -e "  ${BLUE}→${NC} $workflow - ${WORKFLOWS[$workflow]}"
    done
    
    echo ""
}

cmd_status() {
    show_header
    
    echo -e "${CYAN}開發環境狀態${NC}"
    echo "================"
    echo ""
    
    # 檢查工具狀態
    log_info "檢查開發工具..."
    local missing_tools=0
    for tool in "${!DEV_TOOLS[@]}"; do
        local tool_path="$SCRIPT_DIR/${tool}.sh"
        if [ -f "$tool_path" ] && [ -x "$tool_path" ]; then
            echo -e "  ${GREEN}✓${NC} $tool"
        else
            echo -e "  ${RED}✗${NC} $tool"
            missing_tools=$((missing_tools + 1))
        fi
    done
    
    echo ""
    
    # 檢查 Docker 狀態
    log_info "檢查 Docker 環境..."
    if command -v docker &> /dev/null; then
        echo -e "  ${GREEN}✓${NC} Docker 已安裝"
        
        if docker ps &> /dev/null; then
            echo -e "  ${GREEN}✓${NC} Docker 服務運行中"
            
            # 檢查容器狀態
            local containers=("agvc_server" "nginx" "postgres_container" "rosagv")
            for container in "${containers[@]}"; do
                if docker ps --format "{{.Names}}" | grep -q "^$container$"; then
                    echo -e "  ${GREEN}✓${NC} 容器 $container 運行中"
                else
                    echo -e "  ${YELLOW}○${NC} 容器 $container 未運行"
                fi
            done
        else
            echo -e "  ${RED}✗${NC} Docker 服務未運行"
        fi
    else
        echo -e "  ${RED}✗${NC} Docker 未安裝"
    fi
    
    echo ""
    
    # 檢查工作空間狀態
    log_info "檢查工作空間狀態..."
    local workspaces=(
        "agv_ws" "agv_cmd_service_ws" "joystick_ws" "keyence_plc_ws"
        "plc_proxy_ws" "db_proxy_ws" "ecs_ws" "rcs_ws" "ai_wcs_ws"
        "web_api_ws" "kuka_fleet_ws" "sensorpart_ws" "launch_ws"
    )
    
    local built_workspaces=0
    local total_workspaces=${#workspaces[@]}
    
    for workspace in "${workspaces[@]}"; do
        local workspace_path="$PROJECT_ROOT/app/$workspace"
        if [ -d "$workspace_path/install" ] && [ -d "$workspace_path/build" ]; then
            built_workspaces=$((built_workspaces + 1))
        fi
    done
    
    echo -e "  工作空間建置狀態: $built_workspaces/$total_workspaces"
    
    echo ""
    
    # 檢查配置檔案
    log_info "檢查配置檔案..."
    local config_files=(
        "$PROJECT_ROOT/app/config/hardware_mapping.yaml"
        "$PROJECT_ROOT/app/routerconfig.json5"
        "$PROJECT_ROOT/docker-compose.yml"
        "$PROJECT_ROOT/docker-compose.agvc.yml"
    )
    
    local missing_configs=0
    for config_file in "${config_files[@]}"; do
        if [ -f "$config_file" ]; then
            echo -e "  ${GREEN}✓${NC} $(basename "$config_file")"
        else
            echo -e "  ${RED}✗${NC} $(basename "$config_file")"
            missing_configs=$((missing_configs + 1))
        fi
    done
    
    echo ""
    
    # 狀態摘要
    log_info "環境狀態摘要："
    if [ $missing_tools -eq 0 ] && [ $missing_configs -eq 0 ]; then
        echo -e "  ${GREEN}✓${NC} 開發環境就緒"
    else
        echo -e "  ${YELLOW}⚠${NC} 開發環境需要設置 (缺失 $missing_tools 個工具, $missing_configs 個配置)"
    fi
}

cmd_setup() {
    show_header
    
    log_info "初始化開發環境..."
    
    # 執行開發環境設置工作流
    workflow_dev_setup
}

cmd_doctor() {
    show_header
    
    log_info "診斷開發環境問題..."
    
    local issues=()
    local suggestions=()
    
    # 檢查基本工具
    for tool in docker docker-compose git colcon python3; do
        if ! command -v "$tool" &> /dev/null; then
            issues+=("缺少必要工具: $tool")
            case "$tool" in
                docker|docker-compose)
                    suggestions+=("安裝 Docker: https://docs.docker.com/get-docker/")
                    ;;
                git)
                    suggestions+=("安裝 Git: sudo apt-get install git")
                    ;;
                colcon)
                    suggestions+=("安裝 colcon: pip3 install colcon-common-extensions")
                    ;;
                python3)
                    suggestions+=("安裝 Python3: sudo apt-get install python3 python3-pip")
                    ;;
            esac
        fi
    done
    
    # 檢查 Docker 服務
    if command -v docker &> /dev/null && ! docker ps &> /dev/null; then
        issues+=("Docker 服務未運行")
        suggestions+=("啟動 Docker 服務: sudo systemctl start docker")
    fi
    
    # 檢查磁碟空間
    local available_space=$(df / | awk 'NR==2 {print $4}')
    local required_space=2097152  # 2GB in KB
    
    if [ "$available_space" -lt "$required_space" ]; then
        issues+=("磁碟空間不足 (需要至少 2GB)")
        suggestions+=("清理磁碟空間或擴展儲存")
    fi
    
    # 檢查網路連接
    if ! ping -c 1 8.8.8.8 &> /dev/null; then
        issues+=("網路連接問題")
        suggestions+=("檢查網路連接和防火牆設定")
    fi
    
    # 檢查端口衝突
    local required_ports=(7447 8000 8001 8002 5432 80)
    for port in "${required_ports[@]}"; do
        if ss -tuln 2>/dev/null | grep -q ":$port "; then
            issues+=("端口 $port 被佔用")
            suggestions+=("停止佔用端口 $port 的服務或更改配置")
        fi
    done
    
    echo ""
    
    # 顯示診斷結果
    if [ ${#issues[@]} -eq 0 ]; then
        log_success "沒有發現問題，開發環境正常！"
    else
        echo -e "${YELLOW}發現的問題:${NC}"
        for issue in "${issues[@]}"; do
            echo -e "  ${RED}✗${NC} $issue"
        done
        
        echo ""
        echo -e "${CYAN}建議的解決方案:${NC}"
        for suggestion in "${suggestions[@]}"; do
            echo -e "  ${BLUE}→${NC} $suggestion"
        done
    fi
    
    echo ""
}

cmd_clean() {
    show_header
    
    log_info "清理開發檔案..."
    
    read -p "確定要清理所有建置檔案嗎? (yes/no): " confirm
    
    if [ "$confirm" = "yes" ]; then
        # 清理建置檔案
        "$SCRIPT_DIR/build-helper.sh" clean
        
        # 清理部署檔案
        "$SCRIPT_DIR/deploy-helper.sh" cleanup
        
        # 清理 Docker 資源
        log_info "清理 Docker 資源..."
        docker system prune -f
        
        log_success "清理完成！"
    else
        log_info "清理已取消"
    fi
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
    
    # 檢查開發工具是否可用
    if ! check_dev_tools; then
        log_error "部分開發工具缺失，請檢查安裝"
        exit 1
    fi
    
    # 載入工具函數
    load_dev_tool_functions
    
    # 解析參數
    local command=""
    local options=()
    
    if [ $# -eq 0 ]; then
        show_header
        show_usage
        exit 0
    fi
    
    # 解析命令
    command="$1"
    shift
    
    # 收集剩餘參數
    options=("$@")
    
    # 執行對應命令
    case "$command" in
        # 直接工具命令
        "build-helper"|"test-runner"|"code-analyzer"|"deploy-helper")
            if [ -f "$SCRIPT_DIR/${command}.sh" ]; then
                exec "$SCRIPT_DIR/${command}.sh" "${options[@]}"
            else
                log_error "工具不存在: $command"
                exit 1
            fi
            ;;
            
        # 工作流命令
        "dev-setup")
            workflow_dev_setup "${options[@]}"
            ;;
        "code-check")
            workflow_code_check "${options[@]}"
            ;;
        "build-test")
            workflow_build_test "${options[@]}"
            ;;
        "deploy-dev")
            workflow_deploy_dev "${options[@]}"
            ;;
        "deploy-prod")
            workflow_deploy_prod "${options[@]}"
            ;;
        "full-ci")
            local mode="development"
            # 從選項中提取模式
            for ((i=0; i<${#options[@]}; i++)); do
                if [[ "${options[i]}" == "--mode" ]] && [ $((i+1)) -lt ${#options[@]} ]; then
                    mode="${options[$((i+1))]}"
                    break
                fi
            done
            workflow_full_ci "$mode"
            ;;
            
        # 管理命令
        "list")
            cmd_list
            ;;
        "status")
            cmd_status
            ;;
        "setup")
            cmd_setup
            ;;
        "doctor")
            cmd_doctor
            ;;
        "clean")
            cmd_clean
            ;;
            
        # 幫助
        "-h"|"--help"|"help")
            show_header
            show_usage
            ;;
            
        *)
            log_error "未知命令: $command"
            echo ""
            show_usage
            return 1 2>/dev/null || exit 1
            ;;
    esac
}

# ============================================================================
# 便捷函數 (供 source 使用)
# ============================================================================

# 快速建置
dev_build() {
    "$SCRIPT_DIR/build-helper.sh" fast "$@"
}

# 快速測試
dev_test() {
    "$SCRIPT_DIR/test-runner.sh" unit "$@"
}

# 快速代碼檢查
dev_check() {
    "$SCRIPT_DIR/code-analyzer.sh" style "$@"
}

# 快速部署
dev_deploy() {
    "$SCRIPT_DIR/deploy-helper.sh" deploy development "$@"
}

# 顯示狀態
dev_status() {
    cmd_status
}

# 工具說明
show_dev_tools_help() {
    echo -e "${CYAN}RosAGV 開發工具集便捷函數${NC}"
    echo "=========================="
    echo ""
    echo "可用的便捷函數："
    echo "  dev_build [options]      # 快速建置"
    echo "  dev_test [options]       # 快速測試"
    echo "  dev_check [options]      # 快速代碼檢查"
    echo "  dev_deploy [options]     # 快速部署"
    echo "  dev_status              # 顯示狀態"
    echo ""
    echo "使用方式："
    echo "  source scripts/dev-tools/dev-tools.sh"
    echo "  dev_build --workspace agv_ws"
    echo "  dev_test --type unit"
    echo "  dev_check --severity warning"
    echo ""
}

# 如果直接執行此腳本
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
else
    # 被 source 時顯示載入訊息並導出函數
    
    # 導出便捷函數
    export -f dev_build
    export -f dev_test
    export -f dev_check
    export -f dev_deploy
    export -f dev_status
    export -f show_dev_tools_help
    
    # 導出工具函數
    export -f log_info
    export -f log_success
    export -f log_warning
    export -f log_error
    
    # 導出管理命令函數
    export -f cmd_list
    export -f cmd_status
    export -f cmd_doctor
    
    echo -e "\033[0;32m✅ RosAGV 開發工具集已載入\033[0m"
    echo -e "輸入 \033[0;36mshow_dev_tools_help\033[0m 查看可用便捷函數"
fi
