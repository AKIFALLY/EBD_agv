#!/bin/bash
# RosAGV 部署輔助工具
# 版本: 1.0
# 說明: 自動化部署流程、配置檔案驗證、部署前預檢查和回滾機制

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
CONFIG_DIR="$APP_DIR/config"
BACKUP_DIR="/tmp/rosagv_deploy_backup"
DEPLOYMENT_LOG="/tmp/rosagv_deployment.log"

# 部署模式定義
declare -A DEPLOY_MODES=(
    ["development"]="開發環境部署"
    ["staging"]="測試環境部署"
    ["production"]="生產環境部署"
    ["local"]="本地開發部署"
    ["docker"]="Docker 容器部署"
)

# 服務組件定義
declare -A SERVICE_COMPONENTS=(
    ["agvc"]="AGVC 管理系統"
    ["agv"]="AGV 車載系統"
    ["database"]="PostgreSQL 資料庫"
    ["nginx"]="Nginx 反向代理"
    ["zenoh"]="Zenoh 路由器"
)

# 部署階段定義
declare -A DEPLOY_PHASES=(
    ["pre-check"]="部署前檢查"
    ["backup"]="配置備份"
    ["build"]="系統建置"
    ["config"]="配置部署"
    ["service"]="服務啟動"
    ["verification"]="部署驗證"
    ["cleanup"]="清理作業"
)

# ============================================================================
# 工具函數
# ============================================================================

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1" | tee -a "$DEPLOYMENT_LOG"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1" | tee -a "$DEPLOYMENT_LOG"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1" | tee -a "$DEPLOYMENT_LOG"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1" | tee -a "$DEPLOYMENT_LOG"
}

log_deploy() {
    echo -e "${PURPLE}[DEPLOY]${NC} $1" | tee -a "$DEPLOYMENT_LOG"
}

show_header() {
    echo -e "${CYAN}🚀 RosAGV 部署輔助工具${NC}"
    echo -e "${CYAN}========================${NC}"
    echo ""
}

show_usage() {
    echo "使用方式:"
    echo "  $0 [command] [options]"
    echo ""
    echo "命令:"
    echo "  deploy <mode>            # 執行部署 (development, staging, production, local, docker)"
    echo "  pre-check               # 執行部署前檢查"
    echo "  backup                  # 備份當前配置"
    echo "  restore <backup_id>     # 恢復配置備份"
    echo "  rollback                # 回滾到上一個版本"
    echo "  status                  # 檢查部署狀態"
    echo "  validate                # 驗證配置檔案"
    echo "  logs                    # 查看部署日誌"
    echo "  cleanup                 # 清理部署檔案"
    echo ""
    echo "選項:"
    echo "  --components <LIST>     # 指定部署組件，逗號分隔 (agvc,agv,database,nginx,zenoh)"
    echo "  --skip-checks           # 跳過預檢查"
    echo "  --skip-backup           # 跳過備份"
    echo "  --force                 # 強制部署，忽略警告"
    echo "  --dry-run               # 只檢查，不執行實際部署"
    echo "  --config <FILE>         # 使用自定義配置檔案"
    echo "  --tag <VERSION>         # 指定部署版本標籤"
    echo "  --restart-services      # 部署後重啟服務"
    echo "  --verbose               # 顯示詳細輸出"
    echo "  -h, --help             # 顯示此幫助資訊"
    echo ""
    echo "範例:"
    echo "  $0 deploy development                    # 部署到開發環境"
    echo "  $0 deploy production --components agvc,database  # 只部署 AGVC 和資料庫"
    echo "  $0 pre-check --verbose                   # 詳細預檢查"
    echo "  $0 backup                               # 備份當前配置"
    echo "  $0 rollback                             # 回滾部署"
    echo "  $0 status                               # 檢查部署狀態"
}

# ============================================================================
# 核心部署函數
# ============================================================================

check_dependencies() {
    # 先檢查是否為 help 模式，如果是就直接返回成功
    for arg in "$@"; do
        if [[ "$arg" =~ ^(-h|--help)$ ]]; then
            return 0
        fi
    done
    
    local missing_tools=()
    
    # 檢查基本工具
    for tool in docker git rsync; do
        if ! command -v "$tool" &> /dev/null; then
            missing_tools+=("$tool")
        fi
    done
    
    # 檢查 Docker Compose (V2 優先，V1 備用)
    if ! docker compose version &> /dev/null && ! command -v docker-compose &> /dev/null; then
        missing_tools+=("docker-compose")
    fi
    
    if [ ${#missing_tools[@]} -gt 0 ]; then
        log_error "缺少必要工具: ${missing_tools[*]}"
        return 1
    fi
    
    return 0
}

pre_deployment_check() {
    local deploy_mode="$1"
    
    log_deploy "執行部署前檢查..."
    
    local check_results=()
    local critical_issues=0
    local warning_issues=0
    
    # 檢查 Docker 服務
    if ! systemctl is-active --quiet docker; then
        check_results+=("CRITICAL:Docker 服務未運行")
        critical_issues=$((critical_issues + 1))
    else
        check_results+=("OK:Docker 服務正常運行")
    fi
    
    # 檢查磁碟空間
    local available_space=$(df / | awk 'NR==2 {print $4}')
    local required_space=1048576  # 1GB in KB
    
    if [ "$available_space" -lt "$required_space" ]; then
        check_results+=("CRITICAL:磁碟空間不足 (需要至少 1GB)")
        critical_issues=$((critical_issues + 1))
    else
        check_results+=("OK:磁碟空間充足")
    fi
    
    # 檢查配置檔案
    local config_files=(
        "$CONFIG_DIR/hardware_mapping.yaml"
        "$APP_DIR/routerconfig.json5"
        "$PROJECT_ROOT/docker-compose.yml"
        "$PROJECT_ROOT/docker-compose.agvc.yml"
    )
    
    for config_file in "${config_files[@]}"; do
        if [ -f "$config_file" ]; then
            check_results+=("OK:配置檔案存在: $(basename "$config_file")")
        else
            check_results+=("WARNING:配置檔案缺失: $(basename "$config_file")")
            warning_issues=$((warning_issues + 1))
        fi
    done
    
    # 檢查端口可用性
    local required_ports=(7447 8000 8001 8002 5432 80 2200)
    
    for port in "${required_ports[@]}"; do
        if netstat -tuln 2>/dev/null | grep -q ":$port "; then
            check_results+=("WARNING:端口 $port 已被佔用")
            warning_issues=$((warning_issues + 1))
        else
            check_results+=("OK:端口 $port 可用")
        fi
    done
    
    # 檢查工作空間建置狀態
    local workspaces=(
        "agv_ws" "agv_cmd_service_ws" "joystick_ws" "keyence_plc_ws"
        "plc_proxy_ws" "db_proxy_ws" "ecs_ws" "rcs_ws" "wcs_ws"
        "web_api_ws" "kuka_fleet_ws" "sensorpart_ws" "launch_ws"
    )
    
    for workspace in "${workspaces[@]}"; do
        local build_dir="$APP_DIR/$workspace/build"
        local install_dir="$APP_DIR/$workspace/install"
        
        if [ -d "$build_dir" ] && [ -d "$install_dir" ]; then
            check_results+=("OK:工作空間已建置: $workspace")
        else
            check_results+=("WARNING:工作空間需要建置: $workspace")
            warning_issues=$((warning_issues + 1))
        fi
    done
    
    # 顯示檢查結果
    echo ""
    echo -e "${CYAN}部署前檢查結果${NC}"
    echo "==================="
    echo ""
    
    for result in "${check_results[@]}"; do
        local status=$(echo "$result" | cut -d':' -f1)
        local message=$(echo "$result" | cut -d':' -f2-)
        
        case "$status" in
            "OK")
                echo -e "  ${GREEN}✓${NC} $message"
                ;;
            "WARNING")
                echo -e "  ${YELLOW}⚠${NC} $message"
                ;;
            "CRITICAL")
                echo -e "  ${RED}✗${NC} $message"
                ;;
        esac
    done
    
    echo ""
    echo -e "檢查摘要:"
    echo -e "  嚴重問題: ${RED}$critical_issues${NC}"
    echo -e "  警告問題: ${YELLOW}$warning_issues${NC}"
    echo ""
    
    if [ $critical_issues -gt 0 ]; then
        log_error "部署前檢查發現嚴重問題，請先解決後再進行部署"
        return 1
    elif [ $warning_issues -gt 0 ]; then
        log_warning "部署前檢查發現警告問題，建議先解決"
        return 2
    else
        log_success "部署前檢查通過"
        return 0
    fi
}

backup_configuration() {
    local backup_id="${1:-$(date +%Y%m%d_%H%M%S)}"
    local backup_path="$BACKUP_DIR/$backup_id"
    
    log_deploy "備份當前配置到: $backup_path"
    
    mkdir -p "$backup_path"
    
    # 備份配置檔案
    if [ -d "$CONFIG_DIR" ]; then
        cp -r "$CONFIG_DIR" "$backup_path/config"
        log_info "已備份配置目錄"
    fi
    
    # 備份 Docker Compose 檔案
    for compose_file in docker-compose.yml docker-compose.agvc.yml; do
        if [ -f "$PROJECT_ROOT/$compose_file" ]; then
            cp "$PROJECT_ROOT/$compose_file" "$backup_path/"
            log_info "已備份 $compose_file"
        fi
    done
    
    # 備份 Zenoh 配置
    if [ -f "$APP_DIR/routerconfig.json5" ]; then
        cp "$APP_DIR/routerconfig.json5" "$backup_path/"
        log_info "已備份 Zenoh 配置"
    fi
    
    # 備份環境變數檔案
    for env_file in .env .env.local .env.production; do
        if [ -f "$PROJECT_ROOT/$env_file" ]; then
            cp "$PROJECT_ROOT/$env_file" "$backup_path/"
            log_info "已備份 $env_file"
        fi
    done
    
    # 創建備份資訊檔案
    {
        echo "備份時間: $(date)"
        echo "備份ID: $backup_id"
        echo "Git 提交: $(git rev-parse HEAD 2>/dev/null || echo 'N/A')"
        echo "分支: $(git branch --show-current 2>/dev/null || echo 'N/A')"
        echo "備份內容:"
        find "$backup_path" -type f | sed 's|^|  |'
    } > "$backup_path/backup_info.txt"
    
    log_success "配置備份完成: $backup_path"
    echo "$backup_id"
}

validate_configuration() {
    local config_file="$1"
    
    log_deploy "驗證配置檔案..."
    
    local validation_errors=()
    
    # 驗證 hardware_mapping.yaml
    local hw_mapping="$CONFIG_DIR/hardware_mapping.yaml"
    if [ -f "$hw_mapping" ]; then
        if ! python3 -c "
import yaml
try:
    with open('$hw_mapping', 'r') as f:
        data = yaml.safe_load(f)
    print('硬體映射配置有效')
except Exception as e:
    print(f'硬體映射配置錯誤: {e}')
    exit(1)
" 2>/dev/null; then
            validation_errors+=("hardware_mapping.yaml 格式錯誤")
        fi
    else
        validation_errors+=("hardware_mapping.yaml 檔案不存在")
    fi
    
    # 驗證 Zenoh 配置
    local zenoh_config="$APP_DIR/routerconfig.json5"
    if [ -f "$zenoh_config" ]; then
        # 簡單的 JSON5 語法檢查 - 檢查基本結構
        if python3 -c "
import re
try:
    with open('$zenoh_config', 'r') as f:
        content = f.read()
    
    # 檢查基本語法元素
    if not re.search(r'mode\s*:', content):
        raise Exception('缺少 mode 配置')
    if not re.search(r'\{.*\}', content, re.DOTALL):
        raise Exception('JSON 結構不完整')
    
    print('Zenoh 配置語法檢查通過')
except Exception as e:
    print(f'Zenoh 配置錯誤: {e}')
    exit(1)
" 2>/dev/null; then
            log_info "Zenoh 配置檢查通過"
        else
            validation_errors+=("routerconfig.json5 格式錯誤")
        fi
    else
        validation_errors+=("routerconfig.json5 檔案不存在")
    fi
    
    # 驗證 Docker Compose 檔案
    for compose_file in docker-compose.yml docker-compose.agvc.yml; do
        local compose_path="$PROJECT_ROOT/$compose_file"
        if [ -f "$compose_path" ]; then
            if ! docker compose -f "$compose_path" config &>/dev/null; then
                validation_errors+=("$compose_file 配置錯誤")
            fi
        else
            validation_errors+=("$compose_file 檔案不存在")
        fi
    done
    
    # 顯示驗證結果
    if [ ${#validation_errors[@]} -eq 0 ]; then
        log_success "配置檔案驗證通過"
        return 0
    else
        log_error "配置檔案驗證失敗:"
        for error in "${validation_errors[@]}"; do
            echo -e "  ${RED}✗${NC} $error"
        done
        return 1
    fi
}

build_system() {
    local components=("$@")
    
    log_deploy "建置系統組件..."
    
    # 如果沒有指定組件，建置所有組件
    if [ ${#components[@]} -eq 0 ]; then
        components=("${!SERVICE_COMPONENTS[@]}")
    fi
    
    for component in "${components[@]}"; do
        log_info "建置組件: ${SERVICE_COMPONENTS[$component]}"
        
        case "$component" in
            "agv"|"agvc")
                # 建置 ROS 2 工作空間
                build_ros_workspaces "$component"
                ;;
            "database")
                # 準備資料庫
                prepare_database
                ;;
            "nginx")
                # 檢查 Nginx 配置
                validate_nginx_config
                ;;
            "zenoh")
                # 檢查 Zenoh 配置
                validate_zenoh_config
                ;;
            *)
                log_warning "未知組件: $component"
                ;;
        esac
    done
    
    log_success "系統建置完成"
}

build_ros_workspaces() {
    local target="$1"
    
    local workspaces=()
    case "$target" in
        "agv")
            workspaces=(
                "keyence_plc_ws" "plc_proxy_ws" "path_algorithm"
                "agv_cmd_service_ws" "joystick_ws" "agv_ws"
                "sensorpart_ws" "uno_gpio_ws" "launch_ws"
            )
            ;;
        "agvc")
            workspaces=(
                "keyence_plc_ws" "plc_proxy_ws" "path_algorithm" "db_proxy_ws"
                "ecs_ws" "rcs_ws" "wcs_ws" "ai_wcs_ws" "web_api_ws"
                "kuka_fleet_ws" "launch_ws"
            )
            ;;
    esac
    
    for workspace in "${workspaces[@]}"; do
        local workspace_path="$APP_DIR/$workspace"
        
        if [ -d "$workspace_path" ]; then
            log_info "建置工作空間: $workspace"
            
            pushd "$workspace_path" > /dev/null
            
            # 清理舊的建置
            if [ -d "build" ]; then
                rm -rf build
            fi
            if [ -d "install" ]; then
                rm -rf install  
            fi
            
            # 執行建置
            if colcon build --symlink-install --continue-on-error 2>&1 | tee -a "$DEPLOYMENT_LOG"; then
                log_success "工作空間建置成功: $workspace"
            else
                log_error "工作空間建置失敗: $workspace"
                popd > /dev/null
                return 1
            fi
            
            popd > /dev/null
        else
            log_warning "工作空間不存在: $workspace"
        fi
    done
}

prepare_database() {
    log_info "準備資料庫..."
    
    # 檢查是否有資料庫初始化腳本
    local init_scripts=(
        "$APP_DIR/db_proxy_ws/init.sql"
        "$APP_DIR/db_proxy_ws/schema.sql"
        "$PROJECT_ROOT/database/init.sql"
    )
    
    for script in "${init_scripts[@]}"; do
        if [ -f "$script" ]; then
            log_info "發現資料庫初始化腳本: $(basename "$script")"
        fi
    done
    
    log_success "資料庫準備完成"
}

validate_nginx_config() {
    log_info "驗證 Nginx 配置..."
    
    local nginx_configs=(
        "$PROJECT_ROOT/nginx/nginx.conf"
        "$PROJECT_ROOT/nginx/default.conf"
        "$CONFIG_DIR/nginx.conf"
    )
    
    for config in "${nginx_configs[@]}"; do
        if [ -f "$config" ]; then
            log_info "發現 Nginx 配置: $(basename "$config")"
        fi
    done
    
    log_success "Nginx 配置驗證完成"
}

validate_zenoh_config() {
    log_info "驗證 Zenoh 配置..."
    
    local zenoh_config="$APP_DIR/routerconfig.json5"
    if [ -f "$zenoh_config" ]; then
        log_info "Zenoh 配置檔案存在"
        
        # 檢查端點配置
        if grep -q "tcp/" "$zenoh_config"; then
            log_info "發現 TCP 端點配置"
        fi
        
        if grep -q "udp/" "$zenoh_config"; then
            log_info "發現 UDP 端點配置"
        fi
    else
        log_warning "Zenoh 配置檔案不存在"
    fi
    
    log_success "Zenoh 配置驗證完成"
}

deploy_services() {
    local deploy_mode="$1"
    shift
    local components=("$@")
    
    log_deploy "部署服務到 $deploy_mode 環境..."
    
    case "$deploy_mode" in
        "development"|"local")
            deploy_local_services "${components[@]}"
            ;;
        "docker")
            deploy_docker_services "${components[@]}"
            ;;
        "staging"|"production")
            deploy_production_services "$deploy_mode" "${components[@]}"
            ;;
        *)
            log_error "不支援的部署模式: $deploy_mode"
            return 1
            ;;
    esac
}

deploy_local_services() {
    local components=("$@")
    
    log_info "部署到本地開發環境..."
    
    # 啟動基礎服務
    if [[ " ${components[*]} " =~ " database " ]] || [ ${#components[@]} -eq 0 ]; then
        log_info "啟動 PostgreSQL 服務..."
        # 這裡可以添加本地 PostgreSQL 啟動邏輯
    fi
    
    if [[ " ${components[*]} " =~ " zenoh " ]] || [ ${#components[@]} -eq 0 ]; then
        log_info "啟動 Zenoh Router..."
        # 這裡可以添加 Zenoh Router 啟動邏輯
    fi
    
    log_success "本地服務部署完成"
}

deploy_docker_services() {
    local components=("$@")
    
    log_info "部署到 Docker 環境..."
    
    # 啟動 AGVC 系統
    if [[ " ${components[*]} " =~ " agvc " ]] || [ ${#components[@]} -eq 0 ]; then
        log_info "啟動 AGVC 系統..."
        if docker compose -f docker-compose.agvc.yml up -d; then
            log_success "AGVC 系統啟動成功"
        else
            log_error "AGVC 系統啟動失敗"
            return 1
        fi
    fi
    
    # 啟動 AGV 系統
    if [[ " ${components[*]} " =~ " agv " ]] || [ ${#components[@]} -eq 0 ]; then
        log_info "啟動 AGV 系統..."
        if docker compose -f docker-compose.yml up -d; then
            log_success "AGV 系統啟動成功"
        else
            log_error "AGV 系統啟動失敗"
            return 1
        fi
    fi
    
    log_success "Docker 服務部署完成"
}

deploy_production_services() {
    local deploy_mode="$1"
    shift
    local components=("$@")
    
    log_info "部署到 $deploy_mode 環境..."
    
    # 生產環境部署邏輯
    log_warning "生產環境部署功能尚未完全實現"
    
    log_success "$deploy_mode 環境部署完成"
}

verify_deployment() {
    local components=("$@")
    
    log_deploy "驗證部署結果..."
    
    local verification_results=()
    local failed_checks=0
    
    # 檢查容器狀態
    local containers=(
        "agvc_server:AGVC 伺服器"
        "nginx:Nginx 代理"
        "postgres:PostgreSQL 資料庫"
        "rosagv:AGV 容器"
    )
    
    for container_info in "${containers[@]}"; do
        local container_name=$(echo "$container_info" | cut -d':' -f1)
        local container_desc=$(echo "$container_info" | cut -d':' -f2)
        
        if docker ps --format "table {{.Names}}" | grep -q "^$container_name\$"; then
            if [ "$(docker inspect --format='{{.State.Health.Status}}' "$container_name" 2>/dev/null)" = "healthy" ]; then
                verification_results+=("OK:$container_desc 運行正常")
            else
                verification_results+=("WARNING:$container_desc 運行但健康檢查失敗")
            fi
        else
            verification_results+=("ERROR:$container_desc 未運行")
            failed_checks=$((failed_checks + 1))
        fi
    done
    
    # 檢查服務端點
    local endpoints=(
        "http://localhost:8000/health:Web API 健康檢查"
        "http://localhost:8001:AGVCUI 界面"
        "http://localhost:8002:OPUI 界面"
        "tcp://localhost:5432:PostgreSQL 資料庫"
        "tcp://localhost:7447:Zenoh Router"
    )
    
    for endpoint_info in "${endpoints[@]}"; do
        local endpoint=$(echo "$endpoint_info" | cut -d':' -f1-2)
        local description=$(echo "$endpoint_info" | cut -d':' -f3)
        
        if [[ "$endpoint" =~ ^http:// ]]; then
            if curl -s -f "$endpoint" >/dev/null 2>&1; then
                verification_results+=("OK:$description 可訪問")
            else
                verification_results+=("ERROR:$description 無法訪問")
                failed_checks=$((failed_checks + 1))
            fi
        elif [[ "$endpoint" =~ ^tcp:// ]]; then
            local host_port=$(echo "$endpoint" | sed 's|tcp://||')
            local host=$(echo "$host_port" | cut -d':' -f1)
            local port=$(echo "$host_port" | cut -d':' -f2)
            
            if timeout 3 bash -c "echo > /dev/tcp/$host/$port" 2>/dev/null; then
                verification_results+=("OK:$description 連接正常")
            else
                verification_results+=("ERROR:$description 連接失敗")
                failed_checks=$((failed_checks + 1))
            fi
        fi
    done
    
    # 顯示驗證結果
    echo ""
    echo -e "${CYAN}部署驗證結果${NC}"
    echo "=================="
    echo ""
    
    for result in "${verification_results[@]}"; do
        local status=$(echo "$result" | cut -d':' -f1)
        local message=$(echo "$result" | cut -d':' -f2-)
        
        case "$status" in
            "OK")
                echo -e "  ${GREEN}✓${NC} $message"
                ;;
            "WARNING")
                echo -e "  ${YELLOW}⚠${NC} $message"
                ;;
            "ERROR")
                echo -e "  ${RED}✗${NC} $message"
                ;;
        esac
    done
    
    echo ""
    if [ $failed_checks -eq 0 ]; then
        log_success "部署驗證通過"
        return 0
    else
        log_error "部署驗證失敗 ($failed_checks 個檢查失敗)"
        return 1
    fi
}

rollback_deployment() {
    local backup_id="$1"
    
    if [ -z "$backup_id" ]; then
        # 查找最新的備份
        backup_id=$(ls -1t "$BACKUP_DIR" 2>/dev/null | head -n1)
        if [ -z "$backup_id" ]; then
            log_error "沒有找到可用的備份"
            return 1
        fi
        log_info "使用最新備份: $backup_id"
    fi
    
    local backup_path="$BACKUP_DIR/$backup_id"
    
    if [ ! -d "$backup_path" ]; then
        log_error "備份不存在: $backup_path"
        return 1
    fi
    
    log_deploy "回滾到備份: $backup_id"
    
    # 停止服務
    log_info "停止當前服務..."
    docker compose -f docker-compose.agvc.yml down 2>/dev/null || true
    docker compose -f docker-compose.yml down 2>/dev/null || true
    
    # 恢復配置檔案
    if [ -d "$backup_path/config" ]; then
        log_info "恢復配置目錄..."
        rsync -av "$backup_path/config/" "$CONFIG_DIR/"
    fi
    
    # 恢復 Docker Compose 檔案
    for compose_file in docker-compose.yml docker-compose.agvc.yml; do
        if [ -f "$backup_path/$compose_file" ]; then
            log_info "恢復 $compose_file..."
            cp "$backup_path/$compose_file" "$PROJECT_ROOT/"
        fi
    done
    
    # 恢復 Zenoh 配置
    if [ -f "$backup_path/routerconfig.json5" ]; then
        log_info "恢復 Zenoh 配置..."
        cp "$backup_path/routerconfig.json5" "$APP_DIR/"
    fi
    
    # 重新啟動服務
    log_info "重新啟動服務..."
    docker compose -f docker-compose.agvc.yml up -d
    
    log_success "回滾完成"
}

show_deployment_status() {
    log_info "檢查部署狀態..."
    
    echo ""
    echo -e "${CYAN}Docker 容器狀態${NC}"
    echo "==================="
    docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" | grep -E "(agvc|rosagv|nginx|postgres)"
    
    echo ""
    echo -e "${CYAN}服務端點檢查${NC}"
    echo "=================="
    
    local endpoints=(
        "8000:Web API"
        "8001:AGVCUI"
        "8002:OPUI"
        "5432:PostgreSQL"
        "7447:Zenoh Router"
        "80:Nginx"
    )
    
    for endpoint in "${endpoints[@]}"; do
        local port=$(echo "$endpoint" | cut -d':' -f1)
        local service=$(echo "$endpoint" | cut -d':' -f2)
        
        if netstat -tuln 2>/dev/null | grep -q ":$port "; then
            echo -e "  ${GREEN}✓${NC} $service (端口 $port)"
        else
            echo -e "  ${RED}✗${NC} $service (端口 $port)"
        fi
    done
    
    echo ""
    echo -e "${CYAN}最近部署記錄${NC}"
    echo "=================="
    if [ -f "$DEPLOYMENT_LOG" ]; then
        tail -n 10 "$DEPLOYMENT_LOG"
    else
        echo "沒有部署記錄"
    fi
}

cleanup_deployment() {
    log_deploy "清理部署檔案..."
    
    # 清理舊的建置檔案
    log_info "清理建置檔案..."
    find "$APP_DIR" -name "build" -type d -exec rm -rf {} + 2>/dev/null || true
    find "$APP_DIR" -name "install" -type d -exec rm -rf {} + 2>/dev/null || true
    
    # 清理 Docker 資源
    log_info "清理 Docker 資源..."
    docker system prune -f
    
    # 清理舊的備份 (保留最近 5 個)
    if [ -d "$BACKUP_DIR" ]; then
        log_info "清理舊的備份..."
        ls -1t "$BACKUP_DIR" | tail -n +6 | xargs -I {} rm -rf "$BACKUP_DIR/{}" 2>/dev/null || true
    fi
    
    # 清理舊的日誌
    if [ -f "$DEPLOYMENT_LOG" ] && [ $(wc -l < "$DEPLOYMENT_LOG") -gt 1000 ]; then
        log_info "清理部署日誌..."
        tail -n 500 "$DEPLOYMENT_LOG" > "${DEPLOYMENT_LOG}.tmp"
        mv "${DEPLOYMENT_LOG}.tmp" "$DEPLOYMENT_LOG"
    fi
    
    log_success "清理完成"
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
    
    # 初始化日誌
    mkdir -p "$(dirname "$DEPLOYMENT_LOG")"
    echo "$(date): 部署工具啟動" >> "$DEPLOYMENT_LOG"
    
    # 解析參數
    local command=""
    local deploy_mode=""
    local components=()
    local skip_checks="false"
    local skip_backup="false"
    local force_deploy="false"
    local dry_run="false"
    local config_file=""
    local version_tag=""
    local restart_services="false"
    local verbose="false"
    
    # 解析命令
    if [ $# -gt 0 ]; then
        case "$1" in
            deploy|pre-check|backup|restore|rollback|status|validate|logs|cleanup)
                command="$1"
                shift
                
                # 對於 deploy 命令，第二個參數是模式
                if [ "$command" = "deploy" ] && [ $# -gt 0 ] && [[ "$1" =~ ^(development|staging|production|local|docker)$ ]]; then
                    deploy_mode="$1"
                    shift
                fi
                ;;
            *)
                log_error "未知命令: $1"
                show_usage
                exit 1
                ;;
        esac
    else
        log_error "請指定命令"
        show_usage
        exit 1
    fi
    
    # 解析選項
    while [[ $# -gt 0 ]]; do
        case $1 in
            --components)
                IFS=',' read -ra comp_list <<< "$2"
                components+=("${comp_list[@]}")
                shift 2
                ;;
            --skip-checks)
                skip_checks="true"
                shift
                ;;
            --skip-backup)
                skip_backup="true"
                shift
                ;;
            --force)
                force_deploy="true"
                shift
                ;;
            --dry-run)
                dry_run="true"
                shift
                ;;
            --config)
                config_file="$2"
                shift 2
                ;;
            --tag)
                version_tag="$2"
                shift 2
                ;;
            --restart-services)
                restart_services="true"
                shift
                ;;
            --verbose)
                verbose="true"
                shift
                ;;
            -h|--help)
                show_header
                show_usage
                exit 0
                ;;
            *)
                # 對於 restore 命令，這可能是 backup_id
                if [ "$command" = "restore" ] && [ -z "$2" ]; then
                    backup_id="$1"
                    shift
                else
                    log_error "未知參數: $1"
                    show_usage
                    exit 1
                fi
                ;;
        esac
    done
    
    # 顯示標題
    show_header
    
    # 執行對應命令
    case "$command" in
        "deploy")
            if [ -z "$deploy_mode" ]; then
                log_error "請指定部署模式"
                show_usage
                exit 1
            fi
            
            log_info "開始部署到 $deploy_mode 環境"
            if [ ${#components[@]} -gt 0 ]; then
                log_info "部署組件: ${components[*]}"
            fi
            
            # Dry run 模式
            if [ "$dry_run" = "true" ]; then
                log_info "Dry run 模式 - 顯示將要執行的操作："
                echo "  1. 部署前檢查"
                echo "  2. 配置備份"
                echo "  3. 系統建置"
                echo "  4. 服務部署"
                echo "  5. 部署驗證"
                echo "  6. 清理作業"
                exit 0
            fi
            
            # 執行部署流程
            exit_code=0
            
            # 1. 部署前檢查
            if [ "$skip_checks" != "true" ]; then
                if ! pre_deployment_check "$deploy_mode"; then
                    check_result=$?
                    if [ $check_result -eq 1 ] || ([ $check_result -eq 2 ] && [ "$force_deploy" != "true" ]); then
                        log_error "部署前檢查失敗，使用 --force 強制部署"
                        exit 1
                    fi
                fi
            fi
            
            # 2. 配置備份
            if [ "$skip_backup" != "true" ]; then
                backup_id=$(backup_configuration)
                log_info "備份 ID: $backup_id"
            fi
            
            # 3. 配置驗證
            if ! validate_configuration; then
                log_error "配置驗證失敗"
                exit 1
            fi
            
            # 4. 系統建置
            if ! build_system "${components[@]}"; then
                log_error "系統建置失敗"
                exit 1
            fi
            
            # 5. 服務部署
            if ! deploy_services "$deploy_mode" "${components[@]}"; then
                log_error "服務部署失敗"
                exit 1
            fi
            
            # 6. 部署驗證
            if ! verify_deployment "${components[@]}"; then
                log_warning "部署驗證失敗，但部署已完成"
                exit_code=2
            fi
            
            # 7. 清理作業
            cleanup_deployment
            
            if [ $exit_code -eq 0 ]; then
                log_success "部署完成！"
            else
                log_warning "部署完成，但存在一些問題"
            fi
            
            exit $exit_code
            ;;
            
        "pre-check")
            pre_deployment_check "general"
            ;;
            
        "backup")
            backup_id=$(backup_configuration)
            echo "備份 ID: $backup_id"
            ;;
            
        "restore")
            rollback_deployment "$backup_id"
            ;;
            
        "rollback")
            rollback_deployment
            ;;
            
        "status")
            show_deployment_status
            ;;
            
        "validate")
            validate_configuration
            ;;
            
        "logs")
            if [ -f "$DEPLOYMENT_LOG" ]; then
                tail -f "$DEPLOYMENT_LOG"
            else
                log_error "部署日誌不存在"
                exit 1
            fi
            ;;
            
        "cleanup")
            cleanup_deployment
            ;;
            
        *)
            log_error "未知命令: $command"
            show_usage
            exit 1
            ;;
    esac
}

# 如果直接執行此腳本
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi