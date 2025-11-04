#!/bin/bash
# RosAGV AGVC 配置檔案編輯工具
# 版本: 1.0
# 說明: 編輯 AGVC 管理系統配置檔案的便捷工具

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
NC='\033[0m' # No Color

# 配置路徑
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CONFIG_DIR="$PROJECT_ROOT/app/config"
AGVC_CONFIG_DIR="$CONFIG_DIR/agvc"

# 配置類型映射
declare -A CONFIG_MAPPING=(
    ["agvc01"]="$AGVC_CONFIG_DIR/agvc01_config.yaml:AGVC 主節點配置"
    ["primary"]="$AGVC_CONFIG_DIR/agvc01_config.yaml:AGVC 主節點配置"
    ["base"]="$AGVC_CONFIG_DIR/base_config.yaml:AGVC 基礎配置"
    ["base_config"]="$AGVC_CONFIG_DIR/base_config.yaml:AGVC 基礎配置"
    ["web_api"]="$CONFIG_DIR/web_api_config.yaml:Web API 配置"
    ["api"]="$CONFIG_DIR/web_api_config.yaml:Web API 配置"
    ["ecs"]="$CONFIG_DIR/ecs_config.yaml:ECS 設備控制配置"
    ["door"]="$CONFIG_DIR/door_config.yaml:門控系統配置"
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

show_header() {
    echo -e "${CYAN}🖥️ RosAGV AGVC 配置檔案編輯工具${NC}"
    echo -e "${CYAN}===================================${NC}"
    echo ""
}

show_usage() {
    echo "使用方式:"
    echo "  $0 [config_type]      # 編輯特定配置類型"
    echo "  $0 list               # 列出所有可用的配置"
    echo "  $0 -h, --help        # 顯示此幫助資訊"
    echo ""
    echo "可用的配置類型:"
    echo "  agvc01 (或 primary)   # AGVC 主節點配置"
    echo "  base (或 base_config) # AGVC 基礎配置"
    echo "  web_api (或 api)      # Web API 配置"
    echo "  ecs                   # ECS 設備控制配置"
    echo "  door                  # 門控系統配置"
    echo ""
    echo "範例:"
    echo "  $0 agvc01             # 編輯主節點配置"
    echo "  $0 web_api            # 編輯 Web API 配置"
    echo "  $0 list               # 列出所有配置"
}

# ============================================================================
# 主要功能函數
# ============================================================================

check_dependencies() {
    if ! command -v python3 &> /dev/null; then
        log_error "Python3 未安裝，無法進行配置驗證"
        return 1
    fi
    
    if [ ! -d "$CONFIG_DIR" ]; then
        log_error "配置目錄不存在: $CONFIG_DIR"
        return 1
    fi
    
    if [ ! -d "$AGVC_CONFIG_DIR" ]; then
        log_error "AGVC 配置目錄不存在: $AGVC_CONFIG_DIR"
        return 1
    fi
    
    return 0
}

list_agvc_configs() {
    log_info "可用的 AGVC 配置檔案:"
    echo "=========================="
    echo ""
    
    echo -e "${CYAN}🖥️ AGVC 專用配置:${NC}"
    echo "=================="
    for config_file in "$AGVC_CONFIG_DIR"/*.yaml; do
        if [ -f "$config_file" ]; then
            local filename=$(basename "$config_file" .yaml)
            echo -e "  📄 ${BLUE}$filename${NC}"
            
            # 提取設備資訊
            if grep -q "device_id:" "$config_file"; then
                local device_id=$(grep "device_id:" "$config_file" | sed 's/.*device_id: *"\([^"]*\)".*/\1/')
                local device_type=$(grep "device_type:" "$config_file" | sed 's/.*device_type: *"\([^"]*\)".*/\1/' 2>/dev/null || echo "N/A")
                local role=$(grep "role:" "$config_file" | sed 's/.*role: *"\([^"]*\)".*/\1/' 2>/dev/null || echo "N/A")
                local description=$(grep "description:" "$config_file" | sed 's/.*description: *"\?\([^"]*\)"\?.*/\1/')
                
                echo "      ID: $device_id | 類型: $device_type | 角色: $role"
                echo "      描述: $description"
            fi
            echo ""
        fi
    done
    
    echo -e "${CYAN}🔧 其他相關配置:${NC}"
    echo "================"
    local other_configs=(
        "$CONFIG_DIR/web_api_config.yaml:Web API 配置"
        "$CONFIG_DIR/ecs_config.yaml:ECS 設備控制配置"
        "$CONFIG_DIR/door_config.yaml:門控系統配置"
    )
    
    for config_info in "${other_configs[@]}"; do
        local config_path="${config_info%%:*}"
        local config_desc="${config_info##*:}"
        if [ -f "$config_path" ]; then
            local filename=$(basename "$config_path" .yaml)
            echo -e "  📄 ${BLUE}$filename${NC}: $config_desc"
        else
            local filename=$(basename "$config_path" .yaml)
            echo -e "  ❌ ${RED}$filename${NC}: $config_desc (檔案不存在)"
        fi
    done
    echo ""
}

show_config_summary() {
    local config_file="$1"
    local config_type="$2"
    
    echo -e "${CYAN}📋 當前配置摘要:${NC}"
    echo "=================="
    
    # 使用 Python 解析 YAML 並顯示關鍵資訊
    python3 -c "
import yaml
import sys

try:
    with open('$config_file', 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    config_type = '$config_type'
    
    # 顯示設備資訊 (如果存在)
    if 'device_info' in config:
        device_info = config['device_info']
        print('🔧 設備資訊:')
        for key, value in device_info.items():
            print(f'   {key}: {value}')
        print()
    
    # 根據配置類型顯示不同的關鍵參數
    if config_type in ['agvc01', 'primary']:
        if 'agvc_parameters' in config:
            agvc_params = config['agvc_parameters']
            print('⚙️ AGVC 系統參數:')
            key_params = ['system_role', 'max_concurrent_connections', 'max_concurrent_tasks']
            for key in key_params:
                if key in agvc_params:
                    print(f'   {key}: {agvc_params[key]}')
            print()
    
    elif config_type in ['web_api', 'api']:
        # Web API 配置的關鍵參數
        if 'server' in config:
            server_config = config['server']
            print('🌐 API 服務參數:')
            for key, value in server_config.items():
                if key in ['host', 'port', 'cors_origins']:
                    print(f'   {key}: {value}')
            print()
    
    elif config_type == 'ecs':
        # ECS 配置的關鍵參數
        print('🔌 ECS 控制參數:')
        for key, value in config.items():
            if key.startswith(('plc_', 'door_', 'device_')):
                print(f'   {key}: {value}')
                if len([k for k in config.keys() if k.startswith(('plc_', 'door_', 'device_'))]) > 5:
                    print('   ... (更多參數)')
                    break
        print()

except Exception as e:
    print(f'❌ 解析配置檔案時發生錯誤: {e}', file=sys.stderr)
    print('顯示原始檔案前 20 行:')
    with open('$config_file', 'r', encoding='utf-8') as f:
        for i, line in enumerate(f, 1):
            if i > 20:
                break
            print(f'{i:3d}: {line.rstrip()}')
" || {
        log_warning "無法解析配置檔案，直接顯示原始內容"
        head -20 "$config_file"
    }
}

validate_config() {
    local config_file="$1"
    local config_type="$2"
    
    log_info "驗證配置檔案語法..."
    
    python3 -c "
import yaml
import sys

try:
    with open('$config_file', 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    print('✅ YAML 語法檢查通過')
    
    config_type = '$config_type'
    
    # 根據配置類型檢查必要欄位
    if config_type in ['agvc01', 'primary']:
        required_fields = ['device_info']
        for field in required_fields:
            if field not in config:
                print(f'❌ 缺少必要欄位: {field}')
                sys.exit(1)
        
        device_info = config.get('device_info', {})
        if 'device_id' not in device_info:
            print('❌ 缺少 device_info.device_id')
            sys.exit(1)
            
        print('✅ AGVC 必要欄位檢查通過')
    
    elif config_type in ['web_api', 'api']:
        print('✅ Web API 配置格式檢查通過')
    
    elif config_type == 'ecs':
        print('✅ ECS 配置格式檢查通過')
    
    else:
        print('✅ 基本配置格式檢查通過')
    
    print('✅ 配置檔案驗證成功')
    
except yaml.YAMLError as e:
    print(f'❌ YAML 語法錯誤: {e}')
    sys.exit(1)
except Exception as e:
    print(f'❌ 檔案讀取錯誤: {e}')
    sys.exit(1)
"
}

show_post_edit_suggestions() {
    local config_type="$1"
    
    echo ""
    echo -e "${CYAN}🚀 配置變更後的建議操作:${NC}"
    echo "=========================="
    
    case "$config_type" in
        "agvc01"|"primary")
            echo "1. 重啟 AGVC 容器:"
            echo "   docker compose -f docker-compose.agvc.yml restart agvc_server"
            echo ""
            echo "2. 檢查容器日誌:"
            echo "   docker compose -f docker-compose.agvc.yml logs -f agvc_server"
            echo ""
            echo "3. 進入容器檢查系統狀態:"
            echo "   docker compose -f docker-compose.agvc.yml exec agvc_server bash"
            echo "   # 在容器內執行:"
            echo "   source /app/setup.bash && agvc_source"
            echo "   manage status"
            ;;
        "web_api"|"api")
            echo "1. 重啟 Web API 服務:"
            echo "   docker compose -f docker-compose.agvc.yml restart agvc_server"
            echo ""
            echo "2. 檢查 API 服務:"
            echo "   curl -X GET http://localhost:8000/health"
            echo ""
            echo "3. 檢查 API 日誌:"
            echo "   docker compose -f docker-compose.agvc.yml logs -f agvc_server | grep -i api"
            ;;
        "ecs")
            echo "1. 重啟 ECS 服務:"
            echo "   docker compose -f docker-compose.agvc.yml exec agvc_server bash"
            echo "   # 在容器內重啟 ECS 服務:"
            echo "   manage_ecs_core restart"
            echo ""
            echo "2. 檢查 ECS 狀態:"
            echo "   manage_ecs_core status"
            ;;
        *)
            echo "1. 重啟相關 AGVC 服務:"
            echo "   docker compose -f docker-compose.agvc.yml restart agvc_server"
            echo ""
            echo "2. 檢查系統狀態:"
            echo "   docker compose -f docker-compose.agvc.yml exec agvc_server bash"
            echo "   # 在容器內執行:"
            echo "   check_system_status"
            ;;
    esac
    
    echo ""
    echo "通用檢查:"
    echo "4. 檢查主要服務端口:"
    echo "   ss -tuln | grep -E '800[0-2]|7447'"
    echo ""
    echo "5. 監控系統運行:"
    echo "   docker compose -f docker-compose.agvc.yml logs -f agvc_server"
}

edit_agvc_config() {
    local config_type="$1"
    
    # 獲取配置檔案路徑和描述
    local config_info="${CONFIG_MAPPING[$config_type]}"
    if [ -z "$config_info" ]; then
        log_error "未知的配置類型: $config_type"
        echo ""
        log_info "可用的配置類型:"
        for key in "${!CONFIG_MAPPING[@]}"; do
            local info="${CONFIG_MAPPING[$key]}"
            local desc="${info##*:}"
            echo "  - $key: $desc"
        done
        return 1
    fi
    
    local config_file="${config_info%%:*}"
    local config_name="${config_info##*:}"
    
    if [ ! -f "$config_file" ]; then
        log_error "找不到配置檔案: $config_file"
        return 1
    fi
    
    log_info "編輯 $config_name"
    echo "配置檔案: $config_file"
    echo ""
    
    # 顯示配置檔案摘要
    show_config_summary "$config_file" "$config_type"
    
    # 備份原始檔案
    local backup_file="${config_file}.backup.$(date +%Y%m%d_%H%M%S)"
    cp "$config_file" "$backup_file"
    log_success "已備份原始檔案: $backup_file"
    echo ""
    
    # 顯示重要提醒
    echo -e "${YELLOW}⚠️ 重要提醒:${NC}"
    echo "============="
    case "$config_type" in
        "agvc01"|"primary")
            echo "• AGVC 主節點配置變更會影響整個車隊管理"
            echo "• 請謹慎修改系統參數"
            ;;
        "web_api"|"api")
            echo "• Web API 配置變更會影響前端界面"
            echo "• 端口變更需要同步更新 nginx 配置"
            ;;
        "ecs")
            echo "• ECS 配置變更會影響設備控制"
            echo "• PLC 相關參數需要與實際硬體對應"
            ;;
    esac
    echo "• 建議在測試環境先驗證配置變更"
    echo ""
    
    # 使用預設編輯器開啟檔案
    log_info "正在開啟編輯器..."
    ${EDITOR:-nano} "$config_file"
    
    echo ""
    log_success "編輯完成！"
    
    # 驗證配置檔案
    if validate_config "$config_file" "$config_type"; then
        log_success "配置檔案驗證通過"
        show_post_edit_suggestions "$config_type"
    else
        log_error "配置檔案驗證失敗，請檢查語法"
        echo ""
        log_info "可以還原備份檔案:"
        echo "  cp '$backup_file' '$config_file'"
        return 1
    fi
}

# ============================================================================
# 主程式
# ============================================================================

main() {
    show_header
    
    # 參數處理
    case "${1:-}" in
        -h|--help)
            show_usage
            exit 0
            ;;
        list|ls)
            check_dependencies || exit 1
            list_agvc_configs
            exit 0
            ;;
        validate)
            if [ -z "$2" ]; then
                log_error "請提供配置類型進行驗證"
                show_usage
                exit 1
            fi
            check_dependencies || exit 1
            local config_info="${CONFIG_MAPPING[$2]}"
            if [ -n "$config_info" ]; then
                local config_file="${config_info%%:*}"
                if [ -f "$config_file" ]; then
                    validate_config "$config_file" "$2"
                else
                    log_error "找不到配置檔案: $config_file"
                    exit 1
                fi
            else
                log_error "未知的配置類型: $2"
                exit 1
            fi
            exit 0
            ;;
        "")
            log_info "請選擇要編輯的配置類型，或使用 'list' 查看可用選項"
            echo ""
            show_usage
            echo ""
            list_agvc_configs
            exit 0
            ;;
        *)
            check_dependencies || exit 1
            edit_agvc_config "$1"
            ;;
    esac
}

# 當作為腳本執行時調用主程式
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    main "$@"
fi
