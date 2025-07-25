#!/bin/bash
# RosAGV AGV 配置檔案編輯工具
# 版本: 1.0
# 說明: 編輯 AGV 車輛配置檔案的便捷工具

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
CONFIG_DIR="$PROJECT_ROOT/app/config/agv"

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
    echo -e "${CYAN}🚗 RosAGV AGV 配置檔案編輯工具${NC}"
    echo -e "${CYAN}================================${NC}"
    echo ""
}

show_usage() {
    echo "使用方式:"
    echo "  $0 [agv_id]           # 編輯特定 AGV 配置"
    echo "  $0 list               # 列出所有可用的 AGV 配置"
    echo "  $0 -h, --help        # 顯示此幫助資訊"
    echo ""
    echo "範例:"
    echo "  $0 cargo01            # 編輯 cargo01 配置"
    echo "  $0 loader01           # 編輯 loader01 配置"
    echo "  $0 list               # 列出所有 AGV"
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
        log_error "AGV 配置目錄不存在: $CONFIG_DIR"
        return 1
    fi
    
    return 0
}

list_agv_configs() {
    log_info "可用的 AGV 配置檔案:"
    echo "=========================="
    echo ""
    
    local count=0
    for config_file in "$CONFIG_DIR"/*.yaml; do
        if [ -f "$config_file" ]; then
            local filename=$(basename "$config_file" .yaml)
            
            # 跳過基礎配置檔案
            if [ "$filename" = "base_config" ]; then
                continue
            fi
            
            echo -e "  📄 ${BLUE}$filename${NC}"
            
            # 提取設備資訊
            if grep -q "device_id:" "$config_file"; then
                local device_id=$(grep "device_id:" "$config_file" | sed 's/.*device_id: *"\([^"]*\)".*/\1/')
                local device_type=$(grep "device_type:" "$config_file" | sed 's/.*device_type: *"\([^"]*\)".*/\1/')
                local description=$(grep "description:" "$config_file" | sed 's/.*description: *"\([^"]*\)".*/\1/')
                
                echo "      ID: $device_id | 類型: $device_type"
                echo "      描述: $description"
            fi
            echo ""
            ((count++))
        fi
    done
    
    if [ $count -eq 0 ]; then
        log_warning "未找到任何 AGV 配置檔案"
        return 1
    fi
    
    log_info "找到 $count 個 AGV 配置檔案"
}

show_config_summary() {
    local config_file="$1"
    
    echo -e "${CYAN}📋 當前配置摘要:${NC}"
    echo "=================="
    
    # 使用 Python 解析 YAML 並顯示關鍵資訊
    python3 -c "
import yaml
import sys

try:
    with open('$config_file', 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    # 顯示設備資訊
    if 'device_info' in config:
        device_info = config['device_info']
        print('🔧 設備資訊:')
        for key, value in device_info.items():
            print(f'   {key}: {value}')
        print()
    
    # 顯示 AGV 參數
    if 'agv_parameters' in config:
        agv_params = config['agv_parameters']
        print('⚙️ AGV 參數:')
        # 只顯示關鍵參數
        key_params = ['max_speed', 'max_payload', 'safety_distance', 'battery_capacity']
        for key in key_params:
            if key in agv_params:
                print(f'   {key}: {agv_params[key]}')
        print()

except Exception as e:
    print(f'❌ 解析配置檔案時發生錯誤: {e}', file=sys.stderr)
    sys.exit(1)
" || {
        log_warning "無法解析配置檔案，直接顯示原始內容"
        head -20 "$config_file"
    }
}

validate_config() {
    local config_file="$1"
    
    log_info "驗證配置檔案語法..."
    
    python3 -c "
import yaml
import sys

try:
    with open('$config_file', 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    # 檢查必要欄位
    required_fields = ['device_info', 'agv_parameters']
    missing_fields = []
    
    for field in required_fields:
        if field not in config:
            missing_fields.append(field)
    
    if missing_fields:
        print('❌ 缺少必要欄位:', ', '.join(missing_fields))
        sys.exit(1)
    
    # 檢查設備資訊
    device_info = config.get('device_info', {})
    required_device_fields = ['device_id', 'device_type']
    for field in required_device_fields:
        if field not in device_info:
            print(f'❌ 缺少 device_info.{field}')
            sys.exit(1)
    
    print('✅ YAML 語法檢查通過')
    print('✅ 必要欄位檢查通過')
    
except yaml.YAMLError as e:
    print(f'❌ YAML 語法錯誤: {e}')
    sys.exit(1)
except Exception as e:
    print(f'❌ 檔案讀取錯誤: {e}')
    sys.exit(1)
"
}

show_post_edit_suggestions() {
    local agv_id="$1"
    
    echo ""
    echo -e "${CYAN}🚀 配置變更後的建議操作:${NC}"
    echo "=========================="
    echo "1. 驗證配置檔案:"
    echo "   $0 validate $agv_id"
    echo ""
    echo "2. 重啟 AGV 容器以套用新配置:"
    echo "   docker compose -f docker-compose.yml restart rosagv"
    echo ""
    echo "3. 檢查容器日誌:"
    echo "   docker compose -f docker-compose.yml logs -f rosagv"
    echo ""
    echo "4. 進入容器檢查配置載入:"
    echo "   docker compose -f docker-compose.yml exec rosagv bash"
    echo "   # 在容器內執行:"
    echo "   source /app/setup.bash && all_source"
    echo "   ros2 param list | grep $agv_id"
    echo ""
    echo "5. 測試 AGV 功能:"
    echo "   ros2 topic list | grep agv_status"
    echo "   ros2 topic echo /agv_status"
}

edit_agv_config() {
    local agv_id="$1"
    local config_file="$CONFIG_DIR/${agv_id}_config.yaml"
    
    if [ ! -f "$config_file" ]; then
        log_error "找不到 AGV 配置檔案: $config_file"
        echo ""
        log_info "可用的 AGV ID:"
        ls "$CONFIG_DIR" | grep -E ".*_config\.yaml$" | sed 's/_config\.yaml$//' | grep -v base | sed 's/^/  - /'
        return 1
    fi
    
    log_info "編輯 AGV 配置: $agv_id"
    echo "配置檔案: $config_file"
    echo ""
    
    # 顯示配置檔案摘要
    show_config_summary "$config_file"
    
    # 備份原始檔案
    local backup_file="${config_file}.backup.$(date +%Y%m%d_%H%M%S)"
    cp "$config_file" "$backup_file"
    log_success "已備份原始檔案: $backup_file"
    echo ""
    
    # 使用預設編輯器開啟檔案
    log_info "正在開啟編輯器..."
    ${EDITOR:-nano} "$config_file"
    
    echo ""
    log_success "編輯完成！"
    
    # 驗證配置檔案
    if validate_config "$config_file"; then
        log_success "配置檔案驗證通過"
        show_post_edit_suggestions "$agv_id"
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
            list_agv_configs
            exit 0
            ;;
        validate)
            if [ -z "$2" ]; then
                log_error "請提供 AGV ID 進行驗證"
                show_usage
                exit 1
            fi
            check_dependencies || exit 1
            local config_file="$CONFIG_DIR/${2}_config.yaml"
            if [ -f "$config_file" ]; then
                validate_config "$config_file"
            else
                log_error "找不到配置檔案: $config_file"
                exit 1
            fi
            exit 0
            ;;
        "")
            log_info "請選擇要編輯的 AGV ID，或使用 'list' 查看可用選項"
            echo ""
            show_usage
            echo ""
            list_agv_configs
            exit 0
            ;;
        *)
            check_dependencies || exit 1
            edit_agv_config "$1"
            ;;
    esac
}

# 當作為腳本執行時調用主程式
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    main "$@"
fi