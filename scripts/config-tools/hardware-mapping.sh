#!/bin/bash
# RosAGV 硬體映射配置管理工具
# 版本: 1.0
# 說明: 管理硬體映射配置檔案，用於設備身份識別和硬體資訊管理

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

# 配置路徑
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
HARDWARE_MAPPING_FILE="$PROJECT_ROOT/app/config/hardware_mapping.yaml"

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
    echo -e "${CYAN}🔧 RosAGV 硬體映射配置管理工具${NC}"
    echo -e "${CYAN}===================================${NC}"
    echo ""
}

show_usage() {
    echo "使用方式:"
    echo "  $0 [action] [device_id]"
    echo ""
    echo "可用動作:"
    echo "  list, ls              # 列出所有設備詳細資訊"
    echo "  show <device_id>      # 顯示特定設備詳情"
    echo "  edit <device_id>      # 編輯設備配置"
    echo "  mac <device_id>       # 管理 MAC 地址"
    echo "  validate, check       # 驗證配置檔案"
    echo "  overview              # 顯示硬體映射概況 (預設)"
    echo "  -h, --help           # 顯示此幫助資訊"
    echo ""
    echo "範例:"
    echo "  $0                    # 顯示概況"
    echo "  $0 list               # 列出所有設備"
    echo "  $0 show cargo01       # 顯示 cargo01 詳情"
    echo "  $0 edit agvc01        # 編輯 agvc01 配置"
    echo "  $0 mac cargo01        # 管理 cargo01 的 MAC 地址"
    echo "  $0 validate           # 驗證配置檔案"
}

# ============================================================================
# 主要功能函數
# ============================================================================

check_dependencies() {
    if ! command -v python3 &> /dev/null; then
        log_error "Python3 未安裝，無法進行配置解析"
        return 1
    fi
    
    if [ ! -f "$HARDWARE_MAPPING_FILE" ]; then
        log_error "硬體映射配置檔案不存在: $HARDWARE_MAPPING_FILE"
        return 1
    fi
    
    return 0
}

show_hardware_overview() {
    log_info "RosAGV 硬體映射配置概況"
    echo "=================================="
    echo ""
    
    echo "📁 配置檔案: $HARDWARE_MAPPING_FILE"
    echo "📊 檔案大小: $(du -h "$HARDWARE_MAPPING_FILE" | cut -f1)"
    echo "🕒 修改時間: $(stat -c %y "$HARDWARE_MAPPING_FILE" 2>/dev/null || echo "無法獲取")"
    echo ""
    
    # 使用 Python 解析並顯示設備資訊
    python3 -c "
import yaml
import sys

try:
    with open('$HARDWARE_MAPPING_FILE', 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    # 顯示 AGV 設備
    if 'agv_devices' in config:
        print('🚗 AGV 車載設備:')
        print('================')
        agv_count = 0
        for device_id, device_info in config['agv_devices'].items():
            device_type = device_info.get('device_type', 'unknown')
            description = device_info.get('description', 'No description')
            mac_count = len(device_info.get('mac_addresses', []))
            print(f'  📱 {device_id} ({device_type})')
            print(f'      描述: {description}')
            print(f'      MAC 地址數量: {mac_count}')
            print(f'      套件: {device_info.get(\"launch_package\", \"N/A\")}')
            agv_count += 1
        print(f'  總計: {agv_count} 個 AGV 設備')
        print()
    
    # 顯示 AGVC 設備
    if 'agvc_devices' in config:
        print('🖥️ AGVC 管理設備:')
        print('=================')
        agvc_count = 0
        for device_id, device_info in config['agvc_devices'].items():
            device_type = device_info.get('device_type', 'unknown')
            description = device_info.get('description', 'No description')
            role = device_info.get('role', 'N/A')
            mac_count = len(device_info.get('mac_addresses', []))
            services = device_info.get('services', [])
            print(f'  🖥️ {device_id} ({device_type})')
            print(f'      描述: {description}')
            print(f'      角色: {role}')
            print(f'      MAC 地址數量: {mac_count}')
            print(f'      服務數量: {len(services)}')
            agvc_count += 1
        print(f'  總計: {agvc_count} 個 AGVC 設備')
        print()

except Exception as e:
    print(f'❌ 解析配置檔案時發生錯誤: {e}', file=sys.stderr)
    sys.exit(1)
"
}

list_all_devices() {
    log_info "詳細設備清單"
    echo "================"
    echo ""
    
    python3 -c "
import yaml

try:
    with open('$HARDWARE_MAPPING_FILE', 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    # 處理 AGV 設備
    if 'agv_devices' in config:
        print('🚗 AGV 車載設備詳情:')
        print('====================')
        for device_id, info in config['agv_devices'].items():
            print(f'📱 {device_id}')
            print(f'   類型: {info.get(\"device_type\", \"unknown\")}')
            print(f'   描述: {info.get(\"description\", \"N/A\")}')
            print(f'   配置檔案: {info.get(\"config_file\", \"N/A\")}')
            print(f'   啟動套件: {info.get(\"launch_package\", \"N/A\")}')
            print(f'   啟動檔案: {info.get(\"launch_file\", \"N/A\")}')
            
            mac_addresses = info.get('mac_addresses', [])
            print(f'   MAC 地址 ({len(mac_addresses)} 個):')
            for i, mac in enumerate(mac_addresses, 1):
                print(f'     {i}. {mac}')
            print()
    
    # 處理 AGVC 設備
    if 'agvc_devices' in config:
        print('🖥️ AGVC 管理設備詳情:')
        print('=====================')
        for device_id, info in config['agvc_devices'].items():
            print(f'🖥️ {device_id}')
            print(f'   類型: {info.get(\"device_type\", \"unknown\")}')
            print(f'   角色: {info.get(\"role\", \"N/A\")}')
            print(f'   描述: {info.get(\"description\", \"N/A\")}')
            print(f'   配置檔案: {info.get(\"config_file\", \"N/A\")}')
            
            services = info.get('services', [])
            if services:
                print(f'   服務 ({len(services)} 個): {\"、\".join(services)}')
            
            workspaces = info.get('workspaces', [])
            if workspaces:
                print(f'   工作空間數量: {len(workspaces)}')
            
            mac_addresses = info.get('mac_addresses', [])
            print(f'   MAC 地址 ({len(mac_addresses)} 個):')
            for i, mac in enumerate(mac_addresses, 1):
                print(f'     {i}. {mac}')
            print()

except Exception as e:
    print(f'❌ 錯誤: {e}')
"
}

show_device_details() {
    local device_id="$1"
    
    if [ -z "$device_id" ]; then
        log_error "請提供設備 ID"
        return 1
    fi
    
    echo -e "${CYAN}🔍 設備詳細資訊: $device_id${NC}"
    echo "============================="
    echo ""
    
    python3 -c "
import yaml

try:
    with open('$HARDWARE_MAPPING_FILE', 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    device_id = '$device_id'
    device_found = False
    device_category = ''
    
    # 搜尋 AGV 設備
    if 'agv_devices' in config and device_id in config['agv_devices']:
        device_info = config['agv_devices'][device_id]
        device_found = True
        device_category = 'AGV'
        print('🚗 AGV 車載設備')
        print('================')
        
    # 搜尋 AGVC 設備
    elif 'agvc_devices' in config and device_id in config['agvc_devices']:
        device_info = config['agvc_devices'][device_id]
        device_found = True
        device_category = 'AGVC'
        print('🖥️ AGVC 管理設備')
        print('================')
    
    if not device_found:
        print(f'❌ 找不到設備: {device_id}')
        print()
        print('💡 可用的設備 ID:')
        
        if 'agv_devices' in config:
            agv_devices = list(config['agv_devices'].keys())
            if agv_devices:
                print('   🚗 AGV 設備:', ', '.join(agv_devices))
        if 'agvc_devices' in config:
            agvc_devices = list(config['agvc_devices'].keys())
            if agvc_devices:
                print('   🖥️ AGVC 設備:', ', '.join(agvc_devices))
        exit(1)
    
    # 顯示設備詳細資訊
    print(f'設備 ID: {device_id}')
    print(f'設備類別: {device_category}')
    print(f'設備類型: {device_info.get(\"device_type\", \"unknown\")}')
    print(f'描述: {device_info.get(\"description\", \"N/A\")}')
    
    if 'role' in device_info:
        print(f'角色: {device_info[\"role\"]}')
    
    print(f'配置檔案: {device_info.get(\"config_file\", \"N/A\")}')
    
    if 'launch_package' in device_info:
        print(f'啟動套件: {device_info[\"launch_package\"]}')
        print(f'啟動檔案: {device_info.get(\"launch_file\", \"N/A\")}')
    
    if 'services' in device_info:
        services = device_info['services']
        print(f'相關服務 ({len(services)} 個): {\"、\".join(services)}')
    
    if 'workspaces' in device_info:
        workspaces = device_info['workspaces']
        print(f'工作空間數量: {len(workspaces)}')
    
    print()
    print('📡 網路介面資訊:')
    print('===============')
    mac_addresses = device_info.get('mac_addresses', [])
    if mac_addresses:
        for i, mac in enumerate(mac_addresses, 1):
            # 檢查是否有註釋
            if isinstance(mac, str) and '#' in str(mac):
                print(f'  {i}. {mac}')
            else:
                print(f'  {i}. {mac}')
    else:
        print('  ⚠️ 未設定 MAC 地址')
    
    print()

except Exception as e:
    print(f'❌ 錯誤: {e}')
"
}

edit_device_config() {
    local device_id="$1"
    
    if [ -z "$device_id" ]; then
        log_error "請提供設備 ID"
        return 1
    fi
    
    # 檢查設備是否存在
    if ! python3 -c "
import yaml
try:
    with open('$HARDWARE_MAPPING_FILE', 'r') as f:
        config = yaml.safe_load(f)
    found = ('agv_devices' in config and '$device_id' in config['agv_devices']) or \
            ('agvc_devices' in config and '$device_id' in config['agvc_devices'])
    exit(0 if found else 1)
except:
    exit(1)
"; then
        log_error "找不到設備: $device_id"
        
        # 顯示可用設備
        echo ""
        log_info "可用的設備 ID:"
        python3 -c "
import yaml
try:
    with open('$HARDWARE_MAPPING_FILE', 'r') as f:
        config = yaml.safe_load(f)
    if 'agv_devices' in config:
        agv_devices = list(config['agv_devices'].keys())
        if agv_devices:
            print('   🚗 AGV 設備:', ', '.join(agv_devices))
    if 'agvc_devices' in config:
        agvc_devices = list(config['agvc_devices'].keys())
        if agvc_devices:
            print('   🖥️ AGVC 設備:', ', '.join(agvc_devices))
except:
    pass
"
        return 1
    fi
    
    log_info "編輯設備配置: $device_id"
    echo "配置檔案: $HARDWARE_MAPPING_FILE"
    echo ""
    
    echo -e "${YELLOW}⚠️ 重要注意事項:${NC}"
    echo "================="
    echo "• 直接編輯硬體映射檔案會影響設備身份識別"
    echo "• 建議在修改前先備份原始檔案"
    echo "• MAC 地址變更會影響設備自動識別功能"
    echo "• 配置變更後需要重啟對應容器才會生效"
    echo ""
    
    # 備份原始檔案
    local backup_file="${HARDWARE_MAPPING_FILE}.backup.$(date +%Y%m%d_%H%M%S)"
    cp "$HARDWARE_MAPPING_FILE" "$backup_file"
    log_success "已備份原始檔案: $backup_file"
    echo ""
    
    # 顯示當前設備配置
    show_device_details "$device_id"
    echo ""
    
    # 使用編輯器開啟檔案
    log_info "正在開啟編輯器..."
    ${EDITOR:-nano} "$HARDWARE_MAPPING_FILE"
    
    echo ""
    log_success "編輯完成！"
    
    # 驗證配置檔案
    if validate_hardware_config; then
        log_success "配置檔案驗證通過"
        show_post_edit_suggestions "$device_id"
    else
        log_error "配置檔案驗證失敗"
        echo ""
        log_info "可以還原備份檔案:"
        echo "  cp '$backup_file' '$HARDWARE_MAPPING_FILE'"
        return 1
    fi
}

manage_mac_addresses() {
    local device_id="$1"
    
    if [ -z "$device_id" ]; then
        log_error "請提供設備 ID"
        return 1
    fi
    
    echo -e "${CYAN}📡 管理設備 MAC 地址: $device_id${NC}"
    echo "==============================="
    echo ""
    
    # 顯示當前 MAC 地址
    echo -e "${BLUE}📋 當前 MAC 地址:${NC}"
    python3 -c "
import yaml

try:
    with open('$HARDWARE_MAPPING_FILE', 'r') as f:
        config = yaml.safe_load(f)
    
    device_id = '$device_id'
    device_found = False
    
    # 搜尋設備
    for device_type in ['agv_devices', 'agvc_devices']:
        if device_type in config and device_id in config[device_type]:
            device_info = config[device_type][device_id]
            device_found = True
            break
    
    if not device_found:
        print(f'❌ 找不到設備: {device_id}')
        exit(1)
    
    mac_addresses = device_info.get('mac_addresses', [])
    if mac_addresses:
        for i, mac in enumerate(mac_addresses, 1):
            print(f'  {i}. {mac}')
    else:
        print('  ⚠️ 未設定 MAC 地址')

except Exception as e:
    print(f'❌ 錯誤: {e}')
"
    
    echo ""
    echo -e "${CYAN}🔧 可用操作:${NC}"
    echo "1. 顯示當前實際 MAC 地址 (需要在對應容器內執行)"
    echo "2. 自動更新 MAC 地址 (整合現有的 show_device_mac_info 功能)"
    echo "3. 手動編輯硬體映射檔案"
    echo ""
    
    echo -e "${PURPLE}💡 建議操作流程:${NC}"
    echo "================"
    echo "1. 檢查實際 MAC 地址:"
    
    # 根據設備類型給出不同建議
    if [[ "$device_id" =~ ^agv ]]; then
        echo "   docker compose -f docker-compose.yml exec rosagv bash"
        echo "   # 在 AGV 容器內執行:"
    else
        echo "   docker compose -f docker-compose.agvc.yml exec agvc_server bash"
        echo "   # 在 AGVC 容器內執行:"
    fi
    
    echo "   source /app/setup.bash && show_device_mac_info"
    echo ""
    echo "2. 自動更新配置檔案中的 MAC 地址:"
    echo "   # 在對應容器內執行:"
    echo "   show_device_mac_info --update-config"
    echo ""
    echo "3. 手動編輯 (當前操作):"
    echo "   $0 edit $device_id"
    echo ""
    echo "4. 驗證配置變更:"
    echo "   $0 validate"
}

validate_hardware_config() {
    log_info "驗證硬體映射配置檔案"
    echo "========================"
    echo ""
    
    python3 -c "
import yaml
import re

def validate_mac_address(mac):
    \"\"\"驗證 MAC 地址格式\"\"\"
    # 移除可能的註釋
    mac_clean = str(mac).split('#')[0].strip()
    pattern = r'^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$'
    return re.match(pattern, mac_clean) is not None

try:
    with open('$HARDWARE_MAPPING_FILE', 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    print('✅ YAML 語法檢查通過')
    
    errors = []
    warnings = []
    
    # 檢查 AGV 設備
    if 'agv_devices' in config:
        for device_id, device_info in config['agv_devices'].items():
            # 檢查必要欄位
            required_fields = ['device_type', 'config_file', 'launch_package']
            for field in required_fields:
                if field not in device_info:
                    errors.append(f'AGV {device_id} 缺少必要欄位: {field}')
            
            # 檢查 MAC 地址
            mac_addresses = device_info.get('mac_addresses', [])
            if not mac_addresses:
                warnings.append(f'AGV {device_id} 未設定 MAC 地址')
            else:
                for mac in mac_addresses:
                    if not validate_mac_address(mac):
                        errors.append(f'AGV {device_id} MAC 地址格式錯誤: {mac}')
            
            # 檢查配置檔案是否存在
            config_file_path = f'$PROJECT_ROOT/app/config/agv/{device_info.get(\"config_file\", \"\")}'
            import os
            if device_info.get('config_file') and not os.path.exists(config_file_path):
                warnings.append(f'AGV {device_id} 配置檔案不存在: {config_file_path}')
    
    # 檢查 AGVC 設備
    if 'agvc_devices' in config:
        for device_id, device_info in config['agvc_devices'].items():
            # 檢查必要欄位
            required_fields = ['device_type', 'config_file']
            for field in required_fields:
                if field not in device_info:
                    errors.append(f'AGVC {device_id} 缺少必要欄位: {field}')
            
            # 檢查 MAC 地址
            mac_addresses = device_info.get('mac_addresses', [])
            if not mac_addresses:
                warnings.append(f'AGVC {device_id} 未設定 MAC 地址')
            else:
                for mac in mac_addresses:
                    if not validate_mac_address(mac):
                        errors.append(f'AGVC {device_id} MAC 地址格式錯誤: {mac}')
            
            # 檢查配置檔案是否存在
            config_file_path = f'$PROJECT_ROOT/app/config/agvc/{device_info.get(\"config_file\", \"\")}'
            import os
            if device_info.get('config_file') and not os.path.exists(config_file_path):
                warnings.append(f'AGVC {device_id} 配置檔案不存在: {config_file_path}')
    
    # 檢查重複的 MAC 地址
    all_macs = []
    for device_type in ['agv_devices', 'agvc_devices']:
        if device_type in config:
            for device_id, device_info in config[device_type].items():
                mac_addresses = device_info.get('mac_addresses', [])
                for mac in mac_addresses:
                    mac_clean = str(mac).split('#')[0].strip().upper()
                    if mac_clean in all_macs:
                        errors.append(f'重複的 MAC 地址: {mac_clean}')
                    else:
                        all_macs.append(mac_clean)
    
    # 顯示結果
    if errors:
        print('❌ 發現錯誤:')
        for error in errors:
            print(f'   • {error}')
        print()
    
    if warnings:
        print('⚠️ 警告:')
        for warning in warnings:
            print(f'   • {warning}')
        print()
    
    if not errors and not warnings:
        print('✅ 硬體映射配置檔案驗證通過')
    elif not errors:
        print('✅ 硬體映射配置檔案基本正確 (有警告)')
    else:
        print('❌ 硬體映射配置檔案有錯誤，請修正後重新驗證')
        exit(1)

except yaml.YAMLError as e:
    print(f'❌ YAML 語法錯誤: {e}')
    exit(1)
except Exception as e:
    print(f'❌ 驗證過程發生錯誤: {e}')
    exit(1)
"
}

show_post_edit_suggestions() {
    local device_id="$1"
    
    echo ""
    echo -e "${CYAN}🚀 配置變更後的建議操作:${NC}"
    echo "=========================="
    
    # 根據設備類型給出不同建議
    if [[ "$device_id" =~ ^agv ]]; then
        echo "AGV 設備配置變更:"
        echo "1. 重啟 AGV 容器:"
        echo "   docker compose -f docker-compose.yml restart rosagv"
        echo ""
        echo "2. 檢查容器日誌:"
        echo "   docker compose -f docker-compose.yml logs -f rosagv"
        echo ""
        echo "3. 進入容器檢查設備識別:"
        echo "   docker compose -f docker-compose.yml exec rosagv bash"
        echo "   # 在容器內執行:"
        echo "   source /app/setup.bash && check_device_identity"
    else
        echo "AGVC 設備配置變更:"
        echo "1. 重啟 AGVC 容器:"
        echo "   docker compose -f docker-compose.agvc.yml restart agvc_server"
        echo ""
        echo "2. 檢查容器日誌:"
        echo "   docker compose -f docker-compose.agvc.yml logs -f agvc_server"
        echo ""
        echo "3. 進入容器檢查設備識別:"
        echo "   docker compose -f docker-compose.agvc.yml exec agvc_server bash"
        echo "   # 在容器內執行:"
        echo "   source /app/setup.bash && check_device_identity"
    fi
    
    echo ""
    echo "通用檢查:"
    echo "4. 驗證硬體映射配置:"
    echo "   $0 validate"
    echo ""
    echo "5. 檢查設備身份識別:"
    echo "   # 在對應容器內執行:"
    echo "   identify_device_manual"
    echo ""
    echo "6. 檢查 MAC 地址映射:"
    echo "   show_device_mac_info"
}

# ============================================================================
# 主程式
# ============================================================================

main() {
    show_header
    
    # 參數處理
    local action="${1:-overview}"
    local device_id="$2"
    
    case "$action" in
        -h|--help)
            show_usage
            exit 0
            ;;
        list|ls)
            check_dependencies || exit 1
            list_all_devices
            ;;
        show|details)
            check_dependencies || exit 1
            show_device_details "$device_id"
            ;;
        edit)
            check_dependencies || exit 1
            edit_device_config "$device_id"
            ;;
        mac)
            check_dependencies || exit 1
            manage_mac_addresses "$device_id"
            ;;
        validate|check)
            check_dependencies || exit 1
            validate_hardware_config
            ;;
        overview|"")
            check_dependencies || exit 1
            show_hardware_overview
            echo ""
            log_info "使用說明:"
            echo "  $0 list              # 列出所有設備"
            echo "  $0 show <device_id>  # 顯示特定設備詳情"
            echo "  $0 edit <device_id>  # 編輯設備配置"
            echo "  $0 mac <device_id>   # 管理 MAC 地址"
            echo "  $0 validate          # 驗證配置檔案"
            ;;
        *)
            log_error "未知動作: $action"
            echo ""
            show_usage
            exit 1
            ;;
    esac
}

# 當作為腳本執行時調用主程式
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    main "$@"
fi
