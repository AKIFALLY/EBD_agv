#!/bin/bash
# RosAGV Zenoh 路由器配置管理工具
# 版本: 1.0
# 說明: 管理 Zenoh 路由器配置檔案，用於 ROS 2 Zenoh RMW 通訊設定

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
ZENOH_CONFIG_FILE="$PROJECT_ROOT/app/routerconfig.json5"

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
    echo -e "${CYAN}🌐 RosAGV Zenoh Router 配置管理工具${NC}"
    echo -e "${CYAN}=====================================${NC}"
    echo ""
}

show_usage() {
    echo "使用方式:"
    echo "  $0 [action]"
    echo ""
    echo "可用動作:"
    echo "  show, details         # 顯示詳細配置資訊"
    echo "  edit                  # 編輯配置檔案"
    echo "  validate, check       # 驗證配置檔案格式"
    echo "  status                # 檢查 Zenoh 服務狀態"
    echo "  restart               # 重啟服務指南"
    echo "  overview              # 顯示配置概況 (預設)"
    echo "  -h, --help           # 顯示此幫助資訊"
    echo ""
    echo "範例:"
    echo "  $0                    # 顯示配置概況"
    echo "  $0 show               # 顯示詳細配置"
    echo "  $0 edit               # 編輯配置檔案"
    echo "  $0 validate           # 驗證配置格式"
    echo "  $0 status             # 檢查服務狀態"
}

# ============================================================================
# 主要功能函數
# ============================================================================

check_dependencies() {
    if ! command -v python3 &> /dev/null; then
        log_error "Python3 未安裝，無法進行配置解析"
        return 1
    fi
    
    if [ ! -f "$ZENOH_CONFIG_FILE" ]; then
        log_error "Zenoh 配置檔案不存在: $ZENOH_CONFIG_FILE"
        return 1
    fi
    
    return 0
}

show_zenoh_overview() {
    log_info "Zenoh Router 配置概況"
    echo "========================="
    echo ""
    
    echo "📁 配置檔案: $ZENOH_CONFIG_FILE"
    echo "📊 檔案大小: $(du -h "$ZENOH_CONFIG_FILE" | cut -f1)"
    echo "🕒 修改時間: $(stat -c %y "$ZENOH_CONFIG_FILE" 2>/dev/null || echo "無法獲取")"
    echo ""
    
    # 使用 json5 工具解析配置並顯示關鍵資訊
    if ! command -v json5 &> /dev/null; then
        log_error "json5 工具未安裝，無法解析 JSON5 配置檔案"
        log_info "請安裝: npm install -g json5"
        return 1
    fi
    
    # 驗證 JSON5 語法
    if ! json5 --validate "$ZENOH_CONFIG_FILE" >/dev/null 2>&1; then
        log_error "Zenoh 配置檔案語法錯誤"
        log_info "顯示原始檔案前 20 行:"
        head -20 "$ZENOH_CONFIG_FILE" | cat -n
        return 1
    fi
    
    # 解析 JSON5 並顯示配置資訊
    python3 -c "
import json
import sys

try:
    # 使用 json5 工具轉換為標準 JSON
    import subprocess
    result = subprocess.run(['json5', '$ZENOH_CONFIG_FILE'], capture_output=True, text=True)
    if result.returncode != 0:
        print('❌ 無法解析 Zenoh 配置檔案')
        sys.exit(1)
    
    config = json.loads(result.stdout)
    
    if config is None:
        print('❌ 無法解析 Zenoh 配置檔案')
        print('顯示原始檔案前 20 行:')
        with open('$ZENOH_CONFIG_FILE', 'r', encoding='utf-8') as f:
            for i, line in enumerate(f, 1):
                if i > 20:
                    break
                print(f'{i:3d}: {line.rstrip()}')
        exit(1)
    
    # 顯示基本配置資訊
    print('⚙️ 基本配置:')
    print('=============')
    mode = config.get('mode', 'unknown')
    print(f'  模式: {mode}')
    
    if 'id' in config:
        print(f'  節點 ID: {config[\"id\"]}')
    else:
        print('  節點 ID: 自動生成')
    
    print()
    
    # 顯示連接端點
    if 'connect' in config and 'endpoints' in config['connect']:
        endpoints = config['connect']['endpoints']
        print('🔗 連接端點:')
        print('============')
        for i, endpoint in enumerate(endpoints[:5], 1):  # 只顯示前5個
            print(f'  {i}. {endpoint}')
        if len(endpoints) > 5:
            print(f'  ... 還有 {len(endpoints) - 5} 個端點')
        print()
    
    # 顯示監聽端點
    if 'listen' in config and 'endpoints' in config['listen']:
        endpoints = config['listen']['endpoints']
        print('👂 監聽端點:')
        print('============')
        for i, endpoint in enumerate(endpoints, 1):
            print(f'  {i}. {endpoint}')
        print()
    
    # 顯示連接配置摘要
    if 'connect' in config:
        connect_config = config['connect']
        print('🔧 連接配置摘要:')
        print('================')
        
        if 'timeout_ms' in connect_config:
            timeout = connect_config['timeout_ms']
            if isinstance(timeout, dict):
                print(f'  連接超時: router={timeout.get(\"router\", \"N/A\")}, peer={timeout.get(\"peer\", \"N/A\")}, client={timeout.get(\"client\", \"N/A\")}')
            else:
                print(f'  連接超時: {timeout}ms')
        
        if 'retry' in connect_config:
            retry = connect_config['retry']
            print('  重試設定:')
            if 'period_init_ms' in retry:
                print(f'    初始等待: {retry[\"period_init_ms\"]}ms')
            if 'period_max_ms' in retry:
                print(f'    最大等待: {retry[\"period_max_ms\"]}ms')
            if 'period_increase_factor' in retry:
                print(f'    增長因子: {retry[\"period_increase_factor\"]}')
        print()

except Exception as e:
    print(f'❌ 讀取配置檔案時發生錯誤: {e}')
"
}

show_zenoh_details() {
    log_info "Zenoh Router 詳細配置"
    echo "========================="
    echo ""
    
    echo "📁 檔案路徑: $ZENOH_CONFIG_FILE"
    echo "📏 檔案大小: $(wc -l < "$ZENOH_CONFIG_FILE") 行"
    echo ""
    
    # 顯示配置檔案的關鍵部分
    echo -e "${CYAN}🔍 配置檔案內容預覽:${NC}"
    echo "===================="
    
    # 顯示模式配置
    echo ""
    echo -e "${BLUE}📡 運行模式:${NC}"
    grep -E "mode:" "$ZENOH_CONFIG_FILE" | sed 's/^/  /' | head -1
    
    # 顯示連接端點
    echo ""
    echo -e "${BLUE}🔗 連接端點:${NC}"
    sed -n '/endpoints: \[/,/\]/p' "$ZENOH_CONFIG_FILE" | grep -E '(tcp|udp)/' | head -10 | sed 's/^/  /'
    
    # 顯示監聽配置
    echo ""
    echo -e "${BLUE}👂 監聽配置:${NC}"
    sed -n '/listen: {/,/}/p' "$ZENOH_CONFIG_FILE" | grep -v '^\s*//\|^\s*$' | head -10 | sed 's/^/  /'
    
    # 顯示重試配置
    echo ""
    echo -e "${BLUE}🔄 重試配置:${NC}"
    sed -n '/retry: {/,/}/p' "$ZENOH_CONFIG_FILE" | grep -v '^\s*//\|^\s*$' | sed 's/^/  /'
    
    echo ""
    log_info "提示: 使用 '$0 edit' 來編輯完整配置"
}

edit_zenoh_config() {
    log_info "編輯 Zenoh Router 配置"
    echo "========================="
    echo ""
    
    echo -e "${YELLOW}⚠️ 重要提醒:${NC}"
    echo "============="
    echo "1. Zenoh 配置變更會影響整個 ROS 2 通訊網路"
    echo "2. 不正確的配置可能導致 AGV 和 AGVC 無法通訊"
    echo "3. 建議在修改前先備份原始配置檔案"
    echo "4. 配置變更後需要重啟所有相關容器"
    echo "5. 配置檔案使用 JSON5 格式 (支援註釋的 JSON)"
    echo ""
    
    # 備份原始檔案
    local backup_file="${ZENOH_CONFIG_FILE}.backup.$(date +%Y%m%d_%H%M%S)"
    cp "$ZENOH_CONFIG_FILE" "$backup_file"
    log_success "已備份原始檔案: $backup_file"
    echo ""
    
    # 顯示當前重要配置
    echo -e "${CYAN}📊 當前重要配置:${NC}"
    echo "================"
    echo "運行模式:"
    grep -E "mode:" "$ZENOH_CONFIG_FILE" | sed 's/^/  /'
    echo ""
    echo "連接端點 (前5個):"
    sed -n '/endpoints: \[/,/\]/p' "$ZENOH_CONFIG_FILE" | grep -E 'tcp/' | head -5 | sed 's/^/  /'
    echo ""
    
    # 使用編輯器開啟檔案
    log_info "正在開啟編輯器..."
    echo "檔案格式: JSON5 (支援註釋的 JSON)"
    echo ""
    
    ${EDITOR:-nano} "$ZENOH_CONFIG_FILE"
    
    echo ""
    log_success "編輯完成！"
    
    # 驗證配置檔案
    if validate_zenoh_config; then
        log_success "配置檔案驗證通過"
        show_post_edit_suggestions
    else
        log_error "配置檔案驗證失敗"
        echo ""
        log_info "可以還原備份檔案:"
        echo "  cp '$backup_file' '$ZENOH_CONFIG_FILE'"
        return 1
    fi
}

validate_zenoh_config() {
    log_info "驗證 Zenoh 配置檔案"
    echo "======================"
    echo ""
    
    # 使用 json5 工具驗證格式
    if ! command -v json5 &> /dev/null; then
        log_error "json5 工具未安裝，無法驗證 JSON5 配置檔案"
        log_info "請安裝: npm install -g json5"
        return 1
    fi
    
    python3 -c "
import json
import socket
import subprocess
import sys

def validate_endpoint(endpoint):
    \"\"\"驗證端點格式\"\"\"
    try:
        if endpoint.startswith('tcp/'):
            addr_port = endpoint[4:]  # 移除 'tcp/' 前綴
            if ':' in addr_port:
                addr_part, port_part = addr_port.rsplit(':', 1)
                # 處理可能的 interface 參數 (e.g., tcp/192.168.0.1:7447#iface=eth0)
                if '#' in port_part:
                    port_part = port_part.split('#')[0]
                
                try:
                    port = int(port_part)
                    if 1 <= port <= 65535:
                        return True, ''
                    else:
                        return False, f'端口超出範圍: {port}'
                except ValueError:
                    return False, f'無效的端口號: {port_part}'
            else:
                return False, '缺少端口號'
        elif endpoint.startswith('udp/'):
            # 類似 TCP 驗證
            return True, ''
        else:
            return False, f'不支援的協議: {endpoint}'
    except Exception as e:
        return False, str(e)

try:
    print('🔧 解析 JSON5 格式...')
    
    # 使用 json5 工具驗證和解析
    result = subprocess.run(['json5', '--validate', '$ZENOH_CONFIG_FILE'], capture_output=True, text=True)
    if result.returncode != 0:
        print(f'❌ JSON5 格式錯誤: {result.stderr}')
        sys.exit(1)
    
    # 解析為 JSON
    result = subprocess.run(['json5', '$ZENOH_CONFIG_FILE'], capture_output=True, text=True)
    if result.returncode != 0:
        print('❌ 無法解析 JSON5 檔案')
        sys.exit(1)
    
    config = json.loads(result.stdout)
    print('✅ JSON5 格式檢查通過')
    
    # 驗證必要欄位
    errors = []
    warnings = []
    
    # 檢查模式
    if 'mode' not in config:
        errors.append('缺少必要欄位: mode')
    else:
        mode = config['mode']
        if mode not in ['router', 'peer', 'client']:
            errors.append(f'無效的模式: {mode}')
    
    # 檢查連接端點
    if 'connect' in config and 'endpoints' in config['connect']:
        endpoints = config['connect']['endpoints']
        print('🔗 驗證連接端點...')
        
        for endpoint in endpoints:
            valid, error_msg = validate_endpoint(endpoint)
            if not valid:
                errors.append(f'無效的連接端點 {endpoint}: {error_msg}')
            else:
                print(f'  ✅ {endpoint}')
    
    # 檢查監聽端點
    if 'listen' in config and 'endpoints' in config['listen']:
        endpoints = config['listen']['endpoints']
        print('👂 驗證監聽端點...')
        
        for endpoint in endpoints:
            valid, error_msg = validate_endpoint(endpoint)
            if not valid:
                errors.append(f'無效的監聽端點 {endpoint}: {error_msg}')
            else:
                print(f'  ✅ {endpoint}')
    
    # 檢查網路連接性 (僅對連接端點，最多前3個避免太慢)
    if 'connect' in config and 'endpoints' in config['connect']:
        print('🌐 檢查網路連接性...')
        endpoints = config['connect']['endpoints']
        
        for endpoint in endpoints[:3]:  # 只檢查前3個端點
            if endpoint.startswith('tcp/'):
                addr_port = endpoint[4:]
                if ':' in addr_port:
                    addr_part, port_part = addr_port.rsplit(':', 1)
                    # 處理可能的 interface 參數
                    if '#' in port_part:
                        port_part = port_part.split('#')[0]
                    
                    try:
                        port = int(port_part)
                        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        sock.settimeout(2)
                        result = sock.connect_ex((addr_part, port))
                        sock.close()
                        
                        if result == 0:
                            print(f'  ✅ {endpoint} (可連接)')
                        else:
                            warnings.append(f'無法連接到 {endpoint}')
                            print(f'  ⚠️ {endpoint} (無法連接)')
                    except Exception as e:
                        warnings.append(f'連接測試失敗 {endpoint}: {str(e)}')
                        print(f'  ⚠️ {endpoint} (測試失敗)')
    
    # 顯示結果
    print()
    if errors:
        print('❌ 發現錯誤:')
        for error in errors:
            print(f'   • {error}')
        print()
        exit(1)
    
    if warnings:
        print('⚠️ 警告:')
        for warning in warnings:
            print(f'   • {warning}')
        print()
    
    if not errors and not warnings:
        print('✅ Zenoh 配置檔案驗證通過')
    else:
        print('✅ Zenoh 配置檔案格式正確 (有警告)')

except Exception as e:
    print(f'❌ 驗證過程發生錯誤: {e}')
    exit(1)
"
}

check_zenoh_status() {
    log_info "檢查 Zenoh Router 服務狀態"
    echo "============================="
    echo ""
    
    # 檢查進程是否運行 (宿主機上無法直接檢查，提供指令)
    echo -e "${BLUE}🔍 服務狀態檢查:${NC}"
    echo "================="
    echo "⚠️ 注意: Zenoh Router 運行在 Docker 容器內"
    echo ""
    
    echo "檢查 AGV 容器中的 Zenoh Router:"
    echo "  docker compose -f docker-compose.yml exec rosagv bash -c 'pgrep -f rmw_zenohd || echo \"未運行\"'"
    echo ""
    echo "檢查 AGVC 容器中的 Zenoh Router:"
    echo "  docker compose -f docker-compose.agvc.yml exec agvc_server bash -c 'pgrep -f rmw_zenohd || echo \"未運行\"'"
    echo ""
    
    # 檢查端口是否開放 (從宿主機角度)
    echo -e "${BLUE}🌐 網路連接檢查:${NC}"
    echo "================="
    
    # 檢查是否有程序在監聽 7447 端口
    if ss -tuln 2>/dev/null | grep ":7447 " > /dev/null; then
        echo "✅ 端口 7447 已開放 (有服務在監聽)"
        ss -tuln | grep ":7447 " | sed 's/^/  /'
    else
        echo "❌ 端口 7447 未開放"
    fi
    echo ""
    
    # 檢查配置檔案中的端點連接性
    if [ -f "$ZENOH_CONFIG_FILE" ]; then
        echo -e "${BLUE}📡 配置端點連接性:${NC}"
        echo "==================="
        echo "正在檢查配置檔案中的端點..."
        
        # 提取端點並測試連接
        grep -E 'tcp/.*:7447' "$ZENOH_CONFIG_FILE" | grep -v '^\s*//' | head -3 | while read -r line; do
            endpoint=$(echo "$line" | grep -oE 'tcp/[^"]*' | head -1)
            if [ -n "$endpoint" ]; then
                addr_port=${endpoint#tcp/}
                addr=${addr_port%:*}
                port=${addr_port#*:}
                
                # 去除可能的 interface 參數
                port=${port%%#*}
                
                if timeout 2 bash -c "echo > /dev/tcp/$addr/$port" 2>/dev/null; then
                    echo "  ✅ $endpoint (可連接)"
                else
                    echo "  ❌ $endpoint (無法連接)"
                fi
            fi
        done
    fi
    echo ""
    
    echo -e "${PURPLE}💡 詳細檢查指令:${NC}"
    echo "================="
    echo "在 AGV 容器內檢查:"
    echo "  docker compose -f docker-compose.yml exec rosagv bash"
    echo "  source /app/setup.bash && check_zenoh_status"
    echo ""
    echo "在 AGVC 容器內檢查:"
    echo "  docker compose -f docker-compose.agvc.yml exec agvc_server bash"  
    echo "  source /app/setup.bash && check_zenoh_status"
}

show_restart_guide() {
    log_info "Zenoh Router 服務重啟指南"
    echo "============================"
    echo ""
    
    echo -e "${YELLOW}⚠️ 重要提醒:${NC}"
    echo "============"
    echo "1. 重啟 Zenoh 服務會暫時中斷 ROS 2 通訊"
    echo "2. 需要在 AGV 和 AGVC 容器內分別執行"
    echo "3. 建議先驗證配置檔案: $0 validate"
    echo ""
    
    echo -e "${CYAN}🔧 重啟步驟:${NC}"
    echo "==========="
    echo ""
    echo "1. 在 AGV 容器內重啟:"
    echo "   docker compose -f docker-compose.yml exec rosagv bash"
    echo "   # 在容器內執行:"
    echo "   source /app/setup.bash"
    echo "   manage_zenoh restart"
    echo ""
    echo "2. 在 AGVC 容器內重啟:"
    echo "   docker compose -f docker-compose.agvc.yml exec agvc_server bash"
    echo "   # 在容器內執行:"
    echo "   source /app/setup.bash"
    echo "   manage_zenoh restart"
    echo ""
    echo "3. 檢查服務狀態:"
    echo "   $0 status"
    echo ""
    echo "4. 測試 ROS 2 通訊:"
    echo "   # 在任一容器內執行:"
    echo "   ros2 node list"
    echo "   ros2 topic list"
    echo ""
    
    echo -e "${PURPLE}💡 故障排除:${NC}"
    echo "============"
    echo "如果遇到問題，可以檢查："
    echo "- 配置檔案: $0 validate"
    echo "- 網路連接: ping <目標IP>"
    echo "- 端口佔用: ss -tuln | grep 7447"
    echo "- 服務日誌: tail -f /tmp/zenoh_router.log (容器內)"
    echo "- 強制重啟容器:"
    echo "  docker compose -f docker-compose.yml restart rosagv"
    echo "  docker compose -f docker-compose.agvc.yml restart agvc_server"
}

show_post_edit_suggestions() {
    echo ""
    echo -e "${CYAN}📋 配置變更後的後續步驟:${NC}"
    echo "=========================="
    echo "1. 驗證配置檔案:"
    echo "   $0 validate"
    echo ""
    echo "2. 重啟 Zenoh 服務:"
    echo "   $0 restart"
    echo ""
    echo "3. 檢查服務狀態:"
    echo "   $0 status"
    echo ""
    echo "4. 測試系統通訊:"
    echo "   # 在容器內測試 ROS 2 通訊是否正常"
    echo "   docker compose -f docker-compose.yml exec rosagv bash"
    echo "   ros2 node list"
    echo ""
    echo "5. 監控系統日誌:"
    echo "   docker compose -f docker-compose.yml logs -f rosagv"
    echo "   docker compose -f docker-compose.agvc.yml logs -f agvc_server"
}

# ============================================================================
# 主程式
# ============================================================================

main() {
    show_header
    
    # 參數處理
    local action="${1:-overview}"
    
    case "$action" in
        -h|--help)
            show_usage
            exit 0
            ;;
        show|details)
            check_dependencies || exit 1
            show_zenoh_details
            ;;
        edit)
            check_dependencies || exit 1
            edit_zenoh_config
            ;;
        validate|check)
            check_dependencies || exit 1
            validate_zenoh_config
            ;;
        status)
            check_zenoh_status
            ;;
        restart)
            show_restart_guide
            ;;
        overview|"")
            check_dependencies || exit 1
            show_zenoh_overview
            echo ""
            log_info "使用說明:"
            echo "  $0 show      # 顯示詳細配置"
            echo "  $0 edit      # 編輯配置檔案"
            echo "  $0 validate  # 驗證配置檔案"
            echo "  $0 status    # 檢查服務狀態"
            echo "  $0 restart   # 重啟服務指南"
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
