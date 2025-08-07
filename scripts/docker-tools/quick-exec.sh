#!/bin/bash
# RosAGV 容器內快速命令執行工具
# 版本: 1.0
# 說明: 在指定容器內執行命令，自動載入對應環境，支援常用 ROS 2 命令快捷方式

# ============================================================================
# 初始化和設定
# ============================================================================

# 獲取腳本目錄和專案根目錄
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Docker Compose 檔案路徑
AGV_COMPOSE_FILE="$PROJECT_ROOT/docker-compose.yml"
AGVC_COMPOSE_FILE="$PROJECT_ROOT/docker-compose.agvc.yml"

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# ============================================================================
# 輔助函數
# ============================================================================

show_header() {
    echo -e "${CYAN}⚡ RosAGV 容器快速執行工具${NC}"
    echo -e "${CYAN}=============================${NC}"
    echo ""
}

show_help() {
    show_header
    echo -e "${YELLOW}用法:${NC}"
    echo "  $(basename $0) [容器] [命令或快捷方式]"
    echo ""
    echo -e "${YELLOW}容器選項:${NC}"
    echo -e "  ${GREEN}agv${NC}   - AGV 容器 (rosagv)"
    echo -e "  ${GREEN}agvc${NC}  - AGVC 容器 (agvc_server)"
    echo ""
    echo -e "${YELLOW}快捷方式:${NC}"
    echo -e "  ${GREEN}node-list${NC}      - ros2 node list"
    echo -e "  ${GREEN}topic-list${NC}     - ros2 topic list"
    echo -e "  ${GREEN}service-list${NC}   - ros2 service list"
    echo -e "  ${GREEN}topic-echo${NC}     - ros2 topic echo <topic>"
    echo -e "  ${GREEN}topic-info${NC}     - ros2 topic info <topic>"
    echo -e "  ${GREEN}node-info${NC}      - ros2 node info <node>"
    echo -e "  ${GREEN}check-status${NC}   - check_system_status"
    echo -e "  ${GREEN}check-zenoh${NC}    - check_zenoh_status"
    echo -e "  ${GREEN}check-agvc${NC}     - check_agvc_status (僅 AGVC)"
    echo -e "  ${GREEN}build-all${NC}      - build_all"
    echo -e "  ${GREEN}build-ws${NC}       - build_ws <workspace>"
    echo -e "  ${GREEN}test-all${NC}       - test_all"
    echo -e "  ${GREEN}clean-all${NC}      - clean_all"
    echo ""
    echo -e "${YELLOW}直接命令:${NC}"
    echo "  任何其他命令將直接在容器內執行"
    echo ""
    echo -e "${YELLOW}範例:${NC}"
    echo "  $(basename $0) agv node-list"
    echo "  $(basename $0) agvc check-status"
    echo "  $(basename $0) agv \"ros2 topic echo /agv_state\""
    echo "  $(basename $0) agvc \"cd /app/web_api_ws && colcon build\""
    echo ""
    echo -e "${YELLOW}批量執行:${NC}"
    echo "  使用分號分隔多個命令:"
    echo "  $(basename $0) agv \"node-list; topic-list; check-status\""
}

print_status() {
    local icon="$1"
    local color="$2"
    local message="$3"
    echo -e "${color}${icon} ${message}${NC}"
}

print_success() {
    print_status "✅" "$GREEN" "$1"
}

print_error() {
    print_status "❌" "$RED" "$1"
}

print_warning() {
    print_status "⚠️" "$YELLOW" "$1"
}

print_info() {
    print_status "ℹ️" "$BLUE" "$1"
}

# ============================================================================
# 容器檢查函數
# ============================================================================

check_docker_running() {
    if ! docker info >/dev/null 2>&1; then
        print_error "Docker 服務未運行"
        return 1
    fi
    return 0
}

is_container_running() {
    local container_name="$1"
    docker inspect "$container_name" --format '{{.State.Running}}' 2>/dev/null | grep -q "true"
}

# ============================================================================
# 命令轉換函數
# ============================================================================

translate_shortcut() {
    local shortcut="$1"
    shift
    local args="$@"
    
    case "$shortcut" in
        # ROS 2 基本命令
        "node-list")
            echo "ros2 node list"
            ;;
        "topic-list")
            echo "ros2 topic list"
            ;;
        "service-list")
            echo "ros2 service list"
            ;;
        "topic-echo")
            if [ -n "$args" ]; then
                echo "ros2 topic echo $args"
            else
                echo "echo '錯誤: topic-echo 需要主題名稱'; exit 1"
            fi
            ;;
        "topic-info")
            if [ -n "$args" ]; then
                echo "ros2 topic info $args"
            else
                echo "echo '錯誤: topic-info 需要主題名稱'; exit 1"
            fi
            ;;
        "node-info")
            if [ -n "$args" ]; then
                echo "ros2 node info $args"
            else
                echo "echo '錯誤: node-info 需要節點名稱'; exit 1"
            fi
            ;;
        # 系統檢查命令
        "check-status")
            echo "check_system_status"
            ;;
        "check-zenoh")
            echo "check_zenoh_status"
            ;;
        "check-agvc")
            echo "check_agvc_status"
            ;;
        # 構建命令
        "build-all")
            echo "build_all"
            ;;
        "build-ws")
            if [ -n "$args" ]; then
                echo "build_ws $args"
            else
                echo "echo '錯誤: build-ws 需要工作空間名稱'; exit 1"
            fi
            ;;
        "test-all")
            echo "test_all"
            ;;
        "clean-all")
            echo "clean_all"
            ;;
        *)
            # 不是快捷方式，返回原始命令
            echo "$shortcut $args"
            ;;
    esac
}

# ============================================================================
# 執行函數
# ============================================================================

execute_in_agv() {
    local command="$1"
    
    if ! is_container_running "rosagv"; then
        print_error "AGV 容器未運行"
        print_info "請先使用 'scripts/docker-tools/agv-container.sh start' 啟動容器"
        return 1
    fi
    
    print_info "在 AGV 容器中執行命令..."
    
    # 建立執行腳本
    local exec_script="
source /app/setup.bash && agv_source >/dev/null 2>&1
export PS1=''  # 清除提示符避免輸出混亂

# 處理多個命令
IFS=';' read -ra COMMANDS <<< \"$command\"
for cmd in \"\${COMMANDS[@]}\"; do
    cmd=\$(echo \"\$cmd\" | xargs)  # 去除前後空格
    if [ -n \"\$cmd\" ]; then
        echo -e '\033[0;36m➜ 執行: \$cmd\033[0m'
        # 轉換快捷方式
        actual_cmd=\$($(declare -f translate_shortcut); translate_shortcut \$cmd)
        eval \"\$actual_cmd\"
        if [ \$? -ne 0 ]; then
            echo -e '\033[0;31m✗ 命令執行失敗\033[0m'
        fi
        echo ''
    fi
done
"
    
    docker exec -it "rosagv" bash -c "$exec_script"
}

execute_in_agvc() {
    local command="$1"
    
    if ! is_container_running "agvc_server"; then
        print_error "AGVC 容器未運行"
        print_info "請先使用 'scripts/docker-tools/agvc-container.sh start' 啟動容器"
        return 1
    fi
    
    print_info "在 AGVC 容器中執行命令..."
    
    # 建立執行腳本
    local exec_script="
source /app/setup.bash && agvc_source >/dev/null 2>&1
export PS1=''  # 清除提示符避免輸出混亂

# 處理多個命令
IFS=';' read -ra COMMANDS <<< \"$command\"
for cmd in \"\${COMMANDS[@]}\"; do
    cmd=\$(echo \"\$cmd\" | xargs)  # 去除前後空格
    if [ -n \"\$cmd\" ]; then
        echo -e '\033[0;36m➜ 執行: \$cmd\033[0m'
        # 轉換快捷方式
        actual_cmd=\$($(declare -f translate_shortcut); translate_shortcut \$cmd)
        eval \"\$actual_cmd\"
        if [ \$? -ne 0 ]; then
            echo -e '\033[0;31m✗ 命令執行失敗\033[0m'
        fi
        echo ''
    fi
done
"
    
    docker exec -it "agvc_server" bash -c "$exec_script"
}

# ============================================================================
# 快速執行包裝函數
# ============================================================================

# 便捷函數：在 AGV 容器執行
quick_agv() {
    execute_in_agv "$*"
}

# 便捷函數：在 AGVC 容器執行
quick_agvc() {
    execute_in_agvc "$*"
}

# ============================================================================
# 交互式模式
# ============================================================================

interactive_mode() {
    show_header
    echo -e "${YELLOW}進入交互式模式${NC}"
    echo -e "輸入 'help' 查看可用命令，'exit' 退出"
    echo ""
    
    while true; do
        echo -ne "${CYAN}quick-exec> ${NC}"
        read -r input
        
        case "$input" in
            "exit"|"quit")
                print_info "退出交互式模式"
                break
                ;;
            "help")
                show_help
                ;;
            "")
                continue
                ;;
            *)
                # 解析容器和命令
                local container=$(echo "$input" | awk '{print $1}')
                local command=$(echo "$input" | cut -d' ' -f2-)
                
                case "$container" in
                    "agv")
                        execute_in_agv "$command"
                        ;;
                    "agvc")
                        execute_in_agvc "$command"
                        ;;
                    *)
                        print_error "未知容器: $container"
                        print_info "請使用 'agv' 或 'agvc'"
                        ;;
                esac
                ;;
        esac
        echo ""
    done
}

# ============================================================================
# 主程式邏輯
# ============================================================================

main() {
    # 檢查基本依賴
    if ! check_docker_running; then
        exit 1
    fi
    
    # 無參數時進入交互式模式
    if [ $# -eq 0 ]; then
        interactive_mode
        return 0
    fi
    
    local container="$1"
    shift
    local command="$*"
    
    # 檢查是否請求幫助
    if [[ "$container" == "help" || "$container" == "-h" || "$container" == "--help" ]]; then
        show_help
        return 0
    fi
    
    # 如果只有一個參數，可能是請求容器狀態
    if [ -z "$command" ]; then
        print_error "缺少要執行的命令"
        echo ""
        show_help
        return 1
    fi
    
    # 執行命令
    case "$container" in
        "agv")
            execute_in_agv "$command"
            ;;
        "agvc")
            execute_in_agvc "$command"
            ;;
        *)
            print_error "未知容器: $container"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# 如果腳本被直接執行（而非被 source）
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi

# 導出便捷函數供 source 使用
export -f quick_agv
export -f quick_agvc
