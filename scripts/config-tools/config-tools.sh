#!/bin/bash
# RosAGV 配置工具函數集合
# 版本: 1.0
# 說明: 統一的配置管理工具函數集，可以一次載入所有配置工具功能

# ============================================================================
# 初始化和路徑設定
# ============================================================================

# 獲取腳本目錄
CONFIG_TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 檢查並載入所有配置工具腳本
if [ -f "$CONFIG_TOOLS_DIR/edit-agv-config.sh" ]; then
    source "$CONFIG_TOOLS_DIR/edit-agv-config.sh"
fi

if [ -f "$CONFIG_TOOLS_DIR/edit-agvc-config.sh" ]; then
    source "$CONFIG_TOOLS_DIR/edit-agvc-config.sh"
fi

if [ -f "$CONFIG_TOOLS_DIR/hardware-mapping.sh" ]; then
    source "$CONFIG_TOOLS_DIR/hardware-mapping.sh"
fi

if [ -f "$CONFIG_TOOLS_DIR/zenoh-config.sh" ]; then
    source "$CONFIG_TOOLS_DIR/zenoh-config.sh"
fi

# ============================================================================
# 統一的配置工具界面
# ============================================================================

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

show_config_tools_header() {
    echo -e "${CYAN}🔧 RosAGV 配置管理工具套件${NC}"
    echo -e "${CYAN}==============================${NC}"
    echo ""
}

show_config_tools_help() {
    show_config_tools_header
    
    echo -e "${BLUE}📋 可用的配置工具:${NC}"
    echo "=================="
    echo ""
    
    echo -e "${YELLOW}🚗 AGV 配置管理:${NC}"
    echo "  edit_agv_config <agv_id>     # 編輯 AGV 配置"
    echo "  list_agv_configs             # 列出所有 AGV 配置"
    echo "  validate_agv_config <file>   # 驗證 AGV 配置檔案"
    echo ""
    
    echo -e "${YELLOW}🖥️ AGVC 配置管理:${NC}"
    echo "  edit_agvc_config <type>      # 編輯 AGVC 配置"
    echo "  list_agvc_configs            # 列出所有 AGVC 配置"
    echo "  validate_agvc_config <file>  # 驗證 AGVC 配置檔案"
    echo ""
    
    echo -e "${YELLOW}🔧 硬體映射管理:${NC}"
    echo "  show_hardware_overview       # 顯示硬體映射概況"
    echo "  list_all_devices             # 列出所有設備詳情"
    echo "  show_device_details <id>     # 顯示特定設備詳情"
    echo "  edit_device_config <id>      # 編輯設備配置"
    echo "  manage_mac_addresses <id>    # 管理 MAC 地址"
    echo "  validate_hardware_config     # 驗證硬體映射配置"
    echo ""
    
    echo -e "${YELLOW}🌐 Zenoh 配置管理:${NC}"
    echo "  show_zenoh_overview          # 顯示 Zenoh 配置概況"
    echo "  show_zenoh_details           # 顯示詳細 Zenoh 配置"
    echo "  edit_zenoh_config            # 編輯 Zenoh 配置"
    echo "  validate_zenoh_config        # 驗證 Zenoh 配置"
    echo "  check_zenoh_status           # 檢查 Zenoh 服務狀態"
    echo ""
    
    echo -e "${PURPLE}💡 使用方式:${NC}"
    echo "==========="
    echo "1. 載入所有工具函數:"
    echo "   source ./scripts/config-tools/config-tools.sh"
    echo ""
    echo "2. 使用個別工具函數:"
    echo "   list_agv_configs"
    echo "   edit_agv_config cargo01"
    echo "   show_hardware_overview"
    echo "   validate_zenoh_config"
    echo ""
    echo "3. 或者直接執行個別腳本:"
    echo "   ./scripts/config-tools/edit-agv-config.sh list"
    echo "   ./scripts/config-tools/hardware-mapping.sh show cargo01"
    echo ""
    
    echo -e "${GREEN}✅ 配置工具已載入並可使用！${NC}"
}

# 便捷別名函數
agv_config() {
    if [ $# -eq 0 ]; then
        list_agv_configs
    else
        edit_agv_config "$1"
    fi
}

agvc_config() {
    if [ $# -eq 0 ]; then
        list_agvc_configs
    else
        edit_agvc_config "$1"
    fi
}

hardware_config() {
    local action="${1:-overview}"
    local device_id="$2"
    
    case "$action" in
        list|ls)
            list_all_devices
            ;;
        show)
            show_device_details "$device_id"
            ;;
        edit)
            edit_device_config "$device_id"
            ;;
        mac)
            manage_mac_addresses "$device_id"
            ;;
        validate)
            validate_hardware_config
            ;;
        *)
            show_hardware_overview
            ;;
    esac
}

zenoh_config() {
    local action="${1:-overview}"
    
    case "$action" in
        show)
            show_zenoh_details
            ;;
        edit)
            edit_zenoh_config
            ;;
        validate)
            validate_zenoh_config
            ;;
        status)
            check_zenoh_status
            ;;
        restart)
            show_restart_guide
            ;;
        *)
            show_zenoh_overview
            ;;
    esac
}

# 配置工具快速驗證
validate_all_configs() {
    echo -e "${CYAN}🔍 驗證所有配置檔案${NC}"
    echo "======================="
    echo ""
    
    local errors=0
    
    echo -e "${BLUE}1. 驗證硬體映射配置...${NC}"
    if validate_hardware_config; then
        echo -e "${GREEN}✅ 硬體映射配置驗證通過${NC}"
    else
        echo -e "${RED}❌ 硬體映射配置驗證失敗${NC}"
        ((errors++))
    fi
    echo ""
    
    echo -e "${BLUE}2. 驗證 Zenoh 配置...${NC}"
    if validate_zenoh_config; then
        echo -e "${GREEN}✅ Zenoh 配置驗證通過${NC}"
    else
        echo -e "${RED}❌ Zenoh 配置驗證失敗${NC}"
        ((errors++))
    fi
    echo ""
    
    echo -e "${BLUE}3. 檢查 AGV 配置檔案...${NC}"
    local agv_configs=(/home/ct/RosAGV/app/config/agv/*_config.yaml)
    local agv_errors=0
    for config_file in "${agv_configs[@]}"; do
        if [ -f "$config_file" ] && [[ "$(basename "$config_file")" != "base_config.yaml" ]]; then
            if validate_config "$config_file"; then
                echo -e "${GREEN}✅ $(basename "$config_file")${NC}"
            else
                echo -e "${RED}❌ $(basename "$config_file")${NC}"
                ((agv_errors++))
            fi
        fi
    done
    
    if [ $agv_errors -eq 0 ]; then
        echo -e "${GREEN}✅ 所有 AGV 配置驗證通過${NC}"
    else
        echo -e "${RED}❌ $agv_errors 個 AGV 配置檔案有問題${NC}"
        ((errors++))
    fi
    echo ""
    
    # 總結
    if [ $errors -eq 0 ]; then
        echo -e "${GREEN}🎉 所有配置檔案驗證通過！${NC}"
        return 0
    else
        echo -e "${RED}⚠️ 發現 $errors 個配置問題，請檢查並修正${NC}"
        return 1
    fi
}

# 配置工具狀態概覽
config_status_overview() {
    show_config_tools_header
    
    echo -e "${BLUE}📊 配置檔案狀態概覽${NC}"
    echo "===================="
    echo ""
    
    # 檢查各配置檔案是否存在
    local config_files=(
        "/home/ct/RosAGV/app/config/hardware_mapping.yaml:硬體映射配置"
        "/home/ct/RosAGV/app/routerconfig.json5:Zenoh 路由配置"
        "/home/ct/RosAGV/app/config/agvc/agvc01_config.yaml:AGVC 主節點配置" 
        "/home/ct/RosAGV/app/config/web_api_config.yaml:Web API 配置"
        "/home/ct/RosAGV/app/config/ecs_config.yaml:ECS 設備控制配置" 
    )
    
    for config_info in "${config_files[@]}"; do
        local config_path="${config_info%%:*}"
        local config_desc="${config_info##*:}"
        
        if [ -f "$config_path" ]; then
            local size=$(du -h "$config_path" | cut -f1)
            local mtime=$(stat -c %y "$config_path" 2>/dev/null | cut -d' ' -f1)
            echo -e "${GREEN}✅${NC} $config_desc"
            echo "    檔案: $config_path"
            echo "    大小: $size | 修改時間: $mtime"
        else
            echo -e "${RED}❌${NC} $config_desc"
            echo "    檔案不存在: $config_path"
        fi
        echo ""
    done
    
    # 統計 AGV 配置檔案數量
    local agv_config_count=$(ls /home/ct/RosAGV/app/config/agv/*_config.yaml 2>/dev/null | grep -v base_config | wc -l)
    echo -e "${CYAN}📱 AGV 配置檔案: $agv_config_count 個${NC}"
    
    # 顯示可用的工具
    echo ""
    echo -e "${PURPLE}🔧 可用工具:${NC}"
    echo "  config_tools_help    # 顯示詳細說明"
    echo "  validate_all_configs # 驗證所有配置"
    echo "  agv_config [id]      # AGV 配置快速操作"
    echo "  agvc_config [type]   # AGVC 配置快速操作"
    echo "  hardware_config [action] [id]  # 硬體配置操作"
    echo "  zenoh_config [action]          # Zenoh 配置操作"
}

# 設定別名
alias config_tools_help='show_config_tools_help'
alias config_status='config_status_overview'

# ============================================================================
# 主程式 (當直接執行時)
# ============================================================================

main() {
    if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
        # 作為腳本直接執行
        case "${1:-help}" in
            help|--help|-h)
                show_config_tools_help
                ;;
            status)
                config_status_overview
                ;;
            validate)
                validate_all_configs
                ;;
            *)
                echo "RosAGV 配置工具套件"
                echo ""
                echo "使用方式:"
                echo "  source $0           # 載入所有工具函數"
                echo "  $0 help             # 顯示幫助資訊"
                echo "  $0 status           # 顯示配置狀態概覽"
                echo "  $0 validate         # 驗證所有配置檔案"
                ;;
        esac
    else
        # 被 source 載入
        config_status_overview
    fi
}

# 當作為腳本執行時調用主程式
main "$@"
