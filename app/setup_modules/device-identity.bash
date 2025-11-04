#!/bin/bash
# RosAGV Device Identity Module
# 包含設備識別、MAC 地址檢測和配置生成函數

# ============================================================================
# 設備身份識別函數
# ============================================================================

check_device_identity() {
    echo "╔══════════════════════════════════════════════════════════════════════════════╗"
    echo "║ 統一設備身份資訊"
    echo "╚══════════════════════════════════════════════════════════════════════════════╝"

    if [ -f "/app/.device_identity" ]; then
        source /app/.device_identity
        log_success "設備身份已載入: $DEVICE_ID ($CONTAINER_TYPE)"

        echo "🔧 統一設備資訊:"
        echo "  設備 ID: ${DEVICE_ID:-未設定}"
        echo "  容器類型: ${CONTAINER_TYPE:-未設定}"
        echo "  主要 MAC: ${PRIMARY_MAC:-未設定}"
        echo "  識別時間: ${IDENTIFICATION_TIME:-未知}"
        echo "  識別狀態: $([ "$IDENTIFICATION_SUCCESS" = "0" ] && echo "✅ 成功" || echo "❌ 失敗")"
        echo "  識別方法: ${IDENTIFICATION_METHOD:-未知}"

        # 根據容器類型顯示專屬資訊
        case "$CONTAINER_TYPE" in
            "agv")
                if [ -f "/app/.agv_identity" ]; then
                    source /app/.agv_identity
                    echo ""
                    echo "🚗 AGV 專屬資訊:"
                    echo "  AGV ID: ${AGV_ID:-未設定}"
                    echo "  AGV 類型: ${AGV_TYPE:-未設定}"
                    echo "  ROS 命名空間: ${ROS_NAMESPACE:-未設定}"
                    echo "  啟動套件: ${AGV_LAUNCH_PACKAGE:-未設定}"
                    echo "  配置檔案: ${DEVICE_CONFIG_FILE:-未設定}"
                fi
                ;;
            "agvc")
                if [ -f "/app/.agvc_identity" ]; then
                    source /app/.agvc_identity
                    echo ""
                    echo "🖥️ AGVC 專屬資訊:"
                    echo "  AGVC ID: ${AGVC_ID:-未設定}"
                    echo "  AGVC 類型: ${AGVC_TYPE:-未設定}"
                    echo "  AGVC 角色: ${AGVC_ROLE:-未設定}"
                    echo "  ROS 命名空間: ${ROS_NAMESPACE:-未設定}"
                    echo "  配置檔案: ${DEVICE_CONFIG_FILE:-未設定}"
                    echo "  工作空間: ${AGVC_WORKSPACES:-未設定}"
                fi
                ;;
        esac
    else
        log_warning "統一設備身份檔案不存在，請執行身份識別"
        echo "💡 執行 identify_device_manual 進行手動識別"
    fi
}

# 手動觸發統一設備身份識別
identify_device_manual() {
    log_info "手動觸發統一設備身份識別..."
    if [ -f "/app/scripts/config_driven_device_detector.bash" ]; then
        export DEVICE_DEBUG=true
        source /app/scripts/config_driven_device_detector.bash
        log_success "統一設備身份識別完成"
        check_device_identity
    else
        log_error "統一設備識別腳本不存在"
    fi
}

# 顯示設備 MAC 地址資訊和管理建議
show_device_mac_info() {
    local verbose_mode=false
    local update_config=false
    local generate_compose=false

    # 解析參數
    while [[ $# -gt 0 ]]; do
        case $1 in
            --verbose|-v)
                verbose_mode=true
                shift
                ;;
            --update-config|-u)
                update_config=true
                shift
                ;;
            --generate-compose|-g)
                generate_compose=true
                shift
                ;;
            --help|-h)
                echo "用法: show_device_mac_info [選項]"
                echo "選項:"
                echo "  --verbose, -v        顯示詳細的網路介面資訊"
                echo "  --update-config, -u  自動更新配置檔案中的 MAC 地址"
                echo "  --generate-compose, -g 生成固定 MAC 地址的 Docker Compose 配置"
                echo "  --help, -h          顯示此幫助資訊"
                return 0
                ;;
            *)
                log_warning "未知參數: $1"
                shift
                ;;
        esac
    done

    echo "╔══════════════════════════════════════════════════════════════════════════════╗"
    echo "║ 設備 MAC 地址資訊"
    echo "╚══════════════════════════════════════════════════════════════════════════════╝"

    # 讀取設備身份資訊
    local device_id=""
    local device_type=""
    local container_type=""
    local identification_method=""
    local network_mode=""

    if [ -f "/app/.device_identity" ]; then
        source /app/.device_identity
        device_id="$DEVICE_ID"
        container_type="$CONTAINER_TYPE"
        identification_method="$IDENTIFICATION_METHOD"
    else
        log_warning "設備身份檔案不存在，請先執行設備識別"
        device_id="未知"
        container_type="未知"
        identification_method="未執行"
    fi

    # 根據容器類型讀取專屬身份資訊
    case "$container_type" in
        "agv")
            if [ -f "/app/.agv_identity" ]; then
                source /app/.agv_identity
                device_type="$AGV_TYPE"
            fi
            ;;
        "agvc")
            if [ -f "/app/.agvc_identity" ]; then
                source /app/.agvc_identity
                device_type="$AGVC_TYPE"
            fi
            ;;
    esac

    # 檢測網路模式
    _detect_network_mode
    network_mode="$_NETWORK_MODE"

    echo "🔧 當前設備資訊:"
    echo "  設備 ID: ${device_id:-未知}"
    echo "  設備類型: ${device_type:-未知}"
    echo "  容器類型: ${container_type:-未知}"
    echo "  識別方法: ${identification_method:-未知}"
    echo "  網路模式: $network_mode"
    echo ""

    # 獲取實際網路介面 MAC 地址
    _get_actual_mac_addresses "$verbose_mode"

    # 獲取配置檔案中的 MAC 地址
    _get_config_mac_addresses "$device_id" "$container_type"

    # 顯示識別狀態和建議
    _show_identification_status_and_recommendations "$device_id" "$container_type" "$network_mode"

    # 執行額外功能
    if [ "$update_config" = true ]; then
        _update_config_mac_addresses "$device_id" "$container_type"
    fi

    if [ "$generate_compose" = true ]; then
        _generate_compose_config "$device_id" "$container_type"
    fi
}

# MAC 地址資訊顯示函數的簡化別名
mac_info() {
    show_device_mac_info "$@"
}

# 檢測容器網路模式
_detect_network_mode() {
    _NETWORK_MODE="unknown"

    # 檢查是否在容器內
    if [ ! -f "/.dockerenv" ]; then
        _NETWORK_MODE="host (非容器環境)"
        return 0
    fi

    # 檢查網路命名空間
    local host_net_ns=""
    local container_net_ns=""

    # 嘗試獲取宿主機網路命名空間 ID
    if [ -f "/proc/1/ns/net" ]; then
        container_net_ns=$(readlink /proc/1/ns/net 2>/dev/null)
    fi

    # 檢查是否有 host 網路模式的特徵
    # host 模式下容器會看到宿主機的所有網路介面
    local interface_count=$(ls /sys/class/net/ 2>/dev/null | wc -l)
    local docker_interfaces=$(ls /sys/class/net/ 2>/dev/null | grep -E "^(docker|br-|veth)" | wc -l)

    # 檢查是否存在典型的宿主機介面
    if ls /sys/class/net/ 2>/dev/null | grep -qE "^(enp|eth0|wlan)"; then
        # 如果有物理網路介面，可能是 host 模式
        if [ "$interface_count" -gt 3 ] || [ "$docker_interfaces" -gt 0 ]; then
            _NETWORK_MODE="host (使用宿主機網路)"
        else
            _NETWORK_MODE="bridge (容器獨立網路)"
        fi
    else
        # 只有容器內的虛擬介面
        _NETWORK_MODE="bridge (容器獨立網路)"
    fi

    # 檢查是否有 Docker 分配的 MAC 地址模式
    for interface in $(ls /sys/class/net/ 2>/dev/null); do
        if [ -f "/sys/class/net/$interface/address" ]; then
            local mac=$(cat "/sys/class/net/$interface/address" 2>/dev/null | tr '[:lower:]' '[:upper:]')
            # Docker 預設 MAC 地址模式：02:42:xx:xx:xx:xx
            if [[ "$mac" =~ ^02:42: ]]; then
                _NETWORK_MODE="bridge (容器獨立網路，動態 MAC)"
                break
            fi
        fi
    done
}

# 獲取實際網路介面 MAC 地址
_get_actual_mac_addresses() {
    local verbose_mode="$1"
    local primary_mac=""
    local primary_interface=""

    echo "📡 實際網路介面 MAC 地址:"

    # 按優先級順序檢查網路介面
    local interface_priority=("enp4s0" "eth0")
    local found_primary=false

    # 首先檢查高優先級介面
    for interface in "${interface_priority[@]}"; do
        if [ -f "/sys/class/net/$interface/address" ]; then
            local mac=$(cat "/sys/class/net/$interface/address" 2>/dev/null | tr '[:lower:]' '[:upper:]')
            if [ -n "$mac" ] && [ "$mac" != "00:00:00:00:00:00" ]; then
                primary_mac="$mac"
                primary_interface="$interface"
                found_primary=true
                echo "  ✅ $interface: $mac (主要識別 MAC)"
                break
            fi
        fi
    done

    # 如果沒找到高優先級介面，檢查 enx* 介面
    if [ "$found_primary" = false ]; then
        for interface in $(ls /sys/class/net/ 2>/dev/null | grep "^enx" | sort); do
            if [ -f "/sys/class/net/$interface/address" ]; then
                local mac=$(cat "/sys/class/net/$interface/address" 2>/dev/null | tr '[:lower:]' '[:upper:]')
                if [ -n "$mac" ] && [ "$mac" != "00:00:00:00:00:00" ]; then
                    primary_mac="$mac"
                    primary_interface="$interface"
                    found_primary=true
                    echo "  ✅ $interface: $mac (主要識別 MAC - USB 網路)"
                    break
                fi
            fi
        done
    fi

    # 如果還沒找到，檢查其他非虛擬介面
    if [ "$found_primary" = false ]; then
        for interface in $(ls /sys/class/net/ 2>/dev/null); do
            case "$interface" in
                lo|docker0|br-*|veth*) continue ;;
                *)
                    if [ -f "/sys/class/net/$interface/address" ]; then
                        local mac=$(cat "/sys/class/net/$interface/address" 2>/dev/null | tr '[:lower:]' '[:upper:]')
                        if [ -n "$mac" ] && [ "$mac" != "00:00:00:00:00:00" ]; then
                            primary_mac="$mac"
                            primary_interface="$interface"
                            found_primary=true
                            # 檢查是否為 Docker 動態分配的 MAC
                            if [[ "$mac" =~ ^02:42: ]]; then
                                echo "  ⚠️ $interface: $mac (主要識別 MAC - 動態分配，每次重啟會變更)"
                            else
                                echo "  ✅ $interface: $mac (主要識別 MAC)"
                            fi
                            break
                        fi
                    fi
                    ;;
            esac
        done
    fi

    # 顯示其他網路介面（詳細模式或非主要介面）
    if [ "$verbose_mode" = true ] || [ "$found_primary" = false ]; then
        for interface in $(ls /sys/class/net/ 2>/dev/null); do
            if [ "$interface" != "$primary_interface" ]; then
                case "$interface" in
                    lo|docker0|br-*|veth*)
                        if [ "$verbose_mode" = true ]; then
                            echo "  🚫 $interface: (虛擬介面，已排除)"
                        fi
                        ;;
                    *)
                        if [ -f "/sys/class/net/$interface/address" ]; then
                            local mac=$(cat "/sys/class/net/$interface/address" 2>/dev/null | tr '[:lower:]' '[:upper:]')
                            if [ -n "$mac" ] && [ "$mac" != "00:00:00:00:00:00" ]; then
                                case "$interface" in
                                    wlan*) echo "  📱 $interface: $mac (無線網路)" ;;
                                    *) echo "  📡 $interface: $mac" ;;
                                esac
                            fi
                        fi
                        ;;
                esac
            fi
        done
    fi

    # 顯示排除的虛擬介面摘要
    local excluded_interfaces=$(ls /sys/class/net/ 2>/dev/null | grep -E "^(lo|docker0|br-|veth)" | tr '\n' ', ' | sed 's/,$//')
    if [ -n "$excluded_interfaces" ] && [ "$verbose_mode" = false ]; then
        echo "  🚫 已排除: $excluded_interfaces"
    fi

    # 設定全域變數供其他函數使用
    _PRIMARY_MAC="$primary_mac"
    _PRIMARY_INTERFACE="$primary_interface"

    if [ "$found_primary" = false ]; then
        echo "  ❌ 未找到有效的主要 MAC 地址"
    fi

    echo ""
}

# 獲取配置檔案中的 MAC 地址
_get_config_mac_addresses() {
    local device_id="$1"
    local container_type="$2"
    local config_file="/app/config/hardware_mapping.yaml"

    echo "📋 配置檔案中的 MAC 地址:"

    if [ ! -f "$config_file" ]; then
        echo "  ❌ 配置檔案不存在: $config_file"
        _CONFIG_MAC_ADDRESSES=()
        return 1
    fi

    if [ -z "$device_id" ] || [ "$device_id" = "未知" ]; then
        echo "  ⚠️ 設備 ID 未知，無法讀取配置"
        _CONFIG_MAC_ADDRESSES=()
        return 1
    fi

    # 使用 Python 解析 YAML 並獲取 MAC 地址列表
    local mac_addresses_result=$(python3 -c "
import yaml
import sys
try:
    with open('$config_file', 'r') as f:
        config = yaml.safe_load(f)

    devices_key = '${container_type}_devices'
    if devices_key not in config:
        print('ERROR: devices_key_not_found', file=sys.stderr)
        sys.exit(1)

    if '$device_id' not in config[devices_key]:
        print('ERROR: device_not_found', file=sys.stderr)
        sys.exit(1)

    device_config = config[devices_key]['$device_id']
    if 'mac_addresses' not in device_config:
        print('ERROR: mac_addresses_not_found', file=sys.stderr)
        sys.exit(1)

    mac_addresses = device_config['mac_addresses']
    if isinstance(mac_addresses, list):
        for mac in mac_addresses:
            print(mac.upper())
    else:
        print('ERROR: mac_addresses_not_list', file=sys.stderr)
        sys.exit(1)

except Exception as e:
    print(f'ERROR: {e}', file=sys.stderr)
    sys.exit(1)
" 2>/dev/null)

    if [ $? -ne 0 ] || [ -z "$mac_addresses_result" ]; then
        echo "  ⚠️ 無法從配置檔案讀取 MAC 地址"
        echo "    設備 ID: $device_id"
        echo "    容器類型: $container_type"
        _CONFIG_MAC_ADDRESSES=()
        return 1
    fi

    # 將結果轉換為陣列
    _CONFIG_MAC_ADDRESSES=()
    while IFS= read -r mac; do
        if [ -n "$mac" ]; then
            _CONFIG_MAC_ADDRESSES+=("$mac")
        fi
    done <<< "$mac_addresses_result"

    # 顯示配置檔案中的 MAC 地址並與實際 MAC 比對
    local primary_mac="${_PRIMARY_MAC:-}"
    local match_found=false

    for config_mac in "${_CONFIG_MAC_ADDRESSES[@]}"; do
        if [ -n "$primary_mac" ] && [ "$config_mac" = "$primary_mac" ]; then
            echo "  ✅ $config_mac (匹配)"
            match_found=true
        else
            echo "  ❌ $config_mac (不匹配)"
        fi
    done

    if [ ${#_CONFIG_MAC_ADDRESSES[@]} -eq 0 ]; then
        echo "  ⚠️ 配置檔案中未找到 MAC 地址"
    fi

    echo ""

    # 設定匹配狀態供其他函數使用
    _MAC_MATCH_STATUS="$match_found"
}

# 顯示識別狀態和建議
_show_identification_status_and_recommendations() {
    local device_id="$1"
    local container_type="$2"
    local network_mode="$3"

    echo "🎯 識別狀態:"

    # 根據 MAC 地址匹配狀態顯示識別狀態
    if [ "$_MAC_MATCH_STATUS" = true ]; then
        echo "  ✅ MAC 地址匹配，設備識別正常"
        echo "  💡 建議: 配置檔案已包含正確的 MAC 地址"
    else
        echo "  ❌ MAC 地址不匹配，使用預設降級識別"

        # 根據網路模式提供不同的建議
        case "$network_mode" in
            *"bridge"*"動態 MAC"*)
                echo ""
                echo "⚠️ ${container_type^^} 容器 MAC 地址問題:"
                echo "  問題: Bridge 網路模式下，容器 MAC 地址每次 compose up 都會變更"
                echo ""
                echo "🔧 解決方案建議:"
                echo "  1. 固定容器 MAC 地址 (推薦):"
                echo "     在 docker-compose.${container_type}.yml 中添加:"
                echo "     services:"
                echo "       ${container_type}_server:"
                echo "         networks:"
                echo "           ${container_type}_network:"
                if [ ${#_CONFIG_MAC_ADDRESSES[@]} -gt 0 ]; then
                    echo "             mac_address: \"${_CONFIG_MAC_ADDRESSES[0]}\""
                else
                    echo "             mac_address: \"02:42:AC:14:00:10\"  # 請替換為實際 MAC"
                fi
                echo ""
                echo "  2. 使用 host 網路模式:"
                echo "     network_mode: \"host\""
                echo "     (注意: 會與宿主機共享網路，可能有端口衝突)"
                echo ""
                echo "  3. 更新配置檔案 MAC 地址:"
                echo "     執行: show_device_mac_info --update-config"
                ;;
            *"host"*)
                echo ""
                echo "💡 建議操作:"
                echo "  1. 更新配置檔案中的 MAC 地址:"
                if [ -n "$_PRIMARY_MAC" ]; then
                    echo "     將 $_PRIMARY_MAC 添加到 hardware_mapping.yaml"
                fi
                echo "  2. 執行自動更新:"
                echo "     show_device_mac_info --update-config"
                ;;
            *)
                echo ""
                echo "💡 建議檢查網路配置和 MAC 地址設定"
                ;;
        esac
    fi

    echo ""
}

# 自動更新配置檔案中的 MAC 地址
_update_config_mac_addresses() {
    local device_id="$1"
    local container_type="$2"
    local config_file="/app/config/hardware_mapping.yaml"

    echo "🔧 自動更新配置檔案 MAC 地址:"

    if [ -z "$_PRIMARY_MAC" ]; then
        log_error "無法獲取主要 MAC 地址，更新失敗"
        return 1
    fi

    if [ ! -f "$config_file" ]; then
        log_error "配置檔案不存在: $config_file"
        return 1
    fi

    # 檢查 MAC 地址是否已存在於配置中
    for config_mac in "${_CONFIG_MAC_ADDRESSES[@]}"; do
        if [ "$config_mac" = "$_PRIMARY_MAC" ]; then
            log_info "MAC 地址 $_PRIMARY_MAC 已存在於配置檔案中"
            return 0
        fi
    done

    # 備份原始配置檔案
    local backup_file="${config_file}.backup.$(date +%Y%m%d_%H%M%S)"
    cp "$config_file" "$backup_file"
    log_info "已備份原始配置檔案: $backup_file"

    # 使用 Python 更新配置檔案
    python3 -c "
import yaml
import sys
from datetime import datetime

try:
    # 讀取配置檔案
    with open('$config_file', 'r') as f:
        config = yaml.safe_load(f)

    devices_key = '${container_type}_devices'
    if devices_key not in config or '$device_id' not in config[devices_key]:
        print('ERROR: 設備配置不存在', file=sys.stderr)
        sys.exit(1)

    device_config = config[devices_key]['$device_id']

    # 確保 mac_addresses 是列表
    if 'mac_addresses' not in device_config:
        device_config['mac_addresses'] = []
    elif not isinstance(device_config['mac_addresses'], list):
        device_config['mac_addresses'] = [device_config['mac_addresses']]

    # 添加新的 MAC 地址（如果不存在）
    new_mac = '$_PRIMARY_MAC'
    if new_mac not in [mac.upper() for mac in device_config['mac_addresses']]:
        device_config['mac_addresses'].append(new_mac)
        print(f'已添加 MAC 地址: {new_mac}')

    # 添加更新註釋
    if 'description' in device_config:
        device_config['description'] += f' (MAC 更新: {datetime.now().strftime(\"%Y-%m-%d %H:%M\")})'

    # 寫回配置檔案
    with open('$config_file', 'w') as f:
        yaml.dump(config, f, default_flow_style=False, allow_unicode=True, indent=2)

    print('配置檔案更新成功')

except Exception as e:
    print(f'ERROR: {e}', file=sys.stderr)
    sys.exit(1)
"

    if [ $? -eq 0 ]; then
        log_success "配置檔案更新完成"
        log_info "新的 MAC 地址 $_PRIMARY_MAC 已添加到設備 $device_id 的配置中"
    else
        log_error "配置檔案更新失敗"
        return 1
    fi
}

# 生成固定 MAC 地址的 Docker Compose 配置
_generate_compose_config() {
    local device_id="$1"
    local container_type="$2"

    echo "🐳 生成 Docker Compose 配置:"

    if [ "$container_type" != "agvc" ]; then
        log_info "AGV 容器通常使用 host 網路模式，無需固定 MAC 地址"
        return 0
    fi

    local mac_address="${_PRIMARY_MAC:-02:42:AC:14:00:10}"
    if [ ${#_CONFIG_MAC_ADDRESSES[@]} -gt 0 ]; then
        mac_address="${_CONFIG_MAC_ADDRESSES[0]}"
    fi

    local compose_config_file="/tmp/docker-compose.agvc.mac-fixed.yml"

    cat > "$compose_config_file" << EOF
# Docker Compose 配置 - 固定 AGVC 容器 MAC 地址
# 生成時間: $(date)
# 設備 ID: $device_id
# MAC 地址: $mac_address

version: '3.8'

services:
  agvc_server:
    image: yazelin/agvc:latest
    container_name: agvc_server
    restart: unless-stopped

    # 固定 MAC 地址配置
    networks:
      agvc_network:
        mac_address: "$mac_address"

    # 端口映射
    ports:
      - "2200:2200"
      - "3000-3001:3000-3001"
      - "5173:5173"
      - "7447:7447"
      - "8000-8002:8000-8002"

    # 卷掛載
    volumes:
      - ./app:/app
      - ./data:/data
      - ./logs:/logs

    # 環境變數
    environment:
      - CONTAINER_TYPE=agvc
      - MANUAL_DEVICE_ID=$device_id

    # 啟動命令
    command: /bin/bash -c '/app/startup.agvc.bash && tail -f /dev/null'

# 自定義網路配置
networks:
  agvc_network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.20.0.0/16
          gateway: 172.20.0.1

EOF

    log_success "Docker Compose 配置已生成: $compose_config_file"
    echo ""
    echo "📋 使用方式:"
    echo "  1. 停止當前 AGVC 容器:"
    echo "     docker compose -f docker-compose.agvc.yml down"
    echo ""
    echo "  2. 使用新配置啟動:"
    echo "     docker compose -f $compose_config_file up -d"
    echo ""
    echo "  3. 或者將配置合併到現有的 docker-compose.agvc.yml 中"
    echo ""
    echo "⚠️ 注意事項:"
    echo "  - 確保 MAC 地址 $mac_address 在網路中是唯一的"
    echo "  - 固定 MAC 地址後，設備識別將更加穩定"
    echo "  - 建議在生產環境部署前進行測試"
}

# 顯示設備配置資訊
show_device_config() {
    echo "╔══════════════════════════════════════════════════════════════════════════════╗"
    echo "║ 設備配置資訊"
    echo "╚══════════════════════════════════════════════════════════════════════════════╝"

    if [ -f "/app/.device_identity" ]; then
        source /app/.device_identity

        echo "📁 配置檔案檢查:"
        if [ -n "$DEVICE_CONFIG_FILE" ] && [ -f "$DEVICE_CONFIG_FILE" ]; then
            echo "  ✅ 配置檔案存在: $DEVICE_CONFIG_FILE"
            echo "  📊 檔案大小: $(du -h "$DEVICE_CONFIG_FILE" | cut -f1)"
            echo "  🕒 修改時間: $(stat -c %y "$DEVICE_CONFIG_FILE" 2>/dev/null || echo "無法獲取")"
        else
            echo "  ❌ 配置檔案不存在: ${DEVICE_CONFIG_FILE:-未設定}"
        fi

        echo ""
        echo "🗂️ 硬體映射檔案檢查:"
        if [ -f "/app/config/hardware_mapping.yaml" ]; then
            echo "  ✅ 硬體映射檔案存在"
            echo "  📊 檔案大小: $(du -h "/app/config/hardware_mapping.yaml" | cut -f1)"
        else
            echo "  ❌ 硬體映射檔案不存在"
        fi

        echo ""
        echo "📋 日誌檔案檢查:"
        for log_file in "/tmp/device_identification.log" "/tmp/device_hardware_info.log"; do
            if [ -f "$log_file" ]; then
                echo "  ✅ $(basename "$log_file"): $(du -h "$log_file" | cut -f1)"
            else
                echo "  ❌ $(basename "$log_file"): 不存在"
            fi
        done
    else
        log_warning "請先執行設備身份識別"
    fi
}
