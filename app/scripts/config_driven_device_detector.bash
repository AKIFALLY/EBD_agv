#!/bin/bash
# 配置驅動的統一設備身份識別腳本
# 優先級：1. 本地配置文件 2. 手動覆蓋 3. MAC 地址識別 4. 預設值

# 設定日誌檔案
LOG_FILE="/tmp/device_identification.log"
HARDWARE_LOG="/tmp/device_hardware_info.log"
HARDWARE_MAPPING_FILE="/app/config/hardware_mapping.yaml"
LOCAL_CONFIG_FILE="/app/config/local/device.yaml"

# 初始化日誌
echo "=== 配置驅動設備身份識別開始 $(date) ===" > "$LOG_FILE"
echo "=== 硬體資訊收集 $(date) ===" > "$HARDWARE_LOG"

echo "🔍 配置驅動設備身份識別開始..."

# 檢查容器類型
if [ -z "$CONTAINER_TYPE" ]; then
    echo "❌ CONTAINER_TYPE 環境變數未設定"
    return 1
fi

echo "📦 容器類型: $CONTAINER_TYPE"

# =============================================================================
# 從 AGV_NAME 自動衍生配置值的函數
# =============================================================================
derive_values_from_agv_name() {
    local agv_name="$1"

    # 衍生 launch_package（根據名稱前綴）
    case "$agv_name" in
        loader*)
            DERIVED_LAUNCH_PACKAGE="loader_agv"
            DERIVED_DEVICE_TYPE="loader"
            ;;
        cargo*)
            DERIVED_LAUNCH_PACKAGE="cargo_mover_agv"
            DERIVED_DEVICE_TYPE="cargo_mover"
            ;;
        unloader*)
            DERIVED_LAUNCH_PACKAGE="unloader_agv"
            DERIVED_DEVICE_TYPE="unloader"
            ;;
        agvc*)
            DERIVED_LAUNCH_PACKAGE=""
            DERIVED_DEVICE_TYPE="primary_controller"
            ;;
        *)
            DERIVED_LAUNCH_PACKAGE="loader_agv"
            DERIVED_DEVICE_TYPE="generic"
            ;;
    esac

    # 衍生 config_file
    DERIVED_CONFIG_FILE="${agv_name}_config.yaml"

    # 衍生 room_id（從名稱末尾數字提取）
    DERIVED_ROOM_ID=$(echo "$agv_name" | grep -oE '[0-9]+$' | sed 's/^0*//')
    [ -z "$DERIVED_ROOM_ID" ] && DERIVED_ROOM_ID="1"

    echo "  📐 衍生值: launch_package=$DERIVED_LAUNCH_PACKAGE, config_file=$DERIVED_CONFIG_FILE, room_id=$DERIVED_ROOM_ID"
}

# =============================================================================
# 方法 1: 嘗試從本地配置文件讀取（最高優先級）
# =============================================================================
if [ -f "$LOCAL_CONFIG_FILE" ]; then
    echo "📁 發現本地配置文件: $LOCAL_CONFIG_FILE"

    # 使用 Python 讀取 YAML 配置
    local_config=$(python3 -c "
import yaml
import sys
try:
    with open('$LOCAL_CONFIG_FILE', 'r') as f:
        config = yaml.safe_load(f)

    agv_name = config.get('agv_name', '')
    if not agv_name:
        sys.exit(1)

    print(f'AGV_NAME={agv_name}')

    # 讀取可選覆蓋項
    overrides = config.get('overrides', {})
    if overrides:
        if 'launch_package' in overrides:
            print(f\"OVERRIDE_LAUNCH_PACKAGE={overrides['launch_package']}\")
        if 'config_file' in overrides:
            print(f\"OVERRIDE_CONFIG_FILE={overrides['config_file']}\")
        if 'room_id' in overrides:
            print(f\"OVERRIDE_ROOM_ID={overrides['room_id']}\")

    # 讀取 AGVC 專用配置
    agvc_settings = config.get('agvc_settings', {})
    if agvc_settings:
        if 'role' in agvc_settings:
            print(f\"AGVC_ROLE_OVERRIDE={agvc_settings['role']}\")
        if 'workspaces' in agvc_settings:
            print(f\"AGVC_WORKSPACES_OVERRIDE={','.join(agvc_settings['workspaces'])}\")

    sys.exit(0)
except Exception as e:
    print(f'Error: {e}', file=sys.stderr)
    sys.exit(1)
" 2>/dev/null)

    if [ $? -eq 0 ] && [ -n "$local_config" ]; then
        eval "$local_config"

        if [ -n "$AGV_NAME" ]; then
            echo "✅ 從本地配置讀取 AGV_NAME: $AGV_NAME"
            DEVICE_ID="$AGV_NAME"
            IDENTIFICATION_METHOD="local_config"
            IDENTIFICATION_SUCCESS=0

            # 衍生其他配置值
            derive_values_from_agv_name "$AGV_NAME"

            # 應用覆蓋項（如果有）
            LAUNCH_PACKAGE="${OVERRIDE_LAUNCH_PACKAGE:-$DERIVED_LAUNCH_PACKAGE}"
            CONFIG_FILE="${OVERRIDE_CONFIG_FILE:-$DERIVED_CONFIG_FILE}"
            ROOM_ID="${OVERRIDE_ROOM_ID:-$DERIVED_ROOM_ID}"
            DEVICE_TYPE="$DERIVED_DEVICE_TYPE"
            LAUNCH_FILE="launch.py"

            # AGVC 專用配置
            if [[ "$AGV_NAME" == agvc* ]]; then
                ROLE="${AGVC_ROLE_OVERRIDE:-primary}"
                WORKSPACES="${AGVC_WORKSPACES_OVERRIDE:-db_proxy_ws,web_api_ws,ecs_ws,rcs_ws,kuka_wcs_ws}"
            fi
        fi
    else
        echo "⚠️ 本地配置文件解析失敗，繼續其他識別方法"
    fi
else
    echo "📁 本地配置文件不存在: $LOCAL_CONFIG_FILE"
    echo "   使用其他識別方法（手動覆蓋 / MAC 地址 / 預設值）"
fi

# =============================================================================
# 方法 2-4: 若本地配置未成功，使用原有識別邏輯
# =============================================================================
if [ -z "$DEVICE_ID" ]; then
    # 重新賦值給舊變量名以兼容後續代碼
    CONFIG_FILE="$HARDWARE_MAPPING_FILE"

    # 檢查配置檔案是否存在
    if [ ! -f "$CONFIG_FILE" ]; then
        echo "❌ 硬體映射配置檔案不存在: $CONFIG_FILE"
        echo "使用降級處理..."
        case "$CONTAINER_TYPE" in
            "agv") DEVICE_ID="loader02" ;;
            "agvc") DEVICE_ID="agvc01" ;;
            *) echo "❌ 未知容器類型"; return 1 ;;
        esac
        IDENTIFICATION_METHOD="config_file_missing"
        IDENTIFICATION_SUCCESS=1
    else
    # 1. 檢查手動覆蓋設定
    if [ -n "$MANUAL_DEVICE_ID" ]; then
        echo "🔧 檢查手動設定的設備 ID: $MANUAL_DEVICE_ID"
        
        # 使用 Python 驗證手動設定的設備 ID 是否有效
        valid_device=$(python3 -c "
import yaml
import sys
try:
    with open('$CONFIG_FILE', 'r') as f:
        config = yaml.safe_load(f)
    
    devices_key = '${CONTAINER_TYPE}_devices'
    if devices_key in config and '$MANUAL_DEVICE_ID' in config[devices_key]:
        print('$MANUAL_DEVICE_ID')
        sys.exit(0)
    else:
        sys.exit(1)
except Exception as e:
    print(f'Error: {e}', file=sys.stderr)
    sys.exit(1)
" 2>/dev/null)
        
        if [ $? -eq 0 ] && [ -n "$valid_device" ]; then
            DEVICE_ID="$MANUAL_DEVICE_ID"
            echo "✅ 使用手動設定的設備 ID: $DEVICE_ID"
            IDENTIFICATION_METHOD="manual_override"
            IDENTIFICATION_SUCCESS=0
        else
            echo "⚠️ 無效的手動設備 ID: $MANUAL_DEVICE_ID，繼續自動識別"
        fi
    fi
    
    # 2. 如果沒有手動設定，進行 MAC 地址識別
    if [ -z "$DEVICE_ID" ]; then
        echo "🔧 開始 MAC 地址識別..."
        
        # 獲取主要 MAC 地址
        get_primary_mac() {
            echo "獲取 MAC 地址..." >> "$HARDWARE_LOG"
            
            # 優先順序：enp4s0 > eth0 > enx* > 其他
            for interface in enp4s0 eth0 $(ls /sys/class/net/ 2>/dev/null | grep "^enx"); do
                if [ -f "/sys/class/net/$interface/address" ]; then
                    mac=$(cat "/sys/class/net/$interface/address" 2>/dev/null | tr '[:lower:]' '[:upper:]')
                    if [ -n "$mac" ] && [ "$mac" != "00:00:00:00:00:00" ]; then
                        echo "介面 $interface MAC: $mac" >> "$HARDWARE_LOG"
                        echo "$mac"
                        return 0
                    fi
                fi
            done
            
            # 備用方法：獲取第一個非虛擬介面的 MAC
            for mac_file in /sys/class/net/*/address; do
                interface=$(basename $(dirname "$mac_file"))
                case "$interface" in
                    lo|docker0|br-*|veth*) continue ;;
                esac
                
                mac=$(cat "$mac_file" 2>/dev/null | tr '[:lower:]' '[:upper:]')
                if [ -n "$mac" ] && [ "$mac" != "00:00:00:00:00:00" ]; then
                    echo "備用介面 $interface MAC: $mac" >> "$HARDWARE_LOG"
                    echo "$mac"
                    return 0
                fi
            done
            
            echo "無法獲取有效的 MAC 地址" >> "$HARDWARE_LOG"
            return 1
        }
        
        PRIMARY_MAC=$(get_primary_mac)
        if [ $? -eq 0 ] && [ -n "$PRIMARY_MAC" ]; then
            echo "📡 主要 MAC 地址: $PRIMARY_MAC"
            echo "主要 MAC 地址: $PRIMARY_MAC" >> "$LOG_FILE"
            
            # 使用 Python 解析 YAML 並查找匹配的 MAC 地址
            echo "🔍 在配置檔案中查找匹配的 MAC 地址..."
            matched_device=$(python3 -c "
import yaml
import sys
try:
    with open('$CONFIG_FILE', 'r') as f:
        config = yaml.safe_load(f)
    
    devices_key = '${CONTAINER_TYPE}_devices'
    if devices_key not in config:
        print(f'Error: {devices_key} not found in config', file=sys.stderr)
        sys.exit(1)
    
    devices = config[devices_key]
    target_mac = '$PRIMARY_MAC'
    
    for device_id, device_config in devices.items():
        if 'mac_addresses' in device_config:
            mac_list = device_config['mac_addresses']
            if isinstance(mac_list, list):
                # 將所有 MAC 地址轉換為大寫進行比較
                mac_list_upper = [mac.upper() for mac in mac_list]
                if target_mac in mac_list_upper:
                    print(device_id)
                    sys.exit(0)
            else:
                print(f'Warning: mac_addresses for {device_id} is not a list', file=sys.stderr)
    
    sys.exit(1)
except Exception as e:
    print(f'Error parsing YAML: {e}', file=sys.stderr)
    sys.exit(1)
" 2>/dev/null)
            
            if [ $? -eq 0 ] && [ -n "$matched_device" ]; then
                DEVICE_ID="$matched_device"
                echo "✅ 通過 MAC 地址識別設備: $DEVICE_ID"
                IDENTIFICATION_METHOD="mac_address"
                IDENTIFICATION_SUCCESS=0
            else
                echo "⚠️ MAC 地址 $PRIMARY_MAC 未在配置檔案中找到匹配項"
            fi
        else
            echo "❌ 無法獲取有效的 MAC 地址"
        fi
    fi
    
    # 3. 如果仍然無法識別，使用預設值
    if [ -z "$DEVICE_ID" ]; then
        echo "⚠️ 無法識別設備身份，使用預設配置"
        case "$CONTAINER_TYPE" in
            "agv") DEVICE_ID="loader02" ;;
            "agvc") DEVICE_ID="agvc01" ;;
        esac
        IDENTIFICATION_METHOD="default_fallback"
        IDENTIFICATION_SUCCESS=1
    fi
    fi
fi
# === 外層 if [ -z "$DEVICE_ID" ] 結束 ===

echo "🎯 最終設備 ID: $DEVICE_ID"

# 4. 從配置檔案讀取設備配置資訊（僅當未使用本地配置時）
if [ "$IDENTIFICATION_METHOD" != "local_config" ] && [ -f "$CONFIG_FILE" ]; then
    echo "📋 從配置檔案讀取設備配置..."
    device_config=$(python3 -c "
import yaml
import sys
try:
    with open('$CONFIG_FILE', 'r') as f:
        config = yaml.safe_load(f)
    
    devices_key = '${CONTAINER_TYPE}_devices'
    if devices_key in config and '$DEVICE_ID' in config[devices_key]:
        device = config[devices_key]['$DEVICE_ID']
        print(f\"DEVICE_TYPE={device.get('device_type', 'generic')}\")
        print(f\"LAUNCH_PACKAGE={device.get('launch_package', 'loader_agv')}\")
        print(f\"LAUNCH_FILE={device.get('launch_file', 'launch.py')}\")
        print(f\"CONFIG_FILE={device.get('config_file', '${DEVICE_ID}_config.yaml')}\")
        if '$CONTAINER_TYPE' == 'agvc':
            print(f\"ROLE={device.get('role', 'primary')}\")
            workspaces = device.get('workspaces', [])
            print(f\"WORKSPACES={','.join(workspaces)}\")
    else:
        # 使用預設配置
        if '$CONTAINER_TYPE' == 'agv':
            print('DEVICE_TYPE=loader')
            print('LAUNCH_PACKAGE=loader_agv')
            print('LAUNCH_FILE=launch.py')
            print('CONFIG_FILE=${DEVICE_ID}_config.yaml')
        else:
            print('DEVICE_TYPE=primary_controller')
            print('LAUNCH_PACKAGE=')
            print('LAUNCH_FILE=')
            print('CONFIG_FILE=${DEVICE_ID}_config.yaml')
            print('ROLE=primary')
            print('WORKSPACES=db_proxy_ws,web_api_ws,ecs_ws,rcs_ws,wcs_ws')
except Exception as e:
    print(f'Error: {e}', file=sys.stderr)
    sys.exit(1)
")
    
    if [ $? -eq 0 ]; then
        eval "$device_config"
    else
        echo "⚠️ 無法從配置檔案讀取設備配置，使用預設值"
        case "$CONTAINER_TYPE" in
            "agv")
                DEVICE_TYPE="loader"
                LAUNCH_PACKAGE="loader_agv"
                LAUNCH_FILE="launch.py"
                CONFIG_FILE="${DEVICE_ID}_config.yaml"
                ;;
            "agvc")
                DEVICE_TYPE="primary_controller"
                LAUNCH_PACKAGE=""
                LAUNCH_FILE=""
                CONFIG_FILE="${DEVICE_ID}_config.yaml"
                ROLE="primary"
                WORKSPACES="db_proxy_ws,web_api_ws,ecs_ws,rcs_ws,wcs_ws"
                ;;
        esac
    fi
fi

# 5. 設定環境變數
export DEVICE_ID="$DEVICE_ID"

case "$CONTAINER_TYPE" in
    "agv")
        # AGV_NAME 是新的統一識別變量，AGV_ID 保留向後兼容
        export AGV_NAME="$DEVICE_ID"
        export AGV_ID="$DEVICE_ID"
        export AGV_TYPE="$DEVICE_TYPE"
        export ROS_NAMESPACE="/$DEVICE_ID"
        export AGV_LAUNCH_PACKAGE="$LAUNCH_PACKAGE"
        export AGV_LAUNCH_FILE="$LAUNCH_FILE"
        export DEVICE_CONFIG_FILE="/app/config/agv/$CONFIG_FILE"

        echo "🚗 AGV 環境變數設定完成:"
        echo "  AGV_NAME: $AGV_NAME"
        echo "  AGV_ID: $AGV_ID (向後兼容)"
        echo "  AGV_TYPE: $AGV_TYPE"
        echo "  ROS_NAMESPACE: $ROS_NAMESPACE"
        echo "  AGV_LAUNCH_PACKAGE: $AGV_LAUNCH_PACKAGE"
        echo "  DEVICE_CONFIG_FILE: $DEVICE_CONFIG_FILE"
        ;;
        
    "agvc")
        export AGVC_ID="$DEVICE_ID"
        export AGVC_TYPE="$DEVICE_TYPE"
        export AGVC_ROLE="$ROLE"
        export ROS_NAMESPACE="/$DEVICE_ID"
        export DEVICE_CONFIG_FILE="/app/config/agvc/$CONFIG_FILE"
        export AGVC_WORKSPACES="$WORKSPACES"
        
        echo "🖥️ AGVC 環境變數設定完成:"
        echo "  AGVC_ID: $AGVC_ID"
        echo "  AGVC_TYPE: $AGVC_TYPE"
        echo "  AGVC_ROLE: $AGVC_ROLE"
        echo "  ROS_NAMESPACE: $ROS_NAMESPACE"
        echo "  DEVICE_CONFIG_FILE: $DEVICE_CONFIG_FILE"
        echo "  AGVC_WORKSPACES: $AGVC_WORKSPACES"
        ;;
esac

# 6. 創建身份檔案
cat > /app/.device_identity << EOF
DEVICE_ID=$DEVICE_ID
CONTAINER_TYPE=$CONTAINER_TYPE
PRIMARY_MAC=$PRIMARY_MAC
IDENTIFICATION_TIME="$(date)"
IDENTIFICATION_SUCCESS=$IDENTIFICATION_SUCCESS
IDENTIFICATION_METHOD=${IDENTIFICATION_METHOD:-mac_address}
EOF

case "$CONTAINER_TYPE" in
    "agv")
        cat > /app/.agv_identity << EOF
AGV_NAME=$AGV_NAME
AGV_ID=$AGV_ID
AGV_TYPE=$AGV_TYPE
ROS_NAMESPACE=$ROS_NAMESPACE
AGV_LAUNCH_PACKAGE=$AGV_LAUNCH_PACKAGE
AGV_LAUNCH_FILE=$AGV_LAUNCH_FILE
DEVICE_CONFIG_FILE=$DEVICE_CONFIG_FILE
IDENTIFICATION_TIME="$(date)"
IDENTIFICATION_SUCCESS=$IDENTIFICATION_SUCCESS
IDENTIFICATION_METHOD=$IDENTIFICATION_METHOD
EOF
        ;;
    "agvc")
        cat > /app/.agvc_identity << EOF
AGVC_ID=$AGVC_ID
AGVC_TYPE=$AGVC_TYPE
AGVC_ROLE=$AGVC_ROLE
ROS_NAMESPACE=$ROS_NAMESPACE
DEVICE_CONFIG_FILE=$DEVICE_CONFIG_FILE
AGVC_WORKSPACES=$AGVC_WORKSPACES
IDENTIFICATION_TIME="$(date)"
IDENTIFICATION_SUCCESS=$IDENTIFICATION_SUCCESS
EOF
        ;;
esac

echo "✅ 配置驅動設備身份識別完成: $DEVICE_ID"
echo "📁 身份檔案已創建: /app/.device_identity"

[ $IDENTIFICATION_SUCCESS -eq 0 ] && echo "device_detector:success" || echo "device_detector:failed"
return $IDENTIFICATION_SUCCESS
