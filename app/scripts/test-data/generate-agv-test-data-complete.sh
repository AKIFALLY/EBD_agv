#!/bin/bash
# 在 AGV 容器內生成測試用 AGV 狀態資料 (包含完整 IO 資料)
# 此腳本應該在 AGV 容器內執行

echo "📝 在 AGV 容器內產生測試用 AGV 狀態資料 (完整版)..."

# 檢查是否在容器內
if [ ! -f "/app/setup.bash" ]; then
    echo "❌ 錯誤：此腳本必須在 AGV 容器內執行"
    echo "   請使用: docker compose -f docker-compose.yml exec rosagv bash /app/scripts/generate-agv-test-data-complete.sh"
    exit 1
fi

# 使用 Python 生成完整的 IO 資料
cat > /tmp/generate_test_data.py << 'PYTHON_SCRIPT'
#!/usr/bin/env python3
import json
import random

def generate_io_data(active_ratio=0.3):
    """Generate complete IO data with 5 groups x 16 bits each"""
    io_data = {}
    
    # Generate IN_1 to IN_8 (legacy format)
    for i in range(1, 9):
        io_data[f'IN_{i}'] = random.random() < active_ratio
    
    # Generate AGV_INPUT_[1-5]_[1-16] (5 groups, 16 bits each)
    for group in range(1, 6):
        for bit in range(1, 17):
            io_data[f'AGV_INPUT_{group}_{bit}'] = random.random() < active_ratio
    
    # Generate AGV_OUTPUT_[1-5]_[1-16] (5 groups, 16 bits each)
    for group in range(1, 6):
        for bit in range(1, 17):
            io_data[f'AGV_OUTPUT_{group}_{bit}'] = random.random() < active_ratio
    
    return io_data

def generate_door_status():
    """Generate door status data - 8 doors"""
    door_status = {}
    for i in range(1, 9):  # 8 doors (DOOR_OPEN_1 to DOOR_OPEN_8)
        is_open = random.random() < 0.2  # 20% chance door is open
        door_status[f'DOOR_OPEN_{i}'] = 1 if is_open else 0
        door_status[f'DOOR_CLOSE_{i}'] = 0 if is_open else 1
    return door_status

def generate_alarms(has_alarm=False):
    """Generate alarm status data - 96 alarm points"""
    alarms = {}
    for i in range(1, 97):  # 96 alarm points (ALARM_STATUS_1 to ALARM_STATUS_96)
        if has_alarm and i in [1, 3, 7, 15, 22, 45, 67, 89]:  # Some specific alarms for error state
            alarms[f'ALARM_STATUS_{i}'] = True
        else:
            alarms[f'ALARM_STATUS_{i}'] = False
    return alarms

# Generate test data for each AGV
agv_configs = [
    {
        'id': 'loader01',
        'type': 'Loader',
        'state': 'running',
        'io_ratio': 0.4,
        'has_alarm': False,
        'power': 85,
        'x': 1500.5,
        'y': 2000.3,
        'moving': True,
        'contexts': {
            'base': 'AutoState',
            'agv': 'ExecutingState', 
            'robot': 'IdleState'
        }
    },
    {
        'id': 'loader02',
        'type': 'Loader',
        'state': 'idle',
        'io_ratio': 0.1,
        'has_alarm': False,
        'power': 92,
        'x': 500.0,
        'y': 1000.0,
        'moving': False,
        'contexts': {
            'base': 'IdleState',
            'agv': 'IdleState',
            'robot': 'IdleState'
        }
    },
    {
        'id': 'cargo01',
        'type': 'Cargo Mover',
        'state': 'busy',
        'io_ratio': 0.6,
        'has_alarm': False,
        'power': 75,
        'x': 3000.7,
        'y': 1500.2,
        'moving': True,
        'contexts': {
            'base': 'ExecutingState',
            'agv': 'EntranceFlowState',
            'robot': 'ActingState'
        }
    },
    {
        'id': 'unloader01',
        'type': 'Unloader',
        'state': 'running',
        'io_ratio': 0.4,
        'has_alarm': False,
        'power': 68,
        'x': 2500.0,
        'y': 3000.0,
        'moving': False,
        'contexts': {
            'base': 'ExecutingState',
            'agv': 'TakePreDryerState',
            'robot': 'ActingState'
        }
    },
    {
        'id': 'unloader02',
        'type': 'Unloader',
        'state': 'error',
        'io_ratio': 0.05,
        'has_alarm': True,
        'power': 45,
        'x': 100.0,
        'y': 200.0,
        'moving': False,
        'contexts': {
            'base': 'AlarmState',
            'agv': 'ErrorState',
            'robot': 'ErrorState'
        }
    }
]

# Generate and save test data files
for config in agv_configs:
    agv_data = {
        'metadata': {
            'agv_id': config['id'],
            'agv_type': config['type'],
            'timestamp': '2025-08-29T10:00:00',
            'version': '1.0',
            'namespace': f"/{config['id']}",
            'node_name': f"{config['type'].lower().replace(' ', '_')}_agv_node"
        },
        'agv_status': {
            'AGV_ID': config['id'],
            'AGV_Auto': config['state'] != 'error',
            'AGV_MANUAL': config['state'] == 'error',
            'AGV_IDLE': config['state'] == 'idle',
            'AGV_ALARM': config['has_alarm'],
            'AGV_MOVING': config['moving'],
            'X_DIST': config['x'],
            'Y_DIST': config['y'],
            'POWER': config['power']
        },
        'contexts': {
            'base_context': {'current_state': config['contexts']['base']},
            'agv_context': {'current_state': config['contexts']['agv']},
            'robot_context': {'current_state': config['contexts']['robot']}
        },
        'type_specific': {},
        'io_data': generate_io_data(config['io_ratio']),
        'door_status': generate_door_status(),
        'alarms': generate_alarms(config['has_alarm'])
    }
    
    # Add type-specific data
    if config['type'] == 'Loader':
        agv_data['type_specific'] = {
            'agv_ports': {
                'port1': config['state'] == 'running',
                'port2': False,
                'port3': config['state'] == 'running',
                'port4': False
            },
            'work_id': 'WRK-2025-001' if config['state'] == 'running' else None,
            'task_progress': 45.5 if config['state'] == 'running' else None
        }
    elif config['type'] == 'Unloader':
        agv_data['type_specific'] = {
            'batch_processing': {
                'batch_size': 2,
                'current_batch': 1 if config['state'] == 'running' else 0,
                'total_batches': 3 if config['state'] == 'running' else 0
            },
            'agv_carrier_status': {
                'position_1': config['state'] == 'running',
                'position_2': False
            },
            'station_status': {
                'pre_dryer': [True, False, True, False, False, False, False, False] if config['state'] == 'running' else [False]*8,
                'oven_upper': [False, False, False, False],
                'oven_lower': [False, False, False, False]
            }
        }
    elif config['type'] == 'Cargo Mover':
        agv_data['type_specific'] = {
            'hokuyo_status': {
                'hokuyo_1': 'connected',
                'hokuyo_2': 'connected'
            },
            'rack_rotation': {
                'angle': 45.0,
                'is_rotating': False
            },
            'completed': False
        }
    
    # Save individual file
    filename = f"/tmp/agv_status_{config['id']}.json"
    with open(filename, 'w') as f:
        json.dump(agv_data, f, indent=2)
    
    # Count IO points
    io_count = len(agv_data['io_data'])
    active_io = sum(1 for v in agv_data['io_data'].values() if v)
    print(f"✅ {filename}: {io_count} IO points, {active_io} active ({active_io*100//io_count}%)")
    
    # Save first one as main file
    if config['id'] == 'loader01':
        with open('/tmp/agv_status.json', 'w') as f:
            json.dump(agv_data, f, indent=2)
        print(f"✅ /tmp/agv_status.json (main file, using {config['id']} data)")

print("\n📊 IO 資料結構:")
print("   • IN_1 ~ IN_8: 8 個傳統輸入點")
print("   • AGV_INPUT_[1-5]_[1-16]: 80 個輸入點 (5 群組 x 16 位元)")
print("   • AGV_OUTPUT_[1-5]_[1-16]: 80 個輸出點 (5 群組 x 16 位元)")
print("   • 總計: 168 個 IO 點")
PYTHON_SCRIPT

# 執行 Python 腳本
python3 /tmp/generate_test_data.py

echo ""
echo "✅ 已在 AGV 容器內產生完整測試資料檔案"

# 驗證 JSON 格式
echo ""
echo "🔍 驗證 JSON 格式..."
for file in /tmp/agv_status*.json; do
    if python3 -m json.tool "$file" > /dev/null 2>&1; then
        # 計算 IO 點數
        io_count=$(python3 -c "import json; f=open('$file'); d=json.load(f); print(len(d.get('io_data', {})))")
        echo "   ✅ $(basename $file) - 格式正確 (${io_count} IO 點)"
    else
        echo "   ❌ $(basename $file) - 格式錯誤"
    fi
done

echo ""
echo "📌 資料結構總覽:"
echo "   • IO 資料: 168 個點"
echo "     - IN_1 ~ IN_8: 8 個"
echo "     - AGV_INPUT_[1-5]_[1-16]: 80 個 (5x16)"
echo "     - AGV_OUTPUT_[1-5]_[1-16]: 80 個 (5x16)"
echo "   • 門狀態: 16 個點"
echo "     - DOOR_OPEN_[1-8]: 8 個"
echo "     - DOOR_CLOSE_[1-8]: 8 個"
echo "   • 警報狀態: 96 個點"
echo "     - ALARM_STATUS_[1-96]: 96 個"
echo ""
echo "📌 注意事項:"
echo "   • 此腳本必須在 AGV 容器內執行"
echo "   • 總計 280 個狀態點 (168 IO + 16 門 + 96 警報)"
echo "   • 資料格式與生產環境 write_status_to_file() 一致"
echo "   • 狀態名稱使用實際的類別名稱（如 AutoState、IdleState）"