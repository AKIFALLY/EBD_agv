#!/bin/bash
# 在 AGV 容器內生成測試用 AGV 狀態資料
# 此腳本應該在 AGV 容器內執行

echo "📝 在 AGV 容器內產生測試用 AGV 狀態資料..."

# 檢查是否在容器內
if [ ! -f "/app/setup.bash" ]; then
    echo "❌ 錯誤：此腳本必須在 AGV 容器內執行"
    echo "   請使用: docker compose -f docker-compose.yml exec rosagv bash /app/scripts/generate-agv-test-data.sh"
    exit 1
fi

# Loader 01 狀態
cat > /tmp/agv_status_loader01.json << 'EOF'
{
  "metadata": {
    "agv_id": "loader01",
    "agv_type": "Loader",
    "timestamp": "2025-08-29T10:00:00",
    "version": "1.0",
    "namespace": "/loader01",
    "node_name": "loader_agv_node"
  },
  "agv_status": {
    "AGV_ID": "loader01",
    "AGV_Auto": true,
    "AGV_MANUAL": false,
    "AGV_IDLE": false,
    "AGV_ALARM": false,
    "AGV_MOVING": true,
    "X_DIST": 1500.5,
    "Y_DIST": 2000.3,
    "POWER": 85
  },
  "contexts": {
    "base_context": {
      "current_state": "AutoState"
    },
    "agv_context": {
      "current_state": "ExecutingState"
    },
    "robot_context": {
      "current_state": "IdleState"
    }
  },
  "type_specific": {
    "agv_ports": {
      "port1": true,
      "port2": false,
      "port3": true,
      "port4": false
    },
    "work_id": "WRK-2025-001",
    "task_progress": 45.5
  },
  "io_data": {
    "IN_1": true,
    "IN_2": false,
    "IN_3": true,
    "IN_4": false,
    "IN_5": true,
    "IN_6": false,
    "IN_7": true,
    "IN_8": false,
    "AGV_INPUT_1_1": true,
    "AGV_INPUT_1_2": false,
    "AGV_INPUT_1_3": true,
    "AGV_INPUT_1_4": false,
    "AGV_INPUT_1_5": true,
    "AGV_INPUT_1_6": false,
    "AGV_INPUT_1_7": true,
    "AGV_INPUT_1_8": false,
    "AGV_OUTPUT_1_1": false,
    "AGV_OUTPUT_1_2": true,
    "AGV_OUTPUT_1_3": false,
    "AGV_OUTPUT_1_4": true,
    "AGV_OUTPUT_1_5": false,
    "AGV_OUTPUT_1_6": true,
    "AGV_OUTPUT_1_7": false,
    "AGV_OUTPUT_1_8": true
  },
  "door_status": {
    "DOOR_OPEN_1": 0,
    "DOOR_CLOSE_1": 1,
    "DOOR_OPEN_2": 0,
    "DOOR_CLOSE_2": 1,
    "DOOR_OPEN_3": 0,
    "DOOR_CLOSE_3": 1,
    "DOOR_OPEN_4": 0,
    "DOOR_CLOSE_4": 1,
    "DOOR_OPEN_5": 0,
    "DOOR_CLOSE_5": 1,
    "DOOR_OPEN_6": 0,
    "DOOR_CLOSE_6": 1,
    "DOOR_OPEN_7": 0,
    "DOOR_CLOSE_7": 1,
    "DOOR_OPEN_8": 0,
    "DOOR_CLOSE_8": 1
  },
  "alarms": {
    "ALARM_STATUS_1": false,
    "ALARM_STATUS_2": false,
    "ALARM_STATUS_3": false,
    "ALARM_STATUS_4": true,
    "ALARM_STATUS_5": false,
    "ALARM_STATUS_6": false,
    "ALARM_STATUS_7": false,
    "ALARM_STATUS_8": false,
    "ALARM_STATUS_9": false,
    "ALARM_STATUS_10": false
  }
}
EOF

# Loader 02 狀態  
cat > /tmp/agv_status_loader02.json << 'EOF'
{
  "metadata": {
    "agv_id": "loader02",
    "agv_type": "Loader",
    "timestamp": "2025-08-29T10:00:00",
    "version": "1.0",
    "namespace": "/loader02",
    "node_name": "loader_agv_node"
  },
  "agv_status": {
    "AGV_ID": "loader02",
    "AGV_Auto": true,
    "AGV_MANUAL": false,
    "AGV_IDLE": true,
    "AGV_ALARM": false,
    "AGV_MOVING": false,
    "X_DIST": 500.0,
    "Y_DIST": 1000.0,
    "POWER": 92
  },
  "contexts": {
    "base_context": {
      "current_state": "IdleState"
    },
    "agv_context": {
      "current_state": "IdleState"
    },
    "robot_context": {
      "current_state": "IdleState"
    }
  },
  "type_specific": {
    "agv_ports": {
      "port1": false,
      "port2": false,
      "port3": false,
      "port4": false
    },
    "work_id": null,
    "task_progress": null
  },
  "io_data": {
    "IN_1": false,
    "IN_2": false,
    "IN_3": false,
    "IN_4": false,
    "IN_5": false,
    "IN_6": false,
    "IN_7": false,
    "IN_8": false,
    "AGV_INPUT_1_1": false,
    "AGV_INPUT_1_2": false,
    "AGV_INPUT_1_3": false,
    "AGV_INPUT_1_4": false,
    "AGV_INPUT_1_5": false,
    "AGV_INPUT_1_6": false,
    "AGV_INPUT_1_7": false,
    "AGV_INPUT_1_8": false,
    "AGV_OUTPUT_1_1": false,
    "AGV_OUTPUT_1_2": false,
    "AGV_OUTPUT_1_3": false,
    "AGV_OUTPUT_1_4": false,
    "AGV_OUTPUT_1_5": false,
    "AGV_OUTPUT_1_6": false,
    "AGV_OUTPUT_1_7": false,
    "AGV_OUTPUT_1_8": false
  },
  "door_status": {
    "DOOR_OPEN_1": 0,
    "DOOR_CLOSE_1": 1,
    "DOOR_OPEN_2": 0,
    "DOOR_CLOSE_2": 1,
    "DOOR_OPEN_3": 0,
    "DOOR_CLOSE_3": 1,
    "DOOR_OPEN_4": 0,
    "DOOR_CLOSE_4": 1,
    "DOOR_OPEN_5": 0,
    "DOOR_CLOSE_5": 1,
    "DOOR_OPEN_6": 0,
    "DOOR_CLOSE_6": 1,
    "DOOR_OPEN_7": 0,
    "DOOR_CLOSE_7": 1,
    "DOOR_OPEN_8": 0,
    "DOOR_CLOSE_8": 1
  },
  "alarms": {
    "ALARM_STATUS_1": false,
    "ALARM_STATUS_2": false,
    "ALARM_STATUS_3": false,
    "ALARM_STATUS_4": false,
    "ALARM_STATUS_5": false,
    "ALARM_STATUS_6": false,
    "ALARM_STATUS_7": false,
    "ALARM_STATUS_8": false,
    "ALARM_STATUS_9": false,
    "ALARM_STATUS_10": false
  }
}
EOF

# Cargo 01 狀態
cat > /tmp/agv_status_cargo01.json << 'EOF'
{
  "metadata": {
    "agv_id": "cargo01",
    "agv_type": "Cargo Mover",
    "timestamp": "2025-08-29T10:00:00",
    "version": "1.0",
    "namespace": "/cargo01",
    "node_name": "cargo_mover_agv_node"
  },
  "agv_status": {
    "AGV_ID": "cargo01",
    "AGV_Auto": true,
    "AGV_MANUAL": false,
    "AGV_IDLE": false,
    "AGV_ALARM": false,
    "AGV_MOVING": true,
    "X_DIST": 3000.7,
    "Y_DIST": 1500.2,
    "POWER": 75
  },
  "contexts": {
    "base_context": {
      "current_state": "ExecutingState"
    },
    "agv_context": {
      "current_state": "EntranceFlowState"
    },
    "robot_context": {
      "current_state": "ActingState"
    }
  },
  "type_specific": {
    "hokuyo_status": {
      "hokuyo_1": "connected",
      "hokuyo_2": "connected"
    },
    "rack_rotation": {
      "angle": 45.0,
      "is_rotating": false
    },
    "completed": false
  },
  "io_data": {
    "IN_1": true,
    "IN_2": true,
    "IN_3": false,
    "IN_4": true,
    "IN_5": false,
    "IN_6": true,
    "IN_7": false,
    "IN_8": true,
    "AGV_INPUT_1_1": true,
    "AGV_INPUT_1_2": true,
    "AGV_INPUT_1_3": false,
    "AGV_INPUT_1_4": true,
    "AGV_INPUT_1_5": false,
    "AGV_INPUT_1_6": true,
    "AGV_INPUT_1_7": false,
    "AGV_INPUT_1_8": true,
    "AGV_OUTPUT_1_1": true,
    "AGV_OUTPUT_1_2": true,
    "AGV_OUTPUT_1_3": true,
    "AGV_OUTPUT_1_4": false,
    "AGV_OUTPUT_1_5": true,
    "AGV_OUTPUT_1_6": false,
    "AGV_OUTPUT_1_7": true,
    "AGV_OUTPUT_1_8": false
  },
  "door_status": {
    "DOOR_OPEN_1": 1,
    "DOOR_CLOSE_1": 0,
    "DOOR_OPEN_2": 0,
    "DOOR_CLOSE_2": 1,
    "DOOR_OPEN_3": 0,
    "DOOR_CLOSE_3": 1,
    "DOOR_OPEN_4": 0,
    "DOOR_CLOSE_4": 1,
    "DOOR_OPEN_5": 0,
    "DOOR_CLOSE_5": 1,
    "DOOR_OPEN_6": 0,
    "DOOR_CLOSE_6": 1,
    "DOOR_OPEN_7": 0,
    "DOOR_CLOSE_7": 1,
    "DOOR_OPEN_8": 0,
    "DOOR_CLOSE_8": 1
  },
  "alarms": {
    "ALARM_STATUS_1": false,
    "ALARM_STATUS_2": false,
    "ALARM_STATUS_3": false,
    "ALARM_STATUS_4": false,
    "ALARM_STATUS_5": false,
    "ALARM_STATUS_6": false,
    "ALARM_STATUS_7": false,
    "ALARM_STATUS_8": false,
    "ALARM_STATUS_9": false,
    "ALARM_STATUS_10": false
  }
}
EOF

# Unloader 01 狀態
cat > /tmp/agv_status_unloader01.json << 'EOF'
{
  "metadata": {
    "agv_id": "unloader01",
    "agv_type": "Unloader",
    "timestamp": "2025-08-29T10:00:00",
    "version": "1.0",
    "namespace": "/unloader01",
    "node_name": "unloader_agv_node"
  },
  "agv_status": {
    "AGV_ID": "unloader01",
    "AGV_Auto": true,
    "AGV_MANUAL": false,
    "AGV_IDLE": false,
    "AGV_ALARM": false,
    "AGV_MOVING": false,
    "X_DIST": 2500.0,
    "Y_DIST": 3000.0,
    "POWER": 68
  },
  "contexts": {
    "base_context": {
      "current_state": "ExecutingState"
    },
    "agv_context": {
      "current_state": "TakePreDryerState"
    },
    "robot_context": {
      "current_state": "ActingState"
    }
  },
  "type_specific": {
    "batch_processing": {
      "batch_size": 2,
      "current_batch": 1,
      "total_batches": 3
    },
    "agv_carrier_status": {
      "position_1": true,
      "position_2": false
    },
    "station_status": {
      "pre_dryer": [true, false, true, false, false, false, false, false],
      "oven_upper": [false, false, false, false],
      "oven_lower": [false, false, false, false]
    }
  },
  "io_data": {
    "IN_1": true,
    "IN_2": false,
    "IN_3": true,
    "IN_4": true,
    "IN_5": false,
    "IN_6": false,
    "IN_7": true,
    "IN_8": false,
    "AGV_INPUT_1_1": true,
    "AGV_INPUT_1_2": false,
    "AGV_INPUT_1_3": true,
    "AGV_INPUT_1_4": true,
    "AGV_INPUT_1_5": false,
    "AGV_INPUT_1_6": false,
    "AGV_INPUT_1_7": true,
    "AGV_INPUT_1_8": false,
    "AGV_OUTPUT_1_1": true,
    "AGV_OUTPUT_1_2": false,
    "AGV_OUTPUT_1_3": true,
    "AGV_OUTPUT_1_4": false,
    "AGV_OUTPUT_1_5": true,
    "AGV_OUTPUT_1_6": false,
    "AGV_OUTPUT_1_7": true,
    "AGV_OUTPUT_1_8": false
  },
  "door_status": {
    "DOOR_OPEN_1": 0,
    "DOOR_CLOSE_1": 1,
    "DOOR_OPEN_2": 0,
    "DOOR_CLOSE_2": 1,
    "DOOR_OPEN_3": 1,
    "DOOR_CLOSE_3": 0,
    "DOOR_OPEN_4": 0,
    "DOOR_CLOSE_4": 1,
    "DOOR_OPEN_5": 0,
    "DOOR_CLOSE_5": 1,
    "DOOR_OPEN_6": 0,
    "DOOR_CLOSE_6": 1,
    "DOOR_OPEN_7": 0,
    "DOOR_CLOSE_7": 1,
    "DOOR_OPEN_8": 0,
    "DOOR_CLOSE_8": 1
  },
  "alarms": {
    "ALARM_STATUS_1": false,
    "ALARM_STATUS_2": false,
    "ALARM_STATUS_3": false,
    "ALARM_STATUS_4": false,
    "ALARM_STATUS_5": false,
    "ALARM_STATUS_6": false,
    "ALARM_STATUS_7": false,
    "ALARM_STATUS_8": false,
    "ALARM_STATUS_9": false,
    "ALARM_STATUS_10": false
  }
}
EOF

# Unloader 02 狀態 (有警報)
cat > /tmp/agv_status_unloader02.json << 'EOF'
{
  "metadata": {
    "agv_id": "unloader02",
    "agv_type": "Unloader",
    "timestamp": "2025-08-29T10:00:00",
    "version": "1.0",
    "namespace": "/unloader02",
    "node_name": "unloader_agv_node"
  },
  "agv_status": {
    "AGV_ID": "unloader02",
    "AGV_Auto": false,
    "AGV_MANUAL": true,
    "AGV_IDLE": false,
    "AGV_ALARM": true,
    "AGV_MOVING": false,
    "X_DIST": 100.0,
    "Y_DIST": 200.0,
    "POWER": 45
  },
  "contexts": {
    "base_context": {
      "current_state": "AlarmState"
    },
    "agv_context": {
      "current_state": "ErrorState"
    },
    "robot_context": {
      "current_state": "ErrorState"
    }
  },
  "type_specific": {
    "batch_processing": {
      "batch_size": 2,
      "current_batch": 0,
      "total_batches": 0
    },
    "agv_carrier_status": {
      "position_1": false,
      "position_2": false
    },
    "station_status": {
      "pre_dryer": [false, false, false, false, false, false, false, false],
      "oven_upper": [false, false, false, false],
      "oven_lower": [false, false, false, false]
    }
  },
  "io_data": {
    "IN_1": false,
    "IN_2": false,
    "IN_3": false,
    "IN_4": false,
    "IN_5": false,
    "IN_6": false,
    "IN_7": false,
    "IN_8": false,
    "AGV_INPUT_1_1": false,
    "AGV_INPUT_1_2": false,
    "AGV_INPUT_1_3": false,
    "AGV_INPUT_1_4": false,
    "AGV_INPUT_1_5": false,
    "AGV_INPUT_1_6": false,
    "AGV_INPUT_1_7": false,
    "AGV_INPUT_1_8": false,
    "AGV_OUTPUT_1_1": false,
    "AGV_OUTPUT_1_2": false,
    "AGV_OUTPUT_1_3": false,
    "AGV_OUTPUT_1_4": false,
    "AGV_OUTPUT_1_5": false,
    "AGV_OUTPUT_1_6": false,
    "AGV_OUTPUT_1_7": false,
    "AGV_OUTPUT_1_8": false
  },
  "door_status": {
    "DOOR_OPEN_1": 0,
    "DOOR_CLOSE_1": 0,
    "DOOR_OPEN_2": 0,
    "DOOR_CLOSE_2": 0,
    "DOOR_OPEN_3": 0,
    "DOOR_CLOSE_3": 0,
    "DOOR_OPEN_4": 0,
    "DOOR_CLOSE_4": 0,
    "DOOR_OPEN_5": 0,
    "DOOR_CLOSE_5": 0,
    "DOOR_OPEN_6": 0,
    "DOOR_CLOSE_6": 0,
    "DOOR_OPEN_7": 0,
    "DOOR_CLOSE_7": 0,
    "DOOR_OPEN_8": 0,
    "DOOR_CLOSE_8": 0
  },
  "alarms": {
    "ALARM_STATUS_1": true,
    "ALARM_STATUS_2": false,
    "ALARM_STATUS_3": true,
    "ALARM_STATUS_4": false,
    "ALARM_STATUS_5": false,
    "ALARM_STATUS_6": false,
    "ALARM_STATUS_7": true,
    "ALARM_STATUS_8": false,
    "ALARM_STATUS_9": false,
    "ALARM_STATUS_10": false
  }
}
EOF

# 生成主要的 agv_status.json (使用 loader01 作為範例)
cp /tmp/agv_status_loader01.json /tmp/agv_status.json

echo "✅ 已在 AGV 容器內產生 6 個測試資料檔案:"
echo "   • /tmp/agv_status.json (主檔案，使用 loader01 資料)"
echo "   • /tmp/agv_status_loader01.json"
echo "   • /tmp/agv_status_loader02.json"
echo "   • /tmp/agv_status_cargo01.json"
echo "   • /tmp/agv_status_unloader01.json"
echo "   • /tmp/agv_status_unloader02.json"

# 驗證 JSON 格式
echo ""
echo "🔍 驗證 JSON 格式..."
for file in /tmp/agv_status*.json; do
    if python3 -m json.tool "$file" > /dev/null 2>&1; then
        echo "   ✅ $(basename $file) - 格式正確"
    else
        echo "   ❌ $(basename $file) - 格式錯誤"
    fi
done

echo ""
echo "📊 資料結構包含:"
echo "   • metadata: AGV ID、類型、時間戳、版本、namespace、node_name"
echo "   • agv_status: 基本狀態（含 AGV_ID）"
echo "   • contexts: 三層狀態機當前狀態（使用實際狀態名稱）"
echo "   • type_specific: 車型特定資料"
echo "   • io_data: IN_*/AGV_INPUT_*/AGV_OUTPUT_* IO 狀態"
echo "   • door_status: DOOR_OPEN_*/DOOR_CLOSE_* 門狀態"
echo "   • alarms: ALARM_STATUS_* 警報狀態"

echo ""
echo "📌 注意事項:"
echo "   • 此腳本必須在 AGV 容器內執行"
echo "   • 資料格式與生產環境 write_status_to_file() 一致"
echo "   • 狀態名稱使用實際的類別名稱（如 AutoState、IdleState）"