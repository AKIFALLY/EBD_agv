#!/bin/bash

# 測試多車監控系統
# 用於建立模擬的 AGV 狀態檔案以測試多車監控界面

echo "🚀 建立測試用 AGV 狀態檔案..."

# Loader 01 狀態
cat > /tmp/agv_status_loader01.json << 'EOF'
{
  "metadata": {
    "agv_id": "loader01",
    "agv_type": "Loader",
    "timestamp": "2025-08-11T10:00:00",
    "version": "1.0"
  },
  "agv_status": {
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
      "current_state": "NAVIGATING"
    },
    "agv_context": {
      "current_state": "TAKE_TRANSFER"
    },
    "robot_context": {
      "current_state": "ACTING"
    }
  },
  "type_specific": {
    "agv_ports": {
      "port1": true,
      "port2": false,
      "port3": false,
      "port4": false
    },
    "work_id": 31,
    "task_progress": {
      "equipment": "Transfer",
      "action": "Take"
    }
  }
}
EOF

# Loader 02 狀態
cat > /tmp/agv_status_loader02.json << 'EOF'
{
  "metadata": {
    "agv_id": "loader02",
    "agv_type": "Loader",
    "timestamp": "2025-08-11T10:00:00",
    "version": "1.0"
  },
  "agv_status": {
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
      "current_state": "IDLE"
    },
    "agv_context": {
      "current_state": "IDLE"
    },
    "robot_context": {
      "current_state": "IDLE"
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
  }
}
EOF

# Cargo 01 狀態
cat > /tmp/agv_status_cargo01.json << 'EOF'
{
  "metadata": {
    "agv_id": "cargo01",
    "agv_type": "Cargo Mover",
    "timestamp": "2025-08-11T10:00:00",
    "version": "1.0"
  },
  "agv_status": {
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
      "current_state": "EXECUTING"
    },
    "agv_context": {
      "current_state": "ENTRANCE_FLOW"
    },
    "robot_context": {
      "current_state": "ACTING"
    }
  },
  "type_specific": {
    "hokuyo_status": {
      "hokuyo_1": "connected",
      "hokuyo_2": "connected"
    },
    "rack_rotation": false,
    "completed": false
  }
}
EOF

# Cargo 02 狀態 (離線)
# 不建立檔案以模擬離線狀態

# Unloader 01 狀態
cat > /tmp/agv_status_unloader01.json << 'EOF'
{
  "metadata": {
    "agv_id": "unloader01",
    "agv_type": "Unloader",
    "timestamp": "2025-08-11T10:00:00",
    "version": "1.0"
  },
  "agv_status": {
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
      "current_state": "EXECUTING"
    },
    "agv_context": {
      "current_state": "TAKE_PRE_DRYER"
    },
    "robot_context": {
      "current_state": "ACTING"
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
  }
}
EOF

# Unloader 02 狀態 (告警)
cat > /tmp/agv_status_unloader02.json << 'EOF'
{
  "metadata": {
    "agv_id": "unloader02",
    "agv_type": "Unloader",
    "timestamp": "2025-08-11T10:00:00",
    "version": "1.0"
  },
  "agv_status": {
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
      "current_state": "ALARM"
    },
    "agv_context": {
      "current_state": "ERROR"
    },
    "robot_context": {
      "current_state": "ERROR"
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
  }
}
EOF

echo "✅ 已建立測試檔案："
echo "  - /tmp/agv_status_loader01.json (運行中)"
echo "  - /tmp/agv_status_loader02.json (閒置)"
echo "  - /tmp/agv_status_cargo01.json (執行中)"
echo "  - /tmp/agv_status_cargo02.json (離線 - 無檔案)"
echo "  - /tmp/agv_status_unloader01.json (執行中)"
echo "  - /tmp/agv_status_unloader02.json (告警)"
echo ""
echo "📋 測試步驟："
echo "1. 啟動 agvui 服務："
echo "   docker compose -f docker-compose.agvc.yml exec agvc_server bash"
echo "   source /app/setup.bash && agvc_source"
echo "   python3 /app/web_api_ws/src/agvui/agvui/agv_ui_server.py"
echo ""
echo "2. 開啟瀏覽器訪問："
echo "   - http://localhost:8003/test  (測試選擇頁面)"
echo "   - http://localhost:8003/multi (多車監控頁面)"
echo ""
echo "3. 驗證功能："
echo "   - 檢查各 AGV 的線上/離線狀態"
echo "   - 切換不同 AGV 分頁查看詳細資訊"
echo "   - 確認車型特定狀態正確顯示"