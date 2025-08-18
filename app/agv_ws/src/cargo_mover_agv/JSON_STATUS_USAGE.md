# Cargo Mover AGV JSON 狀態記錄系統使用指南

## 📋 功能概述

Cargo Mover AGV 的 JSON 狀態記錄系統能夠將 AGV 的所有狀態變數記錄到 JSON 文件中，包括：

- **繼承的 base 變數**: 來自 `AgvNodebase` 的所有變數
- **core_node 變數**: `AgvCoreNode` 的所有自定義變數  
- **狀態機變數**: 3層狀態機 (Base/Cargo/Robot) 的狀態和變數
- **AgvStatus 細項**: 所有 PLC 狀態細項的 value
- **設備狀態**: Robot 和 Hokuyo 設備的狀態信息

## 📂 文件結構

```
cargo_mover_agv/
├── status_json_recorder.py     # JSON 記錄器類別
├── agv_core_node.py           # 已整合 JSON 記錄功能
├── test_json_status.py        # 測試腳本
└── JSON_STATUS_USAGE.md       # 本使用指南
```

## 🚀 快速開始

### 1. 基本使用

在 `AgvCoreNode` 中已經整合了 JSON 記錄器，可以直接使用：

```python
# 在 AGV 節點運行時
node = AgvCoreNode()

# 保存當前狀態快照
filepath = node.save_status_snapshot()
print(f"狀態已保存到: {filepath}")

# 獲取狀態摘要
summary_json = node.get_status_summary_json()
print(summary_json)

# 在日誌中打印狀態摘要
node.print_status_summary()
```

### 2. 自定義文件名

```python
# 使用自定義文件名
filepath = node.save_status_snapshot("my_custom_status.json")
```

### 3. 持續記錄

```python
# 開始持續狀態記錄 (每5秒記錄一次，最多100個文件)
thread = node.start_continuous_logging(interval_seconds=5.0, max_files=100)

# 持續記錄將在後台運行，不會阻塞主程序
```

## 📊 JSON 輸出格式

完整的 JSON 狀態快照包含以下結構：

```json
{
  "metadata": {
    "timestamp": "2024-01-01T12:00:00.123456",
    "timestamp_unix": 1704110400.123456,
    "recorder_version": "1.0.0",
    "agv_type": "cargo_mover_agv"
  },
  "agv_core_node": {
    "class_info": {
      "class": "AgvCoreNode",
      "module": "cargo_mover_agv.agv_core_node",
      "base_classes": ["AgvNodebase", "Node", "object"]
    },
    "variables": {
      "mission_id": "MISSION_001",
      "AGV_id": 1,
      "pathdata": null,
      "robot_finished": false,
      "plc_heartbeat": 123,
      "..."
    }
  },
  "agv_status": {
    "description": "PLC 狀態和所有細項",
    "variables": {
      "AGV_ID": "CARGO01",
      "POWER": 24.5,
      "AGV_X_SPEED": 100,
      "AGV_Y_SPEED": 50,
      "AGV_Auto": true,
      "AGV_MOVING": false,
      "AGV_ALARM": false,
      "AGV_SLAM_X": 1000,
      "AGV_SLAM_Y": 2000,
      "..."
    }
  },
  "state_machines": {
    "base_context": {
      "context_name": "BaseContext",
      "current_state": {
        "class": "IdleState",
        "module": "agv_base.states.idle_state",
        "state_info": {...}
      },
      "context_variables": {...}
    },
    "cargo_context": {
      "context_name": "CargoContext",
      "current_state": {
        "class": "MissionSelectState",
        "module": "agv_base.agv_states.mission_select_state",
        "state_info": {...}
      },
      "context_variables": {
        "rack_rotation": false,
        "completed": false,
        "..."
      }
    },
    "robot_context": {
      "context_name": "RobotContext",
      "current_state": {
        "class": "IdleState",
        "module": "cargo_mover_agv.robot_states.idle_state",
        "state_info": {...}
      },
      "context_variables": {
        "boxin_up_both_empty": false,
        "boxin_down_both_empty": false,
        "get_rack_port": 1,
        "get_boxin_port": 1,
        "carrier_id": 0,
        "..."
      }
    }
  },
  "devices": {
    "robot": {
      "device_name": "Robot",
      "device_class": "Robot",
      "device_module": "agv_base.robot",
      "device_variables": {
        "parameter": {...},
        "status": "active",
        "..."
      }
    },
    "hokuyo_dms_8bit_1": {
      "device_name": "HokuyoDMS8Bit_1",
      "device_class": "HokuyoDMS8Bit",
      "device_module": "agv_base.hokuyo_dms_8bit",
      "device_variables": {
        "ip_address": "192.168.1.101",
        "status": "connected",
        "..."
      }
    },
    "hokuyo_dms_8bit_2": {
      "device_name": "HokuyoDMS8Bit_2",
      "device_class": "HokuyoDMS8Bit",
      "device_module": "agv_base.hokuyo_dms_8bit",
      "device_variables": {
        "ip_address": "192.168.1.102",
        "status": "connected",
        "..."
      }
    }
  }
}
```

## 🔧 測試方法

### 1. 運行測試腳本

```bash
# 在 AGV 容器內
cd /app/agv_ws/src/cargo_mover_agv
python3 test_json_status.py
```

測試腳本將驗證：
- JSON 記錄器的基本功能
- 完整狀態快照的創建和保存
- 狀態摘要功能
- 文件讀寫操作

### 2. 手動測試

```python
# 創建測試腳本
from cargo_mover_agv.status_json_recorder import CargoAgvStatusJsonRecorder

# 假設有 AGV 節點實例
# agv_node = AgvCoreNode()

recorder = CargoAgvStatusJsonRecorder("/tmp/test_output")
# snapshot = recorder.create_complete_status_snapshot(agv_node)
# filepath = recorder.save_status_to_file(agv_node)
```

## 📁 輸出文件管理

### 默認輸出目錄
- **路徑**: `/app/agv_status_json/`
- **即時狀態文件**: `cargo_agv_{AGV_ID}_current_status.json` (每秒覆蓋更新)
- **手動快照**: `cargo_agv_status_YYYYMMDD_HHMMSS.json`
- **狀態變更**: `cargo_agv_state_change_YYYYMMDD_HHMMSS.json`
- **最終狀態**: `cargo_agv_final_status_YYYYMMDD_HHMMSS.json` (節點停止時保存)

### 文件大小預估
- **完整狀態快照**: 約 50-200 KB (取決於狀態複雜度)
- **狀態摘要**: 約 1-5 KB
- **持續記錄**: 每個文件 50-200 KB

### 清理建議
```bash
# 定期清理舊文件 (保留最近7天)
find /app/agv_status_json/ -name "*.json" -mtime +7 -delete

# 按文件數量清理 (保留最新100個)
ls -t /app/agv_status_json/*.json | tail -n +101 | xargs rm -f

# 注意：即時狀態文件 cargo_agv_*_current_status.json 會自動覆蓋，不需要清理
```

## 🛠️ 高級用法

### 1. 自定義記錄器

```python
from cargo_mover_agv.status_json_recorder import CargoAgvStatusJsonRecorder

# 使用自定義輸出目錄
custom_recorder = CargoAgvStatusJsonRecorder(output_dir="/custom/path")

# 直接使用記錄器 API
snapshot = custom_recorder.create_complete_status_snapshot(agv_node)
filepath = custom_recorder.save_status_to_file(agv_node, "custom_name.json")
```

### 2. 狀態變更時自動記錄

可以在狀態變更回調中添加自動記錄：

```python
def state_changed(self, old_state, new_state):
    # 原有邏輯
    self.common_state_changed(old_state, new_state)
    
    # 自動記錄狀態變更 (可選)
    self.save_status_on_state_change()
```

### 3. 篩選特定變數

如果只需要特定的變數，可以修改 `CargoAgvStatusJsonRecorder` 類：

```python
def record_filtered_status(self, agv_core_node, include_vars=None):
    """只記錄指定的變數"""
    # 實現自定義篩選邏輯
    pass
```

## 📋 注意事項

### 1. 效能考量
- 完整狀態記錄包含大量數據，建議適度使用
- 持續記錄會消耗磁碟空間，注意設置合理的文件數量限制
- 在生產環境中建議使用狀態摘要而非完整快照

### 2. 安全考量
- JSON 文件可能包含敏感信息，確保適當的存取權限
- 定期清理不需要的記錄文件

### 3. 除錯用途
- 記錄的 JSON 文件非常適合：
  - 系統狀態分析
  - 問題重現和除錯  
  - 狀態變化追蹤
  - 系統行為分析

## 🔗 相關文件

- `status_json_recorder.py`: 核心記錄器實現
- `agv_core_node.py`: 整合了 JSON 記錄功能的主節點
- `test_json_status.py`: 完整的測試套件
- `cargo_mover_agv/CLAUDE.md`: Cargo Mover AGV 技術文檔

## 🆘 故障排除

### 常見問題

1. **記錄器初始化失敗**
   ```
   ❌ JSON 狀態記錄器初始化失敗
   ```
   - 檢查輸出目錄權限
   - 確保磁碟空間充足

2. **文件保存失敗**
   ```
   ❌ 保存狀態快照失敗
   ```
   - 檢查輸出目錄是否存在
   - 檢查磁碟空間和權限

3. **序列化錯誤**
   - 檢查是否有無法序列化的物件
   - 查看錯誤日誌獲取詳細信息

### 除錯模式

```python
# 開啟詳細日誌
import logging
logging.basicConfig(level=logging.DEBUG)

# 測試序列化功能
recorder = CargoAgvStatusJsonRecorder()
test_obj = {"test": "value"}
result = recorder.safe_serialize(test_obj)
print(f"序列化結果: {result}")
```