# Cargo AGV JSON 狀態記錄系統

## 📋 功能概述

已為 Cargo Mover AGV 實現完整的 JSON 狀態記錄系統，滿足以下要求：

✅ **記錄所有繼承的 base 變數**: 來自 `AgvNodebase` 的所有變數  
✅ **記錄 core_node 變數**: `AgvCoreNode` 的所有自定義變數  
✅ **記錄狀態機變數**: 3層狀態機 (Base/Cargo/Robot) 的狀態和變數  
✅ **記錄 AgvStatus 細項**: 所有 PLC 狀態細項的 value  
✅ **記錄設備狀態**: Robot 和 Hokuyo 設備的狀態信息  
✅ **輸出目錄**: `/app/agv_status_json/`  
✅ **覆蓋模式**: 每秒更新同一個文件，避免產生大量文件  

## 🚀 自動運行機制

### 即時狀態文件 (自動覆蓋)
- **文件名**: `cargo_agv_{AGV_ID}_current_status.json`
- **更新頻率**: 每 1 秒覆蓋一次
- **位置**: `/app/agv_status_json/cargo_agv_CARGO01_current_status.json`
- **用途**: 提供最新的 AGV 狀態快照

### 其他狀態文件
- **最終狀態**: `cargo_agv_final_status_{timestamp}.json` (節點停止時保存)
- **手動快照**: 通過 API 調用保存的狀態文件
- **狀態變更**: 狀態轉換時的快照文件

## 📊 JSON 文件結構

即時狀態文件包含以下完整結構：

```json
{
  "metadata": {
    "timestamp": "2024-01-01T12:00:00.123456",
    "agv_type": "cargo_mover_agv"
  },
  "agv_core_node": {
    "variables": {
      "mission_id": "...",
      "agv_id": 1,
      "pathdata": null,
      "robot_finished": false,
      "plc_heartbeat": 123
    }
  },
  "agv_status": {
    "variables": {
      "AGV_ID": "CARGO01",
      "POWER": 24.5,
      "AGV_Auto": true,
      "AGV_MOVING": false,
      "AGV_SLAM_X": 1000,
      "AGV_INPUT_1_1": false,
      "AGV_OUTPUT_1_1": false,
      "ALARM_STATUS_1": false
    }
  },
  "state_machines": {
    "base_context": {
      "current_state": {"class": "IdleState"},
      "context_variables": {}
    },
    "cargo_context": {
      "current_state": {"class": "MissionSelectState"},
      "context_variables": {
        "rack_rotation": false,
        "completed": false
      }
    },
    "robot_context": {
      "current_state": {"class": "IdleState"},
      "context_variables": {
        "boxin_up_both_empty": false,
        "get_rack_port": 1,
        "carrier_id": 0
      }
    }
  },
  "devices": {
    "robot": {
      "device_variables": {}
    },
    "hokuyo_dms_8bit_1": {
      "device_variables": {}
    },
    "hokuyo_dms_8bit_2": {
      "device_variables": {}
    }
  }
}
```

## 🔧 使用方法

### 1. 自動運行
啟動 AGV 節點後，系統會自動：
- 創建 `/app/agv_status_json/` 目錄
- 每秒更新即時狀態文件
- 在節點停止時保存最終狀態

### 2. 手動調用
```python
# 在 AGV 節點運行時
node = AgvCoreNode()

# 保存手動快照
filepath = node.save_status_snapshot("manual_snapshot.json")

# 獲取狀態摘要
summary = node.get_status_summary_json()

# 打印狀態摘要到日誌
node.print_status_summary()
```

### 3. 查看即時狀態
```bash
# 查看即時狀態文件
cat /app/agv_status_json/cargo_agv_CARGO01_current_status.json

# 使用 jq 格式化輸出
cat /app/agv_status_json/cargo_agv_CARGO01_current_status.json | jq .

# 監控文件變更
watch -n 1 'ls -la /app/agv_status_json/'
```

## 📁 文件管理

### 文件類型說明
| 文件類型 | 命名格式 | 更新方式 | 用途 |
|---------|----------|----------|------|
| 即時狀態 | `cargo_agv_{AGV_ID}_current_status.json` | 每秒覆蓋 | 最新狀態監控 |
| 手動快照 | `cargo_agv_status_{timestamp}.json` | 手動創建 | 特定時點記錄 |
| 狀態變更 | `cargo_agv_state_change_{timestamp}.json` | 狀態轉換時 | 變更追踪 |
| 最終狀態 | `cargo_agv_final_status_{timestamp}.json` | 節點停止時 | 系統關閉記錄 |

### 磁碟空間管理
- **即時文件**: 固定大小約 50-200 KB，不會持續增長
- **歷史文件**: 根據需要可設置清理策略
- **建議**: 定期清理 7 天以上的歷史快照文件

## 🛠️ 技術實現

### 核心組件
1. **CargoAgvStatusJsonRecorder**: JSON 狀態記錄器類別
2. **AgvCoreNode**: 整合了自動記錄功能
3. **定時器**: 每秒觸發的狀態更新機制
4. **安全序列化**: 處理複雜物件的 JSON 序列化

### 關鍵特性
- **完整性**: 記錄所有變數，包括繼承的 base 變數
- **即時性**: 每秒更新，確保狀態同步
- **穩定性**: 錯誤處理和異常恢復機制
- **效能**: 最小化日誌輸出，避免效能影響

## 📋 驗證檢查

### 檢查文件是否正常生成
```bash
# 檢查目錄是否創建
ls -la /app/agv_status_json/

# 檢查即時文件是否存在且在更新
stat /app/agv_status_json/cargo_agv_*_current_status.json

# 檢查文件內容是否完整
jq '.metadata.timestamp' /app/agv_status_json/cargo_agv_*_current_status.json
```

### 檢查狀態數據完整性
```bash
# 檢查 AGV 狀態變數
jq '.agv_status.variables | keys | length' /app/agv_status_json/cargo_agv_*_current_status.json

# 檢查狀態機信息
jq '.state_machines | keys' /app/agv_status_json/cargo_agv_*_current_status.json

# 檢查設備信息
jq '.devices | keys' /app/agv_status_json/cargo_agv_*_current_status.json
```

## 🚨 故障排除

### 常見問題

1. **目錄未創建**
   ```bash
   # 手動創建目錄
   mkdir -p /app/agv_status_json
   chmod 755 /app/agv_status_json
   ```

2. **文件未更新**
   - 檢查 AGV 節點是否正常運行
   - 檢查日誌中的錯誤信息
   - 驗證磁碟空間是否充足

3. **JSON 格式錯誤**
   ```bash
   # 驗證 JSON 格式
   cat /app/agv_status_json/cargo_agv_*_current_status.json | jq empty
   ```

### 除錯模式
```python
# 在 AGV 節點中啟用詳細日誌
node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

# 手動觸發狀態保存進行測試
node._update_json_status_file()
```

## ✅ 功能確認

- [x] JSON 文件輸出到 `/app/agv_status_json/`
- [x] 使用覆蓋模式，避免文件數量無限增長
- [x] 每秒自動更新即時狀態文件
- [x] 記錄所有繼承的 base 變數
- [x] 記錄 core_node 的所有 self 變數
- [x] 記錄 3層狀態機的狀態和變數
- [x] 記錄 AgvStatus 的所有 PLC 狀態細項
- [x] 記錄 Robot 和 Hokuyo 設備狀態
- [x] 提供安全的物件序列化機制
- [x] 節點停止時保存最終狀態
- [x] 完整的錯誤處理和日誌記錄