# 最終邏輯修正說明

## 📅 修正日期
2025-11-10 19:53

## 🎯 正確理解

### 問題發現
用戶質疑：「為什麼要讀取 PLC？」

這個問題揭露了根本性的邏輯錯誤！

### DM 位址用途
**DM2010~2014 (入口) / DM2020~2024 (出口)**：
- ✅ **系統寫入，PLC 讀取**
- ❌ **不是**從 PLC 讀取！

### carrier_bitmap 資料來源
- **任務建立前**：由 Web UI 給予（存在資料庫 Rack 中）
- **任務建立後**：從 PLC DM3012/3014 (feedback) 讀取

## ✅ 正確的邏輯流程

```
Timer 每 3 秒
  ↓
檢查資料庫：是否有 Rack 在 location_id 26/27？
  ├─ 沒有 Rack → 跳過
  └─ 有 Rack
      ↓
      從資料庫 Rack 讀取：
        - carrier_bitmap
        - carrier_enable_bitmap  
        - direction
      ↓
      判斷是否寫入：
        - 入口：有料（bitmap > 0）才寫入
        - 出口：無料（bitmap = 0）才寫入
      ↓
      檢查是否已有 Task
      ↓
      寫入 PLC DM2010~2014 (入口) 或 DM2020~2024 (出口)：
        - DM[0~1]: carrier_bitmap (32-bit, 小端序)
        - DM[2~3]: carrier_enable_bitmap (32-bit, 小端序)
        - DM[4]: direction 確認值 (16-bit)
```

## 🔧 主要修改

### 1. 刪除錯誤的 PLC 讀取邏輯
**刪除**：
- `_handle_rack_read_response()` - 處理 PLC 讀取回應（不再需要）
- `_convert_direction_angle()` - 轉換角度（改為直接使用 direction 值）
- 從 PLC 讀取 DM2010~2014 的所有程式碼

### 2. 新增正確的寫入邏輯
**新增**：
- `_convert_direction_value()` - 轉換資料庫 direction 為確認值
- `_write_rack_info_to_plc()` - 寫入完整 5 個 words 到 PLC
- `_handle_write_rack_response()` - 處理寫入回應

### 3. 修改主流程
**`_check_and_write_single_transfer_box()`**：
```python
# 修改前（錯誤）
從 PLC 讀取 DM2010~2014 → 解析 → 判斷 → 寫入

# 修改後（正確）
從資料庫讀取 Rack → 解析 → 判斷 → 寫入 PLC DM2010~2014
```

### 4. 更新配置檔案
**移除**：
- `dm_read_rack_start` - 不再需要
- `DM_READ_RACK_COUNT` - 不再需要

**更新**：
- `DM_WRITE_COUNT`: 3 → 5 (現在寫入完整 5 個 words)
- 更新註釋說明 DM 位址的實際用途

## 📊 PLC DM 完整說明

### 系統寫入 → PLC 讀取
```
入口：DM2010~2014
出口：DM2020~2024

格式：
DM[0~1]: carrier_bitmap (32-bit, 小端序)
DM[2~3]: carrier_enable_bitmap (32-bit, 小端序)
DM[4]: direction 確認值 (1=正向, 2=反向)
```

### PLC 寫入 → 系統讀取 (Feedback)
```
入口：DM3012~3013
出口：DM3014~3015

格式：
DM[0~1]: 更新後的 carrier_bitmap (32-bit, 小端序)
```

### PLC 寫入 → 系統讀取 (Work ID)
```
共用：DM3010~3011

格式：
DM[0~1]: work_id (32-bit, 小端序)
```

## 📝 程式碼變更統計

### 修改檔案
1. **config.py**
   - 移除 `dm_read_rack_start` 配置
   - 更新 `DM_WRITE_COUNT`: 3 → 5
   - 移除 `DM_READ_RACK_COUNT`
   - 更新註釋說明

2. **transfer_box_task_build_node.py**
   - 重寫 `_check_and_write_single_transfer_box()` - 從資料庫讀取
   - 刪除 `_handle_rack_read_response()` - 不再需要
   - 刪除 `_convert_direction_angle()` - 改用 direction 值
   - 新增 `_convert_direction_value()` - 轉換 direction
   - 刪除 `_write_carrier_bitmap_to_plc()` - 舊的寫入方法
   - 新增 `_write_rack_info_to_plc()` - 新的完整寫入方法
   - 刪除 `_handle_write_response()` - 舊的回應處理
   - 新增 `_handle_write_rack_response()` - 新的回應處理

### 程式碼行數
- 刪除：~150 行（錯誤的 PLC 讀取邏輯）
- 新增：~90 行（正確的資料庫讀取和完整寫入）
- 淨變化：-60 行（更簡潔清晰）

## ✅ 驗證結果

### 服務狀態
- ✅ Python 語法檢查通過
- ✅ ROS2 建置成功
- ✅ 服務正常啟動
- ✅ 節點已註冊到 ROS2 網路

### 預期日誌
**有 Rack 且滿足寫入條件**：
```
✅ 入口傳送箱 寫入 PLC 成功: 
   Rack ID=123, 
   Carrier=[A=0xffff, B=0x0000], 
   Enable=[A=0xffff, B=0xffff], 
   Direction=1
```

**沒有 Rack**：
```
[DEBUG] 入口傳送箱 Location 27 沒有 Rack，跳過
```

## 🎓 經驗教訓

1. **先確認需求**：在實作前應該先確認資料流向
2. **質疑合理性**：當邏輯看起來奇怪時（讀取後又寫入同一個位址），應該提出質疑
3. **理解 DM 用途**：搞清楚每個 DM 位址是「系統寫入」還是「PLC 寫入」
4. **用戶反饋重要**：用戶的「為什麼」問題往往揭露根本問題

## 📚 相關文檔
- `BUGFIX_LOCATION_CHECK.md` - Location 檢查修正
- `README_PLC_READ.md` - ⚠️ 此文檔已過時，需更新
- `config.py` - 最新配置
- `transfer_box_task_build_node.py` - 最新實作

---

**修正人員**: Claude AI Assistant  
**問題發現**: 用戶質疑  
**嚴重程度**: 高 (根本性邏輯錯誤)  
**修正狀態**: ✅ 已完成並驗證  
**部署時間**: 2025-11-10 19:53:53
