# Empty Rack to Boxout 條件檢查整合總結

## 概述

本文件總結了為 `empty_rack_to_boxout` 功能在整個條件檢查系統中的完整整合情況。

## 更新的組件

### 1. TaskConditionService
**檔案**: `wcs_base/task_condition_service.py`

**新增功能**:
- `check_empty_rack_to_boxout_condition()` 方法
- 整合四個主要判斷條件的統一檢查邏輯
- 支援快取機制
- 詳細的結果資料結構

**關鍵特點**:
```python
def check_empty_rack_to_boxout_condition(self) -> ConditionResult:
    # 1. 檢查空料架區狀態
    # 2. 檢查房間載具狀態  
    # 3. 檢查任務重複
    # 4. 檢查位置佔用
    # 返回滿足條件的房間列表
```

### 2. TaskConditionManager
**檔案**: `wcs_base/task_condition_manager.py`

**更新內容**:
- 在 `_execute_condition_check()` 中新增條件處理
- 在 `_get_cache_group()` 中新增快取群組映射
- 在 `update_all_conditions()` 的基礎條件列表中新增條件

**關鍵更新**:
```python
# 新增條件處理
elif condition_name == "empty_rack_to_boxout_condition":
    return self.condition_service.check_empty_rack_to_boxout_condition()

# 新增快取群組
elif "empty_rack_to_boxout" in condition_name:
    return "task_condition"

# 新增到批次更新列表
basic_conditions = [
    # ... 其他條件
    "empty_rack_to_boxout_condition"
]
```

### 3. TaskConditionStorage
**檔案**: `wcs_base/task_condition_storage.py`

**自動支援**:
- ✅ 無需修改，自動支援新條件
- ✅ 自動存儲到 TaskConditionHistory 表
- ✅ 自動存儲到 TaskConditionCache 表
- ✅ 自動處理快取過期和清理
- ✅ 自動使用中文描述

**支援功能**:
- 歷史記錄保存
- 多層快取管理
- 自動清理過期記錄
- 條件狀態摘要
- 中文描述整合

### 4. ConditionType 枚舉
**檔案**: `db_proxy/models/task_condition_history.py`

**新增類型**:
```python
class ConditionType(str, Enum):
    # ... 其他類型
    TASK_CONDITION = "task_condition"
```

### 5. 條件描述配置
**檔案**: `wcs_base/condition_descriptions.py`

**新增描述**:
```python
CONDITION_DESCRIPTIONS = {
    # ... 其他描述
    "empty_rack_to_boxout_condition": "空架放到出口傳送箱條件檢查",
}

CACHE_GROUP_DESCRIPTIONS = {
    # ... 其他描述
    "task_condition": "任務條件",
}
```

## 整合架構

```
┌─────────────────────────────────────────────────────────────┐
│                    Task Handler                             │
│  (empty_rack_to_boxout.py)                                 │
└─────────────────────┬───────────────────────────────────────┘
                      │ 調用
                      ▼
┌─────────────────────────────────────────────────────────────┐
│                TaskConditionManager                         │
│  - 統一條件檢查介面                                           │
│  - 多層快取管理                                              │
│  - 批次更新支援                                              │
└─────────────────────┬───────────────────────────────────────┘
                      │ 委託
                      ▼
┌─────────────────────────────────────────────────────────────┐
│                TaskConditionService                         │
│  - check_empty_rack_to_boxout_condition()                  │
│  - 具體條件檢查邏輯                                           │
│  - 記憶體快取                                                │
└─────────────────────┬───────────────────────────────────────┘
                      │ 存儲
                      ▼
┌─────────────────────────────────────────────────────────────┐
│                TaskConditionStorage                         │
│  - 資料庫存儲                                                │
│  - 歷史記錄管理                                              │
│  - 資料庫快取                                                │
│  - 自動清理                                                  │
└─────────────────────────────────────────────────────────────┘
```

## 資料流程

### 1. 條件檢查流程
```
1. Task Handler 調用 → TaskConditionManager.check_condition()
2. 檢查記憶體快取 → 如果命中，直接返回
3. 檢查資料庫快取 → 如果命中，更新記憶體快取並返回
4. 執行實際檢查 → TaskConditionService.check_empty_rack_to_boxout_condition()
5. 保存結果到所有快取層和歷史記錄
```

### 2. 批次更新流程
```
1. TaskConditionManager.update_all_conditions()
2. 清除記憶體快取
3. 對每個基礎條件執行檢查
4. 保存到 TaskConditionHistory 表
5. 保存到 TaskConditionCache 表
6. 執行清理作業
```

## 使用方式比較

### 原本方式（複雜）
```python
# 在 empty_rack_to_boxout.py 中
def check_task_conditions(self):
    # 複雜的條件檢查邏輯
    carrier_result = self.get_condition_details("room_have_carrier")
    empty_rack_result = self.get_condition_details("system_empty_rack_area_status")
    # 更多複雜檢查...
    for room in carrier_in_room_list:
        # 複雜的迴圈邏輯...
```

### 新方式（簡化）
```python
# 使用統一條件檢查
def check_task_conditions(self):
    result = self.condition_manager.check_condition("empty_rack_to_boxout_condition")
    
    if result.is_satisfied:
        valid_rooms = result.result_data["valid_rooms"]
        selected_room = valid_rooms[0]
        self.task_room_id = selected_room["room_id"]
        self.task_node_id = selected_room["boxout_node_id"]
        return True
    
    return False
```

## 測試覆蓋

### 提供的測試腳本
1. **`test_empty_rack_to_boxout.py`**
   - 基本條件檢查功能測試
   - 結果資料結構驗證

2. **`example_usage_empty_rack_to_boxout.py`**
   - 使用範例展示
   - 新舊方式比較
   - 管理器方式測試

3. **`test_storage_integration.py`**
   - 存儲整合測試
   - 多層快取測試
   - 生命週期測試
   - 批次更新測試

### 測試執行
```bash
cd wcs_ws/src/wcs_base

# 基本功能測試
python test_empty_rack_to_boxout.py

# 使用範例測試
python example_usage_empty_rack_to_boxout.py

# 存儲整合測試
python test_storage_integration.py
```

## 優勢總結

### 1. 程式碼簡化
- 將複雜的判斷邏輯統一管理
- 減少重複程式碼
- 提高可讀性

### 2. 效能優化
- 多層快取機制
- 避免重複計算
- 批次更新支援

### 3. 維護性提升
- 統一的條件檢查介面
- 集中的配置管理
- 完善的錯誤處理

### 4. 可觀測性
- 詳細的檢查結果
- 歷史記錄追蹤
- 條件狀態摘要

### 5. 擴展性
- 易於新增其他條件
- 統一的架構模式
- 靈活的配置選項

## 後續建議

1. **整合到現有 Handler**：將新的條件檢查整合到 `empty_rack_to_boxout.py`
2. **效能監控**：監控快取命中率和檢查效能
3. **配置調優**：根據實際使用情況調整快取超時和清理策略
4. **擴展應用**：考慮將類似的複雜條件檢查也整合到統一系統中
