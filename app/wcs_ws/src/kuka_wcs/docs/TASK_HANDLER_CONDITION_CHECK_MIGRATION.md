# 任務處理器條件檢查機制遷移總結

## 📋 **遷移概述**

將所有任務處理器從舊的條件檢查機制遷移到新的 `TaskConditionChecker` 架構。

## 🎯 **修改的檔案**

### 1. **empty_rack_to_boxout.py** (範本檔案)
- **start_id**: 1
- **功能**: 空架到出口傳送箱任務
- **狀態**: ✅ 已完成 (作為範本)

### 2. **full_rack_to_manual_receive.py**
- **start_id**: 10
- **功能**: 滿架到人工收料區任務
- **修改內容**:
  - 新增 `TaskConditionChecker` import
  - 初始化條件檢查器
  - 替換 `check_condition()` 方法
  - 新增 `_extract_task_data_from_collected()` 方法

### 3. **rack_rotate_180.py**
- **start_id**: 20
- **功能**: 料架180度旋轉任務
- **修改內容**:
  - 新增 `TaskConditionChecker` import
  - 初始化條件檢查器
  - 替換 `check_condition()` 方法
  - 新增 `_extract_task_data_from_collected()` 方法

### 4. **ready_rack_to_boxin.py**
- **start_id**: 30
- **功能**: 準備區料架到入口傳送箱任務
- **修改內容**:
  - 新增 `TaskConditionChecker` import
  - 初始化條件檢查器
  - 替換 `check_condition()` 方法
  - 新增 `_extract_task_data_from_collected()` 方法

## 🔧 **統一的修改模式**

### **1. Import 新增**
```python
from wcs_base.task_condition_checker import TaskConditionChecker
import json
```

### **2. 初始化條件檢查器**
```python
# 條件檢查相關數據
self.collected_data = {}  # 收集的條件檢查資料

# 初始化條件檢查器（使用即時查詢模式）
self.condition_checker = TaskConditionChecker(
    db_manager=self.db_manager,
    logger=self.node.get_logger(),
    real_time_mode=True,  # 啟用即時查詢模式
    query_timeout=30,     # 設定查詢超時時間
    max_iterations=10     # 設定最大迭代次數
)
```

### **3. 統一的 check_condition() 方法**
```python
def check_condition(self) -> bool:
    """
    基於 task_condition 表格的條件檢查

    使用 TaskConditionChecker 進行條件檢查，
    從指定的 start_id 開始進行條件判斷流程。
    """
    try:
        # 使用條件檢查器進行檢查
        success, collected_data = self.condition_checker.check_conditions_from_id(start_id=X)
        
        if success:
            self.collected_data = collected_data
            self._extract_task_data_from_collected()
            self.find_task = True
            return True
        return False
        
    except Exception as e:
        self.node.get_logger().error(f"❌ 條件檢查失敗: {e}")
        return False
```

### **4. 資料提取方法**
```python
def _extract_task_data_from_collected(self):
    """
    從收集的資料中提取任務相關資訊
    """
    try:
        # 從收集的資料中提取相關資訊
        if 'task_node_id' in self.collected_data:
            self.task_node_id = self.collected_data['task_node_id']
        if 'task_room_id' in self.collected_data:
            self.task_room_id = self.collected_data['task_room_id']
        
        # 收集的資料
        self.node.get_logger().info(f"📊 收集到的所有資料: {self.collected_data}")
        
    except Exception as e:
        self.node.get_logger().error(f"❌ 提取任務資料失敗: {e}")
```

## 📊 **start_id 分配表**

| 任務處理器 | start_id | 功能描述 |
|-----------|----------|----------|
| `empty_rack_to_boxout.py` | 1 | 空架到出口傳送箱 |
| `full_rack_to_manual_receive.py` | 10 | 滿架到人工收料區 |
| `rack_rotate_180.py` | 20 | 料架180度旋轉 |
| `ready_rack_to_boxin.py` | 30 | 準備區料架到入口傳送箱 |

## ✅ **遷移完成的優勢**

### 1. **統一的架構**
- 所有任務處理器使用相同的條件檢查邏輯
- 減少程式碼重複和維護成本

### 2. **靈活的配置**
- 支援即時查詢和預存結果兩種模式
- 可配置的查詢超時和迭代次數

### 3. **更好的除錯**
- 統一的日誌格式
- 詳細的條件檢查過程記錄

### 4. **資料庫驅動**
- 條件邏輯存放在 `task_condition` 表格中
- 修改條件不需要重新編譯程式碼

## 🔍 **後續工作**

### 1. **建立 task_condition 記錄**
需要在 `task_condition` 表格中建立對應的條件記錄：
- ID 1-9: 空架到出口傳送箱相關條件
- ID 10-19: 滿架到人工收料區相關條件
- ID 20-29: 料架180度旋轉相關條件
- ID 30-39: 準備區料架到入口傳送箱相關條件

### 2. **測試驗證**
- 測試每個任務處理器的條件檢查功能
- 驗證資料收集和提取邏輯
- 確認不同 start_id 之間無衝突

### 3. **效能監控**
- 監控條件檢查的執行時間
- 根據需要調整查詢超時和迭代次數

## 🎉 **遷移完成**

所有任務處理器已成功遷移到新的 `TaskConditionChecker` 架構，提供了更統一、更靈活、更易維護的條件檢查機制。
