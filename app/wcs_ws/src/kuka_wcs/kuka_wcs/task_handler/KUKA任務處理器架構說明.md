# KUKA 任務處理器架構說明

## 概述

此目錄包含七大 KUKA 任務判斷處理器，對應完整的倉儲自動化業務流程。每個處理器都遵循統一的三階段處理流程，並基於 `task_condition` 資料表進行條件判斷。

## 七大業務流程模組

### 1. EmptyRackToBoxoutHandler (`empty_rack_to_boxout.py`)
- **起始條件 ID**: 1
- **業務目標**: 空料架搬運到出口傳送箱 - 將系統空料架區的空料架搬運到出口傳送箱
- **關鍵邏輯**: 檢查系統空架區可用性 → 檢查出口傳送箱空間 → 防重複任務
- **優先級**: `CONFIG.PRIORITY_FOR_KUKA_FROM_EMPTY_TO_BOXOUT`

### 2. FullRackToManualReceiveHandler (`full_rack_to_manual_receive.py`)
- **起始條件 ID**: 2
- **業務目標**: 滿料架搬運到人工收料區 - 將房間出口傳送箱的滿料架搬運到人工收料區
- **關鍵邏輯**: 檢查人工收料區空位 → 檢查出口傳送箱滿料架 → 防重複任務
- **優先級**: `CONFIG.PRIORITY_FOR_KUKA_FROM_BOXOUT_TO_MANUAL_RECEIVE`

### 3. RackRotate180Handler (`rack_rotate_180.py`)
- **起始條件 ID**: 3
- **業務目標**: 處理 AGV 料架180度旋轉任務
- **關鍵邏輯**: 檢查等待旋轉狀態的 AGV → 檢查子任務不存在 → 建立旋轉任務
- **優先級**: `CONFIG.PRIORITY_FOR_RACK_ROTATE`
- **特殊處理**: 建立父子任務關係，直接複製父任務的參數

### 4. ReadyRackToBoxinHandler (`ready_rack_to_boxin.py`)
- **起始條件 ID**: 4
- **業務目標**: 將系統準備區的料架搬運到入口傳送箱
- **關鍵邏輯**: 檢查準備區料架可用性 → 檢查入口傳送箱空位 → 防重複任務
- **優先級**: `CONFIG.PRIORITY_FOR_KUKA_FROM_READY_TO_BOXIN`

### 5. EmptyRackToBoxoutOrEmptyareaHandler (`empty_rack_to_boxout_or_emptyarea.py`)
- **起始條件 ID**: 5
- **業務目標**: 將入口傳送箱的空料架搬運到出口傳送箱或系統空料架區
- **關鍵邏輯**: 檢查入口傳送箱空料架 → 檢查出口傳送箱佔用狀態 → 若佔用則檢查系統空料架區空位 → 防重複任務
- **優先級**: `CONFIG.PRIORITY_FOR_KUKA_EMPTY_RACK_TRANSFER`
- **特殊處理**: 支援條件分支邏輯，優先選擇出口傳送箱，若佔用則選擇系統空料架區

### 6. NgRackRecyclingHandler (`ng_rack_recycling_handler.py`)
- **起始條件 ID**: 6
- **業務目標**: 將入口傳送箱的NG料架搬運到NG回收區
- **關鍵邏輯**: 檢查NG回收區空位 → 檢查NG料架存在 → 防重複任務
- **優先級**: `CONFIG.PRIORITY_FOR_KUKA_NG_RACK_RECYCLING`

### 7. ManualEmptyRackRecyclingHandler (`manual_empty_rack_recycling_handler.py`)
- **起始條件 ID**: 7-9
- **業務目標**: 將人工回收空料架區的空料架搬運到系統空料架區
- **關鍵邏輯**: 檢查人工回收區料架 → 檢查空料架回收區空位 → 防重複任務
- **優先級**: `CONFIG.PRIORITY_FOR_KUKA_MANUAL_EMPTY_RACK_RECYCLING`

## 統一架構模式

### 基礎類別繼承
```python
class XxxHandler(BaseTaskHandler):
    def __init__(self, node):
        super().__init__(node)  # 設置 db_manager
        # 初始化條件檢查器
        self.condition_checker = TaskConditionChecker(
            db_manager=self.db_manager,
            logger=self.node.get_logger(),
            real_time_mode=True,
            query_timeout=30,
            max_iterations=50
        )
```

### 三階段處理流程

#### 1. check_condition() - 條件檢查
```python
def check_condition(self) -> bool:
    """基於 task_condition 表格的條件檢查"""
    success, collected_data = self.condition_checker.check_conditions_from_id(start_id=X)
    if success:
        self.collected_data = collected_data
        self._extract_task_data_from_collected()
        self.find_task = True
        return True
    return False
```

#### 2. insert_task() - 任務插入
```python
def insert_task(self) -> bool:
    """插入KUKA任務"""
    # 獲取work配置
    for work in self.db_manager.works:
        if work.id == CONFIG.KUKA_RACK_MOVE:
            self.kuka_rack_move_work = work
            break
    
    # 設定參數
    uuid_nodes = [self.get_uuid(source), self.get_uuid(target)]
    self.kuka_rack_move_work.parameters["nodes"] = uuid_nodes
    self.kuka_rack_move_work.parameters["model"] = CONFIG.KUKA_MODLE_NAME
    
    # 建立任務
    task_data = Task(
        work_id=self.kuka_rack_move_work.id,
        node_id=self.task_node_id,
        name=f"任務描述",
        room_id=self.task_room_id,
        priority=CONFIG.PRIORITY_XXX,
        status_id=CONFIG.WCS_STATUS,
        created_at=datetime.now(timezone.utc),
        updated_at=datetime.now(timezone.utc),
        parameters=self.kuka_rack_move_work.parameters
    )
```

#### 3. check_insert_done() - 插入確認
```python
def check_insert_done(self) -> bool:
    """檢查任務插入是否成功"""
    with self.db_manager.get_session() as session:
        task = self.get_task_by_id(session, self.create_task_result.id)
        if task:
            # 重置狀態，準備下一次檢查
            self.find_task = False
            self.task_inserted = False
            return True
        return False
```

## 條件檢查系統

### TaskConditionChecker 配置
- **real_time_mode**: True - 啟用即時查詢模式
- **query_timeout**: 30 - 查詢超時時間（秒）
- **max_iterations**: 50 - 最大迭代次數

### 條件 ID 分配表
| 處理器 | 起始ID | 條件範圍 | 說明 |
|--------|--------|----------|------|
| EmptyRackToBoxout | 1 | 1, 201-203, 301-303... | 空料架搬運到出口傳送箱 |
| FullRackToManualReceive | 2 | 2, 205-206 | 滿料架搬運到人工收料區 |
| RackRotate180 | 3 | 3 | AGV旋轉檢查 |
| ReadyRackToBoxin | 4 | 4, 房間檢查條件 | 準備區料架到入口傳送箱 |
| EmptyRackToBoxoutOrEmptyarea | 5 | 5, 215-217 | 入口傳送箱空料架到出口傳送箱或系統空料架區 |
| NgRackRecycling | 6 | 6, 220-221, 320-321... | NG料架回收到NG回收區 |
| ManualEmptyRackRecycling | 7 | 7, 8, 9 | 人工空料架回收到系統空料架區 |

## 資料提取模式

### 通用資料提取
```python
def _extract_task_data_from_collected(self):
    """從收集的資料中提取任務相關資訊"""
    if 'room_id' in self.collected_data:
        self.task_room_id = self.collected_data['room_id']
    if 'location' in self.collected_data:
        self.task_node_id = self.collected_data['location']
    # 其他特定欄位提取...
```

### 特殊處理案例

#### 1. 旋轉任務的父子關係
```python
# RackRotate180Handler
task_data = Task(
    parent_task_id=self.parent_task_id,  # 設定父任務
    # 其他參數...
)
```


## 配置依賴

### 必要的 CONFIG 設定
```python
# 工作類型
CONFIG.KUKA_RACK_MOVE  # KUKA料架搬運工作ID
CONFIG.KUKA_MODLE_NAME  # KUKA機器人型號名稱

# 狀態設定
CONFIG.WCS_STATUS  # WCS任務狀態

# 優先級設定
CONFIG.PRIORITY_FOR_KUKA_FROM_EMPTY_TO_BOXOUT
CONFIG.PRIORITY_FOR_KUKA_FROM_BOXOUT_TO_MANUAL_RECEIVE  
CONFIG.PRIORITY_FOR_RACK_ROTATE
CONFIG.PRIORITY_FOR_KUKA_FROM_READY_TO_BOXIN
CONFIG.PRIORITY_FOR_KUKA_EMPTY_RACK_TRANSFER
CONFIG.PRIORITY_FOR_KUKA_NG_RACK_RECYCLING
CONFIG.PRIORITY_FOR_KUKA_MANUAL_EMPTY_RACK_RECYCLING
```

## 位置對應關係

### 房間位置編碼（硬編碼規則）
- **房間1**: 入口 10001 (1*10000+1), 出口 10002 (1*10000+2)
- **房間2**: 入口 20001 (2*10000+1), 出口 20002 (2*10000+2)
- **房間N**: 入口 N0001 (N*10000+1), 出口 N0002 (N*10000+2)

### 特殊區域位置
- **系統空架區**: [31, 32, 33, 34]
- **系統準備區**: [11, 12, 13, 14, 15, 16, 17, 18]
- **人工收料區**: [51, 52, 53, 54, 55]
- **NG回收區**: [71, 72]
- **人工回收空料架區**: [91, 92]
- **空料架回收區**: [51, 52, 53, 54]

### 位置狀態編碼
- **1**: 未知狀態
- **2**: 未佔用
- **3**: 佔用
- **4**: 任務佔用中

## 錯誤處理模式

### 統一錯誤處理
```python
try:
    # 主要邏輯
    pass
except Exception as e:
    self.node.get_logger().error(f"❌ 操作失敗: {e}")
    # 重置狀態
    self.find_task = False
    self.task_inserted = False
    return False
```

### 日誌等級使用
- `✅` - 成功操作
- `❌` - 錯誤情況  
- `⚠️` - 警告情況
- `📊` - 資料資訊
- `🔄` - 處理中狀態
- `📍` - 位置資訊

## 使用方式

### 匯入處理器
```python
from kuka_wcs.task_handler import (
    EmptyRackToBoxoutHandler,
    FullRackToManualReceiveHandler,
    RackRotate180Handler,
    ReadyRackToBoxinHandler,
    EmptyRackToBoxoutOrEmptyareaHandler,
    NgRackRecyclingHandler,
    ManualEmptyRackRecyclingHandler
)
```

### 基本使用流程
```python
# 初始化處理器
handler = EmptyRackToBoxoutOrEmptyareaHandler(node)

# 執行處理流程
if handler.check_condition():
    if handler.insert_task():
        if handler.check_insert_done():
            # 任務處理完成
            pass
```

## 維護注意事項

1. **條件 ID 衝突**: 新增條件時需確保 ID 不重複
2. **優先級管理**: 不同任務類型需設定適當的優先級
3. **狀態重置**: 每次處理完成後必須重置處理器狀態
4. **資源清理**: 確保資料庫連接正確關閉
5. **日誌完整性**: 保持日誌資訊的一致性和可讀性

## 擴展指南

### 新增處理器步驟
1. 繼承 `BaseTaskHandler`
2. 實作三階段方法
3. 定義對應的條件 SQL（在 db_proxy conditions 中）
4. 設定適當的優先級配置
5. 更新 `__init__.py` 匯出清單

### 條件擴展
- 在 `db_proxy_ws/src/db_proxy/db_proxy/sql/init_data/conditions/` 中新增條件檔案
- 確保條件 ID 的唯一性和順序性
- 測試條件邏輯的正確性

這份文檔提供了完整的架構說明，未來查詢時可以快速定位相關資訊，避免重複分析代碼結構。