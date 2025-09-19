# 室內製程基礎調度

## 🎯 倉庫控制系統概覽

RosAGV 的室內製程管理採用 TAFL WCS（Task Automation Flow Language - Warehouse Control System）架構，結合簡化的 RCS 車隊調度系統和 KUKA Fleet 外部整合，提供基礎的製程調度和資源管理。

## 🏗️ 系統架構

### TAFL WCS（流程執行引擎）
```
⚙️ TAFL WCS - YAML 配置驅動引擎
├── YAML 流程定義 (6段式結構)
├── 週期性執行 (如每9秒)
├── 資料庫查詢和更新
├── 任務創建和狀態管理
└── 流程狀態追蹤
```

**核心功能**：
- **配置化流程**: YAML 檔案定義業務流程，無需修改程式碼
- **定期執行**: 根據 execution_interval 週期執行（如每9秒）
- **資料庫操作**: 查詢 locations、racks、tasks，創建和更新記錄
- **同步執行**: 使用同步模式避免記憶體問題

### RCS 簡化調度系統
```
📋 RCS - 簡化車隊調度 (已移除複雜功能)
├── 1秒定時器主迴圈
├── CT 車隊基礎管理
├── KUKA 車隊基礎管理
├── 簡單任務分派邏輯
└── 狀態監控和同步
```

**核心功能**：
- **簡化設計**: 明確移除了複雜的 WCS 適配和優先度調度
- **基本調度**: 1秒循環的簡單任務查詢和分派
- **車隊管理**: CT AGV 和 KUKA AGV 的基礎控制
- **狀態同步**: AGV 狀態監控並同步至資料庫

## 🚛 KUKA Fleet 長距離搬運

### KUKA AGV 車型特性
- **車型**: KUKA KMP 400i (2輪差動車)
- **導航**: KUKA 自有地圖系統（非 ROS 2 導航）
- **載荷**: 專用於 Rack 搬運和 180 度旋轉操作
- **控制**: 透過 KUKA Fleet API 進行任務管理

### KUKA Fleet 基礎整合
```python
# 簡化的 KUKA 任務派發 (來自 simple_kuka_manager.py)
def dispatch(self):
    """KUKA400i AGV 簡單任務派發"""
    # 查詢空閒 KUKA AGV
    idle_kuka400i_agvs = self.kuka_fleet.select_agv(STATUS_IDLE)

    # 基本工作 ID 路由
    if work_id == 210001:  # move
        self._dispatch_move_task(task, agv)
    elif work_id == 220001:  # rack_move
        self._dispatch_rack_move_task(task, agv)
```

**實際功能**：
- **任務派發**: 基於 work_id 的簡單路由邏輯
- **狀態監控**: 每 0.05 秒查詢機器人和容器狀態
- **座標轉換**: KUKA mm 單位轉換為地圖像素 (12.5mm = 1px)
- **資料同步**: 更新 AGV 位置和 Rack 狀態到資料庫

## 📊 六級優先度任務系統

### 任務優先級結構（配置定義，非智能決策）
```
Priority 100: AGV旋轉檢查
├── Rack 需要 180 度轉向以處理 A/B 雙面
├── 確保製程連續性的關鍵操作
└── 最高優先級配置

Priority 80: 滿料架到人工收料區
├── 製程完成 Rack 的最終處理
├── 人工收料區 (位置 51-55)
└── 標準優先級

Priority 60: 系統準備區到房間
├── 投料調度和生產準備
├── 系統準備區 (位置 11-18)
└── 中等優先級

Priority 40: 空料架搬運
├── 房間內空 Rack 轉移
├── 基礎維護優先級
└── 低優先級任務
```

注：所有OCR NG問題在房間入口由Cargo AGV即時處理，不需要NG料架回收任務

### 基礎調度邏輯
- **優先級排序**: 根據 task.priority 欄位簡單排序
- **先進先出**: 同優先級任務按創建時間處理
- **簡單分派**: 查詢空閒 AGV，分派最高優先級任務
- **狀態更新**: 任務狀態從待分派(1)更新為執行中(2)

## 🏭 製程區域管理

### 系統準備區 (11-18)
**功能**: 等待送到房間的 Rack 暫存
- **容量**: 8 個 Rack 位置
- **管理**: 資料庫記錄位置狀態
- **查詢**: TAFL 流程定期檢查並創建搬運任務

### 人工收料區 (51-55)
**功能**: 製程完成後的產品收集
- **容量**: 5 個 Rack 位置
- **流程**:
  1. 滿載 Rack 送達
  2. 人工取出產品
  3. 更新 Rack 狀態為空
  4. 創建回收任務


## 🔄 空料架循環管理

### 基礎回收流程
```
空料架循環流程：
1. 人工收料完成 → 更新資料庫 Rack 狀態
2. TAFL 流程檢查 → 創建回收運輸任務
3. RCS 任務分派 → AGV 執行搬運
4. 送回指定位置 → 更新位置資訊
5. 等待下次使用 → 循環利用
```

### 資源管理
- **狀態追蹤**: 資料庫記錄每個 Rack 的狀態和位置
- **定期檢查**: TAFL 流程週期性檢查需要處理的 Rack
- **簡單分配**: 基於位置和狀態的基礎分配邏輯

## 🚦 交通管制系統

### 基礎交通控制 (TrafficController)
```python
# 實際的交通管制功能
class TrafficController:
    def acquire_traffic_zone(self, traffic_id, agv_id):
        """申請交管區使用權"""
        # 檢查區域是否空閒
        # 設定為 controlled 狀態
        # 記錄佔用 AGV

    def release_traffic_zone(self, traffic_id, agv_id):
        """釋放交管區使用權"""
        # 檢查是否為佔用者
        # 設定為 free 狀態
        # 清除佔用記錄
```

**實際功能**：
- **區域鎖定**: 防止多 AGV 在同一區域
- **簡單管理**: acquire/release 兩個基本操作
- **資料庫記錄**: TrafficZone 表記錄狀態

## 🌐 ECS 設備控制系統

### 門控整合
**基礎功能**：
- **電動門控制**: 開門/關門基本指令
- **雙門互鎖**: 防止同時開啟的安全邏輯
- **狀態監控**: 門控狀態查詢

## 📊 系統監控

### 基礎監控功能
- **任務狀態查詢**: 資料庫查詢任務執行狀態
- **AGV 位置追蹤**: 定期更新 AGV 位置到資料庫
- **流程執行日誌**: TAFL 執行進度和結果記錄
- **簡單統計**: 基本的任務完成數量統計

### 資料庫查詢
```sql
-- 任務統計查詢
SELECT
    status_id,
    COUNT(*) as task_count
FROM task
WHERE created_at >= CURRENT_DATE
GROUP BY status_id;

-- AGV 狀態查詢
SELECT
    id, name, status, x, y
FROM agv
WHERE agv_type IN ('loader', 'unloader', 'cargo');
```

## 🔧 技術實現

### TAFL 流程配置
- **YAML 定義**: 使用 YAML 檔案定義流程邏輯
- **定期執行**: 根據 execution_interval 設定執行週期
- **同步處理**: 避免異步執行的記憶體問題
- **簡單可靠**: 基礎但穩定的執行機制

### RCS 簡化實現
- **單體架構**: 簡單的單一程序設計
- **定時器驅動**: 1秒主迴圈的簡單架構
- **基礎功能**: 專注於核心的任務分派
- **易於維護**: 程式碼簡潔，邏輯清晰

## 🎯 系統特點

### 務實設計
- **簡化架構**: 移除了複雜的優先度調度和 WCS 適配
- **配置驅動**: 透過 YAML 配置調整業務邏輯
- **基礎功能**: 專注於核心的調度和執行
- **穩定可靠**: 簡單的設計帶來穩定性

### 實際能力
- **基本調度**: 提供基礎的任務分派功能
- **狀態管理**: 簡單但有效的狀態追蹤
- **人機協作**: 需要人工參與的半自動化流程
- **漸進改善**: 可根據需求逐步優化

## 🔗 相關資源

- [眼鏡生產完整流程](eyewear-production.md) - 了解端到端業務流程
- [AGV 車型技術詳解](../agv-vehicles/vehicle-types.md) - 三種 AGV 的技術特色
- [KUKA Fleet 整合方案](../technical-details/kuka-integration.md) - 外部系統整合
- [TAFL WCS 系統](../technical-details/tafl-wcs-integration.md) - 流程執行引擎

---

💡 **總結**：室內製程基礎調度展現了 RosAGV 系統的務實設計，通過簡化的 RCS 調度、TAFL 流程執行和基礎的設備整合，實現了穩定可靠的半自動化生產流程。系統專注於核心功能，避免過度複雜化，提供了易於理解和維護的解決方案。