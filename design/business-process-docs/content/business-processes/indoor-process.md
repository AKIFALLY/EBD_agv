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

## 📊 五級優先度任務系統

### 任務優先級結構（配置定義，非動態決策）
```
Priority 100: AGV旋轉檢查 (雙旋轉點) ✅ 已實作
├── 房間入口旋轉 (A面空B面工作) ✅ 已啟用
│   ├── 條件：A面載具已取走，B面有待作業載具
│   ├── 用途：讓B面載具能被Cargo AGV卸載
│   ├── 流程：rack_rotation_room_inlet_aempty_bwork.yaml (enabled: true)
│   └── 測試：test_rack_rotation.py (場景1)
├── 房間出口旋轉 (A面滿B面空) ✅ 已啟用
│   ├── 條件：A面已滿16個載具，B面為空
│   ├── 用途：讓B面繼續收集房間內處理完成的載具
│   ├── 流程：rack_rotation_room_outlet_afull_bempty.yaml (enabled: true)
│   └── 測試：test_rack_rotation.py (場景2)
└── 最高優先級確保機器手臂對接

Priority 80: 滿料架到人工收料區 ✅ 已實作
├── 製程完成 Rack 的最終處理
├── 人工收料區 (位置 51-55)
├── 流程：full_rack_outlet_to_manual_collection.yaml (enabled: true)
├── 功能：滿載或尾批判斷，自動搬運到收料區
└── 測試：test_full_rack_to_collection.py (2個場景)

Priority 60: 系統準備區到房間 ✅ 已實作
├── 投料調度和生產準備
├── 系統準備區 (位置 11-18)
├── 流程：room_dispatch_simple.yaml (enabled: true)
├── 功能：依房間優先級將準備區料架調度到對應房間入口
└── 測試：test_room_dispatch.py (2個場景)

Priority 50: 射出機停車格到系統準備區 ✅ 已實作
├── 料架準備和調度
├── 流程：machine_to_prepare.yaml (enabled: true)
├── 功能：將已派車料架從停車格調度到準備區
└── 測試：test_machine_to_prepare.py (2個場景)

Priority 40: 空料架搬運（三路徑流程）✅ 已實作
├── 流程A：空料架入口→出口（優先）✅ 已實作
│   ├── 條件：A/B面都為空，房間出口位置空閒
│   ├── 用途：準備收集處理完成的Carrier
│   ├── 流程：empty_rack_inlet_to_outlet.yaml (enabled: true)
│   └── 測試：test_parking_flows.py (場景1)
├── 流程B：空料架入口→停車區（備選）✅ 已實作
│   ├── 條件：A/B面都為空，房間出口位置已佔用
│   ├── 目標：系統空車停放區 (31-34)
│   ├── 用途：暫存空Rack，避免出口堵塞
│   ├── 流程：empty_rack_inlet_to_parking.yaml (enabled: true)
│   └── 測試：test_parking_flows.py (場景2)
├── 流程C：停車區→出口（需求調度）✅ 已實作
│   ├── 條件：房間有已完成carrier等待，出口缺rack
│   ├── 用途：從停車區調配rack到出口
│   ├── 流程：parking_to_outlet.yaml (enabled: true)
│   └── 測試：test_parking_flows.py (場景3)
└── 防衝突機制：所有流程都檢查未完成任務，避免重複創建
```

注：
- 所有OCR NG問題在房間入口由Cargo AGV即時處理，不需要NG料架回收任務
- 空料架回收已改為人工手動管理（OPUI-HMI 移出系統 + OPUI 加入料架），不再使用 AGV 自動搬運
- 人工收料區後續全由人工處理，不需要「人工收料區搬運」流程

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
  3. OPUI-HMI「移出系統」(location_id = null)
  4. 人工搬運到系統外倉儲


## 🔄 空料架循環管理（手動管理模式）

### 手動回收流程
```
空料架循環流程：
1. 人工收料完成 → OPUI-HMI「移出系統」(location_id = null)
2. 人工搬運 → 到系統外倉儲空間（非系統管理區域）
3. 需要時 → 人工搬運空 Rack 到射出機旁
4. OPUI 加入料架 → 輸入 Rack 編號（如 "101"）
5. 系統更新 → location_id 分配到停車格，重新進入系統循環
```

### 資源管理
- **系統內活躍**: location_id = 具體位置（停車格、準備區、房間等）
- **系統外儲存**: location_id = null（實體存在但系統不追蹤）
- **手動介入**: 透過 OPUI-HMI 和 OPUI 管理 Rack 進出系統

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