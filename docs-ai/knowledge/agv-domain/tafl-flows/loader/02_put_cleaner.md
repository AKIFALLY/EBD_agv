# Flow 2: loader_put_cleaner.yaml

## 🎯 業務目的
將 Loader AGV 車上的載具放入清洗機下層，進行清洗處理

## 📋 基本信息

| 項目 | 值 |
|------|-----|
| 文件名 | `loader_put_cleaner.yaml` |
| Flow ID | `loader_put_cleaner` |
| 優先級 | 45 |
| 執行間隔 | 12 秒 |
| Work ID | **2030302**（Station-based，只有1個）|

## 🏭 業務場景

### 前置條件
1. Loader AGV 已從入口箱或其他設備取出載具
2. AGV 車上有載具（**status_id=301 清洗中狀態**）需要放入清洗機
3. **清洗機下層（Station 03）有空位**
4. **Presence 感測器確認清洗機為空（硬體驗證）**

### 觸發條件
- Loader AGV Port 1、3 有載具（**至少1個載具，status_id=301**）
- **清洗機下層 Station 03 有空位**（Port 2033-2034，軟體 status="empty"）
- **Presence 感測器確認無載具在席**（硬體 value="0"，雙重驗證）
- 沒有重複的未完成任務

### 執行結果
- 創建 Loader AGV 放料任務
- 任務進入待分派隊列（status_id = 1 PENDING）
- 載具放入清洗機下層進行清洗

## 🔧 技術規格

### 清洗機配置（Station-based，固定方向）

**物理結構**：
- Equipment 203（清洗機 CLEANER）
- 4個 Port（Port 1-4）
- **只使用 Station 03**（**下層進料**）

**Station-Port 映射**（標準設備）：
- **Station 03**: Port 2033-2034（**1格操作**/下層/**只 PUT**）

**Work ID 配置**：
- `2030302`: Station 03 放清洗機（Port 2033-2034，**1格操作**/下層/**只 PUT**）

**固定方向設計**（類似 Unloader 烤箱）：
- **Station 03（下層）**: 只支持 PUT 操作（進料）
- **清洗機內部**: 清洗制程（下層 → 上層）
- **Station 01（上層）**: 只支持 TAKE 操作（出料，Flow 3）

### 批量處理邏輯

**Loader 1格操作**（Station 03）：
- **一次放料放入1格**（Loader 特性：1格操作）
- **需要檢查 AGV 至少有1個載具**（status_id=301 清洗中）
- **需要檢查清洗機至少有1格空位**（Port 2033-2034 任一空閒）
- **雙重驗證機制**：
  - 軟體狀態：eqp_ports.status = "empty"
  - 硬體感測器：Presence signal value = "0"（無載具在席）

## 📝 TAFL Flow 設計

### Metadata
```yaml
metadata:
  id: "loader_put_cleaner"
  name: "Loader AGV 放料到清洗機"
  enabled: true
  version: "1.0.0"
  description: "檢查 Loader AGV 車上載具和清洗機下層空位，創建放料任務"
  author: "TAFL System"
  created_at: "2025-10-16"
  tags: ["loader", "cleaner", "put", "automation"]
```

### Settings
```yaml
settings:
  execution_interval: 12  # 每12秒執行一次
```

### Variables
```yaml
variables:
  priority: 5                     # 修正：priority 必須在 1-10 範圍內
  model: "Loader"                 # AGV 型號
  cleaner_equipment_id: 203       # 清洗機 Equipment ID
  # 固定 Station 配置（只有 Station 03，無需遍歷）
  station: 3                      # Station 03（下層進料）
  work_id: 2030302                # 唯一的 Work ID
  ports: [2033, 2034]             # Port 2033-2034（下層）
  batch_size: 1                   # Loader: 1格操作
  row: "lower"                    # 下層
```

## 🔄 流程邏輯

### 主要步驟

1. 查詢所有啟用的房間（調試通知）
2. 遍歷每個房間（調試通知進入房間）
3. 查詢 Loader AGV（model="Loader", enable=1）
4. 查詢 AGV Port 1、3 載具（≥ 1個，**status_id=301 清洗中**）
5. 查詢清洗機下層 Station 03 空位（≥ 1格，軟體 status="empty"）
6. **雙重驗證**：檢查 Presence 感測器（硬體 value="0" 無載具在席）
7. 檢查重複任務（work_id, room_id, status_id_in=[0,1,2,3]）
8. 創建放料任務（固定 Station 03 配置，**含 presence_verified=true**）

## ⚠️ 注意事項

### 清洗機固定方向設計
- **下層 Station 03（本 Flow）**: Port 2033-2034，**只 PUT**（進料）
- **上層 Station 01（Flow 3）**: Port 2031-2032，**只 TAKE**（出料）
- **固定單向流程**: 下層進料 → 清洗制程 → 上層出料

### 批量處理邏輯
- **Loader 1格操作**: AGV 需 ≥ 1個載具（status_id=301），清洗機需 ≥ 1格空位
- **支持單格放料**（Loader 特性）

### 狀態碼映射
- 前置條件：載具 status_id = 301（**清洗中**，等待放入清洗機）
- 放入清洗機後：載具 status_id 更新為 302（**清洗中**）

### 雙重驗證機制（重要）
- **軟體狀態檢查**: eqp_ports 表 status = "empty"
- **硬體感測器驗證**: eqp_signals 表 Presence signal value = "0"
- **只有兩者都滿足才創建任務**，避免碰撞或重複放料
- 任務參數包含 `presence_verified: true` 標記

## 🧪 測試要點

### 單元測試
1. ✅ AGV 載具數量判斷（>= 1，status_id=301）
2. ✅ 清洗機空位判斷（>= 1，軟體 status="empty"）
3. ✅ **Presence 感測器驗證**（硬體 value="0"）
4. ✅ **雙重驗證邏輯**（軟體 && 硬體）
5. ✅ 重複任務檢查
6. ✅ 調試通知消息

### 整合測試
1. ✅ 與前序流程銜接（載具 status_id=301）
2. ✅ 與 TAKE_CLEANER 銜接（固定方向）
3. ✅ RCS 調度
4. ✅ Presence 感測器模擬測試

## 🔗 相關文檔

- **Loader AGV 代碼**: `/app/agv_ws/src/loader_agv/loader_agv/robot_states/put_cleaner/`
- **Work ID 定義**: 資料庫 `agvc.work` 表
- **後續流程**: Flow 3 - `TAKE_CLEANER`
