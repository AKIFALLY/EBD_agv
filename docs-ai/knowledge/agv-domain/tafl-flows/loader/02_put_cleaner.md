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
2. AGV 車上有載具需要放入清洗機
3. **清洗機下層（Station 03）有空位**

### 觸發條件
- Loader AGV 車上有載具（**至少2個載具，2格批量**）
- **清洗機下層 Station 03 有2格空位**（Port 3-4 全部空閒）
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
- **Station 03**: Port 3-4（**2格批量**/下層/**只 PUT**）

**Work ID 配置**：
- `2030302`: Station 03 放清洗機（Port 3-4，**2格批量**/下層/**只 PUT**）

**固定方向設計**（類似 Unloader 烤箱）：
- **Station 03（下層）**: 只支持 PUT 操作（進料）
- **清洗機內部**: 清洗制程（下層 → 上層）
- **Station 01（上層）**: 只支持 TAKE 操作（出料，Flow 3）

### 批量處理邏輯

**標準2格批量處理**（Station 03）：
- **一次放料放入2格**（2格批量處理）
- **需要檢查 AGV 至少有2個載具**（車上需有2格載具）
- **需要檢查清洗機至少有2格空位**（Port 3-4 全部空閒）
- **不支持部分放料**（要么放2格，要么不放）

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
  priority: 45                    # 高優先級
  model: "LOADER"                 # AGV 型號
  cleaner_equipment_id: 203       # 清洗機 Equipment ID
  # 固定 Station 配置（只有 Station 03，無需遍歷）
  station: 3                      # Station 03（下層進料）
  work_id: 2030302                # 唯一的 Work ID
  ports: [3, 4]                   # Port 3-4（下層）
  batch_size: 2                   # 批量2格
  row: "lower"                    # 下層
```

## 🔄 流程邏輯

### 主要步驟

1. 查詢所有房間
2. 遍歷每個房間
3. 查詢 Loader AGV
4. 查詢 AGV 車上載具（≥ 2個）
5. 查詢清洗機 Station 03 空位（≥ 2格）
6. 檢查重複任務
7. 創建放料任務（固定 Station 03 配置）

## ⚠️ 注意事項

### 清洗機固定方向設計
- **下層 Station 03（本 Flow）**: Port 3-4，**只 PUT**（進料）
- **上層 Station 01（Flow 3）**: Port 1-2，**只 TAKE**（出料）
- **固定單向流程**: 下層進料 → 清洗制程 → 上層出料

### 批量處理邏輯
- **固定2格批量**: 必須有 ≥ 2個載具，清洗機需 ≥ 2格空位
- **不支持部分放料**（要么放2格，要么不放）

### 狀態碼映射
- 放入清洗機後：載具 status_id 更新為 302（清洗中）

## 🧪 測試要點

### 單元測試
1. ✅ AGV 載具數量判斷（>= 2）
2. ✅ 清洗機空位判斷（>= 2）
3. ✅ 重複任務檢查

### 整合測試
1. ✅ 與 TAKE_BOXIN_TRANSFER 銜接
2. ✅ 與 TAKE_CLEANER 銜接（固定方向）
3. ✅ RCS 調度

## 🔗 相關文檔

- **Loader AGV 代碼**: `/app/agv_ws/src/loader_agv/loader_agv/robot_states/put_cleaner/`
- **Work ID 定義**: 資料庫 `agvc.work` 表
- **後續流程**: Flow 3 - `TAKE_CLEANER`
