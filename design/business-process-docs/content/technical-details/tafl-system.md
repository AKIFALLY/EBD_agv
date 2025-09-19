# TAFL 系統概覽

## 🎯 什麼是 TAFL？

TAFL (Task Automation Flow Language) 是 RosAGV 系統中新一代的任務自動化流程語言，取代了舊的 Linear Flow v2 系統。TAFL 提供結構化、標準化的流程定義和執行能力。

## 📋 系統架構

### TAFL 在 RosAGV 中的位置
```
RosAGV 系統架構
├── AGV 車載系統
│   └── 狀態機控制
├── AGVC 管理系統
│   ├── Web API (Port 8000)
│   ├── AGVCUI (Port 8001)
│   └── TAFL WCS ← 您在這裡
│       ├── TAFL Parser
│       ├── TAFL Executor
│       └── TAFL Editor
└── 外部系統整合
    └── KUKA Fleet
```

### 核心組件

#### 1. TAFL Language (語言規範)
- **版本**: v1.1.2
- **格式**: YAML-based
- **特性**: 結構化動詞系統、變數管理、流程控制

#### 2. TAFL Editor (視覺化編輯器)
- **位置**: AGVCUI 中的 `/tafl/editor`
- **功能**: 拖放式流程設計、即時驗證、視覺化預覽
- **輸出**: 標準 TAFL YAML 檔案

#### 3. TAFL WCS (執行引擎)
- **工作空間**: `tafl_wcs_ws`
- **功能**: 解析和執行 TAFL 流程
- **整合**: 與資料庫、ROS 2、外部系統完全整合

## 🚀 主要優勢

### 相比 Linear Flow v2 的改進

| 特性 | Linear Flow v2 | TAFL v1.1.2 | 改進說明 |
|------|----------------|------------|---------|
| **語法結構** | 鬆散的步驟列表 | 結構化動詞系統 | 更清晰的語義 |
| **變數管理** | 簡單字串替換 | 完整變數作用域 | 避免變數污染 |
| **錯誤處理** | 基本錯誤檢查 | 結構化錯誤處理 | 更好的除錯能力 |
| **視覺化** | Linear Flow Designer | TAFL Editor | 更直觀的介面 |
| **條件控制** | 簡單條件 | if/switch 完整支援 | 複雜邏輯處理 |
| **迴圈支援** | 有限 | for 迴圈與過濾 | 集合處理能力 |

### 實際應用優勢

1. **更直觀的流程設計**
   - 使用動詞（verbs）表達意圖
   - 清晰的流程控制結構
   - 視覺化編輯支援

2. **更強大的功能**
   - 支援複雜的條件判斷 (if/else)
   - 迴圈和迭代控制 (for)
   - 多分支選擇 (switch)

3. **更好的維護性**
   - 標準化的語法
   - 完整的文檔支援
   - 內建驗證機制

## 🔧 TAFL 核心功能

### 支援的動詞 (10個)
1. **query** - 查詢資料
2. **check** - 條件檢查
3. **create** - 創建資源
4. **update** - 更新資源
5. **set** - 設定變數
6. **if** - 條件分支
7. **for** - 迴圈迭代
8. **switch** - 多分支選擇
9. **stop** - 停止執行
10. **notify** - 發送通知

### 變數和表達式
- **變數引用**: `${variable_name}`
- **物件屬性**: `${object.property}`
- **陣列索引**: `${array[0]}`
- **算術運算**: `+`, `-`, `*`, `/`
- **比較運算**: `==`, `!=`, `>`, `<`, `>=`, `<=`
- **邏輯運算**: `and`, `or`, `not`

## 📊 TAFL 流程範例

### 簡單的 AGV 任務流程
```yaml
metadata:
  id: simple_agv_task
  name: 簡單 AGV 運輸任務
  version: "1.0"

variables:
  agv_id: "agv_001"
  destination: "station_A"

flow:
  # 創建任務
  - create:
      target: task
      with:
        agv_id: "${agv_id}"
        type: "transport"
      as: new_task

  # 檢查任務狀態
  - check:
      condition: "${new_task.status} == 'created'"
      as: task_ready

  - if:
      condition: "${task_ready}"
      then:
        - update:
            target: agv
            where:
              id: "${agv_id}"
            set:
              destination: "${destination}"
              status: "moving"
        - notify:
            level: info
            message: "AGV ${agv_id} 開始移動到 ${destination}"
      else:
        - notify:
            level: error
            message: "任務創建失敗"
```

### 批次任務處理
```yaml
metadata:
  id: batch_processing
  name: 批次任務處理
  version: "1.1"

flow:
  # 查詢待處理任務
  - query:
      target: tasks
      where:
        status: "pending"
      limit: 10
      as: pending_tasks

  # 迭代處理每個任務
  - for:
      as: task
      in: "${pending_tasks}"
      filter: "${task.priority} > 3"
      do:
        - update:
            target: task
            where:
              id: "${task.id}"
            set:
              status: "processing"
              start_time: "${now()}"

        - notify:
            level: info
            message: "處理任務 ${task.id}"
```

### 優先級分類處理
```yaml
metadata:
  id: priority_handler
  name: 優先級分類處理
  version: "1.0"

variables:
  task_priority: 5

flow:
  - switch:
      expression: "${task_priority}"
      cases:
        - when: 1
          do:
            - set:
                urgency: "low"
                timeout: 3600
        - when: 5
          do:
            - set:
                urgency: "normal"
                timeout: 1800
        - when: 10
          do:
            - set:
                urgency: "high"
                timeout: 600
        - when: "default"
          do:
            - set:
                urgency: "unknown"
                timeout: 1800

  - notify:
      level: info
      message: "任務緊急程度: ${urgency}, 超時時間: ${timeout}秒"
```

## 🔧 開發者資訊

### TAFL 檔案位置
- **正式配置**: `/home/ct/RosAGV/app/config/tafl/`
- **測試檔案**: `/home/ct/RosAGV/app/tafl_ws/migrated_flows/`
- **工作空間**: `/home/ct/RosAGV/app/tafl_ws/`

### 相關工作空間
- **tafl_ws**: TAFL 核心語言實作
- **tafl_wcs_ws**: TAFL WCS 執行引擎
- **web_api_ws**: 包含 TAFL Editor

### 驗證工具
```bash
# 驗證 TAFL 檔案格式
r tafl-validate my_flow.yaml

# 驗證所有 TAFL 檔案
r tafl-validate all

# 列出所有 TAFL 檔案
r tafl-validate list
```

### Python 整合範例
```python
#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/ct/RosAGV/app/tafl_ws/src/tafl')

from tafl.parser import TAFLParser
from tafl.executor import TAFLExecutor

# 解析 TAFL 檔案
parser = TAFLParser()
ast = parser.parse_file('my_flow.tafl.yaml')

# 執行流程
executor = TAFLExecutor()
result = await executor.execute(ast)
```

## 💡 最佳實踐

### 1. 命名規範
- 使用描述性的 flow ID
- 變數名稱使用 snake_case
- 動詞保持簡潔明確

### 2. 流程設計
- 將複雜流程分解為小步驟
- 善用條件判斷避免錯誤
- 適當使用日誌記錄

### 3. 錯誤處理
- 在關鍵步驟使用 check 驗證
- 使用 if/else 處理異常情況
- 記錄詳細的錯誤資訊

### 4. 效能優化
- 使用 query 的 limit 參數
- 在 for 迴圈使用 filter
- 避免深層巢狀迴圈

## ⚠️ 限制和注意事項

### 當前限制
TAFL v1.1.2 目前**不支援**：
- ❌ 刪除操作 (`delete` 動詞)
- ❌ While 迴圈
- ❌ 函數呼叫
- ❌ 外部命令執行
- ❌ 平行執行
- ❌ Break/Continue 語句（使用 stop 作為替代）
- ❌ Return 語句

### 版本相容性
- **v1.1.2**: `set` 只接受字典格式
- **v1.1.3**: `check` 必須包含 `as` 參數

## 🚨 常見問題

### Q: 如何從 Linear Flow 遷移到 TAFL？
A: 系統提供自動遷移工具，但建議重新設計流程以充分利用 TAFL 的新特性。

### Q: TAFL Editor 在哪裡？
A: 在 AGVCUI (Port 8001) 的 `/tafl/editor` 路徑。

### Q: 可以同時使用 Linear Flow 和 TAFL 嗎？
A: 不建議。系統已經完全遷移到 TAFL，Linear Flow 僅供歷史參考。

### Q: 如何處理迴圈中的 break？
A: 使用 `stop` 語句配合 `reason` 參數來終止迴圈執行。

## 🔗 相關文檔
- [TAFL 語言規範](./tafl-language.md)
- [TAFL Editor 使用指南](./tafl-editor.md)
- [從 Linear Flow 遷移](./tafl-migration.md)