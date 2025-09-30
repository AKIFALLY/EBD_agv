# TAFL Parser Workspace

**TAFL (Task Automation Flow Language) v1.1.2** - AGV/WCS 任務自動化流程語言的核心解析器和執行引擎

## 🎯 概述

TAFL 是專為 RosAGV 系統設計的 YAML-based DSL，提供簡潔且強大的流程定義能力：

- **10 個核心動詞**：涵蓋所有自動化需求
- **4-Phase 執行模型**：Settings → Preload → Rules → Flow
- **5-Level 變數作用域**：Loop → Flow → Global → Preload → Rules
- **高效能設計**：預載資料快取、規則最佳化

## 🚀 快速開始

### 建置和安裝

```bash
# 在容器內建置
cd /app/tafl_ws
colcon build --packages-select tafl
source install/setup.bash
```

### 基本使用

```python
from tafl.parser import TAFLParser
from tafl.executor import TAFLExecutor

# 解析 TAFL 檔案
parser = TAFLParser()
with open('flow.tafl.yaml', 'r') as f:
    ast = parser.parse(f.read())

# 執行流程
executor = TAFLExecutor()
result = await executor.execute(ast)
```

### 簡單範例

```yaml
metadata:
  id: example_001
  name: Simple Example
  version: 1.1.2

variables:
  room_id: 1

flow:
  - query:
      target: locations
      where: {room_id: "${room_id}"}
      as: locations  # 注意：使用 as，不是 store_as

  - check:
      condition: "${locations.length} > 0"
      as: has_locations  # CHECK 也使用 as

  - if:
      condition: "${has_locations}"
      then:
        - create:
            target: task
            with: {location_id: "${locations[0].id}"}
            as: new_task  # CREATE 也使用 as
```

## 📚 語言規範

### 6 段式程式結構

```yaml
metadata:    # 可選：程式元資料
settings:    # 可選：執行設定
preload:     # 可選：資料預載（Phase 2）
rules:       # 可選：業務規則（Phase 3）
variables:   # 可選：變數初始化
flow:        # 必要：主流程（Phase 4）
```

### 10 個核心動詞

| 動詞 | 用途 | 語法 |
|------|------|------|
| **query** | 查詢資料 | `query: {target: x, where: {...}, as: result}` |
| **check** | 檢查條件 | `check: {condition: expr, as: result}` |
| **create** | 創建資源 | `create: {target: x, with: {...}, as: result}` |
| **update** | 更新資料 | `update: {target: x, where: {...}, set: {...}}` |
| **if** | 條件判斷 | `if: {condition: expr, then: [...], else: [...]}` |
| **for** | 迴圈處理 | `for: {each: item, in: list, do: [...]}` |
| **switch** | 多分支選擇 | `switch: {on: expr, cases: {...}, default: [...]}` |
| **set** | 設定變數 | `set: {var1: value1, var2: value2}` |
| **stop** | 停止執行 | `stop: {reason: "message"}` |
| **notify** | 發送通知 | `notify: {channel: x, message: "..."}` |

### ⚠️ 重要：統一參數命名

所有動詞都使用 **`as`** 參數儲存結果（不是 `store_as`）：
- ✅ `query: {target: locations, as: result}`
- ✅ `check: {condition: expr, as: is_valid}`
- ✅ `create: {target: task, as: new_task}`
- ❌ ~~`store_as: result`~~ （錯誤寫法）

### 表達式系統

使用 `${}` 進行變數插值和表達式求值：

```yaml
# 變數引用
set: {message: "Count is ${count}"}

# 數學運算
set: {total: "${price * quantity}"}

# 邏輯運算 (Python 風格)
check: {condition: "${x > 0 and y < 10}", as: valid}

# 物件屬性存取
set: {id: "${task.metadata.id}"}
```

## 📁 專案結構

```
tafl_ws/
├── README.md           # 本文件
├── CLAUDE.md          # AI Agent 指導文件
├── src/
│   └── tafl/
│       ├── setup.py
│       ├── tafl/      # 核心模組
│       │   ├── __init__.py
│       │   ├── parser.py        # YAML → AST 解析器
│       │   ├── executor.py      # 4-Phase 執行引擎
│       │   ├── validator.py     # 語法驗證器
│       │   └── ast_nodes.py     # AST 節點定義
│       └── test/      # 測試套件
│           ├── test_parser.py
│           ├── test_executor.py
│           ├── test_validator.py
│           └── test_verbs.py
├── examples/          # 範例檔案
│   ├── task_creation_flow.yaml
│   └── rack_rotation_flow.yaml
├── build/            # 建置輸出
└── install/          # 安裝檔案
```

## 🧪 測試

### 執行測試

```bash
cd /app/tafl_ws/src/tafl
python3 -m pytest test/ -v
```

### 測試覆蓋

- **Parser Tests**: 5 passed
- **Executor Tests**: 13 passed
- **Validator Tests**: 1 passed
- **總計**: 19 passed, 1 skipped

Skipped: `test_external_functions` (需要外部函數註冊)

### 測試特定功能

```bash
# 測試解析器
python3 test/test_parser.py

# 測試執行器
python3 test/test_executor.py

# 測試動詞實作
python3 test/test_verbs.py
```

## 💻 API 使用指南

### Parser API

```python
from tafl.parser import TAFLParser

parser = TAFLParser()

# 從檔案解析
ast = parser.parse_file('flow.yaml')

# 從字串解析
yaml_content = """
flow:
  - set: {counter: 0}
"""
ast = parser.parse_string(yaml_content)
```

### Executor API

```python
from tafl.executor import TAFLExecutor

# 基本執行
executor = TAFLExecutor()
result = await executor.execute(ast)

# 使用自訂函數
def my_function(x, y):
    return x + y

executor = TAFLExecutor(function_registry={
    'my_function': my_function
})
```

### Validator API

```python
from tafl.validator import TAFLValidator

validator = TAFLValidator()

# 驗證 AST
if validator.validate(ast):
    print("✅ Valid TAFL")
else:
    for error in validator.errors:
        print(f"❌ {error}")
```

## ⚙️ 技術細節

### 4-Phase 執行模型

1. **Settings Phase**: 載入執行參數（timeout、retry等）
2. **Preload Phase**: 預載並快取常用資料
3. **Rules Phase**: 定義業務規則（唯讀）
4. **Flow Phase**: 執行主流程邏輯

### 5-Level 變數作用域

優先級從高到低：
1. **Loop Scope**: 迴圈變數（如 for 中的 each）
2. **Flow Scope**: 流程中設定的變數
3. **Global Scope**: variables 段定義的變數
4. **Preload Scope**: 預載的資料快取
5. **Rules Scope**: 規則定義（唯讀）

### 效能指標

- **Parser**: < 10ms (10 statements), < 200ms (200 statements)
- **Executor**: < 1ms per statement (不含外部呼叫)
- **Memory**: ~1MB (typical flow), ~2MB (with variables)

## 🔌 整合狀態

### 已完成
- ✅ 獨立 TAFL 解析器和執行器
- ✅ 完整測試套件
- ✅ 範例檔案

### 進行中
- 🔄 tafl_wcs_ws 系統整合
- 🔄 外部函數註冊機制

### 計劃中
- ⏳ 標準函數庫（sum、avg、count等）
- ⏳ TAFL Editor UI 整合
- ⏳ 除錯工具

## 📖 相關文檔

### 知識庫文檔
- TAFL 語言規範：`@docs-ai/knowledge/system/tafl/tafl-language-specification.md`
- TAFL 開發歷史：`@docs-ai/knowledge/system/tafl/tafl-development-history.md`
- TAFL 使用者指南：`@docs-ai/knowledge/system/tafl/tafl-user-guide.md`

### 工作空間文檔
- TAFL WCS 整合：`../tafl_wcs_ws/CLAUDE.md`
- TAFL Editor：`../web_api_ws/src/agvcui/CLAUDE.md`

## 🔄 版本歷史

- **v1.1.2** (2025-09): 語法標準化，統一使用 `as` 參數
- **v1.1** (2025-08): 新增 preload/rules，4-Phase 執行，5-Level 作用域
- **v1.0** (2025-08): 初始版本，10 個核心動詞

## 📝 授權

內部使用，擎添工業 (Ching Tech) 版權所有