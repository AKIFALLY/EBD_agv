# TAFL Parser Workspace CLAUDE.md

## 📚 Context Loading
../../CLAUDE.md  # 引用根目錄系統文档

# TAFL 專業知識（工作空間層）
@docs-ai/knowledge/system/tafl/tafl-language-specification.md
@docs-ai/knowledge/system/tafl/tafl-development-history.md
@docs-ai/knowledge/system/tafl/tafl-user-guide.md
@docs-ai/knowledge/system/tafl/tafl-editor-specification.md

# 開發標準
@docs-ai/operations/development/testing/testing-standards.md

## 🎯 Module Overview
**TAFL Parser** (Task Automation Flow Language Parser) 是 TAFL v1.1.2 語言的核心解析器和執行引擎，提供完整的 YAML 解析、語法驗證、AST 構建和執行功能。這是所有 TAFL 相關系統的基礎模組。

## 🔧 Core Features
- **TAFL v1.1.2 解析器**: 完整支援 6 段式結構解析（metadata, settings, preload, rules, variables, flow）
- **AST 構建**: 將 YAML 轉換為抽象語法樹 (Abstract Syntax Tree)
- **語法驗證**: 嚴格的語法檢查和錯誤報告
- **執行引擎**: 支援所有 TAFL v1.1.2 動詞執行
- **變數管理**: 5-Level 變數作用域管理
- **擴展機制**: 可插拔的函數和動詞擴展

## 📁 Project Structure
```
tafl_ws/
├── src/
│   └── tafl/
│       ├── setup.py                 # Python 套件設定
│       ├── tafl/                    # TAFL 核心模組
│       │   ├── __init__.py
│       │   ├── parser.py            # TAFL 解析器
│       │   ├── executor.py          # TAFL 執行引擎
│       │   ├── validator.py         # 語法驗證器
│       │   ├── ast_nodes.py         # AST 節點定義
│       │   └── cli.py               # CLI 工具
│       └── test/                    # 測試套件
│           ├── __init__.py
│           ├── test_parser.py       # 解析器測試
│           ├── test_executor.py     # 執行器測試
│           ├── test_validator.py    # 驗證器測試
│           ├── test_verbs.py        # 動詞測試
│           └── test_strict_v112.py  # v1.1.2 嚴格測試
├── examples/                        # 範例 TAFL 檔案
│   ├── rack_rotation_flow.yaml     # 貨架旋轉流程
│   ├── simple_flow.yaml            # 簡單流程範例
│   ├── simple_test.yaml            # 測試流程
│   └── task_creation_flow.yaml     # 任務創建流程
├── README.md                        # 主要文檔
├── CLAUDE.md                        # AI Agent 指導文件
└── run_tests.sh                     # 測試腳本
```

## 🔍 Key Technical Details

**注意**: 以下程式碼為概念性示例，展示架構設計而非實際實作細節。

### Parser Architecture (概念示例)
```python
# TAFL 解析流程概念
class TAFLParser:
    # 實際實作的主要方法：
    # - parse_file(file_path: str) -> TAFLProgram
    # - parse_string(yaml_content: str) -> TAFLProgram
    # - parse_program(data: Dict) -> TAFLProgram

    def parse_string(self, yaml_content: str) -> TAFLProgram:
        # 1. YAML 解析
        data = yaml.safe_load(yaml_content)

        # 2. 調用 parse_program 進行完整解析
        return self.parse_program(data)

    def parse_program(self, data: Dict[str, Any]) -> TAFLProgram:
        # 解析各個段落
        metadata = self._parse_metadata(data.get('metadata', {}))
        settings = self._parse_settings(data.get('settings', {}))
        preload = self._parse_preload(data.get('preload', []))
        rules = self._parse_rules(data.get('rules', {}))
        variables = data.get('variables', {})
        flow = self._parse_statements(data.get('flow', []))

        return TAFLProgram(
            metadata=metadata,
            settings=settings,
            preload=preload,
            rules=rules,
            variables=variables,
            flow=flow
        )
```

### Execution Model (概念示例)
```python
# 4-Phase 執行模型概念
# 實際實作為 async def execute(...)
class TAFLExecutor:
    async def execute(self, program: TAFLProgram):
        # Phase 1: Settings
        self._execute_settings(program.settings)

        # Phase 2: Preload
        self._execute_preload(program.preload)

        # Phase 3: Rules (read-only)
        self._load_rules(program.rules)

        # Phase 4: Variables & Flow
        self._initialize_variables(program.variables)
        await self._execute_flow(program.flow)
```

### Variable Scopes (實際實作)
```python
# 5-Level 變數作用域
# 實際在 TAFLExecutor 中直接管理，非獨立類別
class TAFLExecutor:
    def __init__(self):
        # TAFL v1.1.2: 5-level variable scoping
        self.rules_scope = {}      # Level 1: Rules scope (read-only)
        self.preload_scope = {}    # Level 2: Preload scope (cached)
        self.global_scope = {}     # Level 3: Global scope
        self.flow_scope = {}       # Level 4: Flow scope
        self.loop_scope = {}       # Level 5: Loop scope (current)

    def resolve_variable(self, var_name: str):
        # 從最內層到最外層搜尋
        for scope in [self.loop_scope, self.flow_scope,
                     self.global_scope, self.preload_scope,
                     self.rules_scope]:
            if var_name in scope:
                return scope[var_name]
        raise VariableNotFoundError(var_name)
```

## 🚀 Development Workflow

### Building
```bash
cd /app/tafl_ws
colcon build --packages-select tafl
source install/setup.bash
```

### Testing
```bash
# 執行所有測試
./run_tests.sh

# 測試特定功能
python3 -m pytest src/tafl/test/test_parser.py -v
python3 -m pytest src/tafl/test/test_executor.py -v
```

### Usage Example
```python
from tafl.parser import TAFLParser
from tafl.executor import TAFLExecutor
import asyncio

# 解析 TAFL 檔案
parser = TAFLParser()

# 方法 1: 直接解析檔案
program = parser.parse_file('flow.tafl.yaml')

# 方法 2: 解析字串內容
# with open('flow.tafl.yaml', 'r') as f:
#     program = parser.parse_string(f.read())

# 執行流程（異步執行）
executor = TAFLExecutor()
asyncio.run(executor.execute(program))
```

## 🚨 Common Issues and Solutions

### Issue: YAML 解析錯誤
**問題**: Invalid YAML syntax
**解決**: 檢查 YAML 格式，特別是縮排和特殊字元

### Issue: 未定義的變數
**問題**: Variable not found: ${var_name}
**解決**: 確保變數在使用前已定義，檢查作用域

### Issue: 動詞參數錯誤
**問題**: Missing required parameter for verb
**解決**: 參考 TAFL 規格書確認必要參數

## 🔗 Related Documentation
- TAFL 語言規格: @docs-ai/knowledge/system/tafl/tafl-language-specification.md
- TAFL API 參考: @docs-ai/knowledge/system/tafl/tafl-api-reference.md
- TAFL 使用者指南: @docs-ai/knowledge/system/tafl/tafl-user-guide.md
- TAFL WCS 實作: `app/tafl_wcs_ws/CLAUDE.md`
- TAFL Editor: `app/web_api_ws/src/agvcui/CLAUDE.md`

## 📅 Development Timeline
- **2025-08**: 初始 TAFL v1.0 解析器實作與 Flow WCS 整合
- **2025-08**: 升級至 TAFL v1.1 規格
  - 新增 6 段式結構支援
  - 實作 5-Level 變數作用域
  - 增強動詞支援（switch 範圍、set 多格式）
- **2025-09**: 語法標準化 v1.1.2，統一使用 `as` 參數

## 💡 Design Decisions
1. **純 Python 實作**: 不依賴 ROS 2，可獨立使用
2. **模組化設計**: 解析器、執行器、驗證器分離
3. **可擴展架構**: 支援自定義動詞和函數
4. **嚴格驗證**: 在執行前進行完整的語法和語義檢查