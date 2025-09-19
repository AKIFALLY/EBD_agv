# TAFL Parser Workspace CLAUDE.md

## 📚 Context Loading
../../CLAUDE.md  # 引用根目錄系統文档

# TAFL 專業知識（工作空間層）
@docs-ai/knowledge/system/tafl/tafl-language-specification.md
@docs-ai/knowledge/system/tafl/tafl-implementation-plan.md
@docs-ai/knowledge/system/tafl/tafl-implementation-project.md
@docs-ai/knowledge/system/tafl/tafl-quick-start-guide.md
@docs-ai/knowledge/system/tafl/tafl-editor-specification.md

# 開發標準
@docs-ai/operations/development/testing/testing-standards.md

## 🎯 Module Overview
**TAFL Parser** (Task Automation Flow Language Parser) 是 TAFL v1.1 語言的核心解析器和執行引擎，提供完整的 YAML 解析、語法驗證、AST 構建和執行功能。這是所有 TAFL 相關系統的基礎模組。

## 🔧 Core Features
- **TAFL v1.1 解析器**: 完整支援 6 段式結構解析（metadata, settings, preload, rules, variables, flow）
- **AST 構建**: 將 YAML 轉換為抽象語法樹 (Abstract Syntax Tree)
- **語法驗證**: 嚴格的語法檢查和錯誤報告
- **執行引擎**: 支援所有 TAFL v1.1 動詞執行
- **變數管理**: 5-Level 變數作用域管理
- **擴展機制**: 可插拔的函數和動詞擴展

## 📁 Project Structure
```
tafl_ws/
├── src/
│   └── tafl/                        # TAFL 核心模組
│       ├── __init__.py
│       ├── parser.py                # TAFL 解析器
│       ├── executor.py              # TAFL 執行引擎
│       ├── validator.py            # 語法驗證器
│       ├── ast_nodes.py            # AST 節點定義
│       ├── variables.py            # 變數管理器
│       ├── functions.py            # 內建函數庫
│       └── verbs/                  # 動詞實作
│           ├── __init__.py
│           ├── query.py            # query 動詞
│           ├── check.py            # check 動詞
│           ├── create.py           # create 動詞
│           ├── update.py           # update 動詞
│           ├── call.py             # call 動詞
│           ├── wait.py             # wait 動詞
│           ├── log.py              # log 動詞
│           ├── for_loop.py         # for 迴圈
│           ├── switch.py           # switch 條件
│           └── set.py              # set 變數
├── examples/                       # 範例 TAFL 檔案
│   ├── simple_query.yaml
│   ├── rack_rotation.yaml
│   └── complex_workflow.yaml
├── docs/                           # 技術文檔
│   ├── parser_design.md
│   └── execution_model.md
└── run_tests.sh                    # 測試腳本
```

## 🔍 Key Technical Details

### Parser Architecture
```python
# TAFL 解析流程
class TAFLParser:
    def parse(self, yaml_content: str) -> TAFLDocument:
        # 1. YAML 解析
        raw_data = yaml.safe_load(yaml_content)

        # 2. 結構驗證
        self._validate_structure(raw_data)

        # 3. AST 構建
        ast = self._build_ast(raw_data)

        # 4. 語義分析
        self._semantic_analysis(ast)

        return ast
```

### Execution Model
```python
# 4-Phase 執行模型
class TAFLExecutor:
    def execute(self, ast: TAFLDocument):
        # Phase 1: Settings
        self._execute_settings(ast.settings)

        # Phase 2: Preload
        self._execute_preload(ast.preload)

        # Phase 3: Rules (read-only)
        self._load_rules(ast.rules)

        # Phase 4: Variables
        self._initialize_variables(ast.variables)

        # Execute main flow
        self._execute_flow(ast.flow)
```

### Variable Scopes
```python
# 5-Level 變數作用域
class VariableManager:
    def __init__(self):
        self.scopes = {
            'rules': {},      # Level 1: Rules scope
            'preload': {},    # Level 2: Preload scope
            'global': {},     # Level 3: Global scope
            'flow': {},       # Level 4: Flow scope
            'loop': {}        # Level 5: Loop scope
        }

    def resolve(self, var_name: str):
        # 從最內層到最外層搜尋
        for scope in ['loop', 'flow', 'global', 'preload', 'rules']:
            if var_name in self.scopes[scope]:
                return self.scopes[scope][var_name]
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

# 解析 TAFL 檔案
parser = TAFLParser()
with open('flow.tafl.yaml', 'r') as f:
    ast = parser.parse(f.read())

# 執行流程
executor = TAFLExecutor()
executor.execute(ast)
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
- TAFL 故障排除: @docs-ai/knowledge/system/tafl/tafl-troubleshooting-guide.md
- TAFL WCS 實作: `app/tafl_wcs_ws/CLAUDE.md`
- TAFL Editor: `app/web_api_ws/src/agvcui/CLAUDE.md`

## 📅 Development Timeline
- **2024-12**: 初始 TAFL v1.0 解析器實作
- **2025-01**: 升級至 TAFL v1.1 規格
  - 新增 6 段式結構支援
  - 實作 5-Level 變數作用域
  - 增強動詞支援（switch 範圍、set 多格式）
- **2025-09**: 整合至 tafl_wcs_ws 系統

## 💡 Design Decisions
1. **純 Python 實作**: 不依賴 ROS 2，可獨立使用
2. **模組化設計**: 解析器、執行器、驗證器分離
3. **可擴展架構**: 支援自定義動詞和函數
4. **嚴格驗證**: 在執行前進行完整的語法和語義檢查