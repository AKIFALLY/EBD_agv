# 開發建置測試工具 Prompt

## 🎯 適用場景
- ROS 2 工作空間快速建置
- 單元測試和整合測試執行
- 代碼品質檢查和分析
- 開發環境狀態檢查
- 部署前驗證和測試

## 📋 核心工具概述
RosAGV 開發工具集，提供智能建置、自動化測試、代碼分析和部署輔助功能，支援多工作空間環境的統一開發管理。

## 🔧 開發工具指導

### 🚀 超簡單使用方式 (推薦)
```bash
# 使用統一工具入口 (最簡單)
r dev-status            # 開發環境狀態檢查
r dev-build             # 快速建置 (需容器環境)
r dev-test              # 快速測試 (需容器環境)
r dev-check             # 代碼檢查 (需容器環境)
```

### 載入專業工具集 (進階使用)
```bash
# 載入開發工具集
source scripts/dev-tools/dev-tools.sh

# 顯示可用工具
show_dev_tools_help
```

## 🔧 主要工具說明

### 1. 建置輔助工具 (build-helper.sh)
智能建置輔助工具，支援多種建置配置和工作空間管理。

```bash
scripts/dev-tools/build-helper.sh [config] [options]
```

**建置配置**:
- `fast` - 快速建置，跳過測試
- `full` - 完整建置，包含所有測試
- `incremental` - 增量建置，只建置變更部分
- `debug` - 除錯版本建置
- `release` - 發布版本建置

**常用範例**:
```bash
# 快速建置特定工作空間
scripts/dev-tools/build-helper.sh fast --workspace agv_ws

# 完整建置所有工作空間
scripts/dev-tools/build-helper.sh full

# 增量建置 (只建置有變更的)
scripts/dev-tools/build-helper.sh incremental
```

### 2. 測試執行工具 (test-runner.sh)
全面的測試執行和報告工具，支援多種測試框架。

```bash
scripts/dev-tools/test-runner.sh [test_type] [options]
```

**測試類型**:
- `unit` - 單元測試
- `integration` - 整合測試
- `system` - 系統測試
- `performance` - 性能測試
- `regression` - 回歸測試

**常用範例**:
```bash
# 執行單元測試並生成覆蓋率報告
scripts/dev-tools/test-runner.sh unit --coverage

# 執行整合測試
scripts/dev-tools/test-runner.sh integration --workspace agv_ws

# 性能測試
scripts/dev-tools/test-runner.sh performance --benchmark
```

### 3. 代碼分析工具 (code-analyzer.sh)
代碼品質分析和檢查工具，整合多種靜態分析工具。

```bash
scripts/dev-tools/code-analyzer.sh [analysis_type] [options]
```

**分析類型**:
- `style` - 代碼風格檢查
- `quality` - 代碼品質分析
- `security` - 安全漏洞掃描
- `ros2` - ROS 2 最佳實踐檢查
- `complexity` - 複雜度分析
- `dependencies` - 依賴關係分析
- `performance` - 性能分析
- `documentation` - 文檔完整性檢查

**常用範例**:
```bash
# 代碼風格檢查
scripts/dev-tools/code-analyzer.sh style --severity warning

# 安全掃描
scripts/dev-tools/code-analyzer.sh security --format json

# ROS 2 最佳實踐檢查
scripts/dev-tools/code-analyzer.sh ros2 --workspace agv_ws
```

### 4. 部署輔助工具 (deploy-helper.sh)
應用程式部署管理工具，支援多種部署模式。

```bash
scripts/dev-tools/deploy-helper.sh [mode] [options]
```

**部署模式**:
- `development` - 開發環境部署
- `staging` - 測試環境部署
- `production` - 生產環境部署
- `local` - 本地部署
- `docker` - Docker 容器部署

**常用範例**:
```bash
# 開發環境部署
scripts/dev-tools/deploy-helper.sh development --components agvc

# 生產環境部署 (包含預檢查)
scripts/dev-tools/deploy-helper.sh production --pre-check

# 檢查部署狀態
scripts/dev-tools/deploy-helper.sh status
```

### 5. 統一開發工具 (dev-tools.sh)
整合所有開發工具的統一介面，提供工作流自動化。

```bash
scripts/dev-tools/dev-tools.sh [workflow] [options]
```

**智能工作流**:
- `dev-setup` - 初始化開發環境
- `build-test` - 執行建置和測試流程
- `code-check` - 執行完整代碼品質檢查
- `deploy-dev` - 開發環境部署
- `full-ci` - 完整 CI/CD 流程
- `doctor` - 開發環境診斷

**常用範例**:
```bash
# 初始化開發環境
scripts/dev-tools/dev-tools.sh dev-setup

# 完整建置測試流程
scripts/dev-tools/dev-tools.sh build-test --workspace agv_ws

# CI/CD 完整流程
scripts/dev-tools/dev-tools.sh full-ci --mode production
```

## 🚀 開發工作流程範例

### 標準開發流程
```bash
# 1. 環境設置
scripts/dev-tools/dev-tools.sh dev-setup

# 2. 代碼品質檢查
scripts/dev-tools/dev-tools.sh code-check

# 3. 建置和測試
scripts/dev-tools/dev-tools.sh build-test

# 4. 部署到開發環境
scripts/dev-tools/dev-tools.sh deploy-dev
```

### 持續整合流程
```bash
# CI 流程
scripts/dev-tools/dev-tools.sh full-ci --mode staging

# 包含以下步驟：
# - 環境檢查
# - 代碼分析
# - 建置所有工作空間
# - 執行完整測試套件
# - 生成報告
# - 部署到指定環境
```

### 品質保證流程
```bash
# 全面代碼品質檢查
scripts/dev-tools/code-analyzer.sh style --severity error
scripts/dev-tools/code-analyzer.sh security --format detailed
scripts/dev-tools/code-analyzer.sh ros2 --best-practices

# 性能測試
scripts/dev-tools/test-runner.sh performance --benchmark --report
```

## 🔧 便捷函數使用

### 快速操作函數
```bash
# 載入便捷函數
source scripts/dev-tools/dev-tools.sh

# 使用便捷函數
dev_build --workspace agv_ws            # 快速建置
dev_test --type unit                    # 快速測試
dev_check --severity warning            # 快速代碼檢查
dev_deploy                              # 快速部署
dev_status                              # 顯示狀態
```

### 批量操作
```bash
# 批量工作空間建置
for ws in agv_ws agvc_ws db_proxy_ws; do
    dev_build --workspace $ws
done

# 批量測試
dev_test --type unit --all-workspaces
```

## 📊 報告和輸出格式

### 支援的輸出格式
- **Console**: 終端彩色輸出
- **JSON**: 結構化數據，便於自動化處理
- **HTML**: 詳細的網頁報告
- **XML/JUnit**: 適用於 CI/CD 系統
- **CSV**: 表格數據，便於分析

### 報告生成範例
```bash
# 生成 HTML 測試報告
scripts/dev-tools/test-runner.sh unit --format html --output test-report.html

# 生成 JSON 代碼分析報告
scripts/dev-tools/code-analyzer.sh quality --format json --output analysis.json

# 生成建置摘要報告
scripts/dev-tools/build-helper.sh full --report --output build-summary.txt
```

## 🔧 進階功能

### 並行建置
```bash
# 並行建置多個工作空間
scripts/dev-tools/build-helper.sh full --parallel --jobs 4

# 並行測試執行
scripts/dev-tools/test-runner.sh unit --parallel --workers 2
```

### 條件式操作
```bash
# 只有在代碼變更時才建置
scripts/dev-tools/build-helper.sh incremental --if-changed

# 只有在測試通過時才部署
scripts/dev-tools/deploy-helper.sh development --if-tests-pass
```

### 自訂配置
```bash
# 使用自訂配置檔案
scripts/dev-tools/dev-tools.sh build-test --config custom-build.yaml

# 設定環境變數
BUILD_TYPE=release scripts/dev-tools/build-helper.sh full
```

## 🚨 故障排除

### 建置問題
```bash
# 清理建置快取
scripts/dev-tools/build-helper.sh clean --all

# 重新建置依賴
scripts/dev-tools/build-helper.sh full --clean-deps

# 除錯模式建置
scripts/dev-tools/build-helper.sh debug --verbose
```

### 測試失敗
```bash
# 詳細測試輸出
scripts/dev-tools/test-runner.sh unit --verbose --debug

# 單獨執行失敗的測試
scripts/dev-tools/test-runner.sh unit --test-filter failed_test_name

# 生成測試覆蓋率報告
scripts/dev-tools/test-runner.sh unit --coverage --coverage-report
```

### 部署問題
```bash
# 檢查部署前置條件
scripts/dev-tools/deploy-helper.sh pre-check --environment development

# 回滾到上一版本
scripts/dev-tools/deploy-helper.sh rollback --version previous

# 檢查部署狀態
scripts/dev-tools/deploy-helper.sh status --detailed
```

## 💡 最佳實踐

### 開發環境設定
1. **初始設置**: 始終使用 `dev-setup` 初始化環境
2. **依賴管理**: 定期檢查和更新依賴關係
3. **環境隔離**: 使用 Docker 容器確保環境一致性

### 代碼品質管理
1. **提交前檢查**: 每次提交前執行代碼品質檢查
2. **持續監控**: 定期執行全面代碼分析
3. **標準遵循**: 嚴格遵循 ROS 2 最佳實踐

### 測試策略
1. **分層測試**: 單元測試 → 整合測試 → 系統測試
2. **覆蓋率要求**: 維持 80% 以上的測試覆蓋率
3. **性能基準**: 定期執行性能測試，建立基準線

### 部署管理
1. **環境一致**: 確保開發、測試、生產環境一致
2. **漸進部署**: 採用藍綠部署或金絲雀發布
3. **監控告警**: 部署後持續監控系統狀態

## 📋 工具快速參考

| 工具 | 主要用途 | 關鍵命令 |
|------|----------|----------|
| `build-helper.sh` | 智能建置管理 | `fast`, `full`, `incremental` |
| `test-runner.sh` | 測試執行報告 | `unit`, `integration`, `performance` |
| `code-analyzer.sh` | 代碼品質分析 | `style`, `security`, `ros2` |
| `deploy-helper.sh` | 部署管理 | `development`, `production`, `status` |
| `dev-tools.sh` | 統一工作流 | `build-test`, `code-check`, `full-ci` |
| 便捷函數 | 快速操作 | `dev_build`, `dev_test`, `dev_check` |