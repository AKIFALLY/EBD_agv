# CLAUDE.md

## 系統概述
ROS 2 Jazzy + Zenoh RMW企業級AGV車隊管理系統，採用雙環境Docker架構(AGV車載+AGVC管理)。

**⚠️ 重要**: 所有ROS 2程式必須在Docker容器內執行，宿主機無ROS 2環境。

## 核心架構
- **雙環境**: AGV車載系統 + AGVC管理系統
- **技術棧**: ROS 2 Jazzy + Zenoh RMW + Docker
- **車輛類型**: Cargo, Loader, Unloader
- **外部整合**: KUKA Fleet系統
- **Web界面**: AGVCUI管理台 + OPUI操作界面

## AI 開發助手指導

### 🔍 核心開發原則
**⚠️ 重要：程式開發時必須遵循以下核心原則**

#### 基於實際程式碼開發
- **追蹤變數來源**: 遇到不明變數時，必須查看 `import` 語句找出定義來源
- **搜尋相關檔案**: 使用 `rg` 工具深入搜尋變數定義和使用範例
- **理解後行動**: 完全理解變數意義、用途和上下文後才進行修改
- **禁止推測**: 絕不使用推測或預想的內容，只使用實際程式內的定義

#### 實施方法
```bash
# 1. 追蹤變數定義
rg "variable_name.*=" --type py -C 3        # 查找變數定義
rg "import.*variable_name" --type py        # 查找導入來源
rg "from.*import.*variable_name" --type py  # 查找具體導入

# 2. 理解變數上下文
rg "variable_name" --type py -C 5           # 查看使用上下文
rg "class.*variable_name|def.*variable_name" --type py  # 查找相關類/函數

# 3. 驗證理解正確性
# 在多個檔案中確認變數一致性使用
# 檢查型別提示和文檔字串
```

### 搜尋策略
- **精確搜尋**: 使用 Grep 工具（基於 ripgrep）進行代碼搜尋
- **模式匹配**: 使用 Glob 工具進行檔案路徑匹配
- **複雜查詢**: 使用 Task 工具處理多步驟搜尋和分析任務
- **並行搜尋**: 同時執行多個搜尋以提高效率

### 模組文檔索引
當涉及以下功能領域時，請讀取對應的詳細 CLAUDE.md：

#### 🚗 AGV 車載系統
- **AGV狀態機**: `app/agv_ws/src/agv_base/CLAUDE.md` - 3層狀態架構詳解
- **車型實現**: `app/agv_ws/src/{cargo_mover_agv,loader_agv,unloader_agv}/CLAUDE.md`
- **手動控制**: `app/agv_cmd_service_ws/CLAUDE.md` - 手動指令服務
- **搖桿控制**: `app/joystick_ws/CLAUDE.md` - USB搖桿整合
- **感測器處理**: `app/sensorpart_ws/CLAUDE.md` - 感測器數據處理

#### 🖥️ AGVC 管理系統  
- **Web API**: `app/web_api_ws/CLAUDE.md` - FastAPI + Socket.IO 詳解
- **資料庫操作**: `app/db_proxy_ws/CLAUDE.md` - PostgreSQL ORM和CRUD
- **設備控制**: `app/ecs_ws/CLAUDE.md` - 門控系統和設備管理
- **倉庫控制**: `app/wcs_ws/CLAUDE.md` - WCS智能調度系統
- **機器人控制**: `app/rcs_ws/CLAUDE.md` - RCS和交通管理

#### 🔗 通訊與整合
- **PLC通訊**: `app/keyence_plc_ws/CLAUDE.md` - Keyence PLC協議
- **PLC代理**: `app/plc_proxy_ws/CLAUDE.md` - ROS 2 PLC服務
- **KUKA整合**: `app/kuka_fleet_ws/CLAUDE.md` - KUKA Fleet Adapter
- **路徑規劃**: `app/path_algorithm/CLAUDE.md` - A*演算法實現

### 🔧 配置管理工具集

專用配置管理工具，用於系統配置、連線測試和問題診斷：

#### Zenoh 路由器配置管理
```bash
scripts/config-tools/zenoh-config.sh [action]
```
**主要功能**：
- `validate/check` - 驗證配置檔案格式和連線狀況
- `show/details` - 顯示詳細配置資訊
- `edit` - 編輯配置檔案（含備份功能）
- `status` - 檢查 Zenoh 服務狀態
- `restart` - 重啟服務指南
- `overview` - 顯示配置概況（預設）

**配置檔案**: `/app/routerconfig.json5` (JSON5格式)
**常用範例**:
```bash
# 檢查配置和連線狀況
scripts/config-tools/zenoh-config.sh validate

# 編輯配置檔案
scripts/config-tools/zenoh-config.sh edit
```

#### 硬體映射配置管理
```bash
scripts/config-tools/hardware-mapping.sh [action] [device_id]
```
**主要功能**：
- `validate/check` - 驗證硬體映射配置
- `list/ls` - 列出所有設備詳細資訊
- `show <device_id>` - 顯示特定設備詳情
- `edit <device_id>` - 編輯設備配置
- `mac <device_id>` - 管理 MAC 地址
- `overview` - 顯示硬體映射概況（預設）

**配置檔案**: `/app/config/hardware_mapping.yaml`
**常用範例**:
```bash
# 檢查硬體映射配置
scripts/config-tools/hardware-mapping.sh validate

# 檢查特定設備
scripts/config-tools/hardware-mapping.sh show cargo01

# 管理設備 MAC 地址
scripts/config-tools/hardware-mapping.sh mac agvc01
```

#### 連線測試最佳實踐
**快速連線測試**:
```bash
# 測試單一端點
timeout 3 bash -c "echo > /dev/tcp/192.168.100.100/7447" 2>/dev/null && echo "✅ 可連接" || echo "❌ 無法連接"

# 批量測試多個端點
for endpoint in "192.168.100.100:7447" "192.168.10.3:7447"; do
    ip=$(echo $endpoint | cut -d: -f1)
    port=$(echo $endpoint | cut -d: -f2)
    if timeout 3 bash -c "echo > /dev/tcp/$ip/$port" 2>/dev/null; then
        echo "✅ tcp/$endpoint (可連接)"
    else
        echo "❌ tcp/$endpoint (無法連接)"
    fi
done
```

**配置管理工作流程**:
1. **測試連線** - 使用連線測試確認端點可達性
2. **更新配置** - 根據測試結果調整配置檔案
3. **驗證配置** - 使用驗證工具確認配置正確性
4. **重啟服務** - 套用配置變更

**故障排除**:
- 配置格式錯誤：使用對應工具的 `validate` 指令
- 連線問題：使用 `/dev/tcp/` 快速測試網路連通性
- 服務狀態：使用 `status` 指令檢查服務運行狀況

### 🐳 Docker 管理工具集

專用 Docker 容器管理工具，提供便捷的容器操作和狀態監控：

#### 工具載入方式
```bash
# 載入所有 Docker 工具函數
source scripts/docker-tools/docker-tools.sh

# 查看可用命令
show_docker_tools_help
```

#### 主要工具說明
1. **agv-container.sh** - AGV 容器專用管理
   - 支援動作: start, stop, restart, exec, logs, health, status
   - 特色: 自動載入 agv_source 環境

2. **agvc-container.sh** - AGVC 容器專用管理
   - 支援動作: start, stop, restart, exec, logs, health, services
   - 特色: 管理完整 AGVC 系統 (含 PostgreSQL, Nginx)

3. **container-status.sh** - 容器狀態檢查
   - 支援動作: all, agv, agvc, ports, resources, network, health
   - 特色: 智能健康診斷和問題建議

4. **quick-exec.sh** - 容器內快速執行
   - 支援快捷方式: node-list, topic-list, check-status 等
   - 特色: 自動環境載入，批量命令執行

#### 常用操作範例
```bash
# 系統管理
all_start                    # 一鍵啟動整個系統
all_health                   # 全面健康檢查
quick_diagnose               # 快速診斷問題

# 開發操作
agv_enter                    # 進入 AGV 開發環境
agvc_enter                   # 進入 AGVC 開發環境
quick_agv "build_all"        # 在 AGV 容器構建
quick_agvc "check_agvc_status"  # 檢查 AGVC 狀態
```

### 🔍 系統診斷和監控工具集

專用系統健康檢查和服務監控工具，提供全方位的系統診斷功能：

#### 工具載入方式
```bash
# 載入所有系統工具函數
source scripts/system-tools/system-tools.sh

# 查看可用命令
show_system_tools_help
```

#### 主要工具說明
1. **health-check.sh** - 全面系統健康檢查
   - 檢查項目: 系統環境、配置檔案、容器狀態、網路連接、服務健康、ROS 2 環境
   - 支援模式: 快速檢查、完整檢查、修復模式、報告生成、定期檢查
   - 特色: 智能評分系統、問題診斷建議、自動修復功能

2. **service-monitor.sh** - 服務狀態監控
   - 監控服務: Zenoh Router, PostgreSQL, Nginx, Web API, AGV/AGVC 容器
   - 監控功能: 實時狀態監控、自動重啟機制、警報系統、依賴關係檢查
   - 特色: 連續監控、歷史記錄、服務詳情分析

3. **system-tools.sh** - 統一系統工具集
   - 整合功能: 健康檢查、服務監控的便捷介面
   - 組合操作: 全面檢查、緊急診斷模式
   - 特色: 一鍵操作、智能診斷、問題修復

#### 常用操作範例
```bash
# 健康檢查
system_health                # 完整健康檢查
system_quick_check           # 快速健康檢查
system_health_fix            # 檢查並嘗試修復問題
system_health_report         # 生成詳細報告

# 服務監控
system_status                # 顯示所有服務狀態
system_monitor --auto-restart # 啟動自動監控與重啟
system_watch                 # 連續監控服務狀態
system_restart postgres      # 重啟特定服務

# 組合診斷
system_full_check            # 執行全面系統檢查
system_emergency_check       # 緊急診斷和修復模式
```

### 📊 日誌分析工具集

專用日誌分析和故障診斷工具，提供智能錯誤識別和解決建議：

#### 工具載入方式
```bash
# 載入所有日誌分析工具函數
source scripts/log-tools/log-tools.sh

# 查看可用命令
show_log_tools_help
```

#### 主要工具說明
1. **log-analyzer.sh** - 智能日誌分析
   - 錯誤模式: 12種預定義錯誤模式 (CRITICAL, ERROR, EXCEPTION, TIMEOUT, CONNECTION 等)
   - 嚴重程度: 5級分級系統 (嚴重錯誤到警告)
   - 分析功能: 統計分析、時間軸、解決建議、JSON輸出
   - 日誌來源: 支援 AGV/AGVC/PostgreSQL/Nginx 容器及自定義檔案

2. **log-tools.sh** - 統一日誌工具集
   - 便捷操作: 快速掃描、錯誤查找、統計分析
   - 高級功能: 即時監控、完整診斷、清理建議
   - 報告生成: 自動生成結構化分析報告

#### 常用操作範例
```bash
# 基本分析
log_analyze agv              # 分析 AGV 容器日誌
log_analyze agvc --stats     # 分析 AGVC 日誌並顯示統計
log_quick_scan               # 快速掃描所有容器錯誤

# 深度分析
log_find_errors "timeout"    # 查找超時錯誤
log_timeline agvc            # 顯示 AGVC 錯誤時間軸
log_suggestions all          # 獲取解決建議

# 監控和報告
log_monitor agv 10           # 每10秒監控 AGV 日誌
log_export_report            # 生成完整分析報告
log_full_diagnosis           # 執行完整診斷

# 直接使用分析工具
scripts/log-tools/log-analyzer.sh all --severity 3 --suggestions
```

### 🌐 網路診斷工具集

專用網路診斷和通訊測試工具，提供全方位的網路連接性、性能和故障排除功能：

#### 工具載入方式
```bash
# 載入所有網路診斷工具函數
source scripts/network-tools/network-tools.sh

# 查看可用命令
show_network_tools_help
```

#### 主要工具說明
1. **zenoh-network.sh** - Zenoh 網路專用診斷
   - 連接檢查: Zenoh Router 連接性和端點可達性測試
   - 性能測試: 通訊延遲和吞吐量測試  
   - 配置分析: Zenoh 配置檔案驗證和最佳化建議
   - 故障排除: 完整的 Zenoh 通訊診斷流程

2. **port-check.sh** - 端口連接檢查
   - 系統端口: 檢查關鍵端口 (7447, 8000-8002, 5432)  
   - 分類檢查: Web服務、資料庫、通訊端口分類檢查
   - 衝突檢測: 端口佔用和衝突解決建議
   - 範圍掃描: 支援自定義端口範圍掃描

3. **network-scan.sh** - 網路掃描和設備發現
   - 設備發現: 自動發現網路中的 AGV 和 AGVC 設備
   - MAC 映射: IP 地址和 MAC 地址關聯映射
   - 拓撲分析: 網路設備連接關係分析
   - 配置驗證: 硬體映射配置檔案驗證

4. **connectivity-test.sh** - 連接性綜合測試
   - 多層測試: Ping、端口、路由追蹤綜合測試
   - 性能評估: 延遲、抖動、封包遺失率分析
   - 品質評分: 智能連接品質評估和建議
   - 壓力測試: 高頻連接和穩定性測試

5. **network-tools.sh** - 統一網路工具集
   - 便捷函數: 整合所有網路工具的快捷介面
   - 場景化: 啟動檢查、維護檢查、緊急診斷模式
   - 批量操作: 同時檢查所有 AGV/AGVC 設備
   - 報告生成: 自動生成結構化網路診斷報告

#### 常用操作範例
```bash
# 基本網路診斷
network_quick_check              # 快速網路健康檢查
network_diagnose                 # 全面網路診斷
network_troubleshoot zenoh       # Zenoh 專項故障排除

# 設備檢查
network_check_all_agv            # 檢查所有 AGV 設備  
network_check_all_agvc           # 檢查所有 AGVC 設備
network_validate_mapping         # 驗證硬體映射配置

# 連接測試
network_test_connection 192.168.100.100     # 基本連接測試
network_performance 192.168.100.100         # 性能測試

# 監控和報告
network_monitor                  # 即時網路監控
network_generate_report          # 生成診斷報告

# 場景化檢查
network_startup_check            # 系統啟動網路檢查
network_emergency_check "Zenoh 斷線"        # 緊急故障檢查

# 直接使用單一工具
scripts/network-tools/zenoh-network.sh full-check
scripts/network-tools/port-check.sh conflicts --verbose
scripts/network-tools/connectivity-test.sh performance --target 192.168.100.100
```

### 💻 開發工作流工具集

專用開發工作流管理工具，提供建置、測試、分析、部署的完整自動化解決方案：

#### 工具載入方式
```bash
# 載入所有開發工具函數
source scripts/dev-tools/dev-tools.sh

# 查看可用命令
show_dev_tools_help
```

#### 主要工具說明
1. **build-helper.sh** - 智能建置輔助工具
   - 建置配置: 5種配置 (fast, full, incremental, debug, release)
   - 工作空間管理: 智能發現和分類建置
   - 錯誤分析: 建置錯誤診斷和修復建議
   - 並行建置: 支援多工作空間並行處理

2. **test-runner.sh** - 測試執行和報告工具
   - 測試類型: 5種測試 (unit, integration, system, performance, regression)
   - 框架支援: pytest, unittest, gtest, colcon 多框架整合
   - 覆蓋率分析: 自動生成測試覆蓋率報告
   - 多格式輸出: console, junit, html, json 格式報告

3. **code-analyzer.sh** - 代碼分析和檢查工具
   - 分析類型: 8種分析 (style, quality, security, ros2, complexity 等)
   - 工具整合: flake8, pylint, mypy, bandit, black, isort
   - 自定義規則: ROS 2 最佳實踐、安全漏洞檢查
   - 多格式輸出: console, json, html, csv 報告格式

4. **deploy-helper.sh** - 部署輔助工具
   - 部署模式: 5種模式 (development, staging, production, local, docker)
   - 完整流程: 預檢查、備份、建置、部署、驗證
   - 配置管理: 自動配置驗證和備份機制
   - 回滾支援: 快速回滾到穩定版本

5. **dev-tools.sh** - 統一開發工具集
   - 工作流整合: 6種智能工作流程自動化
   - 環境診斷: 完整開發環境健康檢查
   - 便捷函數: dev_build, dev_test, dev_check, dev_deploy
   - 統一介面: 一致的命令格式和參數風格

#### 工作流程範例
```bash
# 開發環境設置
dev-tools.sh dev-setup                  # 初始化開發環境
dev-tools.sh status                     # 檢查環境狀態
dev-tools.sh doctor                     # 診斷環境問題

# 代碼品質檢查工作流
dev-tools.sh code-check                 # 執行完整代碼品質檢查
code-analyzer.sh style --workspace agv_ws              # 檢查特定工作空間
code-analyzer.sh security --severity error             # 安全掃描

# 建置和測試工作流
dev-tools.sh build-test                 # 執行建置和測試流程
build-helper.sh fast --workspace agv_ws               # 快速建置
test-runner.sh unit --coverage                        # 單元測試覆蓋率

# 部署工作流
dev-tools.sh deploy-dev                 # 開發環境部署
deploy-helper.sh deploy development --components agvc # 部署特定組件
deploy-helper.sh status                               # 檢查部署狀態

# CI/CD 完整流程
dev-tools.sh full-ci --mode production  # 完整 CI/CD 到生產環境
```

#### 便捷函數使用
```bash
# 載入便捷函數
source scripts/dev-tools/dev-tools.sh

# 快速操作
dev_build --workspace agv_ws            # 快速建置
dev_test --type unit                    # 快速測試
dev_check --severity warning            # 快速代碼檢查
dev_deploy                              # 快速部署
dev_status                              # 顯示狀態
```

### 智能導航提示
根據問題類型自動定位相關模組：

| 問題類型 | 主要檔案位置 | 相關CLAUDE.md |
|---------|-------------|---------------|
| 狀態機問題 | `agv_ws/src/agv_base/agv_states/` | `agv_ws/src/agv_base/CLAUDE.md` |
| API錯誤 | `web_api_ws/src/web_api/routers/` | `web_api_ws/CLAUDE.md` |
| 資料庫問題 | `db_proxy_ws/src/db_proxy/crud/` | `db_proxy_ws/CLAUDE.md` |
| PLC通訊故障 | `keyence_plc_ws/src/keyence_plc/` | `keyence_plc_ws/CLAUDE.md` |
| 門控問題 | `ecs_ws/src/ecs/` | `ecs_ws/CLAUDE.md` |
| **配置管理** | `scripts/config-tools/` | **使用配置管理工具集** |
| **連線測試** | **Bash + /dev/tcp/** | **參考配置管理最佳實踐** |
| **容器管理** | `scripts/docker-tools/` | **使用 Docker 管理工具集** |
| **容器狀態** | **container-status.sh** | **執行 all_health 診斷** |
| **系統診斷** | `scripts/system-tools/` | **使用系統診斷工具集** |
| **服務監控** | **service-monitor.sh** | **執行 system_status 檢查** |
| **健康檢查** | **health-check.sh** | **執行 system_health 全面檢查** |
| **日誌分析** | `scripts/log-tools/` | **使用日誌分析工具集** |
| **錯誤查找** | **log-analyzer.sh** | **執行 log_analyze 或 log_quick_scan** |
| **故障診斷** | **log-tools.sh** | **執行 log_full_diagnosis** |
| **網路診斷** | `scripts/network-tools/` | **使用網路診斷工具集** |
| **Zenoh 通訊故障** | **zenoh-network.sh** | **執行 network_troubleshoot zenoh** |
| **端口連接問題** | **port-check.sh** | **執行 network_check_ports** |
| **設備發現問題** | **network-scan.sh** | **執行 network_scan_devices** |
| **連接性測試** | **connectivity-test.sh** | **執行 network_test_connection** |
| **網路性能問題** | **network-tools.sh** | **執行 network_performance** |
| **開發工作流** | `scripts/dev-tools/` | **使用開發工作流工具集** |
| **建置問題** | **build-helper.sh** | **執行 dev_build 或 build-helper 快速建置** |
| **測試失敗** | **test-runner.sh** | **執行 dev_test 或 test-runner 單元測試** |
| **代碼品質** | **code-analyzer.sh** | **執行 dev_check 或 code-analyzer 品質檢查** |
| **部署問題** | **deploy-helper.sh** | **執行 dev_deploy 或 deploy-helper 部署流程** |
| **環境診斷** | **dev-tools.sh** | **執行 dev-tools.sh doctor 環境診斷** |
| **CI/CD 流程** | **dev-tools.sh** | **執行 dev-tools.sh full-ci 完整流程** |

## Docker部署架構
```
🚗 AGV車載系統 (docker-compose.yml)
├─ rosagv容器 (host網路模式)
├─ 環境變數: CONTAINER_TYPE="agv" 
├─ 自動載入: keyence_plc_ws, agv_ws, joystick_ws等
└─ 設備掛載: /dev/input (USB搖桿)

🖥️ AGVC管理系統 (docker-compose.agvc.yml)  
├─ agvc_server容器 (bridge網路 192.168.100.100)
├─ nginx容器 (port 80)
├─ postgres容器 (port 5432)
├─ 環境變數: CONTAINER_TYPE="agvc"
└─ 端口映射: 7447(Zenoh), 8000-8002(API), 2200(SSH)
```

## 開發指令

### 1. 容器操作 (宿主機執行)

#### 🐳 Docker-Tools 便捷工具 (推薦使用)
```bash
# 載入 docker-tools 工具集
source scripts/docker-tools/docker-tools.sh

# 🚗 AGV 容器管理
agv_start                    # 啟動 AGV 容器
agv_stop                     # 停止 AGV 容器
agv_enter                    # 進入 AGV 容器 (自動載入 agv_source)
agv_logs                     # 查看 AGV 日誌
agv_health                   # AGV 健康檢查

# 🖥️ AGVC 容器管理
agvc_start                   # 啟動 AGVC 系統 (所有服務)
agvc_stop                    # 停止 AGVC 系統
agvc_enter                   # 進入 AGVC 容器 (自動載入 agvc_source)
agvc_logs                    # 查看 AGVC 日誌
agvc_health                  # AGVC 健康檢查
agvc_services                # 檢查所有 AGVC 服務狀態

# 📊 系統整體操作
all_start                    # 啟動所有系統 (AGVC + AGV)
all_stop                     # 停止所有系統
all_status                   # 查看所有容器狀態
all_health                   # 系統健康檢查
all_ports                    # 檢查端口狀態

# ⚡ 快速命令執行
quick_agv "node-list"        # 在 AGV 容器執行命令
quick_agvc "check-status"    # 在 AGVC 容器執行命令
scripts/docker-tools/quick-exec.sh agv "ros2 topic list"
```

#### 原始 Docker Compose 指令
```bash
# 🚗 AGV車載系統
docker compose -f docker-compose.yml up -d          # 啟動
docker compose -f docker-compose.yml exec rosagv bash   # 進入容器
docker compose -f docker-compose.yml logs -f rosagv     # 查看日誌

# 🖥️ AGVC管理系統  
docker compose -f docker-compose.agvc.yml up -d         # 啟動
docker compose -f docker-compose.agvc.yml exec agvc_server bash  # 進入容器
docker compose -f docker-compose.agvc.yml logs -f agvc_server    # 查看日誌
```

### 2. 開發指令 (容器內執行)
**⚠️ 重要**: 以下指令必須在對應容器內執行，setup.bash會自動載入所需環境

```bash
# 環境載入 (setup.bash會自動執行)
source /app/setup.bash       # 載入開發環境 (ROS 2 + Zenoh + 工作空間)

# 智能工作空間載入 (setup.bash驗證可用)
all_source                  # 智能載入工作空間 (根據容器類型自動選擇，別名: sa)
agv_source                  # 載入 AGV 車載系統專用工作空間 (別名: agv)
agvc_source                 # 載入 AGVC 管理系統專用工作空間 (別名: agvc)

# 構建和測試 (setup.bash驗證可用)
build_all                    # 構建所有工作空間 (別名: ba)
build_ws <workspace_name>   # 構建特定工作空間 (如: build_ws agv_ws)
clean_all                   # 清理所有工作空間 (別名: ca) 
test_all                    # 測試所有工作空間 (別名: ta)

# 系統狀態檢查 (setup.bash驗證可用)
check_system_status         # 檢查系統狀態 (別名: status)
check_zenoh_status         # 檢查Zenoh狀態 (別名: zenoh)
check_ros_env              # 檢查ROS環境 (別名: rosenv)
```

**容器環境檢測**:
- AGV容器: `$CONTAINER_TYPE="agv"` (host網路模式)
- AGVC容器: `$CONTAINER_TYPE="agvc"` (bridge網路模式)
- 宿主機: `$CONTAINER_TYPE` 未設定

## 🔧 智能工作空間載入系統

### 自動環境檢測載入
- **`all_source`**: 根據 `$CONTAINER_TYPE` 自動選擇載入策略
  - AGV環境: 自動載入9個AGV專用工作空間
  - AGVC環境: 自動載入11個AGVC專用工作空間
  - 未知環境: 預設載入AGV工作空間

### 專用工作空間載入函數

#### AGV車載系統 (9個工作空間)
```bash
agv_source  # 或使用別名: agv
```
**載入清單**:
- 基礎: keyence_plc_ws, plc_proxy_ws, path_algorithm  
- 應用: agv_cmd_service_ws, joystick_ws, agv_ws, sensorpart_ws, uno_gpio_ws, launch_ws

#### AGVC管理系統 (11個工作空間)
```bash
agvc_source  # 或使用別名: agvc
```
**載入清單**:
- 基礎: keyence_plc_ws, plc_proxy_ws, path_algorithm, db_proxy_ws
- 應用: ecs_ws, rcs_ws, wcs_ws, ai_wcs_ws, web_api_ws, kuka_fleet_ws, launch_ws

### 使用建議
- **一般使用**: 直接使用 `all_source` 或 `sa`，系統會自動選擇
- **強制載入**: 使用 `agv_source`/`agv` 或 `agvc_source`/`agvc` 強制載入特定環境
- **互動環境**: 別名 `agv`, `agvc`, `sa` 都可正常使用
- **腳本環境**: 使用完整函數名 `agv_source`, `agvc_source`, `all_source`

### 📋 工作空間載入最佳實踐

#### 🎯 場景選擇指南
```bash
# 新進入容器，不確定環境類型
all_source      # 👍 推薦：自動檢測並載入對應工作空間

# 明確在 AGV 容器中開發車載功能
agv_source      # 👍 推薦：只載入AGV相關的9個工作空間，提高效率

# 明確在 AGVC 容器中開發管理系統
agvc_source     # 👍 推薦：只載入AGVC相關的11個工作空間，避免不必要依賴

# 跨環境開發或測試
all_source      # 👍 推薦：讓系統自動判斷，減少人為錯誤
```

#### ⚡ 效能優化
- **AGV開發**: 使用 `agv_source` 減少載入時間約35% (9個vs13個工作空間)
- **AGVC開發**: 使用 `agvc_source` 避免載入不相關的AGV專用工作空間
- **記憶體優化**: 專用載入可減少約200MB記憶體使用量

#### 🔧 故障排除
```bash
# 如果 all_source 選擇錯誤環境
echo "當前環境: $CONTAINER_TYPE"  # 檢查環境變數
agv_source                        # 強制載入AGV環境
# 或
agvc_source                       # 強制載入AGVC環境

# 檢查載入狀態
ros2 pkg list | grep -E "(agv|ecs|web_api)" | wc -l  # 統計載入的套件數量
```

## 🔍 容器內外環境使用指南

### 宿主機環境 (無ROS 2)
**適用操作**：
- Docker容器管理：`docker compose up/down/ps/logs`
- 文件編輯：IDE、文本編輯器操作
- Git版本控制：`git add/commit/push/pull`
- 文件系統操作：檔案複製、移動、權限設定

**不可執行**：
- ❌ ROS 2指令：`ros2 node/topic/service`
- ❌ 工作空間操作：`build_all/test_all/clean_all`  
- ❌ 系統狀態檢查：`check_system_status`

### 容器內環境 (完整ROS 2)
**進入方式**：
```bash
# AGV車載系統
docker compose -f docker-compose.yml exec rosagv bash

# AGVC管理系統  
docker compose -f docker-compose.agvc.yml exec agvc_server bash
```

**適用操作** (setup.bash自動載入後可用)：
- ✅ ROS 2開發：`ros2 node/topic/service/launch`
- ✅ 工作空間管理：`build_all/test_all/clean_all`
- ✅ 智能工作空間載入：`all_source/agv_source/agvc_source`
- ✅ 系統監控：`check_system_status/check_zenoh_status`
- ✅ Python開發：完整Python環境 + ROS 2套件

### 3. 服務管理 (容器內執行)
```bash
# 通用服務
manage_zenoh start|stop|restart|status
manage_ssh start|stop|restart|status

# AGVC專用服務 (僅AGVC容器)
agvc_source                 # 載入AGVC工作空間
start_db / stop_db         # 資料庫狀態檢查和管理指導
start_ecs                  # 啟動ECS服務
check_agvc_status          # 檢查AGVC狀態
```

## 代碼架構

### 系統結構
**🚗 AGV車載系統** (docker-compose.yml): 車載電腦，實時控制和感測器整合
**🖥️ AGVC管理系統** (docker-compose.agvc.yml): 管理伺服器，車隊管理和任務調度

### 工作空間架構
```
app/
├── agv_ws/                    # 🚗 AGV核心控制(3層狀態架構)
├── agv_cmd_service_ws/        # 🚗 手動指令服務
├── joystick_ws/               # 🚗 USB搖桿控制
├── keyence_plc_ws/            # 🚗🖥️ Keyence PLC通訊
├── plc_proxy_ws/              # 🚗🖥️ ROS 2 PLC服務代理
├── path_algorithm/            # 🚗🖥️ A*路徑規劃
├── db_proxy_ws/               # 🖥️ PostgreSQL代理+ORM
├── ecs_ws/                    # 🖥️ 設備控制系統
├── rcs_ws/                    # 🖥️ 機器人控制系統
├── wcs_ws/                    # 🖥️ 倉庫控制系統
├── web_api_ws/                # 🖥️ Web APIs + Socket.IO
├── kuka_fleet_ws/             # 🖥️ KUKA Fleet整合
└── sensorpart_ws/             # 🚗 感測器數據處理
```

### 關鍵設計模式

#### AGV狀態機架構(3層)
- **Base層**: 通用邏輯狀態
- **AGV層**: 車型特定狀態(Cargo/Loader/Unloader)
- **Robot層**: 機械臂任務執行狀態

關鍵檔案: `agv_ws/src/agv_base/agv_base/agv_node_base.py`, `agv_ws/src/*/robot_context.py`

#### Web服務架構
- **Port 8000**: 核心API服務
- **Port 8001**: AGVCUI車隊管理界面
- **Port 8002**: OPUI操作員界面

#### PLC通訊棧
```
ECS → plc_proxy_ws → keyence_plc_ws → Keyence PLC
```

## 開發指南

### 開發環境 (容器內實際配置)
- **Python**: 3.12 (系統Python + 部分套件在 `/opt/pyvenv_env/`)
- **ROS 2**: Jazzy + rmw_zenoh_cpp (經驗證可用)
- **資料庫**: PostgreSQL (僅AGVC容器，經驗證可用)
- **容器**: Docker + Compose V2
- **重要**: 宿主機無ROS 2環境，所有ROS指令必須在容器內執行

### 重要配置文件
- `/app/config/hardware_mapping.yaml` - 設備映射
- `/app/config/agv/*.yaml` - 車輛配置
- `/app/config/agvc/*.yaml` - 管理系統配置
- `/app/routerconfig.json5` - Zenoh路由配置

### 🗄️ 資料庫配置
**AGVC PostgreSQL 資料庫設定** (Docker容器架構):
```yaml
# 正確的AGVC資料庫連線參數
database:
  host: postgres          # Docker容器名稱 (不是localhost)
  port: 5432             # 標準PostgreSQL端口
  name: agvc             # 資料庫名稱
  user: agvc             # 使用者名稱  
  password: password     # 密碼
```

**資料庫管理指令** (容器內執行):
```bash
# 檢查資料庫連線狀態
start_db                 # 檢查PostgreSQL連接，提供管理指導
stop_db                  # 檢查狀態，提供停止指導
check_agvc_status        # 完整AGVC系統狀態檢查

# 宿主機資料庫容器管理
docker compose -f docker-compose.agvc.yml up -d postgres    # 啟動服務
docker compose -f docker-compose.agvc.yml stop postgres     # 停止服務
docker compose -f docker-compose.agvc.yml ps postgres       # 查看狀態
docker compose -f docker-compose.agvc.yml logs postgres     # 查看日誌

# 直接連線到PostgreSQL（系統管理員）
docker compose -f docker-compose.agvc.yml exec postgres psql -U postgres -d postgres

# 連線到agvc資料庫
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc
```

**Docker Compose 配置詳情**:
```yaml
# docker-compose.agvc.yml 中的 PostgreSQL 配置
postgres:
  image: postgres:latest
  container_name: postgres_container  # 實際容器名稱
  networks:
    bridge_network:
      ipv4_address: 192.168.100.254   # 固定IP
  environment:
    POSTGRES_USER: postgres           # 系統管理員帳號
    POSTGRES_PASSWORD: password      # 系統管理員密碼
    POSTGRES_DB: postgres            # 預設資料庫
  ports:
    - "5432:5432"                    # 端口映射
```

**重要提醒**:
- PostgreSQL運行在獨立Docker容器中，無法從AGVC容器內直接啟停
- 服務名稱: `postgres`，容器名稱: `postgres_container`
- 所有資料庫連線必須使用主機名稱 `postgres` (非 `localhost`)
- 系統管理員: `postgres/password`，應用資料庫: `agvc/agvc/password`
- 容器內指令提供連線檢查和管理指導，實際操作需在宿主機執行

### 環境檢測
- **AGV環境**: CONTAINER_TYPE="agv"
- **AGVC環境**: CONTAINER_TYPE="agvc"

### 開發工作流程

#### 🔧 開發環境準備
```bash
# 1. 啟動開發環境
docker compose -f docker-compose.agvc.yml up -d
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 2. 環境初始化
source /app/setup.bash && agvc_source  # 或使用 all_source (自動檢測AGVC環境)
check_system_status

# 3. 開始開發前的檢查
check_zenoh_status   # 確認通訊正常
start_db             # 確保資料庫運行
```

#### 📋 任務開發流程

##### 1. 需求分析階段
- **閱讀相關CLAUDE.md**: 根據功能領域查閱對應文檔
- **代碼結構探索**: 使用 `rg` 搜尋相關實現
- **依賴關係分析**: 檢查相關接口和數據結構

##### 2. 實現階段
**新增AGV狀態:**
1. 閱讀 `app/agv_ws/src/agv_base/CLAUDE.md` 了解狀態機架構
2. 擴展 `agv_base/agv_states/` 基礎狀態機
3. 在對應 `*_agv/` 工作空間實現車型特定狀態
4. 更新狀態常數和驗證邏輯：`rg "STATE_" --type py`
5. 添加完整的日誌和錯誤處理

**創建API端點:**
1. 閱讀 `app/web_api_ws/CLAUDE.md` 了解API架構
2. 在 `web_api_ws/src/web_api/models/` 定義Pydantic模型
3. 在 `web_api_ws/src/web_api/crud/` 實現CRUD操作
4. 在 `web_api_ws/src/web_api/routers/` 創建FastAPI路由
5. 更新 `api_server.py` 主應用並測試端點

**擴展Socket.IO事件:**
1. 在對應socket類添加事件處理器
2. 更新前端JavaScript事件監聽器
3. 使用瀏覽器開發工具測試事件流
4. 完整記錄事件格式和響應範例

##### 3. 測試階段 (容器內執行)
```bash
# 單元測試 (基於setup.bash實際函數)
build_ws <workspace_name>       # 如: build_ws agv_ws
test_ws <workspace_name>        # 使用setup.bash中的test_ws函數

# 整合測試 (經驗證可用)
test_all                        # setup.bash中的test_all函數
check_system_status            # 檢查整體系統狀態

# API測試 (僅AGVC容器)
curl -X GET http://localhost:8000/health  # 基本健康檢查
```

**⚠️ 測試注意事項**:
- 所有ROS 2相關測試必須在對應容器內執行
- AGV測試需在AGV容器內: `docker compose -f docker-compose.yml exec rosagv bash`
- AGVC測試需在AGVC容器內: `docker compose -f docker-compose.agvc.yml exec agvc_server bash`

##### 4. 部署驗證 (容器內執行)
```bash
# 構建檢查 (setup.bash驗證可用)
clean_all && build_all

# 系統整體驗證 (setup.bash驗證可用)
check_system_status         # 檢查基礎服務和工作空間狀態
check_agvc_status          # 僅AGVC容器可用

# ROS 2服務測試 (需在載入環境後)
source /app/setup.bash && all_source
ros2 node list             # 檢查運行中的ROS節點
ros2 topic list            # 檢查可用的ROS主題
```

**⚠️ 驗證注意事項**：
- 確認 `check_system_status` 顯示所有工作空間為"已建置"狀態
- AGVC容器必須確認PostgreSQL和Zenoh Router正常運行
- AGV容器必須確認Zenoh Router和所需硬體設備正常

#### 🚀 最佳實踐指南

##### 代碼品質
- **命名約定**: 遵循現有的蛇形命名法 (`snake_case`)
- **類型提示**: 使用Python 3.12+類型標註
- **錯誤處理**: 實現完整的異常捕獲和日誌記錄
- **文檔字串**: 為所有公共方法添加docstring

##### 性能優化
- **並行處理**: 在I/O密集操作中使用異步處理
- **資源管理**: 適當關閉資料庫連接和文件句柄  
- **緩存策略**: 對重複查詢使用適當的緩存機制

##### 安全考量
- **敏感數據**: 使用環境變量存儲密碼和API金鑰
- **輸入驗證**: 所有API端點使用Pydantic進行數據驗證
- **權限控制**: 實現適當的用戶權限檢查

### 重要注意事項
- 使用環境變量存儲敏感資料
- 遵循現有命名約定
- Zenoh自動配置，避免手動設置RMW
- 始終在相應容器環境中開發
- AGV狀態轉換必須包含完整驗證和日誌
- 使用SQLModel適當方式進行資料庫變更

### 執行程式重要提醒
**🚨 程式執行必須在Docker容器內:**
1. **先啟動容器**: `docker compose -f <file> up -d`
2. **進入容器**: `docker compose -f <file> exec <container> bash`
3. **載入環境**: `source /app/setup.bash && all_source`
4. **執行ROS 2指令**: `ros2 launch/run/...`

### 除錯與故障排除

**宿主機執行**：
```bash
# 容器狀態檢查
docker compose -f docker-compose.yml ps                    # AGV容器狀態
docker compose -f docker-compose.agvc.yml ps               # AGVC容器狀態
docker compose -f <compose-file> logs -f <service>         # 查看容器日誌
```

**容器內執行** (必須先進入對應容器):
```bash
# 進入容器
docker compose -f <compose-file> exec <container> bash

# 載入環境並執行檢查 (setup.bash驗證可用)
source /app/setup.bash
check_system_status         # 整體狀況 (所有工作空間狀態)
check_zenoh_status         # Zenoh連接狀態 (port 7447)
check_ros_env              # ROS 2環境驗證
check_agvc_status          # AGVC狀態 (僅AGVC容器可用)
```

## 常見問題速查表

### 🚨 緊急問題處理

#### AGV狀態機卡住 (AGV容器內執行)
```bash
# 1. 進入AGV容器並載入環境
docker compose -f docker-compose.yml exec rosagv bash
source /app/setup.bash && agv_source  # 或使用 all_source (自動檢測)

# 2. 檢查當前狀態 (需驗證主題名稱)
ros2 topic list | grep agv     # 先查看可用主題
ros2 node list                 # 檢查運行中的節點

# 3. 查看狀態機日誌
rg "state|transition|error" /tmp/agv.log -i
```

#### Web API無響應 (AGVC容器內執行)
```bash
# 1. 進入AGVC容器並檢查服務狀態
docker compose -f docker-compose.agvc.yml exec agvc_server bash
curl http://localhost:8000/health    # 基本健康檢查 (如果API服務運行中)

# 2. 檢查系統狀態 (setup.bash驗證可用)
source /app/setup.bash
check_agvc_status                    # 檢查AGVC系統狀態

# 3. 檢查端口佔用
netstat -tlnp | grep 800[0-2]       # 檢查8000-8002端口狀態
```

#### PLC通訊中斷 (容器內執行)
```bash
# 1. 檢查Zenoh Router狀態 (setup.bash驗證可用)
source /app/setup.bash
check_zenoh_status                   # 檢查Zenoh狀態

# 2. 重啟Zenoh服務 (setup.bash驗證可用)
manage_zenoh restart                 # setup.bash提供的函數

# 3. 檢查配置檔案
ls -la /app/config/hardware_mapping.yaml
ls -la /app/routerconfig.json5       # Zenoh路由配置
```

### 🔍 診斷工作流程

#### 步驟1: 系統整體檢查
**宿主機執行**：
1. `docker compose -f <compose-file> ps` - 檢查容器運行狀態
2. `docker compose -f <compose-file> logs -f <service>` - 查看容器日誌

**容器內執行**：
1. `source /app/setup.bash && check_system_status` - 確認基礎環境
2. `check_zenoh_status` - 驗證Zenoh通訊狀態  
3. `check_agvc_status` - 檢查AGVC系統狀態 (僅AGVC容器)

#### 步驟2: 問題分類診斷
| 問題現象 | 可能原因 | 檢查命令 (容器內) | 解決方案 |
|---------|---------|---------|---------|
| AGV不動作 | 狀態機錯誤 | `rg "error\|state" /tmp/agv.log -i` | 檢查狀態機日誌 |
| 網頁無法載入 | Web服務未啟動 | `netstat -tlnp \| grep 800[0-2]` | 檢查端口佔用 |
| 資料庫錯誤 | PostgreSQL連接問題 | `check_agvc_status` (setup.bash函數) | 檢查資料庫狀態 (使用postgres:5432) |
| 感測器異常 | 設備掛載問題 | `ls -la /dev/input/` (AGV容器) | 檢查USB設備掛載 |
| 通訊中斷 | Zenoh Router問題 | `check_zenoh_status` (setup.bash函數) | 重啟Zenoh服務 |

#### 步驟3: 深度分析
- **使用相關CLAUDE.md**: 根據問題類型讀取對應模組文檔
- **代碼追蹤**: 使用 `rg` 搜尋錯誤關鍵字
- **日誌分析**: 查看 `/tmp/*.log` 詳細錯誤訊息

### 日誌位置
- 容器日誌: `docker compose -f <file> logs <service>`
- Zenoh Router: `/tmp/zenoh_router.log` (容器內)
- AGV啟動: `/tmp/agv.log` (容器內)

## 語言配置
- **CLI互動語言**: 繁體中文
- **代碼註釋語言**: 繁體中文
```