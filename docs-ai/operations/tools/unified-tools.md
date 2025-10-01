# 統一工具系統

## 🎯 適用場景
- 系統診斷、容器管理、開發工作流程的統一工具使用
- 提供宿主機和容器內的工具整合方案
- 簡化日常開發和維護操作

## 🎯 核心原則
- **環境分離**: 宿主機工具 vs 容器內工具完全分離
- **前提條件**: 每個工具都有明確的執行環境要求
- **統一入口**: 優先使用 `r` 命令系列

## 🖥️ 第一部分：宿主機工具（統一入口）

### ⚠️ 執行前提
**使用 `r` 工具集之前，必須將 RosAGV 目錄加入 PATH 環境變數**

- **[宿主機]** 在 `~/RosAGV` 目錄執行
- **[宿主機]** 確保 `/home/ct/RosAGV` 已加入 PATH
- **[宿主機]** 對應容器必須已啟動

#### PATH 配置
在 `~/.bashrc` 中添加以下設定：
```bash
# [宿主機] RosAGV 工具路徑配置
export PATH="/home/ct/RosAGV:$PATH"

# 或者根據您的實際安裝路徑調整
# export PATH="/path/to/your/RosAGV:$PATH"
```

設定完成後，重新載入環境：
```bash
# [宿主機] 重新載入環境
source ~/.bashrc
```

驗證配置是否正確：
```bash
# [宿主機] 驗證配置
which r                    # 應該顯示 /home/ct/RosAGV/r
r menu                     # 應該顯示工具選單
```

### 系統診斷工具
```bash
# [宿主機] 統一診斷入口
r                         # 顯示所有可用工具
r quick-diag             # 快速綜合診斷
r agvc-check             # AGVC 系統健康檢查
r agv-check              # AGV 系統健康檢查
r containers-status      # 容器狀態檢查
r network-check          # 網路連接檢查
r zenoh-check            # Zenoh 連接檢查
```

### 配置管理工具
```bash
# [宿主機] 配置管理
r zenoh-config           # Zenoh Router 配置管理
r hardware-config        # 硬體映射配置管理
r tafl-validate [file]   # TAFL 檔案格式驗證
```

### 容器操作工具
```bash
# [宿主機] Docker Compose 操作
docker compose -f docker-compose.yml up -d          # 啟動 AGV 容器
docker compose -f docker-compose.agvc.yml up -d     # 啟動 AGVC 容器系統
docker compose -f docker-compose.agvc.yml ps        # 檢查容器狀態
docker compose -f docker-compose.agvc.yml logs [service]  # 查看日誌
```

## 🔧 第二部分：專業工具集（需要載入）

### ⚠️ 載入前提
**[宿主機]** 必須先載入工具集：
```bash
# [宿主機] 載入 Docker 管理工具
source scripts/docker-tools/docker-tools.sh
```

### AGVC 系統管理工具
載入後可用的專業工具：
```bash
# [宿主機] AGVC 系統生命週期管理
agvc_start               # 啟動 AGVC 系統 (所有服務)
agvc_stop                # 停止 AGVC 系統
agvc_restart             # 重啟 AGVC 系統
agvc_status              # 查看 AGVC 系統狀態
agvc_logs                # 查看 AGVC 系統日誌
agvc_health              # AGVC 系統健康檢查
agvc_services            # 檢查所有 AGVC 服務狀態
```

### 容器進入工具
```bash
# [宿主機] 快速進入容器
agv_enter                # 進入 AGV 容器
agvc_enter               # 進入 AGVC 容器 (自動載入 agvc_source)
quick_agvc "command"     # 快速執行 AGVC 容器內指令
```

### 專業診斷工具集
```bash
# [宿主機] 載入各專業工具集
source scripts/system-tools/system-tools.sh   # 系統診斷工具
source scripts/network-tools/network-tools.sh # 網路診斷工具
source scripts/log-tools/log-tools.sh         # 日誌分析工具
source scripts/dev-tools/dev-tools.sh         # 開發工具集

# 載入後可用的功能
system_health              # 完整健康檢查
network_troubleshoot       # 網路通訊診斷
log_analyze agv           # AGV 日誌分析
dev_build                 # 自動建置
```

## 📦 第三部分：容器內工具（進入容器後）

### ⚠️ 執行前提
**必須先進入容器**：
```bash
# [宿主機] 標準進入方式
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 或使用專業工具（需先載入 docker-tools.sh）
agvc_enter
```

### 環境載入工具
```bash
# [容器內] 環境設置
source /app/setup.bash    # 載入基本環境
all_source                # 自動載入工作空間 (別名: sa)
agv_source                # 載入 AGV 工作空間
agvc_source               # 載入 AGVC 工作空間

# [容器內] 常用別名
ba                        # build_all - 建置所有工作空間
sa                        # all_source - 載入所有工作空間
```

### 服務管理工具
```bash
# [容器內] 服務管理 (載入 setup.bash 後可用)
manage_web_api_launch {start|stop|restart|status}  # Web API 服務群組
manage_zenoh {start|stop|restart|status}           # Zenoh Router
manage_ssh {start|stop|restart|status}             # SSH 服務
```

### 結構化資料處理
```bash
# [容器內] JSON5 配置處理 (Zenoh 配置)
json5 /app/routerconfig.json5 | jq '.mode'
json5 /app/routerconfig.json5 | jq '.listen.endpoints[]'

# [容器內] YAML 配置處理
yq '.services.agvc_server.ports' /path/to/compose.yml
```

## 🚀 第四部分：完整工作流示例

### 系統診斷工作流
```bash
# 步驟1: [宿主機] 快速診斷
cd ~/RosAGV
r quick-diag

# 步驟2: [宿主機] 檢查容器狀態
r containers-status

# 步驟3: [宿主機] 檢查具體問題
r agvc-check              # 或 r network-check, r zenoh-check
```

### 服務重啟工作流
```bash
# 步驟1: [宿主機] 載入專業工具
cd ~/RosAGV
source scripts/docker-tools/docker-tools.sh

# 步驟2: [宿主機] 停止和啟動服務
agvc_stop
agvc_start

# 步驟3: [宿主機] 驗證健康狀態
agvc_health
```

### 開發工作流
```bash
# 步驟1: [宿主機] 進入容器
cd ~/RosAGV
source scripts/docker-tools/docker-tools.sh
agvc_enter

# 步驟2: [容器內] 載入環境
source /app/setup.bash
agvc_source

# 步驟3: [容器內] 建置和重啟
ba                              # 建置所有工作空間
sa                              # 重新載入環境
manage_web_api_launch restart   # 重啟 Web 服務
```

### 複雜指令執行（bash -i 模式）
```bash
# [宿主機] 一次性執行複雜容器內指令
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
source /app/setup.bash &&
agvc_source &&
manage_web_api_launch stop &&
ba &&
sa &&
manage_web_api_launch start
"
```

## 🔧 配置管理詳細說明

### Zenoh 路由器配置管理
```bash
# [宿主機] 統一工具入口 (推薦)
r zenoh-config             # 顯示配置概況和使用說明

# [宿主機] 直接使用專業工具
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

### 硬體映射配置管理
```bash
# [宿主機] 統一工具入口 (推薦)
r hardware-config         # 顯示硬體映射概況和使用說明

# [宿主機] 直接使用專業工具
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

## 🛠️ TAFL 語言工具

### TAFL 驗證
```bash
# [宿主機] TAFL (Task Automation Flow Language) 驗證工具
r tafl-validate [file]      # 驗證單個 TAFL 檔案
r tafl-validate all         # 驗證所有 TAFL 檔案
r tafl-validate list        # 列出所有 TAFL 檔案
r tafl-validate help        # 顯示使用說明
```

**TAFL 檔案位置**:
- **正式配置**: `/home/ct/RosAGV/app/config/tafl/flows/` - TAFL 流程檔案存放位置

## 📊 問題診斷對照表
| 問題類型 | 執行環境 | 使用工具 | 檔案位置 |
|---------|---------|---------|---------|
| 容器問題 | **[宿主機]** | `r containers-status` | - |
| 網路問題 | **[宿主機]** | `r network-check` | - |
| Zenoh 問題 | **[宿主機]** | `r zenoh-check` | `/app/routerconfig.json5` |
| 服務問題 | **[宿主機]** | `r agvc-check` | - |
| 狀態機問題 | **[容器內]** | ROS2 工具 | `agv_ws/src/agv_base/agv_states/` |
| API錯誤 | **[容器內]** | 日誌分析 | `web_api_ws/src/web_api/routers/` |
| 資料庫問題 | **[宿主機]** | Docker 工具 | `db_proxy_ws/src/db_proxy/crud/` |

## 💡 使用技巧總結
1. **統一入口優先**: 使用 `r` 命令處理日常操作
2. **環境明確分離**: 清楚區分宿主機和容器內操作
3. **前提條件檢查**: 執行前確認環境和載入狀態
4. **工作流程化**: 使用完整工作流而非單一命令
5. **專業工具深入**: 複雜問題使用對應的專業工具集

## 🔗 交叉引用
- 系統診斷: docs-ai/operations/guides/system-diagnostics.md
- 容器管理: docs-ai/operations/deployment/container-management.md
- 故障排除: docs-ai/operations/guides/troubleshooting.md
- 開發環境: docs-ai/operations/development/docker-development.md