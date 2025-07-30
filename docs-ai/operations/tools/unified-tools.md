# 統一工具系統

## 🎯 適用場景
- 系統診斷、容器管理、開發工作流程的統一工具使用
- 提供宿主機和容器內的工具整合方案
- 簡化日常開發和維護操作

## 📋 工具系統概覽

RosAGV 提供完整的工具生態系統，分為宿主機統一工具和容器內專業工具集兩個層次。

### 宿主機統一工具 (r 命令)
```bash
r                           # 顯示所有可用工具
r agvc-check               # AGVC 系統健康檢查
r agv-check                # AGV 系統健康檢查
r containers-status        # 容器狀態檢查
r network-check            # 網路連接檢查
r quick-diag               # 快速綜合診斷
```

### 容器內專業工具集
```bash
# Docker 工具集
source scripts/docker-tools/docker-tools.sh
# 或載入完整的配置工具集
source scripts/config-tools/config-tools.sh

# AGVC 系統管理
agvc_start                 # 啟動 AGVC 系統 (所有服務)
agvc_stop                  # 停止 AGVC 系統
agvc_restart               # 重啟 AGVC 系統
agvc_status                # 查看 AGVC 系統狀態
agvc_logs                  # 查看 AGVC 系統日誌
agvc_health                # AGVC 系統健康檢查
agvc_services              # 檢查所有 AGVC 服務狀態

# 容器進入和快速執行
agv_enter                  # 進入 AGV 容器
agvc_enter                 # 進入 AGVC 容器 (自動載入 agvc_source)
quick_agvc "command"       # 快速執行 AGVC 容器內指令

# 系統健康檢查
all_health                 # 智能健康檢查

# 系統診斷工具集
source scripts/system-tools/system-tools.sh
system_health              # 完整健康檢查
system_quick_check         # 快速診斷

# 系統健康監控
scripts/system-tools/health-check.sh [component]    # 系統健康檢查

# 網路診斷工具集
source scripts/network-tools/network-tools.sh
network_troubleshoot zenoh # Zenoh 通訊診斷
network_test_connection    # 連接測試

# 獨立網路工具
scripts/network-tools/network-scan.sh [target]     # 網路掃描
scripts/network-tools/zenoh-network.sh [action]    # Zenoh 網路診斷

# 日誌分析工具集
source scripts/log-tools/log-tools.sh
log_analyze agv            # AGV 日誌分析
log_quick_scan             # 快速錯誤掃描

# 開發工具集
source scripts/dev-tools/dev-tools.sh  
dev_build                  # 智能建置
dev_test                   # 執行測試
dev_check                  # 代碼檢查
```

## 🔧 配置管理工具集

### Zenoh 路由器配置管理
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

### 硬體映射配置管理
```bash
scripts/config-tools/hardware-mapping.sh [action] [device_id]
```

### 環境配置管理
```bash
# AGV 環境配置編輯
scripts/config-tools/edit-agv-config.sh [config_type]

# AGVC 環境配置編輯  
scripts/config-tools/edit-agvc-config.sh [config_type]
```
**主要功能**：
- `validate/check` - 驗證硬體映射配置
- `list/ls` - 列出所有設備詳細資訊
- `show <device_id>` - 顯示特定設備詳情
- `edit <device_id>` - 編輯設備配置
- `mac <device_id>` - 管理 MAC 地址
- `overview` - 顯示硬體映射概況（預設）

**配置檔案**: `/app/config/hardware_mapping.yaml`

### 結構化資料處理工具
**⚠️ 現代工具：jq/yq 已安裝在容器內，提供專業的結構化資料處理**

#### JSON5 配置處理 (json5 + jq)
**⚠️ 注意：RosAGV 使用 JSON5 格式 (routerconfig.json5)，需要先轉換**

```bash
# Zenoh 配置分析 (JSON5 格式)
json5 /app/routerconfig.json5 | jq '.mode'                    # 查看運行模式
json5 /app/routerconfig.json5 | jq '.listen.endpoints[]'      # 查看監聽端點
json5 /app/routerconfig.json5 | jq '.transport.unicast.lowlatency'  # 查看效能配置
json5 /app/routerconfig.json5 | jq '.connect.endpoints[]'     # 查看連接端點

# JSON5 格式驗證和處理
json5 --validate /app/routerconfig.json5                      # 驗證 JSON5 語法
json5 /app/routerconfig.json5 | jq . | head -20              # 預覽配置結構
json5 /app/routerconfig.json5 | jq -C . | less               # 彩色分頁顯示

# 配置查詢和分析
json5 /app/routerconfig.json5 | jq 'keys'                     # 查看所有頂層配置項
json5 /app/routerconfig.json5 | jq '.plugins | keys'          # 查看插件列表
```

#### YAML 配置處理 (yq)
```bash
# Docker Compose 配置分析
yq '.services.agvc_server.ports' docker-compose.agvc.yml     # 查看服務端口
yq '.networks' docker-compose.agvc.yml                       # 查看網路配置
yq '.services.*.image' docker-compose.agvc.yml               # 查看所有映像

# 硬體映射配置分析
yq '.devices[] | select(.type == "agv")' /app/config/hardware_mapping.yaml
yq '.devices[].mac_address' /app/config/hardware_mapping.yaml

# 配置驗證和格式化
yq . docker-compose.agvc.yml                             # 驗證 YAML 格式
```

#### 配置管理最佳實踐
```bash
# 備份配置檔案
cp /app/routerconfig.json5 /app/routerconfig.json5.backup

# JSON5 配置修改 (需要手動編輯或轉換)
# ⚠️ 注意：JSON5 → JSON → 修改 → 手動還原為 JSON5
json5 /app/routerconfig.json5 | jq '.listen.endpoints[0] = "tcp/0.0.0.0:7448"' > /tmp/config.json
# 然後需要手動將 JSON 轉回 JSON5 格式 (加入註解等)

# 推薦：直接編輯 JSON5 檔案 (保留註解和格式)
vim /app/routerconfig.json5
# 或使用配置工具
scripts/config-tools/zenoh-config.sh edit

# YAML 配置安全修改
yq '.services.agvc_server.ports[0] = "8001:8000"' docker-compose.agvc.yml > /tmp/compose.yml && mv /tmp/compose.yml docker-compose.agvc.yml

# 配置驗證流程
json5 --validate /app/routerconfig.json5                      # 驗證語法
json5 /app/routerconfig.json5 | jq empty                      # 驗證結構
```

### 連線測試最佳實踐
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

## 📊 智能導航提示
根據問題類型自動定位相關模組：

| 問題類型 | 主要檔案位置 | 相關工具 |
|---------|-------------|---------|
| 狀態機問題 | `agv_ws/src/agv_base/agv_states/` | `log_analyze agv` |
| API錯誤 | `web_api_ws/src/web_api/routers/` | `r agvc-check` |
| 資料庫問題 | `db_proxy_ws/src/db_proxy/crud/` | `start_db` |
| PLC通訊故障 | `keyence_plc_ws/src/keyence_plc/` | `network_test_connection <PLC_IP>` |
| 門控問題 | `ecs_ws/src/ecs/` | `r quick-diag` |
| **配置管理** | `scripts/config-tools/` | **配置管理工具集** |
| **容器管理** | `scripts/docker-tools/` | **Docker 管理工具集** |
| **系統診斷** | `scripts/system-tools/` | **系統診斷工具集** |
| **網路診斷** | `scripts/network-tools/` | **網路診斷工具集** |
| **日誌分析** | `scripts/log-tools/` | **日誌分析工具集** |
| **開發工作流** | `scripts/dev-tools/` | **開發工作流工具集** |

## 💡 使用策略
- **統一入口優先**: 使用 `r` 命令處理日常操作
- **專業工具深入**: 複雜問題使用對應的專業工具集
- **便捷函數組合**: 載入工具集後使用便捷函數提高效率
- **場景化選擇**: 根據具體問題類型選擇最適合的工具

## 🛠️ 維護和驗證工具

### 文檔引用檢查
```bash
# 檢查 CLAUDE.md 文件中的 @docs-ai/ 引用是否存在
scripts/check-claude-references.sh

# 批量更新 CLAUDE.md 文件中的 @docs-ai/ 引用路徑
scripts/update-claude-references.sh
```

### Docker 配置驗證
```bash
# 驗證 Docker 容器配置的完整性和可用性
scripts/validate-docker-config.sh
```

### 容器專用管理工具
```bash
# AGV 容器專用管理
scripts/docker-tools/agv-container.sh [action]     # AGV 容器管理
scripts/docker-tools/agvc-container.sh [action]    # AGVC 容器管理
scripts/docker-tools/quick-exec.sh [command]       # 快速容器指令執行
```

**主要功能**：
- `start/stop/restart` - 容器生命週期管理
- `enter` - 進入容器環境
- `status` - 檢查容器狀態
- `logs` - 查看容器日誌
- `health` - 健康檢查

## 🔗 交叉引用
- **維護工具指南**: @docs-ai/operations/maintenance/unified-tools.md - 日常運維和故障排除的實用指南
- 系統診斷: @docs-ai/operations/maintenance/system-diagnostics.md
- 容器管理: @docs-ai/operations/deployment/container-management.md
- 故障排除: @docs-ai/operations/maintenance/troubleshooting.md
- 開發環境: @docs-ai/operations/development/docker-development.md