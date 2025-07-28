# RosAGV 工具快速參考指南

## 🚀 一分鐘上手

### 最重要的一個字母：`r`
```bash
r                    # 顯示所有可用工具
r agvc-check         # 每日健康檢查
r containers-status  # 檢查容器狀態
r quick-diag         # 快速診斷
```

## 📋 常用命令速查表

### 🔍 系統診斷
| 命令 | 用途 | 適用場景 |
|------|------|----------|
| `r agvc-check` | AGVC 健康檢查 | 每日例行檢查 |
| `r system-health` | 完整健康檢查 | 深度系統診斷 |
| `r quick-diag` | 快速綜合診斷 | 故障排除 |

### 🐳 容器管理
| 命令 | 用途 | 適用場景 |
|------|------|----------|
| `r containers-status` | 檢查容器狀態 | 確認系統運行 |
| `r agvc-start` | 啟動 AGVC 系統 | 系統啟動 |
| `r agvc-stop` | 停止 AGVC 系統 | 系統關閉 |

### 🌐 網路診斷
| 命令 | 用途 | 適用場景 |
|------|------|----------|
| `r network-check` | 端口連接檢查 | 網路問題診斷 |
| `r zenoh-check` | Zenoh 連接檢查 | 通訊問題排除 |

### 📋 日誌分析
| 命令 | 用途 | 適用場景 |
|------|------|----------|
| `r log-scan` | 日誌錯誤掃描 | 發現系統問題 |
| `r log-errors` | 高級錯誤掃描 | 深度錯誤分析 |

### 🛠️ 開發工具
| 命令 | 用途 | 環境要求 |
|------|------|----------|
| `r dev-status` | 開發環境狀態 | 宿主機 |
| `r dev-build` | 快速建置 | 容器內 |
| `r dev-test` | 快速測試 | 容器內 |
| `r dev-check` | 代碼檢查 | 容器內 |

## 🚨 故障排除快速流程

### 系統無回應
```bash
1. r containers-status    # 檢查容器狀態
2. r agvc-start          # 嘗試啟動系統
3. r agvc-check          # 檢查啟動結果
4. r log-errors          # 如果仍有問題，檢查日誌
```

### 網路連接問題
```bash
1. r network-check       # 檢查端口狀態
2. r zenoh-check        # 檢查 Zenoh 連接
3. r quick-diag         # 綜合診斷
```

### 容器啟動失敗
```bash
1. r containers-status   # 確認當前狀態
2. r log-errors         # 檢查錯誤日誌
3. r agvc-stop          # 停止異常服務
4. r agvc-start         # 重新啟動
```

## 🔧 進階工具使用

### 載入專業工具集
```bash
# Docker 管理工具
source scripts/docker-tools/docker-tools.sh

# 系統診斷工具
source scripts/system-tools/system-tools.sh

# 網路診斷工具
source scripts/network-tools/network-tools.sh

# 日誌分析工具
source scripts/log-tools/log-tools.sh

# 開發工具
source scripts/dev-tools/dev-tools.sh
```

### 專業工具便捷函數
```bash
# Docker 管理
all_health              # 智能健康檢查
all_status              # 容器狀態
agvc_enter              # 進入 AGVC 容器

# 系統診斷
system_health_check     # 完整健康檢查
quick_diagnose          # 快速診斷

# 網路診斷
check_zenoh_connectivity # Zenoh 連接檢查
check_api_endpoints     # API 端點檢查

# 日誌分析
log_quick_scan          # 快速日誌掃描
analyze_system_logs     # 系統日誌分析

# 開發工具
dev_build               # 智能建置
dev_test                # 執行測試
```

## 📂 重要檔案位置

### 工具腳本
```bash
rosagv-tools.sh                              # 統一工具入口
scripts/docker-tools/docker-tools.sh         # Docker 工具集
scripts/system-tools/system-tools.sh         # 系統工具集
scripts/network-tools/network-tools.sh       # 網路工具集
scripts/log-tools/log-tools.sh              # 日誌工具集
scripts/dev-tools/dev-tools.sh              # 開發工具集
```

### 配置檔案
```bash
docker-compose.agvc.yml                      # AGVC 系統配置
app/config/zenoh/routerconfig.json5          # Zenoh 配置
```

### 日誌檔案
```bash
docker logs agvc_server                      # AGVC 容器日誌
docker logs postgres_container               # PostgreSQL 日誌
docker logs nginx                            # Nginx 日誌
```

## 💡 使用技巧

### 日常維護
```bash
# 每日健康檢查
r agvc-check && r containers-status && r network-check

# 每週深度檢查
r system-health && r log-scan
```

### 開發工作流程
```bash
# 1. 檢查開發環境 (宿主機)
r dev-status

# 2. 進入容器環境
source scripts/docker-tools/docker-tools.sh
agvc_enter

# 3. 在容器內開發
r dev-build && r dev-test && r dev-check
```

### 故障排除組合
```bash
# 快速診斷組合
r quick-diag && r log-errors && r network-check

# 深度分析組合
r system-health && r containers-status && r zenoh-check
```

## ⚠️ 重要注意事項

### 環境區分
- **宿主機**: 使用 `r` 命令和基本診斷
- **容器內**: 使用專業工具集和開發工具
- **開發工具**: 大部分需要在容器內執行

### 容器狀態
- **正常運行**: `agvc_server`, `postgres_container`, `nginx`
- **AGV 容器**: 通常在實際車輛上運行，開發環境可能不存在

### 工具特性
- **無需安裝**: 直接使用，無需修改系統配置
- **環境安全**: 不會關閉終端，正確顯示錯誤信息
- **智能檢測**: 自動識別 AGV/AGVC 環境

## 🔗 詳細文檔參考

### 核心工具文檔
- @docs-ai/AI_LEARNING_GUIDE.md - AI 助理學習指南
- @docs-ai/operations/maintenance/unified-tools.md - 統一工具使用指南
- @docs-ai/operations/deployment/container-management.md - Docker 容器管理
- @docs-ai/operations/maintenance/system-diagnostics.md - 系統健康檢查

### 專業工具文檔
- @docs-ai/operations/maintenance/system-diagnostics.md - 網路診斷
- @docs-ai/operations/maintenance/log-analysis.md - 日誌分析
- @docs-ai/operations/development/build-and-test.md - 開發建置測試

### 使用指南
- @docs-ai/USAGE_GUIDE.md - Prompts 使用指南
- `TOOLS_USAGE_GUIDE.md` - 工具使用指南

## 🎯 快速解決方案

### 系統無法啟動
```bash
r containers-status → r agvc-start → r agvc-check
```

### 網路無法連接
```bash
r network-check → r zenoh-check → r quick-diag
```

### 日誌有錯誤
```bash
r log-scan → r log-errors → 根據錯誤類型處理
```

### 開發環境問題
```bash
r dev-status → 進入容器 → r dev-build
```
