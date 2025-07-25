# 🛠️ RosAGV 宿主機工具使用指南

## 🎯 快速開始 - 只需要記住一個字母：`r`

```bash
# 🚀 超短命令 (推薦)
r                    # 顯示工具選單
r agvc-check         # 執行健康檢查
r containers-status  # 檢查容器狀態
r quick-diag         # 快速綜合診斷

# 📝 完整命令 (備選)
bash rosagv-tools.sh agvc-check
```

> 💡 **提示**: 命令 `r` 是 `rosagv-tools.sh` 的快捷方式，只需要打一個字母！

### 🎯 最常用的 3 個命令
```bash
r agvc-check         # 每日健康檢查
r containers-status  # 檢查容器運行狀態  
r quick-diag         # 遇到問題時快速診斷
```

### 🔧 安裝說明 (給同事)
1. 確保在 RosAGV 專案目錄
2. 執行一次：`source ~/.bashrc` 
3. 就可以使用 `r` 命令了！

## 📋 完整工具清單

### 🔍 系統診斷工具
| 命令 | 說明 | 適用環境 |
|------|------|----------|
| `agvc-check` | AGVC 管理主機快速健康檢查 | AGVC 主機 |
| `agv-check` | AGV 車載系統快速健康檢查 | AGV 車輛 |
| `system-health` | 完整系統健康檢查 | 通用 |
| `quick-diag` | 快速綜合診斷 | 通用 |

### 🐳 容器管理工具
| 命令 | 說明 |
|------|------|
| `containers-status` | 檢查所有容器狀態 |
| `agv-start` | 啟動 AGV 容器 |
| `agv-stop` | 停止 AGV 容器 |
| `agvc-start` | 啟動 AGVC 系統 |
| `agvc-stop` | 停止 AGVC 系統 |

### 🌐 網路診斷工具
| 命令 | 說明 |
|------|------|
| `network-check` | 系統端口檢查 |
| `zenoh-check` | Zenoh 連接檢查 |

### 📋 日誌分析工具
| 命令 | 說明 |
|------|------|
| `log-scan` | 日誌錯誤掃描 |
| `log-errors` | 高級錯誤掃描 |

### 🛠️ 開發工具
| 命令 | 說明 | 注意事項 |
|------|------|----------|
| `dev-status` | 開發環境狀態 | 宿主機可用 |
| `dev-build` | 快速建置 | 需容器環境 |
| `dev-test` | 快速測試 | 需容器環境 |
| `dev-check` | 代碼檢查 | 需容器環境 |

## 💡 使用範例

### 日常運維
```bash
# 每日健康檢查
bash rosagv-tools.sh agvc-check

# 檢查容器狀態
bash rosagv-tools.sh containers-status

# 網路診斷
bash rosagv-tools.sh network-check
```

### 故障排除
```bash
# 快速診斷
bash rosagv-tools.sh quick-diag

# 深度日誌分析
bash rosagv-tools.sh log-errors

# Zenoh 連接問題
bash rosagv-tools.sh zenoh-check
```

### 開發工作
```bash
# 檢查開發環境
bash rosagv-tools.sh dev-status

# 在容器內建置 (需先進入容器)
docker compose exec agvc_server bash
# 然後在容器內執行
bash rosagv-tools.sh dev-build
```

## 🔧 工具特性

### ✅ 優點
- **無需安裝**: 直接使用，無需修改系統配置
- **環境安全**: 不會關閉終端，正確顯示錯誤信息
- **智能檢測**: 自動識別 AGV/AGVC 環境
- **完整選單**: 內建幫助和說明
- **便攜性**: 可在任何有 bash 的環境使用

### ⚠️ 注意事項
- 部分開發工具需要在對應的 Docker 容器內執行
- 日誌分析工具會分析最近 1000 行日誌
- 網路檢查工具檢查本機端口狀態
- 健康檢查會生成詳細的系統報告

## 📞 支援

如果遇到問題：
1. 使用 `bash rosagv-tools.sh tools-help` 查看詳細說明
2. 使用 `bash rosagv-tools.sh quick-diag` 進行故障診斷
3. 檢查相關容器是否正常運行

## 🔄 版本歷史

- **v1.0**: 初始版本，整合所有 RosAGV 管理工具
- 替代原有的別名系統，提供更穩定的工具介面