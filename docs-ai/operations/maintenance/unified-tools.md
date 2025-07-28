# RosAGV 宿主機統一工具系統

## 🎯 適用場景
- 宿主機環境的日常運維管理
- 快速系統診斷和故障排除
- 容器狀態檢查和管理
- 開發環境狀態監控
- 新手快速上手 RosAGV 工具
- AI 助理學習 RosAGV 工具使用

## 📋 統一工具概述

### 超簡單使用方式
RosAGV 提供統一的工具入口，只需要記住一個字母：`r`

```bash
# 🚀 最簡單的使用方式
r                    # 顯示工具選單
r agvc-check         # 執行健康檢查
r containers-status  # 檢查容器狀態
r quick-diag         # 快速綜合診斷
```

### 完整命令格式
```bash
# 完整命令格式 (備選)
bash rosagv-tools.sh [command]
./rosagv-tools.sh [command]
```

## 🔧 核心工具分類

### 🔍 系統診斷工具
- **`agvc-check`**: AGVC 管理主機快速健康檢查
- **`agv-check`**: AGV 車載系統快速健康檢查  
- **`system-health`**: 完整系統健康檢查
- **`quick-diag`**: 快速綜合診斷

### 🐳 容器管理工具
- **`containers-status`**: 檢查所有容器狀態
- **`agv-start/stop`**: AGV 容器啟停管理
- **`agvc-start/stop`**: AGVC 系統啟停管理

### 🌐 網路診斷工具
- **`network-check`**: 系統端口檢查
- **`zenoh-check`**: Zenoh 連接檢查

### 📋 日誌分析工具
- **`log-scan`**: 日誌錯誤掃描
- **`log-errors`**: 高級錯誤掃描

### 🛠️ 開發工具
- **`dev-status`**: 開發環境狀態 (宿主機可用)
- **`dev-build`**: 快速建置 (需容器環境)
- **`dev-test`**: 快速測試 (需容器環境)
- **`dev-check`**: 代碼檢查 (需容器環境)

## 📂 工具系統架構

### 宿主機統一入口
```
rosagv-tools.sh (別名: r)
├── 系統診斷模組
├── 容器管理模組  
├── 網路診斷模組
├── 日誌分析模組
└── 開發工具模組
```

### Scripts 工具集整合
統一工具會調用 `scripts/` 目錄下的專業工具集：
- `scripts/system-tools/` - 系統診斷
- `scripts/docker-tools/` - 容器管理
- `scripts/network-tools/` - 網路診斷
- `scripts/log-tools/` - 日誌分析
- `scripts/dev-tools/` - 開發工具

## 🚀 使用場景範例

### 日常運維檢查
```bash
# 每日例行檢查
r agvc-check              # AGVC 主機健康檢查
r containers-status       # 檢查容器運行狀態
r network-check          # 網路連接檢查
```

### 故障排除流程
```bash
# 遇到問題時的診斷流程
r quick-diag             # 1. 快速綜合診斷
r log-errors             # 2. 深度日誌分析  
r zenoh-check           # 3. Zenoh 連接專項檢查
```

### 開發環境檢查
```bash
# 開發前的環境檢查
r dev-status            # 檢查開發環境狀態
r containers-status     # 確認容器運行正常

# 進入容器後的開發操作
# (需要先進入對應容器)
r dev-build            # 快速建置
r dev-test             # 執行測試
r dev-check            # 代碼品質檢查
```

## 🔧 工具特性與優勢

### ✅ 核心優勢
- **無需安裝**: 直接使用，無需修改系統配置
- **環境安全**: 不會關閉終端，正確顯示錯誤信息
- **智能檢測**: 自動識別 AGV/AGVC 環境
- **完整選單**: 內建幫助和說明系統
- **便攜性**: 可在任何有 bash 的環境使用
- **一致介面**: 統一的命令格式和輸出風格

### 🎯 設計理念
- **簡化操作**: 一個字母 `r` 解決所有工具調用
- **分層設計**: 統一入口 + 專業工具集
- **環境適應**: 自動檢測並適配不同環境
- **錯誤友好**: 清晰的錯誤提示和解決建議

## ⚠️ 使用注意事項

### 環境依賴
- **宿主機工具**: 大部分診斷工具可在宿主機直接使用
- **容器內工具**: 開發相關工具需要在對應 Docker 容器内執行
- **權限要求**: 某些系統檢查可能需要適當的權限

### 工具限制
- 日誌分析工具預設分析最近 1000 行日誌
- 網路檢查工具主要檢查本機端口狀態
- 健康檢查會生成詳細報告，可能需要一些時間

## 🚨 故障排除

### 工具本身問題
```bash
# 查看工具幫助
r tools-help

# 檢查工具版本和狀態  
r --version
r --check
```

### 常見問題解決
1. **命令找不到**: 確保在 RosAGV 專案目錄下執行
2. **權限問題**: 檢查腳本執行權限
3. **容器工具失效**: 確認目標容器正在運行
4. **診斷結果異常**: 使用 `quick-diag` 獲取完整診斷資訊

## 💡 進階使用技巧

### 組合使用
```bash
# 完整診斷流程
r agvc-check && r network-check && r log-scan

# 開發工作流程
r dev-status && r containers-status
# 然後進入容器執行開發工具
```

### 輸出重定向
```bash
# 保存診斷結果
r system-health > system-report.txt
r log-errors > error-analysis.log
```

### 定期檢查
```bash
# 可以設置定期檢查任務
# 建議每日執行 agvc-check 或 agv-check
```

## 📋 工具清單參考表

| 類別 | 命令 | 用途 | 環境需求 |
|------|------|------|----------|
| 診斷 | `agvc-check` | AGVC 主機健康檢查 | 宿主機 |
| 診斷 | `agv-check` | AGV 車載健康檢查 | 宿主機 |
| 診斷 | `system-health` | 完整系統健康檢查 | 宿主機 |
| 診斷 | `quick-diag` | 快速綜合診斷 | 宿主機 |
| 容器 | `containers-status` | 容器狀態檢查 | 宿主機 |
| 容器 | `agv-start/stop` | AGV 容器管理 | 宿主機 |
| 容器 | `agvc-start/stop` | AGVC 系統管理 | 宿主機 |
| 網路 | `network-check` | 端口連接檢查 | 宿主機 |
| 網路 | `zenoh-check` | Zenoh 連接檢查 | 宿主機 |
| 日誌 | `log-scan` | 日誌錯誤掃描 | 宿主機 |
| 日誌 | `log-errors` | 高級錯誤分析 | 宿主機 |
| 開發 | `dev-status` | 開發環境狀態 | 宿主機 |
| 開發 | `dev-build` | 快速建置 | 容器內 |
| 開發 | `dev-test` | 快速測試 | 容器內 |
| 開發 | `dev-check` | 代碼品質檢查 | 容器內 |