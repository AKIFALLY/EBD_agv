# 統一工具系統 (r 命令)

## 🎯 快速開始

RosAGV 提供強大的統一工具系統，只需記住一個字母 `r` 即可存取所有管理功能。

### 環境設定
```bash
# 在 ~/.bashrc 中添加
export PATH="/home/ct/RosAGV:$PATH"

# 重新載入環境
source ~/.bashrc

# 驗證配置
which r                    # 應該顯示 /home/ct/RosAGV/r
r                          # 顯示工具選單
```

## 📋 工具分類

### 🔍 系統診斷工具

| 命令 | 功能 | 說明 |
|------|------|------|
| `r agvc-check` | AGVC 健康檢查 | 檢查 AGVC 管理系統所有服務狀態 |
| `r agv-check` | AGV 健康檢查 | 檢查 AGV 車載系統狀態 |
| `r quick-diag` | 快速綜合診斷 | 一鍵執行完整系統診斷 |
| `r system-health` | 完整健康檢查 | 深度系統健康狀態分析 |

### 🐳 容器管理工具

| 命令 | 功能 | 說明 |
|------|------|------|
| `r containers-status` | 容器狀態檢查 | 顯示所有容器運行狀態 |
| `r agvc-start` | 啟動 AGVC | 啟動 AGVC 管理系統 |
| `r agvc-stop` | 停止 AGVC | 停止 AGVC 管理系統 |
| `r agv-start` | 啟動 AGV | 啟動 AGV 車載系統 |
| `r agv-stop` | 停止 AGV | 停止 AGV 車載系統 |

### 🌐 網路診斷工具

| 命令 | 功能 | 說明 |
|------|------|------|
| `r network-check` | 網路連接檢查 | 檢查所有服務端口連通性 |
| `r zenoh-check` | Zenoh 連接檢查 | 檢查 Zenoh Router 通訊狀態 |

### ⚙️ 配置管理工具

| 命令 | 功能 | 說明 |
|------|------|------|
| `r zenoh-config` | Zenoh 配置管理 | 管理 Zenoh Router 配置 |
| `r hardware-config` | 硬體映射配置 | 管理硬體設備映射配置 |

### 🔎 TAFL 語言工具

| 命令 | 功能 | 說明 |
|------|------|------|
| `r tafl-validate [file]` | 驗證 TAFL 檔案 | 檢查 TAFL 語法和邏輯 |
| `r tafl-validate all` | 驗證所有檔案 | 批量驗證所有 TAFL 檔案 |
| `r tafl-validate list` | 列出 TAFL 檔案 | 顯示所有可用的 TAFL 檔案 |

## 🚀 常用工作流程

### 日常維護流程
```bash
# 每日系統檢查
r agvc-check
r containers-status
r network-check

# 問題診斷
r quick-diag
r zenoh-check

# 查看詳細日誌
docker compose -f docker-compose.agvc.yml logs -f agvc_server
```

### 開發工作流程
```bash
# 重啟服務
r agvc-stop
r agvc-start

# 配置管理
r zenoh-config
r hardware-config

# TAFL 開發
r tafl-validate my_flow.yaml
```

### 故障排除流程
```bash
# 第一階段：快速評估
r quick-diag

# 第二階段：定位問題
r network-check
r zenoh-check

# 第三階段：深度分析
r system-health
```

## 🔧 進階功能

### Zenoh 配置管理
```bash
# 查看配置概況
r zenoh-config

# 直接使用專業工具
scripts/config-tools/zenoh-config.sh validate  # 驗證配置
scripts/config-tools/zenoh-config.sh edit      # 編輯配置
scripts/config-tools/zenoh-config.sh status    # 服務狀態
```

### 硬體映射管理
```bash
# 查看硬體映射
r hardware-config

# 詳細操作
scripts/config-tools/hardware-mapping.sh list          # 列出設備
scripts/config-tools/hardware-mapping.sh show device_1 # 顯示特定設備
scripts/config-tools/hardware-mapping.sh edit device_1 # 編輯設備配置
```

## 📊 工具詳細說明

### agvc-check 健康檢查
執行內容：
1. 檢查 Docker 容器狀態
2. 驗證網路連通性
3. 檢查資料庫連接
4. 驗證 Web 服務回應
5. 檢查 Zenoh 通訊

輸出範例：
```
🏥 AGVC 系統健康檢查
===================
✅ Docker 容器: 運行中
✅ PostgreSQL: 連接正常
✅ Web API (8000): 回應正常
✅ AGVCUI (8001): 回應正常
✅ OPUI (8002): 回應正常
✅ Zenoh Router: 運行中
⚠️  警告: CPU 使用率偏高 (>80%)
```

### quick-diag 快速診斷
包含檢查項目：
- 容器運行狀態
- 網路端口檢查
- 服務健康狀態
- 資源使用情況
- 最近錯誤日誌

### tafl-validate 驗證工具
驗證內容：
- YAML 語法正確性
- TAFL 動詞識別
- 必要參數完整性
- 變數引用檢查
- 邏輯結構合理性

## 💡 使用技巧

### 別名設定
在 `~/.bashrc` 中添加常用別名：
```bash
alias rcheck='r agvc-check'
alias rdiag='r quick-diag'
alias rstatus='r containers-status'
```

### 組合使用
```bash
# 完整系統檢查
r agvc-check && r containers-status && r network-check

# 批量操作
for check in agvc-check network-check zenoh-check; do
    echo "執行: r $check"
    r $check
done
```

### 輸出重定向
```bash
# 儲存診斷結果
r quick-diag > ~/diagnosis_$(date +%Y%m%d).log

# 持續監控
watch -n 5 'r containers-status'
```

## 🛠️ 故障排除

### 常見問題

#### r 命令找不到
```bash
# 檢查 PATH 設定
echo $PATH | grep RosAGV

# 手動執行
/home/ct/RosAGV/r

# 重新設定 PATH
export PATH="/home/ct/RosAGV:$PATH"
```

#### 權限問題
```bash
# 確保執行權限
chmod +x /home/ct/RosAGV/r
chmod +x /home/ct/RosAGV/rosagv-tools.sh

# Docker 權限
sudo usermod -aG docker $USER
```

#### 容器連接失敗
```bash
# 檢查 Docker 服務
systemctl status docker

# 檢查容器狀態
docker ps -a

# 重啟 Docker
sudo systemctl restart docker
```

## 🔗 相關文檔
- [系統診斷](system-diagnostics.md)
- [服務管理工具](service-management.md)
- [Docker 開發環境](development.md)
- [故障排除指南](troubleshooting.md)

## 📝 工具開發指南

### 新增自定義工具
在 `rosagv-tools.sh` 中添加：
```bash
"my-tool")
    echo "執行自定義工具..."
    # 工具邏輯
    ;;
```

### 工具命名規範
- 使用連字符分隔: `agvc-check`
- 動詞-名詞格式: `check-system`
- 簡短明確: 不超過15個字元

---
*最後更新: 2025-09-18*