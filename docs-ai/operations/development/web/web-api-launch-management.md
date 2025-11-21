# Web 服務統一管理機制

## 🏗️ 環境架構

RosAGV 的 Web 服務分布於兩個容器環境：

| 環境 | 管理函數 | 管理服務 | 端口 |
|------|---------|---------|------|
| **AGVC 容器** | `manage_web_api_launch` | api_server, agvcui, opui | 8000, 8001, 8002 |
| **AGV 容器** | `manage_web_agv_launch` | agv_ui_server | 8003 |

## 🎯 適用場景
- Web 服務群組的統一管理
- 啟動腳本中的自動啟動控制
- 手動服務管理和故障排除
- 開發和測試環境的靈活配置

## 📋 功能概述

RosAGV 實現了統一的 Web 服務管理機制，與 Zenoh Router 管理保持一致的 API 介面，提供完整的服務生命週期管理。

### 核心特性
- **統一 API**: 與 `manage_zenoh` 保持一致的管理介面
- **自動啟動控制**: 支援開關控制的自動啟動機制
- **健康監控**: 完整的進程、子服務、端口狀態檢查
- **優雅管理**: 重複啟動檢查、優雅停止、進程清理

## 🚀 管理函式

### manage_web_api_launch 函式
定義在 `/app/setup.bash` 中，提供統一的服務管理介面：

```bash
manage_web_api_launch {start|stop|restart|status}
```

#### 支援的操作

| 操作 | 功能描述 | 返回值 |
|------|----------|--------|
| `start` | 啟動 Web API Launch 服務群組 | 0=成功, 1=失敗 |
| `stop` | 停止所有相關進程並清理 | - |
| `restart` | 重新啟動服務群組 | 0=成功, 1=失敗 |
| `status` | 顯示詳細的服務和端口狀態 | 0=運行中, 1=未運行 |

### manage_web_agv_launch 函式

**環境**: AGV 容器
**定義**: `/app/setup_modules/node-management.bash`

提供 AGV 車載監控界面 (AGVUI) 的統一管理介面：

```bash
manage_web_agv_launch {start|stop|restart|status}
```

#### 支援的操作

| 操作 | 功能描述 | 返回值 |
|------|----------|--------|
| `start` | 啟動 AGVUI 車載監控服務 (Port 8003) | 0=成功, 1=失敗 |
| `stop` | 停止服務並清理進程 | - |
| `restart` | 重新啟動服務 | 0=成功, 1=失敗 |
| `status` | 顯示服務和端口狀態 | 0=運行中, 1=未運行 |

#### 使用範例

```bash
# [AGV 容器內] 啟動 AGVUI
manage_web_agv_launch start

# [AGV 容器內] 檢查狀態
manage_web_agv_launch status

# [AGV 容器內] 重啟服務
manage_web_agv_launch restart
```

#### 自動啟動控制

在 `/app/startup.agv.bash` 中控制自動啟動：

```bash
# 設定自動啟動開關 (true=啟動, false=跳過)
AUTO_START_WEB_AGV_LAUNCH=true

# 根據開關決定是否啟動 AGVUI
if [ "$AUTO_START_WEB_AGV_LAUNCH" = "true" ]; then
    echo "🌐 啟動 AGVUI 車載監控..."
    manage_web_agv_launch start
else
    echo "⏸️ AGVUI 自動啟動已停用"
fi
```

## 🔧 自動啟動控制

### 啟動腳本整合
在 `/app/startup.agvc.bash` 中整合了自動啟動控制：

```bash
# 設定自動啟動開關 (true=啟動, false=跳過)
AUTO_START_WEB_API_LAUNCH=true

# 根據開關決定是否啟動 Web API Launch
if [ "$AUTO_START_WEB_API_LAUNCH" = "true" ]; then
    echo "🌐 啟動 Web API Launch 服務群組..."
    manage_web_api_launch start
else
    echo "⏸️ Web API Launch 自動啟動已停用 (AUTO_START_WEB_API_LAUNCH=false)"
fi
```

### 開關控制
```bash
# 啟用自動啟動
AUTO_START_WEB_API_LAUNCH=true

# 停用自動啟動 (用於測試或調試)
AUTO_START_WEB_API_LAUNCH=false
```

## 🛠️ 使用指導

### 基本操作
```bash
# 在容器內執行 (載入 setup.bash 後可用)
manage_web_api_launch start     # 啟動服務
manage_web_api_launch stop      # 停止服務
manage_web_api_launch restart   # 重啟服務
manage_web_api_launch status    # 檢查狀態
```

### 啟動服務
```bash
# 啟動 Web API Launch 服務群組
manage_web_api_launch start

# 預期輸出:
# 🚀 啟動 Web API Launch 服務群組...
# ✅ Web API Launch 已啟動 (PID: 211)
# 🔍 檢查 Web 服務端口狀態...
# ✅ Web API 端口 8000 已開啟
# ✅ AGVCUI 端口 8001 已開啟
# ✅ OPUI 端口 8002 已開啟
```

### 檢查狀態
```bash
# 查看詳細狀態
manage_web_api_launch status

# 預期輸出:
# ✅ Web API Launch 正在運行 (PID: 211)
# 🔍 子服務狀態：
#   ✅ AGVCUI 服務運行中
#   ✅ OPUI 服務運行中
#   ✅ Web API 服務運行中
# 🔍 端口狀態：
#   ✅ 端口 8000 已開啟
#   ✅ 端口 8001 已開啟
#   ✅ 端口 8002 已開啟
```

### 停止服務
```bash
# 停止所有相關服務
manage_web_api_launch stop

# 功能:
# - 停止主進程 (launch)
# - 清理所有子進程 (agvc_ui_server, op_ui_server, api_server)
# - 移除 PID 檔案
# - 確保完全清理
```

## 🔍 技術實現

### 服務架構
```
Web API Launch 服務群組
├── ros2 launch web_api_launch launch.py (主進程)
├── agvc_ui_server (PID: xxx) - Port 8001
├── op_ui_server (PID: xxx) - Port 8002
└── api_server (PID: xxx) - Port 8000
```

### 檔案管理
```bash
# 日誌檔案
/tmp/web_api_launch.log         # 服務啟動和運行日誌

# PID 檔案
/tmp/web_api_launch.pid         # 主進程 PID 檔案
```

### 重複啟動檢查
```bash
# 檢查邏輯
if [ -f "$WEB_API_PID_FILE" ] && pgrep -F "$WEB_API_PID_FILE" > /dev/null; then
    echo "✅ Web API Launch 已經在運行中 (PID: $(cat $WEB_API_PID_FILE))"
    return 0
fi
```

### 健康檢查機制
1. **主進程檢查**: 驗證 launch 進程是否運行
2. **子服務檢查**: 檢查三個子服務的運行狀態
3. **端口檢查**: 驗證 8000-8002 端口是否開啟
4. **PID 檔案管理**: 確保 PID 檔案的準確性

## 🧪 測試和調試

### 開發測試流程
```bash
# 1. 停用自動啟動進行測試
# 修改 startup.agvc.bash: AUTO_START_WEB_API_LAUNCH=false

# 2. 重啟容器
docker compose -f docker-compose.agvc.yml restart agvc_server

# 3. 手動測試各種操作
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "
source /app/setup.bash > /dev/null 2>&1
manage_web_api_launch start
manage_web_api_launch status
manage_web_api_launch stop
"

# 4. 重新啟用自動啟動
# 修改 startup.agvc.bash: AUTO_START_WEB_API_LAUNCH=true
```

### 故障排除
```bash
# 查看日誌
tail -f /tmp/web_api_launch.log

# 檢查進程狀態
ps aux | grep -E "(web_api_launch|agvc_ui_server|op_ui_server|api_server)"

# 檢查端口佔用
ss -tuln | grep -E ":(8000|8001|8002)"

# 手動清理進程 (如果函式停止失敗)
pkill -f "web_api_launch"
pkill -f "agvc_ui_server"
pkill -f "op_ui_server"
pkill -f "api_server"
rm -f /tmp/web_api_launch.pid
```

## 🔄 與其他服務的對比

### 統一管理模式對比

| 服務 | 管理函式 | 操作 | 特點 |
|------|----------|------|------|
| **Zenoh Router** | `manage_zenoh` | start/stop/restart/status | 單一進程管理 |
| **Web API Launch** | `manage_web_api_launch` | start/stop/restart/status | 多進程群組管理 |
| **SSH Service** | `manage_ssh` | start/stop/restart/status | 系統服務管理 |

### API 一致性
```bash
# 所有服務管理函式都遵循相同的 API 格式
manage_zenoh {start|stop|restart|status}
manage_web_api_launch {start|stop|restart|status}
manage_ssh {start|stop|restart|status}
```

## 📚 最佳實踐

### 開發環境建議
1. **測試時停用自動啟動**: 設定 `AUTO_START_WEB_API_LAUNCH=false`
2. **使用統一函式**: 優先使用 `manage_web_api_launch` 而非直接操作
3. **檢查服務狀態**: 定期使用 `status` 操作檢查服務健康度
4. **查看詳細日誌**: 問題排除時檢查 `/tmp/web_api_launch.log`

### 生產環境建議
1. **啟用自動啟動**: 設定 `AUTO_START_WEB_API_LAUNCH=true`
2. **監控服務狀態**: 定期執行健康檢查
3. **日誌輪轉**: 配置適當的日誌輪轉策略
4. **資源監控**: 監控服務的 CPU 和記憶體使用

## 🔗 交叉引用

### 相關文檔
- 統一工具系統: docs-ai/operations/tools/unified-tools.md
- Docker 開發環境: docs-ai/operations/development/docker-development.md
- 系統診斷工具: docs-ai/operations/guides/system-diagnostics.md
- 雙環境架構: docs-ai/context/system/dual-environment.md

### 相關模組
- Launch 工作空間: `app/launch_ws/CLAUDE.md`
- Web API 服務: `app/web_api_ws/CLAUDE.md`
- Setup 腳本: `app/setup.bash`
- 啟動腳本: `app/startup.agvc.bash`