# 服務管理工具

## 🎯 統一服務管理 API

RosAGV 提供統一的服務管理介面，所有核心服務都遵循相同的管理 API，確保操作的一致性和可預測性。

## 📋 核心服務列表

### Web API Launch 服務群組
**manage_web_api_launch** - 管理所有 Web 相關服務

包含的服務：
- `ros2 launch web_api_launch launch.py` (主進程)
- `agvc_ui_server` (Port 8001)
- `op_ui_server` (Port 8002)
- `api_server` (Port 8000)

### Zenoh Router 服務
**manage_zenoh** - 管理 Zenoh 通訊路由器

### SSH 服務
**manage_ssh** - 管理容器內 SSH 服務

## 🔧 統一管理介面

所有服務管理工具都遵循相同的介面：

```bash
manage_<service> {start|stop|restart|status}
```

### 操作說明

| 命令 | 功能 | 說明 |
|------|------|------|
| `start` | 啟動服務 | 檢查是否已運行，避免重複啟動 |
| `stop` | 停止服務 | 優雅關閉，清理相關進程 |
| `restart` | 重啟服務 | 執行 stop 後再 start |
| `status` | 狀態檢查 | 顯示詳細運行狀態 |

## 🚀 Web API Launch 管理

### 基本操作
```bash
# 進入容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash
source /app/setup.bash && agvc_source

# 服務管理
manage_web_api_launch start     # 啟動服務群組
manage_web_api_launch stop      # 停止服務群組
manage_web_api_launch restart   # 重啟服務群組
manage_web_api_launch status    # 檢查狀態
```

### 一行指令（從宿主機）
```bash
# 使用 bash -i 確保環境載入
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
source /app/setup.bash &&
agvc_source &&
manage_web_api_launch restart
"
```

### 自動啟動控制
在 `startup.agvc.bash` 中設定：
```bash
AUTO_START_WEB_API_LAUNCH=true   # 容器啟動時自動啟動
AUTO_START_WEB_API_LAUNCH=false  # 手動管理（測試用）
```

### 狀態輸出範例
```
🔍 Web API Launch 服務狀態檢查
================================
✅ ros2 launch 主進程: 運行中 (PID: 1234)
✅ API Server (8000): 運行中 (PID: 1235)
✅ AGVCUI (8001): 運行中 (PID: 1236)
✅ OPUI (8002): 運行中 (PID: 1237)

服務健康狀態:
- API 回應時間: 12ms
- WebSocket 連接數: 5
- 記憶體使用: 256MB
```

## ⚙️ 常用工作流程

### 開發工作流程
```bash
# 重建並重啟服務
manage_web_api_launch stop
ba  # build_all
sa  # source_all
manage_web_api_launch start

# 或一行完成
manage_web_api_launch stop && ba && sa && manage_web_api_launch start
```

### TAFL Editor 重建
```bash
# 只重建 AGVCUI
cd /app/web_api_ws
colcon build --packages-select agvcui
manage_web_api_launch restart
```

### 除錯模式
```bash
# 停止自動管理
manage_web_api_launch stop

# 手動啟動以查看輸出
ros2 launch web_api_launch launch.py
```

## 📊 服務監控

### 即時日誌查看
```bash
# 查看所有 Web 服務日誌
tail -f /tmp/web_api_launch.log

# 查看特定服務
tail -f /tmp/agvcui.log
tail -f /tmp/opui.log
tail -f /tmp/api_server.log
```

### 資源監控
```bash
# 查看服務資源使用
ps aux | grep -E "ros2|python" | grep -v grep

# 監控端口使用
ss -tulpn | grep -E "8000|8001|8002"

# 即時資源監控
htop -p $(pgrep -f "web_api")
```

### 健康檢查端點
```bash
# API 健康檢查
curl http://localhost:8000/health

# AGVCUI 健康檢查
curl http://localhost:8001/health

# OPUI 健康檢查
curl http://localhost:8002/health
```

## 🔍 故障排除

### 服務無法啟動

#### 端口被占用
```bash
# 檢查端口占用
ss -tulpn | grep 8000

# 找出占用進程
lsof -i :8000

# 強制釋放端口
kill -9 $(lsof -t -i :8000)
```

#### 依賴服務未啟動
```bash
# 檢查 ROS 2 daemon
ros2 daemon status
ros2 daemon stop
ros2 daemon start

# 檢查 PostgreSQL
psql -h 192.168.100.254 -U postgres -c "SELECT 1"
```

#### 權限問題
```bash
# 檢查檔案權限
ls -la /app/web_api_ws/

# 修復權限
chmod +x /app/web_api_ws/install/setup.bash
```

### 服務異常停止

#### 查看錯誤日誌
```bash
# 查看崩潰日誌
journalctl -u web_api_launch

# 查看 Python 錯誤
cat /tmp/web_api_launch.err
```

#### 記憶體不足
```bash
# 檢查記憶體
free -h

# 查看記憶體使用最多的進程
ps aux --sort=-%mem | head -10

# 清理記憶體
sync && echo 3 > /proc/sys/vm/drop_caches
```

## 🛠️ 進階配置

### 服務參數調整
編輯 `/app/launch_ws/src/web_api_launch/launch/launch.py`:
```python
# 調整服務參數
parameters=[
    {'port': 8000},
    {'workers': 4},
    {'timeout': 300}
]
```

### 環境變數設定
```bash
# 在 setup.bash 中設定
export WEB_API_PORT=8000
export WEB_API_WORKERS=4
export WEB_API_LOG_LEVEL=INFO
```

### 自定義服務管理
創建新的管理函數：
```bash
manage_my_service() {
    case "$1" in
        start)
            echo "啟動 my_service..."
            # 啟動邏輯
            ;;
        stop)
            echo "停止 my_service..."
            # 停止邏輯
            ;;
        restart)
            manage_my_service stop
            manage_my_service start
            ;;
        status)
            echo "檢查 my_service 狀態..."
            # 狀態檢查
            ;;
        *)
            echo "用法: manage_my_service {start|stop|restart|status}"
            ;;
    esac
}
```

## 📝 最佳實踐

### 服務管理原則
1. **優雅關閉**: 總是使用 stop 而非 kill -9
2. **狀態檢查**: 操作前先執行 status
3. **日誌記錄**: 保留操作日誌以便追蹤
4. **錯誤處理**: 檢查命令返回值
5. **超時保護**: 設定合理的啟動超時

### 建議的重啟順序
```bash
1. manage_zenoh stop
2. manage_web_api_launch stop
3. 系統更新或修改
4. manage_zenoh start
5. manage_web_api_launch start
```

### 監控建議
- 設定健康檢查定期執行
- 配置告警閾值（CPU > 80%, Memory > 90%）
- 保留 7 天的服務日誌
- 定期檢查端口連通性

## 🔗 相關文檔
- [統一工具系統](unified-tools.md)
- [系統診斷](system-diagnostics.md)
- [Docker 開發環境](development.md)
- [Web API 開發](../technical-details/ros2-integration.md)

---
*最後更新: 2025-09-18*