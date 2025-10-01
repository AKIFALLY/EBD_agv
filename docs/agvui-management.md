# AGVUI 管理指南

## 概述
AGVUI 是 AGV 車載監控界面，可顯示完整的 330+ PLC 屬性資料，支援多車監控和即時更新。

## 管理方式

### 使用 manage_agvui 函數（推薦）
```bash
# 在容器內或透過容器執行
source /app/setup.bash

# 管理指令
manage_agvui start    # 啟動服務
manage_agvui stop     # 停止服務
manage_agvui restart  # 重啟服務
manage_agvui status   # 檢查狀態
manage_agvui logs     # 查看日誌
```

### 從宿主機管理
```bash
# 啟動 AGVUI
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && manage_agvui start"

# 檢查狀態
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && manage_agvui status"

# 停止服務
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && manage_agvui stop"
```

## 自動啟動配置

### AGV 車載環境
在 `/app/startup.agv.bash` 中配置：
```bash
# =====================================
# 🖥️ AGVUI 服務自動啟動配置
# =====================================
AUTO_START_AGVUI=true  # 設定為 true 啟用自動啟動，false 停用
```

當設定為 `true` 時，AGVUI 會在容器啟動時自動運行。

## 功能特點

### 多車支援
- 支援 6 台 AGV 同時監控（loader01, loader02, cargo01, cargo02, unloader01, unloader02）
- 自動識別本機 AGV ID
- 測試模式支援 URL 參數覆寫

### 即時更新
- 每秒自動更新狀態資料
- DOM 更新策略，不影響捲動位置
- Socket.IO 即時通訊

### 視覺化顯示
- 多欄位網格佈局，一目了然
- IO/警報狀態網格化顯示（綠色=ON，灰色=OFF）
- 分類顯示：基本資訊、位置狀態、速度狀態、門控狀態等

## 存取界面

### 生產環境
- AGV 車載監控：`http://<AGV_IP>:8003`
- 顯示本機 AGV 的完整 330+ 屬性

### 測試環境
- AGVC 中央監控：`http://192.168.100.100:8003`
- 測試頁面：`http://192.168.100.100:8003/test`
- 指定 AGV：`http://192.168.100.100:8003/?agv_id=loader01`

## 狀態檔案

### 單機模式（實際 AGV）
- 讀取 `/tmp/agv_status.json`
- 由 AGV 系統即時產生

### 多機模式（測試/中央監控）
- 讀取 `/tmp/agv_status_*.json`
- 使用測試腳本產生：
```bash
/home/ct/RosAGV/scripts/test-agvui-multi.sh
```

## 故障排除

### 服務無法啟動
```bash
# 檢查端口佔用
docker compose -f docker-compose.agvc.yml exec agvc_server ss -tulpn | grep 8003

# 檢查日誌
docker compose -f docker-compose.agvc.yml exec agvc_server tail -f /tmp/agvui.log

# 清理舊進程
docker compose -f docker-compose.agvc.yml exec agvc_server ps aux | grep agv_ui_server
docker compose -f docker-compose.agvc.yml exec agvc_server kill <PID>
```

### 資料不顯示
```bash
# 檢查狀態檔案
docker compose -f docker-compose.agvc.yml exec agvc_server ls -la /tmp/agv_status*.json

# 產生測試資料
/home/ct/RosAGV/scripts/test-agvui-multi.sh

# 複製到容器
docker cp /tmp/agv_status_loader01.json agvc_server:/tmp/
```

## 技術架構

- **後端**: FastAPI + Socket.IO + ROS 2
- **前端**: JavaScript + Bulma CSS
- **通訊**: Socket.IO WebSocket + 檔案系統
- **部署**: Docker 容器化

## 相關檔案

- 主程式：`/app/web_api_ws/src/agvui/agvui/agv_ui_server.py`
- 前端邏輯：`/app/web_api_ws/src/agvui/agvui/static/js/agvPage.js`
- 管理函數：`/app/setup.bash` 中的 `manage_agvui`
- 自動啟動：`/app/startup.agv.bash`
- 測試腳本：`/home/ct/RosAGV/scripts/test-agvui-multi.sh`