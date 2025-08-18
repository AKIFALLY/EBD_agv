# AGVUI 監控系統架構與實作

## 🎯 適用場景
- 理解 AGV UI (AGVUI) 監控系統的架構和功能
- 開發和部署 AGV 車載監控界面
- 測試和診斷 AGV 狀態顯示問題

## 📋 系統概述

### AGVUI 定位
**AGV User Interface (AGVUI)** 是 RosAGV 系統的車載監控界面，運行在 port 8003，為 AGV 設備提供簡化的 Web 界面，用於顯示 AGV 狀態、任務資訊和基本控制功能。

### 核心特性
- **輕量級設計**: 專為車載設備資源限制環境設計
- **自適應模式**: 自動識別單機部署或多機測試環境
- **即時更新**: 透過 Socket.IO 提供即時狀態更新
- **檔案監控**: 定時讀取狀態檔案並廣播更新

## 🏗️ 系統架構

### 技術棧
- **後端**: FastAPI + Socket.IO + ROS 2
- **前端**: 原生 JavaScript + Bulma CSS + miniStore
- **通訊**: WebSocket (Socket.IO) + 檔案系統監控
- **部署**: Docker 容器內運行，port 8003

### 核心模組
```
agvui/
├── agv_ui_server.py      # 主伺服器 (FastAPI + Socket.IO)
├── agv_ui_socket.py      # Socket.IO 事件處理
├── agv_ui_ros.py         # ROS 2 節點整合
├── static/               # 前端靜態資源
│   ├── js/              # JavaScript 邏輯
│   └── css/             # 樣式檔案
└── templates/           # HTML 模板
    ├── agv.html        # 主監控頁面
    └── test.html       # 測試選擇頁面
```

## 🔧 狀態檔案載入機制

### 雙模式自適應
AGVUI 會根據環境自動決定載入模式：

#### 1. 多機模式（測試/模擬環境）
- **檔案格式**: `/tmp/agv_status_<agv_id>.json`
- **AGV 列表**: loader01, loader02, cargo01, cargo02, unloader01, unloader02
- **用途**: 測試環境或中央監控系統
- **特點**: 同時監控多台 AGV 狀態

#### 2. 單機模式（實際 AGV 部署）
- **檔案格式**: `/tmp/agv_status.json`
- **用途**: 實際 AGV 車載部署
- **特點**: 只監控本機 AGV 狀態

### 身份識別機制
```python
# 載入順序
1. 讀取 /app/.device_identity 確認容器類型
2. 如果是 AGV 容器，讀取 /app/.agv_identity
3. 備用：從環境變數 AGV_ID 讀取
4. 無法識別時顯示所有 AGV 狀態
```

## 🚀 測試工具

### 模擬資料產生腳本
**位置**: `/home/ct/RosAGV/scripts/test-agvui-multi.sh`

**功能**:
- 生成 6 台 AGV 的測試狀態檔案
- 包含 330+ 個狀態屬性
- 自動複製到 Docker 容器內
- 支援隨機數據生成

**使用方式**:
```bash
# 生成測試資料
bash /home/ct/RosAGV/scripts/test-agvui-multi.sh

# 訪問測試頁面
http://localhost:8003/test       # 選擇要監控的 AGV
http://localhost:8003/?agv_id=loader01  # 直接監控特定 AGV
```

### 單機測試腳本
**位置**: `/home/ct/RosAGV/scripts/test-agvui.sh`
- 生成單一 AGV 狀態檔案
- 適用於單機部署測試

## 📊 狀態資料結構

### 核心狀態屬性
```json
{
  "AGV_ID": "loader01",
  "MAGIC": 13243,
  "timestamp": "2025-08-15T17:28:00+08:00",
  "namespace": "loader01",
  "AGV_Auto": 1,
  "AGV_Manual": 0,
  "AGV_MOVING": 0,
  "AGV_SLAM_X": 3456,
  "AGV_SLAM_Y": 2890,
  "AGV_SLAM_ANGLE": 45.6,
  "POWER": 85,
  "AGV_SPEED": 120.5,
  // ... 330+ 屬性
}
```

### 狀態分類
- **基本狀態**: AGV_ID, 模式, 移動狀態
- **位置資訊**: SLAM 座標, PGV 座標, 角度
- **系統狀態**: 電量, 速度, 目標點
- **I/O 狀態**: Input_1-100, Output_1-100
- **警報狀態**: Alarm_1-50
- **PLC 記憶體**: PLC_D1-80

## 🔍 開發與部署

### 啟動服務
```bash
# 在 AGVC 容器內
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash
source /app/setup.bash && agvc_source

# 啟動 AGVUI
python3 -m agvui.agv_ui_server

# 或使用 ROS 2
ros2 run agvui agv_ui_server
```

### API 端點
- `GET /` - 主監控頁面
- `GET /test` - 測試選擇頁面
- `GET /api/identity` - 取得本機 AGV 身份

### Socket.IO 事件
- `agv_status_update` - AGV 狀態更新事件
- `connect` - 客戶端連接事件
- `disconnect` - 客戶端斷開事件

## 🚨 故障排除

### 常見問題

#### 無狀態更新
1. 檢查狀態檔案是否存在：
```bash
docker compose -f docker-compose.agvc.yml exec agvc_server ls -la /tmp/agv_status*.json
```

2. 執行測試腳本生成資料：
```bash
bash /home/ct/RosAGV/scripts/test-agvui-multi.sh
```

#### 服務無法啟動
1. 檢查端口佔用：
```bash
netstat -tulpn | grep :8003
```

2. 檢查環境載入：
```bash
source /app/setup.bash && agvc_source
```

## 💡 最佳實踐

### 部署建議
1. **車載部署**: 使用單機模式，只讀取 `/tmp/agv_status.json`
2. **測試環境**: 使用多機模式，支援多 AGV 監控
3. **資源優化**: 壓縮前端資源，實施瀏覽器快取

### 開發建議
1. **狀態檔案**: 確保 JSON 格式正確，包含必要欄位
2. **更新頻率**: 每秒讀取一次狀態檔案
3. **錯誤處理**: 處理 JSON 解析錯誤和檔案不存在情況

## 🔗 交叉引用
- AGVC 工作空間: @docs-ai/context/workspaces/agvc-workspaces.md
- Web API 系統: `app/web_api_ws/CLAUDE.md`
- Docker 開發: @docs-ai/operations/development/docker-development.md
- 系統診斷: @docs-ai/operations/maintenance/system-diagnostics.md