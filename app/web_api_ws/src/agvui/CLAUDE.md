# agvui - AGV車載監控界面

## 專案概述
agvui是RosAGV系統的AGV車載監控界面，為AGV設備提供簡化的Web界面，用於顯示AGV狀態、任務資訊和基本控制功能。基於FastAPI + Socket.IO技術，專為車載設備的有限資源環境設計。

## 核心模組

### 後端架構
- **AgvUiServer** (`agv_ui_server.py`): 主要FastAPI伺服器和Socket.IO整合
- **AgvUiSocket** (`agv_ui_socket.py`): Socket.IO事件處理
- **AgvUiRos** (`agv_ui_ros.py`): ROS 2節點整合

### 前端架構 (簡化設計)
- **靜態資源** (`static/`): CSS、JavaScript資源
- **模板** (`templates/`): 簡化的HTML模板
- **狀態管理**: 輕量級miniStore

## 關鍵檔案

### 後端核心檔案
- `/agvui/agv_ui_server.py` - 主要FastAPI應用伺服器
- `/agvui/agv_ui_socket.py` - Socket.IO連接管理
- `/agvui/agv_ui_ros.py` - ROS 2節點背景服務

### 前端資源
```
static/
├── css/
│   ├── bulma_1_0_4.min.css     # Bulma CSS框架
│   ├── materialdesignicons.css # Material Design圖標
│   └── opui-bulma-extend.css   # 自定義樣式擴展
├── js/
│   ├── agvPage.js              # AGV頁面主要邏輯
│   ├── navbar.js               # 導航欄功能
│   ├── notify.js               # 通知系統
│   ├── socket.js               # Socket.IO客戶端
│   └── lib/                    # 第三方庫
└── store/
    ├── index.js                # 狀態管理入口
    └── miniStore.js            # 輕量級狀態管理
```

### 模板檔案
```
templates/
├── agv.html                    # AGV監控主頁面
├── base.html                   # 基礎模板
└── navbar.html                 # 導航欄模板
```

## 開發指令

### 基本操作
```bash
# 進入AGVC容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash
source /app/setup.bash && all_source

# 構建agvui
build_ws web_api_ws
# 或單獨構建
colcon build --packages-select agvui

# 啟動AGVUI伺服器
cd /app/web_api_ws/src/agvui
python3 agvui/agv_ui_server.py
```

### 開發模式啟動
```bash
# 使用ROS 2啟動 (entry_point方式)
ros2 run agvui agv_ui_server

# 直接執行Python模組
python3 -m agvui.agv_ui_server
```

### 服務配置
```python
# agv_ui_server.py 預設配置
HOST = "0.0.0.0"
PORT = 8003  # 已修改為8003，避免與agvcui (8001) 衝突
CORS_ORIGINS = "*"
```

## 整合點

### 與其他專案整合
- **agv_base**: 接收AGV狀態更新事件
- **agv_interfaces**: 使用AGV狀態和命令訊息類型
- **web_api_ws**: 共享web_api_ws工作空間
- **Socket.IO**: 與後端系統進行即時通訊

### 技術棧
- **後端**: FastAPI + Socket.IO + ROS 2
- **前端**: 原生JavaScript + Bulma CSS + miniStore
- **通訊**: WebSocket (Socket.IO)
- **狀態管理**: 輕量級miniStore

## 服務特點

### 簡化設計
- **輕量級**: 專為車載設備資源限制設計
- **單頁應用**: 主要顯示AGV狀態和基本資訊
- **即時更新**: 透過Socket.IO接收即時狀態更新
- **響應式**: 支援觸控和小螢幕設備

### 功能範圍
- AGV狀態監控 (電池、位置、任務狀態)
- 基本系統資訊顯示
- 即時通知和警告
- 簡化的導航界面

## 開發注意事項

### 端口配置
```bash
# agvui現在使用port 8003，避免與agvcui (port 8001) 衝突
# 端口分配：
# - 8000: Web API Gateway
# - 8001: AGVCUI (車隊管理系統)
# - 8002: OPUI (操作員界面)
# - 8003: AGVUI (AGV車載監控)
```

### ROS 2整合
```python
# AgvUiRos類別負責ROS 2節點背景服務
# 在background thread中運行，避免阻塞FastAPI
class AgvUiRos:
    def start(self):
        # 背景執行ROS 2節點
        pass
```

### Socket.IO事件
```javascript
// 前端事件監聽範例
socket.on('agv_status_update', (data) => {
    // 更新AGV狀態顯示
});

socket.on('task_update', (data) => {
    // 更新任務資訊
});
```

## 故障排除

### 常見問題

#### 端口衝突
```bash
# 檢查端口佔用
netstat -tulpn | grep :8003

# 如需修改服務端口
# 編輯agv_ui_server.py中的PORT設定 (目前為8003)
```

#### Socket.IO連接失敗
```bash
# 檢查Socket.IO服務
curl http://localhost:8003/socket.io/

# 檢查CORS設定
# 確認cors_allowed_origins="*"設定
```

#### ROS 2節點問題
```bash
# 檢查ROS 2環境
ros2 node list | grep agv_ui

# 檢查節點日誌
ros2 node info /agv_ui_node
```

### 除錯建議
- 使用瀏覽器開發工具檢查WebSocket連接
- 監控FastAPI日誌查看請求處理狀況
- 檢查ROS 2節點是否正常運行
- 確認與其他Web服務的端口不衝突

## 部署建議

### 車載部署
- 建議部署在AGV容器中，使用host網路模式
- 配置合適的端口避免與其他服務衝突
- 優化前端資源大小以適應車載網路環境
- 考慮離線模式下的基本功能

### 資源優化
- 壓縮CSS和JavaScript檔案
- 使用CDN或本地快取第三方庫
- 實施適當的瀏覽器快取策略
- 監控記憶體使用避免車載設備資源耗盡