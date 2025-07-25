# web_api_ws CLAUDE.md

## 模組概述
Web API服務系統，提供RESTful API、Socket.IO實時通訊與管理界面

## 專案結構 (實際驗證)
```
src/
├── web_api/        # API Gateway服務 (Port 8000) - PLC/交管/門控整合
├── agvcui/         # 車隊管理系統 (Port 8001) - 完整MVC架構
├── opui/           # 操作員界面 (Port 8002) - 任務管理界面  
└── agvui/          # AGV車載監控界面
```

## 服務架構 (基於實際檔案結構)

### Web服務端口 (docker-compose.agvc.yml驗證)
- **8000**: `web_api` - API Gateway (PLC、KUKA Fleet、交管整合)
- **8001**: `agvcui` - 車隊管理系統 (完整資料庫操作和UI)
- **8002**: `opui` - 操作員任務管理界面

### 實際技術棧
- **web_api**: FastAPI + 外部系統整合 (無完整MVC架構)
- **agvcui**: 完整MVC架構 + 資料庫操作 + 前端界面
- **opui**: Vue.js界面 + 任務管理功能
- **資料庫**: PostgreSQL (透過setup.bash的資料庫服務)

## 開發指令

### 環境設定 (AGVC容器內)
```bash
source /app/setup.bash
agvc_source  # 載入AGVC工作空間
cd /app/web_api_ws
```

### 服務啟動 (AGVC容器內執行)
```bash
# 必須先進入AGVC容器並載入環境
docker compose -f docker-compose.agvc.yml exec agvc_server bash
source /app/setup.bash && agvc_source

# API Gateway服務 (實際路徑驗證)
python3 src/web_api/web_api/api_server.py

# AGVCUI車隊管理系統
python3 src/agvcui/agvcui/agvc_ui_server.py

# OPUI操作員界面  
python3 src/opui/opui/opui_server.py

# 或使用ROS2啟動 (如果已建置)
ros2 run web_api api_server
ros2 run agvcui agvc_ui_server
ros2 run opui opui_server
```

### 構建與測試 (setup.bash驗證可用)
```bash
# 使用setup.bash中的函數
build_ws web_api_ws

# 檢查服務狀態
curl http://localhost:8000/health    # web_api健康檢查
curl http://localhost:8001/         # agvcui界面
curl http://localhost:8002/         # opui界面
```

## API開發指南 (基於實際架構)

### Web API服務 (API Gateway模式)
**實際路徑**: `src/web_api/routers/` (驗證存在)
- `door.py` - 門控系統API
- `plc.py` - PLC設備API  
- `traffic.py` - 交通管理API
- `kuka.py` - KUKA Fleet整合API

### AGVCUI系統 (完整MVC架構)
**實際路徑**: `src/agvcui/` (驗證存在完整結構)
- `database/` - 資料庫操作層
- `routers/` - 完整的API路由系統
- `static/` & `templates/` - 前端資源

### 新增API端點流程
1. **Web API**: 在 `src/web_api/routers/` 添加新路由檔案
2. **AGVCUI**: 在 `src/agvcui/routers/` 實現完整CRUD操作
3. **註冊**: 在對應的 `*_server.py` 中註冊路由

### 實際API結構範例
```python
# web_api/routers/door.py (外部系統整合)
@router.get("/door/{door_id}/status")
async def get_door_status(door_id: str):
    # PLC整合邏輯
    
# agvcui/routers/*.py (完整業務邏輯)  
@router.get("/agv/{agv_id}/status")
async def get_agv_status(agv_id: str):
    # 完整的資料庫查詢和業務邏輯
```

## Socket.IO事件管理

### 事件命名空間
- `/agv` - AGV狀態更新
- `/system` - 系統狀態事件
- `/task` - 任務管理事件

### 新增Socket.IO事件
1. 在對應socket類別添加事件處理器
2. 更新前端JavaScript事件監聽
3. 測試事件流與資料格式

```python
@socketio.on('connect', namespace='/agv')
def handle_agv_connect():
    emit('status', {'message': 'AGV Socket Connected'})
```

## 前端開發指南

### OPUI (Vue.js 3)
- **架構**: 完整重構的現代化界面
- **組件**: 模組化Vue組件設計
- **狀態管理**: Vuex/Pinia整合
- **開發**: 支援熱重載開發模式

### AGVCUI
- **功能**: 車隊管理與監控
- **整合**: 與核心API緊密整合
- **即時更新**: Socket.IO實時資料

## 配置管理

### 服務配置 (docker-compose.agvc.yml驗證)
- **端口映射**: 8000-8002:8000-8002 (實際配置)
- **資料庫**: PostgreSQL透過setup.bash的start_db/stop_db管理
- **環境變數**: CONTAINER_TYPE="agvc" (容器環境檢測)

### 實際配置檔案
- **Web API配置**: `/app/config/web_api_config.yaml` (實際存在)
- **硬體映射**: `/app/config/hardware_mapping.yaml`
- **Zenoh配置**: `/app/routerconfig.json5`

## 測試與調試

### API測試
```bash
# FastAPI自動文檔
curl http://localhost:8000/docs

# Socket.IO測試
python -c "import socketio; sio = socketio.Client(); sio.connect('http://localhost:8000')"
```

### 前端調試
- 瀏覽器開發工具：檢查Network與Console
- Vue Devtools：組件狀態調試
- Socket.IO調試：查看事件流

## 部署注意事項

### 容器內服務
- 所有服務運行在AGVC容器內
- 通過nginx反向代理對外服務
- 支援SSL終端與負載均衡

### 安全考量
- CORS政策正確配置
- API認證與授權機制
- 敏感資料環境變數管理

## 故障排除

### 常見問題
1. **端口衝突**: 檢查8000-8002端口佔用
2. **資料庫連接失敗**: 確認db_proxy服務狀態
3. **Socket.IO斷線**: 檢查網路連接與防火牆
4. **前端資源載入失敗**: 確認nginx配置

### 日誌位置
- API日誌：容器內stdout
- Nginx日誌：`/var/log/nginx/`
- 瀏覽器日誌：開發工具Console

## 重要提醒
- API變更需更新前端界面
- Socket.IO事件要確保向後兼容
- 資料庫變更透過db_proxy進行
- 所有服務必須在AGVC容器內運行