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

## 🔧 開發工具指南

### 宿主機操作 (Docker 容器管理)

#### AGVC 容器管理工具
```bash
# 載入 Docker 工具集
source scripts/docker-tools/docker-tools.sh

# AGVC 系統基本操作
agvc_start                   # 啟動 AGVC 系統 (所有服務)
agvc_stop                    # 停止 AGVC 系統
agvc_restart                 # 重啟 AGVC 系統
agvc_status                  # 查看 AGVC 系統狀態
agvc_logs                    # 查看 AGVC 系統日誌
agvc_health                  # AGVC 系統健康檢查
agvc_services                # 檢查所有 AGVC 服務狀態

# 快速進入 AGVC 開發環境
agvc_enter                   # 進入 AGVC 容器 (自動載入 agvc_source)

# 快速執行 AGVC 容器內指令
quick_agvc "check_agvc_status"        # 檢查 AGVC 狀態
quick_agvc "curl http://localhost:8000/health"  # API 健康檢查
quick_agvc "build_ws web_api_ws"      # 建置 Web API 工作空間
```

#### Web 服務診斷工具 (宿主機執行)
```bash
# Web 服務狀態檢查
scripts/system-tools/service-monitor.sh status    # 所有服務監控
scripts/docker-tools/container-status.sh agvc     # AGVC 容器詳細狀態

# API 服務測試
curl http://localhost:8000/health     # API Gateway 健康檢查
curl http://localhost:8001/           # AGVCUI 界面檢查
curl http://localhost:8002/           # OPUI 界面檢查

# AGVC 日誌分析
scripts/log-tools/log-analyzer.sh agvc --stats     # AGVC 日誌統計
scripts/log-tools/log-analyzer.sh agvc --timeline  # 錯誤時間軸

# 網路和端口診斷
scripts/network-tools/port-check.sh system         # 系統端口檢查
scripts/network-tools/connectivity-test.sh performance --target localhost
```

#### 資料庫管理工具 (宿主機執行)
```bash
# PostgreSQL 容器管理
docker compose -f docker-compose.agvc.yml up -d postgres    # 啟動資料庫
docker compose -f docker-compose.agvc.yml stop postgres     # 停止資料庫
docker compose -f docker-compose.agvc.yml logs postgres     # 查看資料庫日誌

# 資料庫連接測試
scripts/network-tools/port-check.sh --port 5432 --host localhost  # 資料庫端口檢查
quick_agvc "start_db"                # 檢查資料庫連接狀態
```

#### 開發工作流工具 (宿主機執行)
```bash
# 載入開發工具集
source scripts/dev-tools/dev-tools.sh

# Web API 工作空間開發
dev_build --workspace web_api_ws     # 建置 Web API 工作空間
dev_test --workspace web_api_ws      # 測試 Web API 工作空間
dev_check --workspace web_api_ws --severity warning  # 代碼品質檢查

# 完整開發流程
scripts/dev-tools/build-helper.sh fast --workspace web_api_ws    # 快速建置
scripts/dev-tools/test-runner.sh unit --workspace web_api_ws     # 單元測試
scripts/dev-tools/code-analyzer.sh style --workspace web_api_ws  # 代碼風格檢查
```

### 容器內操作 (ROS 2 和 Web 開發)

#### 環境設定 (AGVC容器內)
```bash
source /app/setup.bash
agvc_source  # 載入AGVC工作空間 (或使用 all_source 自動檢測)
cd /app/web_api_ws
```

#### 服務啟動 (容器內執行)
```bash
# 【方法1: 透過宿主機工具】(推薦)
# 在宿主機執行：
source scripts/docker-tools/docker-tools.sh
agvc_enter  # 自動進入 AGVC 容器並載入環境

# 或使用快速執行
quick_agvc "python3 src/web_api/web_api/api_server.py"      # API Gateway
quick_agvc "python3 src/agvcui/agvcui/agvc_ui_server.py"   # AGVCUI 系統
quick_agvc "python3 src/opui/opui/opui_server.py"          # OPUI 界面

# 【方法2: 手動進入容器】
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

#### 構建與測試

##### 宿主機建置和測試 (推薦)
```bash
# 使用開發工具進行建置
source scripts/dev-tools/dev-tools.sh
dev_build --workspace web_api_ws    # 建置 Web API 工作空間
dev_test --workspace web_api_ws     # 測試 Web API 工作空間

# 或直接使用工具腳本
scripts/dev-tools/build-helper.sh fast --workspace web_api_ws
scripts/dev-tools/test-runner.sh unit --workspace web_api_ws

# Web 服務狀態檢查
curl http://localhost:8000/health    # web_api健康檢查
curl http://localhost:8001/         # agvcui界面
curl http://localhost:8002/         # opui界面
```

##### 容器內建置 (setup.bash驗證可用)
```bash
# 【方法1: 透過宿主機工具】(推薦)
quick_agvc "build_ws web_api_ws"     # 在 AGVC 容器內建置

# 【方法2: 手動進入容器】
agvc_enter  # 進入容器
build_ws web_api_ws                  # 使用setup.bash中的函數
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

## 🔍 測試與調試

### 宿主機測試工具 (推薦)

#### API 和服務測試
```bash
# Web 服務健康檢查
curl http://localhost:8000/health    # API Gateway
curl http://localhost:8001/          # AGVCUI 界面
curl http://localhost:8002/          # OPUI 界面

# FastAPI 自動文檔
curl http://localhost:8000/docs      # API 文檔界面

# AGVC 系統狀態檢查
source scripts/docker-tools/docker-tools.sh
agvc_health                          # AGVC 系統健康檢查
agvc_services                        # 所有服務狀態

# 日誌分析和調試
scripts/log-tools/log-analyzer.sh agvc --stats       # AGVC 日誌統計
scripts/log-tools/log-analyzer.sh agvc --timeline    # 錯誤時間軸
scripts/log-tools/log-analyzer.sh agvc --suggestions # 解決建議
```

#### 網路和端口診斷
```bash
# 端口連接檢查
scripts/network-tools/port-check.sh system           # 系統端口檢查
scripts/network-tools/port-check.sh --port 8000-8002 # Web 服務端口

# 服務性能測試
scripts/network-tools/connectivity-test.sh performance --target localhost
```

### 容器內調試工具

#### API 測試 (容器內)
```bash
# 【方法1: 透過宿主機工具】(推薦)
quick_agvc "curl http://localhost:8000/health"       # API 健康檢查
quick_agvc "python -c \"import socketio; sio = socketio.Client(); sio.connect('http://localhost:8000')\""

# 【方法2: 手動進入容器】
agvc_enter  # 進入容器
curl http://localhost:8000/docs      # FastAPI自動文檔
# Socket.IO測試
python -c "import socketio; sio = socketio.Client(); sio.connect('http://localhost:8000')"
```

### 前端調試
- **瀏覽器開發工具**: 檢查Network與Console
- **Vue Devtools**: 組件狀態調試  
- **Socket.IO調試**: 查看事件流

## 部署注意事項

### 容器內服務
- 所有服務運行在AGVC容器內
- 通過nginx反向代理對外服務
- 支援SSL終端與負載均衡

### 安全考量
- CORS政策正確配置
- API認證與授權機制
- 敏感資料環境變數管理

## 🛠️ 故障排除

### 系統診斷工作流程

#### 第一步：快速系統檢查 (宿主機執行)
```bash
# 完整系統健康檢查
scripts/system-tools/health-check.sh --quick

# AGVC 系統狀態檢查
source scripts/docker-tools/docker-tools.sh
agvc_health                          # AGVC 系統健康檢查
agvc_services                        # 所有服務狀態檢查
```

#### 第二步：專項診斷 (宿主機執行)
```bash
# Web 服務診斷
scripts/network-tools/port-check.sh system          # 端口檢查
curl http://localhost:8000/health                   # API 服務檢查
curl http://localhost:8001/ > /dev/null && echo "AGVCUI OK" || echo "AGVCUI Failed"

# AGVC 日誌分析
scripts/log-tools/log-analyzer.sh agvc --stats      # 日誌統計分析
scripts/log-tools/log-analyzer.sh agvc --timeline   # 錯誤時間軸
scripts/log-tools/log-analyzer.sh agvc --suggestions # 解決建議

# 資料庫診斷
docker compose -f docker-compose.agvc.yml ps postgres  # 資料庫容器狀態
scripts/network-tools/port-check.sh --port 5432 --host localhost
```

### 常見問題及解決方案

#### 1. **AGVC 容器無法啟動**
```bash
# 宿主機診斷步驟
agvc_status                          # 查看容器狀態
agvc_logs                           # 查看啟動日誌
scripts/docker-tools/container-status.sh agvc  # 詳細診斷報告
```

#### 2. **端口衝突 (8000-8002)**
```bash
# 檢查端口佔用
scripts/network-tools/port-check.sh --port 8000-8002 --verbose
netstat -tlnp | grep -E "800[0-2]"  # 查看端口佔用進程

# 解決方案
agvc_stop && agvc_start              # 重啟 AGVC 系統
```

#### 3. **資料庫連接失敗**
```bash
# 資料庫狀態檢查
docker compose -f docker-compose.agvc.yml ps postgres
quick_agvc "start_db"                # 檢查資料庫連接

# 資料庫重啟
docker compose -f docker-compose.agvc.yml restart postgres
```

#### 4. **Socket.IO 斷線**
```bash
# 網路連接檢查
scripts/network-tools/connectivity-test.sh performance --target localhost
quick_agvc "netstat -tlnp | grep 8000"  # 檢查 Socket.IO 服務

# 防火牆檢查
sudo ufw status                      # 檢查防火牆狀態
```

#### 5. **前端資源載入失敗**
```bash
# Nginx 配置檢查
docker compose -f docker-compose.agvc.yml ps nginx
docker compose -f docker-compose.agvc.yml logs nginx

# Nginx 重啟
docker compose -f docker-compose.agvc.yml restart nginx
```

### 日誌位置和分析
```bash
# 宿主機日誌分析 (推薦)
scripts/log-tools/log-analyzer.sh agvc --stats      # 統計分析
scripts/log-tools/log-analyzer.sh agvc --severity 3  # 嚴重錯誤

# 容器日誌位置
# - API日誌：容器內stdout (透過 agvc_logs 查看)
# - Nginx日誌：/var/log/nginx/ (透過 docker logs 查看)
# - 瀏覽器日誌：開發工具Console
```

## 💡 重要提醒

### 開發環境使用原則
- **🖥️ 宿主機**: 使用 `scripts/` 工具進行容器管理、服務診斷、API 測試
- **🐳 容器內**: 執行 Web 服務、ROS 2 相關指令、資料庫操作
- **📡 推薦方式**: 使用 `agvc_enter` 進入容器，使用 `quick_agvc` 執行容器內指令

### Web 開發最佳實踐
- **API變更**: 需同步更新前端界面和文檔
- **Socket.IO事件**: 確保向後兼容性和完整測試
- **資料庫變更**: 透過 db_proxy 進行，避免直接操作
- **服務部署**: 所有 Web 服務必須在 AGVC 容器內運行

### 故障排除最佳實踐
1. **優先使用宿主機工具**: 快速診斷和服務檢查
2. **多層次檢查**: 容器→服務→端口→網路→資料庫
3. **日誌分析為主**: 使用 `scripts/log-tools/` 進行智能分析
4. **服務隔離**: 分別檢查 API Gateway、AGVCUI、OPUI 服務