# web_api - API Gateway 核心服務

## 📚 Context Loading
../CLAUDE.md  # 引用上層 web_api_ws 工作空間文档

## 📋 套件概述
web_api 是 Web API 工作空間中的 **API Gateway 服務**，專注於外部系統整合。基於 FastAPI 框架，提供標準化的 HTTP API 端點，統一管理 PLC 控制、門控制、交通管制、地圖匯入、KUKA Fleet 整合等核心功能。

**🎯 定位**: Port 8000 的 API Gateway，負責外部系統整合中心

## 核心模組

### 主要類別
- **ApiServer** (`api_server.py`): 主要API伺服器類，整合所有路由器
- **路由器模組**: 各功能域的API端點實現

### API路由架構
```
routers/
├── plc.py              # PLC控制API
├── door.py             # 門控制API  
├── traffic.py          # 交通管制API
├── map_importer.py     # 地圖匯入API
└── kuka.py             # KUKA Fleet整合API
```

## 關鍵檔案

### 核心檔案
- `/web_api/api_server.py` - 主要API伺服器，整合所有功能模組
- `/web_api/routers/__init__.py` - 路由器模組初始化
- `/docs/README.md` - API文檔說明

### API路由檔案
- `/web_api/routers/plc.py` - PLC設備控制API端點
- `/web_api/routers/door.py` - 門控制系統API端點  
- `/web_api/routers/traffic.py` - 交通管制區域API端點
- `/web_api/routers/map_importer.py` - 地圖數據匯入API端點
- `/web_api/routers/kuka.py` - KUKA Fleet系統整合API端點

### 測試檔案
- `/tests/README.md` - 測試說明文檔
- `/tests/test_kuka_api.py` - KUKA API測試
- `/tests/test_parameters_update.py` - 參數更新測試
- `/tests/quick_test.py` - 快速測試腳本

## 🚀 套件特定啟動

### API Gateway 服務啟動
```bash
# 【推薦方式】透過上層工作空間工具
# 參考: ../CLAUDE.md 開發環境設定

# 【直接啟動】API Gateway 服務
cd /app/web_api_ws/src/web_api
python3 web_api/api_server.py

# 開發模式 (自動重載)
uvicorn web_api.api_server:app --host 0.0.0.0 --port 8000 --reload
```

### 套件特定測試
```bash
# API Gateway 專項測試
python3 -m pytest tests/ -v
python3 tests/test_kuka_api.py       # KUKA 整合測試
python3 tests/quick_test.py          # 快速功能驗證
python3 tests/test_parameters_update.py  # 參數更新測試

# API 健康檢查
curl http://localhost:8000/health
curl http://localhost:8000/docs      # Swagger UI
```

## 📊 API Gateway 特定配置

### 服務器配置 (api_server.py)
```python
HOST = "0.0.0.0"      # API Gateway 監聽地址
PORT = 8000           # API Gateway 端口 (外部系統整合)
LOG_LEVEL = "debug"   # 詳細日誌用於外部系統調試
```

### 路由器整合配置
- **PLC 整合**: `PlcClientNode('plc_client', 'agvc')`
- **門控整合**: `/app/config/door_config.yaml`
- **交通管制**: ConnectionPoolManager 資料庫池整合
- **KUKA Fleet**: 資料庫池管理，支援任務狀態回調接收

## 🔗 外部系統整合點

### API Gateway 特有整合
- **plc_proxy_ws**: 透過 PlcClientNode 進行 PLC 設備控制
- **ecs_ws**: 使用 DoorLogic 進行門控制系統整合
- **db_proxy_ws**: 透過 ConnectionPoolManager 存取資料庫
- **traffic_manager**: 交通管制區域管理 (KUKA AGV 交管)
- **kuka_fleet_ws**: KUKA Fleet 系統整合和任務狀態回調

### KUKA Fleet 整合詳細說明
web_api 透過 `/interfaces/api/amr/missionStateCallback` 端點接收 KUKA Fleet Manager 的任務狀態回調：

**🔧 回調實作**: `routers/kuka.py`
- **端點**: `POST /interfaces/api/amr/missionStateCallback`
- **功能**: 接收 KUKA Fleet 任務狀態更新並儲存至資料庫
- **狀態類型**: 支援 12 種任務狀態 (MOVE_BEGIN, ARRIVED, COMPLETED 等)
- **資料模型**: 使用 `MissionStateCallbackData` Pydantic 模型驗證

**📋 API 規格參考**:
- **完整 API 規格**: @docs-ai/knowledge/protocols/kuka-fleet-api.md
- **回調處理規格**: @docs-ai/knowledge/protocols/kuka-fleet-callback.md

### API端點規範
```bash
# PLC控制API
GET  /plc/status                    # PLC狀態查詢
POST /plc/read_data                 # 讀取PLC數據
POST /plc/write_data                # 寫入PLC數據

# 門控制API
GET  /door/status                   # 門狀態查詢
POST /door/open                     # 開門指令
POST /door/close                    # 關門指令

# 交通管制API
GET  /traffic/areas                 # 交通區域列表
POST /traffic/acquire               # 申請交通區域
POST /traffic/release               # 釋放交通區域

# 地圖匯入API
POST /map/import                    # 匯入地圖數據
GET  /map/status                    # 地圖狀態查詢

# KUKA Fleet API (詳細規格參考 @docs-ai/knowledge/protocols/kuka-fleet-api.md)
POST /interfaces/api/amr/missionStateCallback  # 任務狀態回調接收 (實際實作)
```

## 🧪 API Gateway 專項測試

### 外部系統整合測試
```bash
# 核心 API 端點測試
curl -X GET http://localhost:8000/health
curl -X GET http://localhost:8000/docs  # API 文檔

# PLC 整合測試
curl -X POST http://localhost:8000/plc/read_data \
  -H "Content-Type: application/json" \
  -d '{"address": "DM100", "length": 10}'

# KUKA Fleet 回調測試
curl -X POST http://localhost:8000/interfaces/api/amr/missionStateCallback \
  -H "Content-Type: application/json" \
  -d '{"missionId": "test001", "state": "COMPLETED"}'

# 交通管制整合測試
curl -X POST http://localhost:8000/traffic/acquire \
  -H "Content-Type: application/json" \
  -d '{"area_id": "area_01", "agv_id": "kuka01"}'
```

## 🚨 API Gateway 專項故障排除

**⚠️ 通用故障排除請參考**: ../CLAUDE.md 故障排除章節

### 套件特有問題

#### 外部系統整合失敗
```bash
# KUKA Fleet 回調接收問題
curl -X GET http://localhost:8000/health
# 檢查 `/interfaces/api/amr/missionStateCallback` 端點

# PLC 通訊異常
ros2 service call /plc/reconnect
curl -X GET http://localhost:8000/plc/status

# 交通管制整合問題
curl -X GET http://localhost:8000/traffic/areas
```

#### API Gateway 特有配置
```bash
# 檢查 Port 8000 專用配置
netstat -tulpn | grep :8000

# 檢查路由器模組載入
python3 -c "from web_api.routers import kuka, plc, door, traffic, map_importer"
```

### API 文檔和除錯
- **Swagger UI**: http://localhost:8000/docs (互動式 API 測試)
- **ReDoc**: http://localhost:8000/redoc (API 文檔)
- **OpenAPI 規範**: 自動生成，支援外部系統整合測試