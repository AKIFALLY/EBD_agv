# web_api - 核心Web API服務

## 專案概述
web_api是RosAGV系統的核心Web API服務，提供RESTful API接口整合各系統模組。基於FastAPI框架，統一管理PLC控制、門控制、交通管制、地圖匯入、KUKA Fleet整合等功能，為前端界面和外部系統提供標準化的HTTP API。

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

## 開發指令

### 基本操作
```bash
# 進入AGVC容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash
source /app/setup.bash && all_source

# 構建web_api_ws
build_ws web_api_ws
# 或單獨構建
colcon build --packages-select web_api

# 啟動API伺服器
cd /app/web_api_ws/src/web_api
python3 web_api/api_server.py
```

### 開發模式啟動
```bash
# 使用uvicorn開發模式
uvicorn web_api.api_server:app --host 0.0.0.0 --port 8000 --reload

# 檢查API文檔
curl http://localhost:8000/docs
curl http://localhost:8000/redoc
```

### 測試指令
```bash
# 執行API測試
cd /app/web_api_ws/src/web_api
python3 -m pytest tests/ -v

# 執行特定測試
python3 tests/test_kuka_api.py
python3 tests/quick_test.py
python3 tests/test_parameters_update.py

# 檢查API健康狀態
curl http://localhost:8000/health
```

## 配置設定

### API伺服器配置
```python
# api_server.py 配置
HOST = "0.0.0.0"
PORT = 8000
LOG_LEVEL = "debug"

# 資料庫連接
DATABASE_URL = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
```

### 路由器配置
- PLC客戶端配置: `PlcClientNode('plc_client', 'agvc')`
- 門控制配置: `/app/config/door_config.yaml`
- 交通控制: ConnectionPoolManager整合
- KUKA整合: 資料庫池管理

### 環境變數
```bash
export API_HOST="0.0.0.0"              # API伺服器主機
export API_PORT="8000"                 # API伺服器端口
export DATABASE_URL="postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
export LOG_LEVEL="debug"               # 日誌級別
```

## 整合點

### 與其他專案整合
- **plc_proxy_ws**: 透過PlcClientNode進行PLC控制
- **ecs_ws**: 使用DoorLogic進行門控制
- **db_proxy_ws**: 透過ConnectionPoolManager存取資料庫
- **traffic_manager**: 交通管制區域管理
- **kuka_fleet_ws**: KUKA Fleet系統整合
- **agvcui**: 提供API給管理界面
- **opui**: 提供API給操作界面

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

# KUKA Fleet API
GET  /kuka/robots                   # 機器人列表
POST /kuka/dispatch_task            # 派遣任務
GET  /kuka/task_status              # 任務狀態查詢
```

## 測試方法

### API功能測試
```bash
# 健康檢查
curl -X GET http://localhost:8000/health

# PLC API測試
curl -X GET http://localhost:8000/plc/status
curl -X POST http://localhost:8000/plc/read_data \
  -H "Content-Type: application/json" \
  -d '{"address": "DM100", "length": 10}'

# 門控制API測試
curl -X GET http://localhost:8000/door/status
curl -X POST http://localhost:8000/door/open \
  -H "Content-Type: application/json" \
  -d '{"door_id": "door_01"}'

# 交通管制API測試
curl -X GET http://localhost:8000/traffic/areas
curl -X POST http://localhost:8000/traffic/acquire \
  -H "Content-Type: application/json" \
  -d '{"area_id": "area_01", "agv_id": "agv01"}'
```

### 整合測試
```bash
# 執行完整API測試套件
python3 -m pytest tests/ -v --tb=short

# KUKA API整合測試
python3 tests/test_kuka_api.py

# 參數更新測試
python3 tests/test_parameters_update.py

# 快速功能驗證
python3 tests/quick_test.py
```

### 負載測試
```bash
# 使用Apache Bench進行負載測試
ab -n 1000 -c 10 http://localhost:8000/health

# 使用curl進行併發測試
for i in {1..10}; do
  curl -X GET http://localhost:8000/plc/status &
done
wait
```

## 故障排除

### 常見問題

#### API伺服器啟動失敗
```bash
# 檢查端口佔用
netstat -tulpn | grep :8000
sudo lsof -i :8000

# 檢查依賴模組
python3 -c "import fastapi, uvicorn; print('Dependencies OK')"

# 檢查配置檔案
cat /app/config/door_config.yaml
```

#### PLC連接失敗
```bash
# 檢查PLC客戶端狀態
curl -X GET http://localhost:8000/plc/status

# 重新初始化PLC連接
ros2 service call /plc/reconnect

# 檢查網路連通性
ping <PLC_IP_ADDRESS>
```

#### 資料庫連接問題
```bash
# 檢查資料庫連接
python3 -c "
from db_proxy.connection_pool_manager import ConnectionPoolManager
db = ConnectionPoolManager('postgresql+psycopg2://agvc:password@192.168.100.254/agvc')
print('Database connection OK')"

# 檢查PostgreSQL服務
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c 'SELECT 1;'
```

#### API回應緩慢
```bash
# 檢查API回應時間
curl -w "@curl-format.txt" -X GET http://localhost:8000/health

# 監控API日誌
tail -f /var/log/web_api.log

# 檢查系統資源
htop
df -h
```

### 除錯技巧
- 啟用debug級別日誌觀察詳細錯誤信息
- 使用FastAPI自動生成的`/docs`端點測試API
- 監控uvicorn日誌掌握請求處理狀況
- 使用Postman或curl進行API端點測試
- 檢查各路由器模組的依賴服務狀態

### 效能監控
- API回應時間分析
- 併發請求處理能力
- 資料庫連接池使用率
- PLC通訊延遲監控
- 記憶體和CPU使用情況
- 錯誤率和成功率統計

### API文檔
- 訪問 `http://localhost:8000/docs` 查看Swagger UI
- 訪問 `http://localhost:8000/redoc` 查看ReDoc文檔
- 使用OpenAPI規範進行API測試和整合