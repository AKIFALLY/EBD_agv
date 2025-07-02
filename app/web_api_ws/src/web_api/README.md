# Web API 專案

本專案提供 Web API 服務，包含多個 API 端點用於與不同系統整合。

## 專案結構

```
web_api/
├── README.md                    # 本文件
├── docs/                        # 文檔目錄
│   ├── README.md               # 文檔索引
│   └── kuka_api.md            # Kuka API 說明文件
├── tests/                       # 測試工具目錄
│   ├── README.md               # 測試說明
│   ├── test_kuka_api.py       # Kuka API 測試腳本
│   └── create_test_task.py    # 測試任務創建工具
├── web_api/                     # 主要程式碼
│   ├── __init__.py
│   ├── api_server.py          # API 伺服器主程式
│   └── routers/               # API 路由器
│       ├── __init__.py
│       ├── door.py            # 門控制 API
│       ├── kuka.py            # Kuka 系統 API
│       ├── map_importer.py    # 地圖匯入 API
│       ├── plc.py             # PLC API
│       └── traffic.py         # 交通管制 API
├── package.xml                  # ROS2 套件配置
├── setup.py                     # Python 套件設定
└── setup.cfg                    # 套件配置
```

## API 端點

### 已實現的 API

1. **Kuka API** (`/interfaces/api/amr/`)
   - `POST /missionStateCallback` - 接收 Kuka 系統任務狀態回報
   - 詳細說明：[docs/kuka_api.md](docs/kuka_api.md)

2. **PLC API** (`/plc/`)
   - `GET /get_data/{device_type}/{key}` - 讀取 PLC 資料

3. **交通管制 API** (`/traffic/`)
   - `POST /acquire` - 取得交管區使用權
   - `POST /release` - 釋放交管區使用權
   - `POST /acquire_by_name` - 依名稱取得交管區使用權
   - `POST /release_by_name` - 依名稱釋放交管區使用權

4. **門控制 API** (`/door/`)
   - `POST /control` - 門控制指令
   - `POST /state` - 查詢門狀態

5. **地圖匯入 API** (`/map_importer/`)
   - `POST /upload-kuka-map/` - 上傳 Kuka 地圖
   - `POST /upload-ct-map/` - 上傳 CT 地圖
   - `DELETE /delete-kuka-map` - 刪除 Kuka 地圖

## 快速開始

### 1. 環境準備

確保已安裝必要的依賴：
- ROS2
- PostgreSQL
- Python 3.8+

### 2. 啟動服務

```bash
# 進入工作空間
cd /app/web_api_ws

# 建置專案
colcon build --symlink-install

# 載入環境
source install/setup.bash

# 啟動 API 伺服器
ros2 run web_api api_server
```

伺服器將在 `http://localhost:8000` 啟動。

### 3. 測試 API

```bash
# 創建測試資料
python tests/create_test_task.py

# 測試 Kuka API
python tests/test_kuka_api.py
```

## 開發指南

### 添加新的 API 端點

1. 在 `web_api/routers/` 目錄下創建新的路由器檔案
2. 實現 API 邏輯
3. 在 `api_server.py` 中註冊新的路由器
4. 添加相應的測試和文檔

### 路由器範例

```python
# web_api/routers/example.py
from fastapi import APIRouter
from pydantic import BaseModel

class ExampleData(BaseModel):
    message: str

def create_example_router():
    router = APIRouter(prefix="/example", tags=["Example"])
    
    @router.post("/test")
    async def test_endpoint(data: ExampleData):
        return {"received": data.message}
    
    return router
```

### 註冊路由器

```python
# api_server.py
from web_api.routers.example import create_example_router

# 在 ApiServer.__init__ 中添加
self.app.include_router(create_example_router())
```

## 配置

### 資料庫連接

預設資料庫連接字串：
```
postgresql+psycopg2://agvc:password@192.168.100.254/agvc
```

### 伺服器設定

- 主機：`0.0.0.0`
- 端口：`8000`
- 日誌級別：`DEBUG`

## 故障排除

### 常見問題

1. **端口被佔用**
   ```bash
   # 檢查端口使用情況
   lsof -i :8000
   
   # 終止佔用端口的程序
   kill -9 <PID>
   ```

2. **資料庫連接失敗**
   - 檢查 PostgreSQL 服務是否運行
   - 確認連接字串正確
   - 檢查網路連接

3. **模組匯入錯誤**
   ```bash
   # 確保已載入 ROS2 環境
   source install/setup.bash
   
   # 檢查 Python 路徑
   echo $PYTHONPATH
   ```

## 相關專案

- [db_proxy](../../db_proxy_ws/) - 資料庫代理服務
- [agvcui](../agvcui/) - Web 使用者介面
- [kuka_fleet_adapter](../../kuka_fleet_ws/) - Kuka 系統適配器

## 授權

本專案採用 MIT 授權條款。
