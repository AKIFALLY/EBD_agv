# 技術棧詳細說明

## 🎯 RosAGV 技術架構概覽

RosAGV 採用現代化的技術棧，結合 ROS 2、容器化、微服務等先進技術，構建穩定可靠的企業級 AGV 控制系統。

## 🏗️ 技術架構分層

### 應用層
```
應用層組件
├── AGVCUI - 管理員界面 (Port 8001)
├── OPUI - 操作員界面 (Port 8002)
├── AGV 車型特定邏輯
│   ├── Cargo Mover AGV
│   ├── Loader AGV
│   └── Unloader AGV
└── 業務流程邏輯
```

### 服務層
```
服務層架構
├── Web API (FastAPI + Socket.IO)
├── ROS 2 Services
│   ├── AGV 控制服務
│   ├── PLC 通訊服務
│   └── 路徑規劃服務
└── 外部系統整合
    ├── KUKA Fleet Adapter
    └── WMS 整合介面
```

### 通訊層
```
通訊技術棧
├── Zenoh RMW - 高效能 ROS 2 通訊
├── PLC 協定 - Keyence 工業通訊
├── HTTP/WebSocket - Web 服務通訊
└── Socket.IO - 即時雙向通訊
```

### 資料層
```
資料管理
├── PostgreSQL 16 - 主要資料庫
├── Redis - 快取和會話管理
├── 配置檔案 (YAML/JSON5)
└── 日誌系統 (結構化日誌)
```

### 基礎層
```
基礎設施
├── Docker 容器化
├── Ubuntu 24.04 LTS
├── 硬體設備整合
└── 網路基礎設施
```

## 🤖 ROS 2 生態系統

### ROS 2 Jazzy
- **版本**: Jazzy Jalapa (最新 LTS)
- **選擇原因**: 長期支援、穩定性、現代化設計
- **核心特性**: 
  - 改進的 DDS 實作
  - 更好的即時性能
  - 增強的安全性
  - 完善的工具鏈

### Zenoh RMW
```python
# Zenoh RMW 配置
export RMW_IMPLEMENTATION=rmw_zenohd

# 高效能通訊特性
- 本地通訊: < 100μs
- 跨網路通訊: < 10ms  
- 自動服務發現
- 網路透明性
```

### Python 3.12
- **語言特性**: 最新 Python 特性支援
- **效能提升**: 更好的執行效能
- **生態系統**: 豐富的第三方套件
- **開發體驗**: 優秀的開發工具支援

## 🐳 容器化技術

### Docker Compose V2
```yaml
# 雙環境容器配置
AGV 車載環境:
  - 容器: rosagv (單容器)
  - 網路: host (直接硬體存取)
  - 工作空間: 11個 (6專用 + 4共用基礎 + 2共用應用 - db_proxy_ws, launch_ws)
  - 服務: AGV 控制、PLC 通訊、感測器處理、搖桿控制、AGVUI

AGVC 管理環境:
  - 容器: agvc_server, postgres, nginx, pgadmin (4個容器)
  - 網路: bridge (192.168.100.0/24)
  - 工作空間: 13個 (7專用 + 4共用基礎 + 2共用應用 - agv_ws監控, launch_ws)
  - 服務: Web API、資料庫、反向代理、KUKA WCS、KUKA Fleet
```

### Ubuntu 24.04 LTS
- **基礎映像**: ubuntu:24.04
- **系統特性**: 
  - 最新 LTS 版本
  - ROS 2 Jazzy 官方支援
  - 長期安全更新
  - 現代化系統工具

## 🌐 Web 技術棧

### FastAPI
```python
# 高效能 Web 框架
from fastapi import FastAPI
from fastapi.middleware.socketio import SocketIOMiddleware

app = FastAPI(
    title="RosAGV API",
    version="1.0.0",
    docs_url="/api/docs"
)

# 特性優勢
- 高效能 (基於 Starlette)
- 自動 API 文檔
- 型別檢查
- 異步支援
```

### Socket.IO
- **即時通訊**: 雙向即時資料交換
- **自動重連**: 網路中斷自動恢復
- **房間管理**: 支援多用戶管理
- **跨瀏覽器**: 完整的瀏覽器相容性

### PostgreSQL 16
```sql
-- 企業級資料庫特性
CREATE DATABASE agvc;
CREATE USER agvc WITH PASSWORD 'password';

-- 核心功能
- ACID 事務支援
- 複雜查詢優化
- 並發控制
- 資料完整性保證
- 內建 pgAdmin4 管理工具 (Port 5050)
```

## 🏭 工業通訊

### Keyence PLC 協定
```python
# PLC 通訊範例
class KeyencePlcCommunication:
    def __init__(self, ip="192.168.2.101", port=8501):
        self.ip = ip
        self.port = port
    
    def read_device(self, device_type, address):
        # 讀取 PLC 設備資料
        command = f"RD {device_type}{address}\r\n"
        return self.send_command(command)
```

### KUKA Fleet 整合
- **API 版本**: KUKA Fleet Manager v2.x
- **通訊協定**: REST API + WebSocket
- **整合功能**: 任務同步、狀態監控、協調控制

## 📊 效能特性

### 系統效能指標
```
效能基準
├── ROS 2 通訊延遲: < 1ms (本地)
├── Web API 響應時間: < 100ms
├── 資料庫查詢: < 50ms (平均)
├── PLC 通訊延遲: < 10ms
└── 系統吞吐量: > 1000 req/s
```

### 資源使用
- **記憶體**: 每容器 2-4GB
- **CPU**: 多核心並行處理
- **網路**: Gigabit 乙太網路
- **儲存**: SSD 推薦 (NVMe 更佳)

## 📦 工作空間架構

### AGV 車載工作空間 (11個含共用)

#### 專用工作空間 (6個)
| 工作空間 | 功能描述 |
|---------|---------|
| `agv_ws` | 核心 AGV 控制和狀態機 |
| `agv_cmd_service_ws` | 手動指令服務介面 |
| `joystick_ws` | 搖桿控制整合 |
| `sensorpart_ws` | 感測器資料處理 (3D相機、OCR) |
| `uno_gpio_ws` | GPIO 控制服務 |
| `web_api_ws` | AGVUI 車載監控介面 |

#### 共用基礎設施 (4個)
| 工作空間 | 功能描述 |
|---------|---------|
| `shared_constants_ws` | 系統級常數定義 (跨環境共用) |
| `keyence_plc_ws` | Keyence PLC 通訊協議 |
| `plc_proxy_ws` | PLC 代理服務 |
| `path_algorithm` | 路徑規劃演算法 |

#### 共用應用 (2個)
| 工作空間 | 功能描述 |
|---------|---------|
| `db_proxy_ws` | 本地資料存取 |
| `launch_ws` | ROS 2 啟動編排 |

### AGVC 管理工作空間 (13個含共用)

#### 專用工作空間 (7個)
| 工作空間 | 功能描述 |
|---------|---------|
| `web_api_ws` | Web API 和 Socket.IO 服務 |
| `db_proxy_ws` | 資料庫代理和 ORM |
| `ecs_ws` | 設備控制系統 (門控) |
| `rcs_ws` | 機器人控制系統 |
| `kuka_wcs_ws` | KUKA WCS 系統 (當前 WCS 實作) |
| `wcs_ws` | WCS 工作空間 (流程控制邏輯) |
| `kuka_fleet_ws` | KUKA Fleet 系統整合 |

#### 共用基礎設施 (4個)
| 工作空間 | 功能描述 |
|---------|---------|
| `shared_constants_ws` | 系統級常數定義 (跨環境共用) |
| `keyence_plc_ws` | Keyence PLC 通訊 (共用) |
| `plc_proxy_ws` | PLC 代理服務 (共用) |
| `path_algorithm` | 路徑規劃演算法 (共用) |

#### 共用應用 (2個)
| 工作空間 | 功能描述 |
|---------|---------|
| `agv_ws` | AGV 介面定義 (AGVC 監控需要) |
| `launch_ws` | ROS 2 Launch 系統管理 |

#### 已棄用 (2個)
| 工作空間 | 狀態 |
|---------|------|
| ~~`tafl_ws`~~ | ⚠️ 已棄用 - TAFL 語言核心實作 |
| ~~`tafl_wcs_ws`~~ | ⚠️ 已棄用 - TAFL WCS (已被 kuka_wcs_ws 取代) |

## 🔧 開發工具鏈

### 程式碼品質
```bash
# 靜態分析工具
pylint --rcfile=.pylintrc src/
mypy src/
black src/
isort src/

# 測試框架
pytest tests/
pytest --cov=src tests/
```

### 結構化資料處理
```bash
# JSON 處理
jq '.key' file.json

# YAML 處理  
yq '.services.agvc_server.ports' docker-compose.agvc.yml

# JSON5 處理 (Zenoh 配置)
json5 /app/routerconfig.json5 | jq '.mode'
json5 --validate /app/routerconfig.json5
```

### 監控和日誌
```python
# 結構化日誌
import logging
import json

logger = logging.getLogger(__name__)
logger.info(json.dumps({
    "event": "agv_task_completed",
    "agv_id": "cargo_01",
    "task_id": "T123456",
    "duration": 45.2
}))
```

## 🚀 部署架構

### 生產環境
```yaml
# 生產部署配置
production:
  agv_nodes: 3  # 3台 AGV
  agvc_replicas: 2  # AGVC 高可用部署
  database: 
    - primary: postgres-primary
    - replica: postgres-replica
  monitoring:
    - prometheus
    - grafana
    - alertmanager
```

### 開發環境
- **本地開發**: Docker Compose 一鍵啟動
- **持續整合**: GitHub Actions 自動化測試
- **程式碼審查**: Pull Request 工作流程

## 💡 技術決策記錄

### 為什麼選擇 ROS 2 Jazzy？
- **長期支援**: LTS 版本保證
- **現代化**: 基於現代軟體工程實踐
- **生態系統**: 豐富的機器人算法庫
- **工業級**: 符合工業應用需求

### 為什麼選擇 Zenoh RMW？
- **效能**: 比傳統 DDS 更高效
- **網路適應**: 更好的跨網路能力
- **簡化**: 減少 DDS 配置複雜性
- **未來**: ROS 2 的發展方向

### 為什麼選擇 FastAPI？
- **效能**: 接近 Node.js 的效能
- **開發體驗**: 自動文檔和型別檢查
- **現代**: 基於 Python 3.6+ 特性
- **生態**: 豐富的插件和中間件

## 🔗 相關文檔
- [雙環境架構](dual-environment.md) - 架構設計詳解
- [ROS 2 技術整合](../technical-details/ros2-integration.md) - ROS 2 實作細節
- [部署指導](../operations/deployment.md) - 完整部署操作