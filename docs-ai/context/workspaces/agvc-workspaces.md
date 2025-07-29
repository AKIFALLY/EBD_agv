# AGVC 管理工作空間概覽

## 🎯 適用場景
- 理解 AGVC 管理系統的工作空間結構和職責分工
- 為 AGVC 相關開發提供架構指導
- 解決跨工作空間的依賴和整合問題

## 📋 AGVC 工作空間架構

### 工作空間總覽 (11個)
AGVC 管理系統包含 11 個專用工作空間，每個工作空間負責特定的管理功能，形成完整的車隊管理和控制系統。

```
AGVC 管理系統工作空間
├── web_api_ws/                # Web API 和 Socket.IO
├── db_proxy_ws/               # 資料庫代理服務
├── ecs_ws/                    # 設備控制系統
├── rcs_ws/                    # 機器人控制系統
# (wcs_ws 已整合至 ai_wcs_ws)
├── kuka_fleet_ws/             # KUKA Fleet 整合
├── ai_wcs_ws/                 # AI 倉庫控制系統
├── keyence_plc_ws/            # Keyence PLC 通訊 (共用)
├── plc_proxy_ws/              # PLC 代理服務 (共用)
├── path_algorithm/            # 路徑規劃演算法 (共用)
└── [1個預留工作空間]          # 未來擴展使用
```

## 🌐 Web 服務工作空間

### web_api_ws/ - Web API 和 Socket.IO
**職責**: 提供 Web API 服務、即時通訊、用戶介面

#### 套件結構
```
web_api_ws/src/
├── web_api/                   # 核心 API 服務
│   ├── main.py               # FastAPI 應用入口
│   ├── routers/              # API 路由模組
│   ├── models/               # 資料模型
│   ├── services/             # 業務邏輯服務
│   └── websocket/            # Socket.IO 處理
├── agvcui/                   # AGVC 管理員界面
│   ├── static/               # 靜態資源
│   ├── templates/            # HTML 模板
│   └── js/                   # JavaScript 邏輯
├── opui/                     # 操作員界面
│   ├── static/               # 靜態資源
│   ├── templates/            # HTML 模板
│   └── js/                   # JavaScript 邏輯
└── agvui/                    # AGV 狀態顯示界面
    ├── static/               # 靜態資源
    └── templates/            # HTML 模板
```

#### 服務端口配置
- **Port 8000**: 核心 API 服務 (FastAPI + Socket.IO)
- **Port 8001**: AGVCUI 管理員界面
- **Port 8002**: OPUI 操作員界面
- **Port 8003**: AGVUI 狀態顯示界面

#### 核心功能
- RESTful API 服務
- Socket.IO 即時通訊
- 用戶認證和授權
- 資料驗證和序列化
- 跨域資源共享 (CORS)

## 🗄️ 資料管理工作空間

### db_proxy_ws/ - 資料庫代理服務
**職責**: 資料庫操作封裝、ORM 管理、資料一致性

#### 套件結構
```
db_proxy_ws/src/
├── db_proxy/                 # 資料庫代理核心
│   ├── models/               # SQLModel 資料模型
│   ├── crud/                 # CRUD 操作封裝
│   ├── schemas/              # Pydantic 模式
│   ├── database.py           # 資料庫連接管理
│   └── migrations/           # Alembic 遷移腳本
└── db_interfaces/            # 資料庫介面定義
    ├── msg/                  # 資料訊息定義
    └── srv/                  # 資料服務定義
```

#### 資料模型
- **AGV 管理**: AGV 基本資訊、狀態、配置
- **任務管理**: 任務定義、執行狀態、歷史記錄
- **用戶管理**: 用戶帳號、權限、操作日誌
- **系統配置**: 系統參數、設備配置、告警設定

#### 技術特性
- SQLModel ORM (基於 SQLAlchemy 2.x)
- 異步資料庫操作
- 連接池管理
- 自動遷移支援

## 🏭 控制系統工作空間

### ecs_ws/ - 設備控制系統 (Equipment Control System)
**職責**: 設備狀態監控、設備控制指令、設備生命週期管理

#### 核心功能
- 設備註冊和發現
- 設備狀態即時監控
- 設備控制指令下發
- 設備故障檢測和處理
- 設備維護排程

#### 管理設備類型
- AGV 車輛設備
- 充電樁設備
- 感測器設備
- PLC 控制器
- 網路設備

### rcs_ws/ - 機器人控制系統 (Robot Control System)
**職責**: CT 和 KUKA 車隊控制、任務分派協調、AGV 狀態監控

#### 套件結構
```
rcs_ws/src/
├── rcs/                         # RCS 車隊控制核心
│   ├── rcs_core.py             # RCS 核心節點 - 1秒定時器協調中心
│   ├── ct_manager.py           # CT 車隊管理器
│   ├── kuka_manager.py         # KUKA 車隊管理器
│   ├── kuka_dispatcher.py      # KUKA 任務派發器
│   ├── kuka_dispatcher_v2.py   # KUKA 派發器 V2
│   ├── kuka_robot.py           # KUKA 機器人控制
│   ├── kuka_container.py       # KUKA 容器管理
│   ├── kuka_config_manager.py  # KUKA 配置管理
│   ├── kuka_config_cli.py      # 配置管理 CLI 工具
│   ├── kuka_monitor.py         # KUKA 監控模組
│   ├── task_status_simulator.py # 任務狀態模擬器
│   ├── rack_state_manager.py   # 架台狀態管理
│   ├── wcs_priority_scheduler.py # WCS 優先級調度器
│   └── wcs_task_adapter.py     # WCS 任務適配器
├── rcs_interfaces/             # RCS 介面定義 (CMake 專案)
└── traffic_manager/            # 交通管理模組
    └── traffic_controller.py   # 交通區域控制器
```

#### 核心功能
- **統一車隊協調**: 1秒定時器主迴圈協調 CT 和 KUKA 車隊
- **AGV 狀態監控**: 訂閱 `/agv/state_change` 和 `/agv/status` 主題
- **任務分派引擎**: WCS 任務讀取和車隊任務派發
- **KUKA 車隊整合**: 完整的 KUKA Fleet 管理和配置
- **交通管制**: 交通區域控制和衝突避免

### ⚠️ wcs_ws/ - 已整合至 ai_wcs_ws
**說明**: 原本的 wcs_ws 倉庫控制系統功能已完全整合至 ai_wcs_ws 中，現在由 AI WCS 統一決策引擎提供更強大的智能倉庫控制功能。

**遷移說明**: 
- 原 WCS 功能現已由 `ai_wcs_ws` 的統一決策引擎實現
- 七大業務流程統一調度管理
- Work ID 分類管理系統 (220001, 230001, 100001, 100002等)
- 詳細功能請參考 `app/ai_wcs_ws/CLAUDE.md`

## 🤖 AI 和整合工作空間

### ai_wcs_ws/ - AI 倉庫控制系統
**職責**: AI 決策引擎、智能最佳化、預測分析

#### 套件結構
```
ai_wcs_ws/src/
├── ai_wcs/                   # AI 控制核心
│   ├── decision_engine.py    # 決策引擎
│   ├── optimization.py       # 最佳化演算法
│   ├── prediction.py         # 預測分析
│   └── learning.py           # 機器學習
└── ai_interfaces/            # AI 介面定義
    ├── msg/                  # AI 訊息
    └── srv/                  # AI 服務
```

#### AI 功能
- 任務分配最佳化
- 路徑規劃最佳化
- 負載預測和平衡
- 異常檢測和預警
- 效能分析和改進建議

### kuka_fleet_ws/ - KUKA Fleet 整合
**職責**: 與 KUKA Fleet 系統的整合和通訊

#### 整合功能
- KUKA Fleet API 整合
- 任務同步和狀態同步
- 資料格式轉換
- 錯誤處理和重試
- 連接狀態監控

#### 通訊協定
- HTTP/HTTPS API 呼叫
- JSON 資料格式
- 認證和授權
- 即時狀態同步
- 事件通知機制

## 🔄 工作空間載入和管理

### 環境載入
```bash
# 智能載入 (自動檢測 AGVC 環境)
all_source             # 或別名: sa

# 強制載入 AGVC 工作空間
agvc_source           # 載入所有 AGVC 工作空間

# 檢查載入狀態
echo $ROS_WORKSPACE   # 顯示當前載入的工作空間
```

### 建置管理
```bash
# 建置所有 AGVC 工作空間
build_all             # 智能建置腳本

# 建置特定工作空間
colcon build --packages-select web_api
colcon build --packages-up-to db_proxy

# 測試執行
colcon test --packages-select web_api_ws
```

### 服務啟動
```bash
# 啟動 Web 服務
start_web_api         # 啟動 Web API 服務
start_agvcui          # 啟動管理員界面
start_opui            # 啟動操作員界面

# 啟動控制系統
start_ecs             # 啟動設備控制系統
start_rcs             # 啟動機器人控制系統
start_wcs             # 啟動倉庫控制系統
```

## 🔧 開發指導

### 跨工作空間開發
1. **服務導向**: 每個工作空間提供明確的服務介面
2. **資料一致性**: 透過 db_proxy_ws 確保資料一致性
3. **事件驅動**: 使用 ROS 2 主題進行事件通訊
4. **API 標準化**: 統一的 API 設計和錯誤處理

### Web 開發模式
1. **前後端分離**: API 服務與前端界面分離
2. **即時通訊**: 使用 Socket.IO 進行即時資料更新
3. **狀態管理**: 前端使用統一的狀態管理機制
4. **響應式設計**: 支援多種設備和螢幕尺寸

### 資料庫設計
1. **正規化設計**: 避免資料冗餘和不一致
2. **索引最佳化**: 針對查詢模式設計索引
3. **事務管理**: 確保資料操作的原子性
4. **效能監控**: 監控查詢效能和資料庫負載

## 📊 系統整合

### 服務間通訊
```
通訊模式
├── ROS 2 主題: 事件通知和狀態廣播
├── ROS 2 服務: 同步請求和回應
├── ROS 2 動作: 長時間運行的任務
├── HTTP API: Web 界面和外部系統
└── Socket.IO: 即時資料推送
```

### 資料流向
```
資料流向
Web 界面 → Web API → DB Proxy → PostgreSQL
         ↓
ROS 2 服務 → 控制系統 → AGV 車載系統
         ↓
外部系統 ← KUKA Fleet ← AI 決策引擎
```

### 錯誤處理
1. **分層錯誤處理**: 每層都有適當的錯誤處理機制
2. **錯誤傳播**: 錯誤資訊正確傳播到上層
3. **恢復機制**: 自動恢復和手動恢復選項
4. **日誌記錄**: 完整的錯誤日誌和追蹤

## 📋 最佳實踐

### 程式碼組織
- 每個工作空間專注於單一職責
- 使用清晰的模組化設計
- 實作適當的抽象層
- 保持介面的穩定性

### 效能最佳化
- 資料庫查詢最佳化
- 快取機制實作
- 異步處理長時間任務
- 負載均衡和擴展性設計

### 安全性考量
- API 認證和授權
- 資料驗證和清理
- SQL 注入防護
- 跨站腳本攻擊防護

## 🔗 交叉引用
- AGV 工作空間: @docs-ai/context/workspaces/agv-workspaces.md
- 共用組件: @docs-ai/context/workspaces/shared-components.md
- Web 開發: @docs-ai/operations/development/web-development.md
- 資料庫操作: @docs-ai/operations/development/database-operations.md
- 車隊管理: @docs-ai/knowledge/automation/fleet-coordination.md
