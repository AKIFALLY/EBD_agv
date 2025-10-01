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
├── tafl_ws/                   # TAFL 語言核心實作
├── tafl_wcs_ws/               # TAFL WCS (目前使用的 WCS 系統)
├── kuka_fleet_ws/             # KUKA Fleet 整合
├── keyence_plc_ws/            # Keyence PLC 通訊 (共用)
├── plc_proxy_ws/              # PLC 代理服務 (共用)
├── path_algorithm/            # 路徑規劃演算法 (共用)
└── launch_ws/                 # Launch 編排服務
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

## 🏗️ 基礎設施工作空間 (部分與 AGV 共用)

### shared_constants_ws/ - 共享常數定義 (與 AGV 共用)
**職責**: 系統級常數、配置常數、通用數據結構定義

#### 核心功能
- 系統通用常數定義
- 跨工作空間共享的數據結構
- 配置參數標準化
- 介面定義的基礎常數

#### 跨環境共用特性
- **AGV 和 AGVC 共用**: 確保兩個環境使用相同的常數定義
- **載入優先級**: 最高優先級，所有其他工作空間都依賴此工作空間
- **一致性保證**: 避免不同環境間的常數不一致問題

### keyence_plc_ws/ - Keyence PLC 通訊 (與 AGV 共用)
**職責**: 與 Keyence PLC 設備的直接通訊

#### 通訊協定
- Keyence 專有協定
- TCP/IP 乙太網路連接
- 即時資料交換
- 錯誤處理和重連

#### 資料交換
- **讀取**: 感測器狀態、設備狀態
- **寫入**: 控制指令、配置參數
- **監控**: 連接狀態、通訊品質

### plc_proxy_ws/ - PLC 代理服務 (與 AGV 共用)
**職責**: PLC 通訊的 ROS 2 服務封裝

#### 服務功能
- ROS 2 服務介面
- 資料格式轉換
- 非同步通訊處理
- 錯誤處理和重試

#### 介面設計
- 標準化的 ROS 2 訊息格式
- 服務和動作介面
- 主題發布和訂閱
- 參數配置支援

### path_algorithm/ - 路徑規劃演算法 (與 AGV 共用)
**職責**: 路徑規劃、導航演算法實作

#### 演算法支援
- **A* 演算法**: 最短路徑搜尋
- **RRT**: 快速隨機樹規劃
- **動態視窗法**: 即時避障
- **純追蹤**: 路徑跟隨控制

#### AGVC 使用場景
- 車隊路徑規劃協調
- 全局路徑最佳化
- 多 AGV 路徑衝突避免
- 任務路線規劃

### agv_ws/ - AGV 核心控制 (AGVC 監控需要)
**職責**: AGVC 需要監控 AGV 狀態和事件，引入 AGV 核心介面

#### AGVC 使用原因
- **狀態監控**: 需要訂閱 AGV 狀態主題
- **介面定義**: 使用 agv_interfaces 中定義的訊息和服務
- **事件處理**: 監聽 AGV 狀態變化事件
- **指令下發**: 向 AGV 發送控制指令

#### 包含的 AGV 介面
- AGV 狀態訊息定義
- AGV 控制服務介面
- AGV 事件和狀態轉換
- 車型特定參數定義

## 🗺️ 資料管理工作空間

### db_proxy_ws/ - 資料庫代理服務 (AGVC 專用基礎)
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

### tafl_ws/ - TAFL 語言核心實作
**職責**: TAFL (Task Automation Flow Language) 語言核心實作，提供語法解析、執行和驗證功能

#### 套件結構
```
tafl_ws/src/
├── tafl/                       # TAFL 核心套件
│   ├── parser.py               # TAFL 語法解析器
│   ├── executor.py             # TAFL 執行引擎
│   ├── validator.py            # TAFL 語法驗證器
│   ├── ast_nodes.py           # 抽象語法樹節點
│   └── cli.py                 # 命令列介面工具
└── examples/                   # 範例 TAFL 檔案
    ├── simple_flow.yaml       # 簡單流程範例
    ├── task_creation_flow.yaml # 任務建立範例
    └── rack_rotation_flow.yaml # 架台輪轉範例
```

#### 核心功能
- **語法解析**: 解析 YAML 格式的 TAFL 流程檔案
- **執行引擎**: 執行 TAFL 流程邏輯和控制流
- **語法驗證**: 驗證 TAFL 檔案格式和語法正確性
- **AST 處理**: 建立和操作抽象語法樹
- **CLI 工具**: 提供命令列工具進行 TAFL 開發和測試

#### TAFL 語言特性
- **10個核心動詞**: query, check, create, update, if, for, switch, set, stop, notify
- **變數系統**: 支援變數定義和引用 (`${variable}`)
- **條件邏輯**: if/else 條件判斷和 switch 多分支
- **迴圈處理**: for 迴圈和 foreach 遍歷
- **資料查詢**: 支援資料庫查詢和條件過濾

### tafl_wcs_ws/ - TAFL WCS 系統
**職責**: 目前使用的 WCS 實作，基於 TAFL (Task Automation Flow Language) 的倉庫控制系統

#### 套件結構
```
tafl_wcs_ws/src/
├── tafl_wcs/                   # TAFL WCS 核心
│   ├── tafl_executor.py        # TAFL 執行引擎
│   ├── tafl_monitor.py         # TAFL 監控服務
│   ├── tafl_integration.py     # TAFL 整合層
│   ├── database.py             # 資料庫存取層
│   ├── decorators.py           # 裝飾器函數註冊
│   └── functions/              # 內建函數庫
└── launch/                     # Launch 檔案
    └── tafl_wcs_launch.py
```

#### 核心特色
- **流程執行**: 線性流程執行模式
- **變數解析**: 支援 `${variable}` 變數引用語法
- **條件執行**: `skip_if` 和 `skip_if_not` 條件控制
- **迴圈支援**: `foreach` 迴圈處理
- **平行分支**: `parallel` 平行執行
- **43個內建函數**: 完整的 query, check, task, action, control 函數庫

## 🤖 整合工作空間

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
# 自動載入 (自動檢測 AGVC 環境)
all_source             # 或別名: sa

# 強制載入 AGVC 工作空間
agvc_source           # 載入所有 AGVC 工作空間 (包含共用基礎設施和 tafl_wcs_ws)

# 檢查載入狀態
echo $ROS_WORKSPACE   # 顯示當前載入的工作空間
```

### 工作空間載入順序

#### 基礎設施工作空間 (優先載入)
1. **shared_constants_ws**: 最高優先級，定義系統通用常數
2. **keyence_plc_ws**: PLC 通訊基礎
3. **plc_proxy_ws**: PLC 服務封裝
4. **path_algorithm**: 路徑規劃算法
5. **agv_ws**: AGV 介面定義 (AGVC 監控需要)
6. **db_proxy_ws**: 資料庫代理服務 (AGVC 核心基礎)

#### AGVC 應用工作空間 (依序載入)
7. **ecs_ws**: 設備控制系統
8. **rcs_ws**: 機器人控制系統
9. **tafl_ws**: TAFL 語言核心實作
10. **tafl_wcs_ws**: TAFL WCS (目前使用的 WCS 實作)
11. **web_api_ws**: Web API 和使用者介面
12. **kuka_fleet_ws**: KUKA Fleet 外部整合
13. **launch_ws**: AGVC 啟動編排服務

### 建置管理
```bash
# 建置所有 AGVC 工作空間
build_all             # 自動建置腳本 (包含共用基礎設施)

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

## 📋 工作空間總結

### 與 AGV 共用的基礎設施工作空間 (4個)
- **shared_constants_ws**: 系統級常數定義 (跨環境共用)
- **keyence_plc_ws**: PLC 通訊基礎 (AGV 和 AGVC 都需要)
- **plc_proxy_ws**: PLC 服務封裝 (統一的 ROS 2 介面)
- **path_algorithm**: 路徑規劃算法 (AGV 執行，AGVC 協調)

### AGVC 特殊基礎工作空間 (2個)
- **agv_ws**: AGV 介面定義 (AGVC 監控 AGV 狀態需要)
- **db_proxy_ws**: 資料庫代理服務 (AGVC 核心資料基礎)

### AGVC 專用應用工作空間 (6個)
- **ecs_ws**: 設備控制系統
- **rcs_ws**: 機器人控制系統 (車隊協調)
- **tafl_ws**: TAFL 語言核心實作 (語法解析、執行、驗證)
- **tafl_wcs_ws**: TAFL WCS (目前使用的 WCS 實作)
- **web_api_ws**: Web API 和使用者介面
- **kuka_fleet_ws**: KUKA Fleet 外部整合
- **launch_ws**: AGVC 啟動編排

### 雙環境工作空間設計原則
- **基礎共用**: 4個基礎設施工作空間在兩個環境中保持一致
- **職責分離**: AGV 專注車載控制，AGVC 專注車隊管理
- **介面統一**: 透過共用工作空間確保介面一致性
- **資料隔離**: AGVC 擁有專用的資料管理基礎設施

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
- AGV 工作空間: docs-ai/context/workspaces/agv-workspaces.md
- Web 開發: docs-ai/operations/development/web/web-development.md
- 資料庫操作: docs-ai/operations/development/database-operations.md
- 車隊管理: docs-ai/knowledge/protocols/kuka-fleet-api.md
