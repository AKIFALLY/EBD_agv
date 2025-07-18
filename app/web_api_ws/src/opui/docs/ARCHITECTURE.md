# OPUI 簡化架構文件

## 📋 概述

OPUI (Operator User Interface) 是一個基於 ROS2 的 AGV 操作員介面系統，經過重構後採用更簡化、模組化的架構，適合新手維護和擴充。

## 🏗️ 新架構設計

### 整體架構圖

```
┌─────────────────────────────────────────────────────────────┐
│                    OPUI 簡化架構                              │
├─────────────────────────────────────────────────────────────┤
│  前端 (Frontend)                                             │
│  ┌─────────────────┐  ┌─────────────────┐  ┌──────────────┐ │
│  │   統一狀態管理    │  │    API 通訊層    │  │   主應用程式   │ │
│  │   (store.js)    │  │   (api.js)      │  │   (app.js)   │ │
│  └─────────────────┘  └─────────────────┘  └──────────────┘ │
├─────────────────────────────────────────────────────────────┤
│  後端 (Backend)                                              │
│  ┌─────────────────┐  ┌─────────────────┐  ┌──────────────┐ │
│  │   核心服務層     │  │   業務邏輯層     │  │   資料庫層    │ │
│  │   (core/)       │  │ (task_service)  │  │ (database/)  │ │
│  └─────────────────┘  └─────────────────┘  └──────────────┘ │
├─────────────────────────────────────────────────────────────┤
│  監控與 API                                                  │
│  ┌─────────────────┐  ┌─────────────────┐                   │
│  │   任務監控       │  │   REST API      │                   │
│  │ (monitoring/)   │  │   (api/)        │                   │
│  └─────────────────┘  └─────────────────┘                   │
└─────────────────────────────────────────────────────────────┘
```

## 📁 新的目錄結構

```
opui/
├── opui/
│   ├── core/                    # 核心服務層
│   │   ├── op_ui_server.py     # FastAPI 伺服器和路由處理
│   │   ├── op_ui_socket.py     # Socket.IO 事件處理
│   │   ├── task_service.py     # 任務業務邏輯服務
│   │   └── device_auth.py      # 設備授權驗證
│   ├── database/               # 資料庫層
│   │   └── operations.py       # 資料庫操作 (使用 db_proxy)
│   ├── api/                   # REST API 路由
│   │   ├── _client.py         # 客戶端相關 API
│   │   ├── agv.py             # AGV 相關 API
│   │   ├── license.py         # 授權相關 API
│   │   ├── node.py            # 節點相關 API
│   │   ├── process_settings.py # 製程設定 API
│   │   └── product.py         # 產品相關 API
│   ├── constants/             # 常數定義
│   │   ├── parking_status.py  # 停車位狀態常數
│   │   └── task_status.py     # 任務狀態常數
│   ├── monitoring/            # 監控模組
│   │   └── task_monitor.py    # 任務監控
│   ├── frontend/              # 前端資源
│   │   ├── static/
│   │   │   ├── index.js       # 共用功能 (全域初始化、Store狀態管理、Socket連線)
│   │   │   ├── js/
│   │   │   │   ├── store.js   # 統一狀態管理 (appStore)
│   │   │   │   ├── api.js     # API 通訊層
│   │   │   │   ├── notify.js  # 通知系統
│   │   │   │   ├── pages/     # 頁面專用功能模組
│   │   │   │   │   ├── homePage.js    # Home 頁面專用功能
│   │   │   │   │   ├── settingPage.js # Settings 頁面專用功能
│   │   │   │   │   └── rackPage.js    # Rack 頁面專用功能
│   │   │   │   └── lib/       # 第三方庫
│   │   │   │       ├── miniStore.js   # 輕量級狀態管理庫
│   │   │   │       └── socket.io.js   # Socket.IO 客戶端庫
│   │   │   ├── css/           # 樣式檔案
│   │   │   ├── fonts/         # 字型檔案
│   │   │   └── favicon.png    # 網站圖示
│   │   └── templates/         # HTML 模板
│   │       ├── base.html      # 基礎模板
│   │       ├── navbar.html    # 導航列模板
│   │       ├── home.html      # 操作主頁
│   │       ├── setting.html   # 設定頁面
│   │       ├── rack.html      # 料架管理頁面
│   │       └── unauthorized.html # 未授權頁面
├── tests/                     # 統一測試目錄
├── docs/                      # 文件目錄
├── config/                    # 配置檔案
└── README.md
```

## 🔧 核心組件說明

### 1. 前端架構 (Frontend)

#### 統一狀態管理 (store.js)
- **目的**: 將原本分散的 5 個 Store 整合為單一 Store
- **優點**: 
  - 減少狀態同步複雜度
  - 統一的狀態更新邏輯
  - 更容易除錯和維護

```javascript
// 統一的應用程式狀態
const appStore = createStore('opuiAppState', {
    user: { clientId, machineId, isConnected },
    operation: { left: {...}, right: {...} },
    data: { products, machines, rooms, parking },
    tasks: { active: {...} },
    ui: { loading, notifications }
});
```

#### API 通訊層 (api.js)
- **目的**: 統一處理前後端通訊
- **功能**: 
  - Socket.IO 連線管理
  - 事件處理封裝
  - 錯誤處理統一化

#### 共用功能模組 (index.js)
- **目的**: 統一管理應用程式初始化和共用邏輯
- **功能**:
  - 全域初始化
  - Store 狀態管理
  - Socket 連線處理
  - 事件綁定管理

#### 頁面專用功能模組 (pages/)
- **homePage.js**: Home 頁面專用功能（產品選擇、數量設定、房號選擇）
- **settingPage.js**: Settings 頁面專用功能（產品管理、系統設定）
- **rackPage.js**: Rack 頁面專用功能（料架管理）

### 2. 後端架構 (Backend)

#### 核心服務層 (core/)
- **op_ui_server.py**: FastAPI 伺服器和 HTTP 路由處理，專責 Web 伺服器管理
- **op_ui_socket.py**: Socket.IO 事件處理和即時通訊功能，專責 Socket.IO 邏輯
- **task_service.py**: 任務相關業務邏輯，統一處理任務創建、取消等操作
- **device_auth.py**: 設備授權驗證，處理客戶端認證

#### 資料庫層 (database/)
- **operations.py**: 原 db.py，包含所有資料庫操作函數
- 統一的資料庫連線和 CRUD 操作

#### 監控模組 (monitoring/)
- **task_monitor.py**: 任務狀態監控，獨立的監控邏輯

## 🚀 開發指南

### 環境設定

1. **建置專案**
```bash
cd /app/web_api_ws
colcon build --packages-select opui --symlink-install
source install/setup.bash  # 或執行 all_source
```

2. **運行服務**
```bash
ros2 run opui op_ui_server
```

3. **訪問介面**
- 主頁面: http://localhost:8002/
- 設定頁面: http://localhost:8002/setting

### 開發流程

#### 新增功能
1. **前端共用**: 在 `index.js` 中新增共用邏輯
2. **前端頁面**: 在對應的 `pages/*.js` 中新增頁面專用邏輯
3. **API**: 在 `api.js` 中新增通訊方法
4. **後端**: 在 `task_service.py` 中新增業務邏輯
5. **Socket**: 在 `op_ui_socket.py` 中新增事件處理

#### 修改狀態
1. 使用 `stateHelpers` 中的輔助函數
2. 避免直接操作 `appStore.setState()`
3. 狀態變更會自動同步到伺服器

#### 除錯技巧
1. **前端**: 使用瀏覽器開發者工具查看 console.log
2. **後端**: 查看終端輸出的 print 訊息
3. **狀態**: 在瀏覽器 localStorage 中查看狀態資料

## 🔄 架構特色

當前架構採用模組化設計，具有以下特色：

1. **前後端分離**: `op_ui_server.py` 專責 HTTP 路由，`op_ui_socket.py` 專責 Socket.IO 功能
2. **頁面功能分離**: 每個頁面都有專用的 JavaScript 檔案處理其特定功能
3. **統一狀態管理**: 使用 miniStore 提供輕量級但功能完整的狀態管理
4. **模組化設計**: 清晰的職責分離，易於維護和擴展

## 📝 維護指南

### 常見問題

1. **Socket 連線失敗**
   - 檢查伺服器是否正常運行
   - 確認 CORS 設定
   - 查看瀏覽器 Network 標籤

2. **狀態不同步**
   - 檢查 `stateHelpers` 使用是否正確
   - 確認 Socket 連線狀態
   - 查看 localStorage 中的狀態資料

3. **任務創建失敗**
   - 檢查資料庫連線
   - 確認任務參數完整性
   - 查看後端錯誤日誌

### 效能優化

1. **前端**:
   - 使用 `console.debug` 而非 `console.log` 用於除錯
   - 避免頻繁的狀態更新
   - 使用事件委派減少事件監聽器

2. **後端**:
   - 使用連線池管理資料庫連線
   - 避免在 Socket 事件中進行重複的資料庫查詢
   - 使用異步處理提高並發性能

## 🧪 測試

測試檔案統一放在 `tests/` 目錄中：

```bash
# 運行所有測試
cd /app/web_api_ws/src/opui
colcon test --packages-select opui --event-handlers console_direct+

# 運行特定測試
python -m pytest tests/test_specific.py -v
```

## 📚 進階主題

### 擴充新功能

1. **新增 Socket 事件**:
   - 在 `socket_handler.py` 中新增事件處理方法
   - 在 `api.js` 中新增對應的前端方法
   - 更新 `app.js` 中的事件綁定

2. **新增資料表操作**:
   - 在 `database/operations.py` 中新增 CRUD 函數
   - 在 `task_service.py` 中新增業務邏輯
   - 更新前端狀態管理

3. **新增頁面**:
   - 在 `frontend/templates/` 中新增 HTML 模板
   - 在 `core/server.py` 中新增路由
   - 在 `app.js` 中新增頁面初始化邏輯

這個簡化的架構讓 OPUI 系統更容易理解、維護和擴充，特別適合新手開發者快速上手。

## 🎯 快速開始指南

### 第一次使用

1. **了解專案結構**: 先閱讀本文件的目錄結構部分
2. **運行專案**: 按照環境設定步驟啟動服務
3. **查看範例**: 觀察現有的叫車/派車功能如何實作
4. **小幅修改**: 從修改 UI 文字或樣式開始練習

### 常用開發任務

#### 修改 UI 文字
- 檔案位置: `frontend/templates/*.html`
- 修改後重新啟動服務即可看到變化

#### 新增按鈕功能
1. 在 HTML 中新增按鈕元素
2. 在對應的 `pages/*.js` 中新增事件綁定
3. 在 `api.js` 中新增 Socket 通訊方法
4. 在 `op_ui_socket.py` 中新增事件處理

#### 修改資料庫操作
1. 在 `database/operations.py` 中修改或新增函數
2. 在 `task_service.py` 中使用新的資料庫函數
3. 測試功能是否正常運作

記住：小步快跑，每次只修改一個小功能，測試通過後再繼續下一個。
