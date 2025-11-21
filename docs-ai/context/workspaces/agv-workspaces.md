# AGV 車載工作空間概覽

## 🎯 適用場景
- 理解 AGV 車載系統的工作空間結構和職責分工
- 為 AGV 相關開發提供架構指導
- 解決跨工作空間的依賴和整合問題

## 📋 AGV 工作空間架構

### 工作空間總覽 (11個：6專用 + 4共用基礎 + 1共用應用)
AGV 車載系統包含 11 個工作空間（含共用），每個工作空間負責特定的功能領域，形成完整的車載控制系統。

```
AGV 車載系統工作空間
├── 專用工作空間 (6個)
│   ├── agv_ws/                    # 核心 AGV 控制
│   ├── agv_cmd_service_ws/        # 手動指令服務
│   ├── joystick_ws/               # 搖桿控制整合
│   ├── sensorpart_ws/             # 感測器資料處理
│   ├── uno_gpio_ws/               # GPIO 控制服務
│   └── web_api_ws/                # AGVUI 車載監控介面
├── 共用基礎設施 (4個)
│   ├── shared_constants_ws/       # 系統級常數定義
│   ├── keyence_plc_ws/            # Keyence PLC 通訊
│   ├── plc_proxy_ws/              # PLC 代理服務
│   └── path_algorithm/            # 路徑規劃演算法
└── 共用應用工作空間 (2個)
    ├── db_proxy_ws/               # 本地資料存取
    └── launch_ws/                 # ROS 2 啟動編排
```

## 🤖 核心控制工作空間

### agv_ws/ - AGV 核心控制
**職責**: AGV 狀態機、車型實作、核心控制邏輯

#### 套件結構
```
agv_ws/src/
├── agv_base/                  # 基礎狀態機架構
│   ├── agv_node_base.py      # AGV 節點基類
│   ├── agv_states/           # Base 層狀態定義
│   ├── context_abc.py        # Context 抽象基類
│   └── event.py              # 事件處理機制
├── cargo_mover_agv/          # Cargo 車型實作
│   ├── cargo_context.py     # Cargo 車型 Context
│   └── robot_context.py     # Cargo 機械臂 Robot
├── loader_agv/               # Loader 車型實作
│   ├── loader_context.py    # Loader 車型 Context
│   └── robot_context.py     # Loader 機械臂 Robot
└── unloader_agv/             # Unloader 車型實作
    ├── unloader_context.py  # Unloader 車型 Context
    └── robot_context.py     # Unloader 機械臂 Robot
```

#### 3層狀態機架構
```
📊 Base 層 (通用邏輯狀態)
├── agv_base/agv_states/*.py
├── 基礎狀態定義和轉換邏輯
└── 所有車型共用的狀態處理

📋 AGV 層 (車型特定狀態)  
├── *_agv/*_context.py
├── Cargo/Loader/Unloader 特定行為
└── 車型專用狀態和參數

🤖 Robot 層 (機械臂執行狀態)
├── *_agv/robot_context.py
├── 機械臂任務執行邏輯
└── 硬體動作控制狀態
```

#### 車型特性 (基於實際代碼)
- **Cargo Mover**: 貨物搬運車，架台與傳送箱間搬運，具備 Hokuyo 8bit 光通訊模組 (左右側)
- **Loader**: 裝載車，多工位載料操作，具備機械臂和動態端口管理
- **Unloader**: 卸載車，後段工位卸料操作，具備分揀功能和機械臂

### agv_cmd_service_ws/ - 手動指令服務
**職責**: 手動控制指令處理、遠端操作支援

#### 核心功能
- 手動模式切換
- 遠端控制指令接收
- 安全檢查和驗證
- 自動/手動模式協調

#### 關鍵特性
- 優先級管理：手動指令優先於自動任務
- 安全機制：防止危險操作
- 狀態同步：與主狀態機同步

## 🕹️ 人機介面工作空間

### joystick_ws/ - 搖桿控制整合
**職責**: USB 搖桿整合、手動操作支援

#### 硬體支援
- USB 搖桿設備檢測
- 多種搖桿型號支援
- 按鍵映射配置
- 死區和靈敏度調整

#### 控制功能
- 方向控制：前進、後退、轉向
- 速度控制：變速操作
- 功能按鍵：緊急停止、模式切換
- 狀態指示：LED 或震動回饋

#### 安全機制
- 死人開關：必須持續按住才能操作
- 速度限制：手動模式下的最大速度限制
- 緊急停止：立即停止所有動作

## 📡 感測器處理工作空間

### sensorpart_ws/ - 感測器資料處理
**職責**: 感測器資料收集、處理、融合

#### 實際感測器和設備
- **Hokuyo 8bit 光通訊模組**: 與 PLC 進行光通訊資料傳輸
- **麥克納姆輪編碼器**: 輪速和位置回饋
- **機械臂感測器**: 位置和狀態回饋
- **端口檢測**: 工位和 AGV 端口狀態檢測

#### 資料處理功能
- 光通訊資料解析和處理
- 端口狀態資料融合
- 機械臂感測器資料處理
- 即時資料發布到 ROS 2 主題

#### 安全功能
- 端口狀態安全檢查
- 機械臂位置安全監控
- 光通訊異常檢測
- 工位安全狀態監控

## 🔌 通訊整合工作空間

### keyence_plc_ws/ - Keyence PLC 通訊
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

### plc_proxy_ws/ - PLC 代理服務
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

## 🗺️ 導航規劃工作空間

### path_algorithm/ - 路徑規劃演算法
**職責**: 路徑規劃、導航演算法實作

#### 演算法支援
- **A* 演算法**: 最短路徑搜尋
- **RRT**: 快速隨機樹規劃
- **動態視窗法**: 即時避障
- **純追蹤**: 路徑跟隨控制

#### 功能特性
- 靜態地圖規劃
- 動態障礙物避障
- 多目標點規劃
- 路徑最佳化

#### 整合介面
- ROS 2 導航堆疊整合
- 自定義規劃器介面
- 即時路徑更新
- 規劃結果視覺化

## 🔄 工作空間載入和管理

### 環境載入
```bash
# 自動載入 (自動檢測 AGV 環境)
all_source             # 或別名: sa

# 強制載入 AGV 工作空間
agv_source            # 載入所有 AGV 工作空間 (包含共用基礎設施)

# 檢查載入狀態
echo $ROS_WORKSPACE   # 顯示當前載入的工作空間
```

### 工作空間載入順序

#### 基礎設施工作空間 (優先載入)
1. **shared_constants_ws**: 最高優先級，定義系統通用常數
2. **keyence_plc_ws**: PLC 通訊基礎
3. **plc_proxy_ws**: PLC 服務封裝
4. **path_algorithm**: 路徑規劃算法

#### AGV 應用工作空間 (依序載入)
5. **agv_cmd_service_ws**: 手動指令服務
6. **joystick_ws**: 搖桿控制
7. **agv_ws**: 核心 AGV 控制
8. **sensorpart_ws**: 感測器處理
9. **uno_gpio_ws**: GPIO 控制
10. **launch_ws**: 啟動編排服務

### 建置管理
```bash
# 建置所有 AGV 工作空間
build_all             # 自動建置腳本 (包含共用基礎設施)

# 建置特定工作空間
colcon build --packages-select agv_base
colcon build --packages-up-to cargo_mover_agv

# 測試執行
colcon test --packages-select agv_ws
```

### 啟動管理
```bash
# 啟動 AGV 核心系統
ros2 launch loader_agv launch.py

# 啟動特定服務
ros2 run agv_cmd_service cmd_service_node
ros2 run joystick_ws joystick_node
```

## 📋 工作空間總結

### 與 AGVC 共用的基礎設施工作空間 (4個)
- **shared_constants_ws**: 系統級常數定義 (跨環境共用)
- **keyence_plc_ws**: PLC 通訊基礎 (AGV 和 AGVC 都需要)
- **plc_proxy_ws**: PLC 服務封裝 (統一的 ROS 2 介面)
- **path_algorithm**: 路徑規劃算法 (AGV 執行，AGVC 協調)

### AGV 專用應用工作空間 (6個)
- **agv_cmd_service_ws**: 手動指令服務
- **joystick_ws**: 搖桿控制整合
- **agv_ws**: 核心 AGV 控制 (狀態機、車型實作)
- **sensorpart_ws**: 感測器資料處理
- **uno_gpio_ws**: GPIO 控制服務
- **launch_ws**: AGV 啟動編排

### 共用工作空間設計理念
- **一致性**: 確保 AGV 和 AGVC 使用相同的基礎服務
- **維護性**: 基礎設施變更只需在一處進行
- **效率性**: 避免重複開發相同功能
- **可靠性**: 經過雙環境驗證的穩定基礎

## 🔧 開發指導

### 跨工作空間開發
1. **依賴管理**: 使用 package.xml 聲明依賴關係
2. **介面定義**: 在 agv_interfaces 中定義共用介面
3. **配置管理**: 使用 ROS 2 參數系統
4. **測試策略**: 單元測試 + 整合測試

### 狀態機開發
1. **Base 層**: 實作通用狀態邏輯
2. **AGV 層**: 實作車型特定行為
3. **Robot 層**: 實作機械臂控制
4. **事件處理**: 使用統一的事件機制

### 硬體整合
1. **抽象層**: 使用硬體抽象層隔離硬體差異
2. **驅動程式**: 實作標準化的驅動介面
3. **配置檔案**: 使用 YAML 配置硬體參數
4. **錯誤處理**: 實作完善的錯誤處理機制

## 📋 最佳實踐

### 程式碼組織
- 每個工作空間專注於單一職責
- 使用清晰的命名規範
- 實作適當的抽象層
- 保持介面的穩定性

### 測試策略
- 單元測試：測試個別功能模組
- 整合測試：測試工作空間間的整合
- 硬體在環測試：使用實際硬體測試
- 模擬測試：使用模擬器進行測試

### 文檔維護
- 每個工作空間維護獨立的 CLAUDE.md
- 記錄 API 變更和相容性
- 提供使用範例和最佳實踐
- 定期更新架構文檔

## 🔗 交叉引用
- AGVC 工作空間: docs-ai/context/workspaces/agvc-workspaces.md
- ROS 2 開發指導: docs-ai/operations/development/ros2/ros2-development.md
- 車型領域知識: docs-ai/knowledge/agv-domain/vehicle-types.md
