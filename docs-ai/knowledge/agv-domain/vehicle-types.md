# AGV 車型特性和應用場景

## 🎯 適用場景
- 理解不同 AGV 車型的特性和應用場景
- 為車型特定功能開發提供領域知識
- 選擇合適的車型配置和參數設定

## 📋 RosAGV 支援的車型

### 車型概覽
RosAGV 系統支援三種主要的 AGV 車型，每種車型都有特定的應用場景和技術特性：

```
🚛 Cargo Mover AGV    # 貨物搬運車
🏗️ Loader AGV         # 裝載車
🏭 Unloader AGV       # 卸載車
```

**⚠️ 重要事實：所有 3 種 AGV 都使用麥克納姆輪且都具備機械臂**

## 🚛 Cargo Mover AGV (貨物搬運車)

### 基本特性 (基於實際代碼)
- **主要功能**: 架台與傳送箱間貨物搬運
- **載重能力**: 500kg (來自 cargo01_config.yaml)
- **運行速度**: 最大 1.5 m/s (來自配置)
- **安全距離**: 0.5m (來自配置)
- **工作流程**: ENTRANCE/EXIT 流程控制

### 實際機械結構
```
Cargo Mover 結構 (基於實際代碼)
├── 移動底盤
│   ├── 麥克納姆輪系統 (全向移動)
│   ├── PLC 控制系統 (IP: 192.168.2.101:8501)
│   └── Hokuyo 8bit 光通訊模組 (左右側)
├── 機械臂系統
│   ├── Robot 控制器 (agv_base.robot.Robot)
│   ├── PLC 通訊介面 (透過 plc_proxy)
│   ├── PGNO 指令系統 (40000-40009, 50000)
│   └── 參數管理 (CargoRobotParameter)
├── 光通訊系統
│   ├── Hokuyo 8bit 光通訊模組 (左右側)
│   ├── PLC 資料通訊
│   └── 8bit 資料傳輸
└── 控制系統
    ├── 3層狀態機架構 (Base-AGV-Robot)
    ├── ROS 2 節點管理
    └── Zenoh RMW 通訊
```

### 實際應用場景
- **架台搬運**: 從架台(Rack)取料運送至傳送箱(Transfer)
- **入口流程**: ENTRANCE 流程 - 架台→等待旋轉→傳送箱
- **出口流程**: EXIT 流程 - 傳送箱→等待旋轉→架台
- **光通訊**: 左右側 Hokuyo 8bit 光通訊模組提供 PLC 資料通訊功能

### 實際技術特點
- **Hokuyo 系統**: 左右側 8bit 光通訊模組提供 PLC 資料通訊
- **麥克納姆輪**: 全向移動能力，精確定位
- **機械臂整合**: 具備完整的 Robot 控制系統
- **3層狀態機**: 複雜的狀態管理和控制邏輯

### 實際控制邏輯特性
```python
# Cargo Mover 實際狀態 (基於 cargo_mover_agv 代碼)
class CargoRobotStates:
    # Robot PGNO 定義 (來自 robot.py)
    CHG_PARA = "40000"           # 參數變更
    PHOTO_RACK_UP = "40001"      # 架台上層拍照
    PHOTO_RACK_DOWN = "40002"    # 架台下層拍照  
    PHOTO_BOX_IN = "40003"       # 入口箱拍照
    PHOTO_BOX_OUT = "40004"      # 出口箱拍照
    IDLE = "50000"               # 機械臂閒置
    
    # 實際參數管理 (CargoRobotParameter)
    rack_port: int              # 架台端口 (1-32)
    boxin_port: int             # 入口箱端口
    boxout_port: int            # 出口箱端口
```

## 🏗️ Loader AGV (裝載車)

### 基本特性 (基於實際代碼)
- **主要功能**: 多工位自動載料操作
- **機械臂**: Robot 控制系統 (共用 agv_base.robot.Robot)
- **工作流程**: Take Transfer, Put/Take Cleaner, Put/Take Soaker, Put Pre-dryer
- **完整度**: 具備完整測試套件和實作
- **AGV 端口**: 動態端口管理 (port1-4)

### 實際機械結構
```
Loader AGV 結構 (基於 loader_agv 代碼)
├── 移動底盤
│   ├── 麥克納姆輪系統 (全向移動)
│   ├── 精密定位系統
│   └── 3層狀態機控制
├── 機械臂系統
│   ├── Robot 控制器 (agv_base.robot.Robot)
│   ├── LoaderRobotParameter 參數管理
│   ├── PLC 通訊介面
│   └── PGNO 指令系統
├── 端口管理系統
│   ├── AGV 端口 (port1-4)
│   ├── 工位端口 (Transfer, Cleaner, Soaker, Pre-dryer)
│   ├── 端口狀態檢測
│   └── 動態端口分配
├── 光通訊系統
│   ├── Hokuyo 8bit 光通訊模組 (前方)
│   └── PLC 資料通訊
└── 控制系統
│   ├── LoaderContext (AGV 層狀態)
│   ├── RobotContext (Robot 層狀態)
│   └── 完整測試覆蓋
```

### 實際應用場景
- **Take Transfer**: 從傳送箱取料至 AGV 端口
- **Put/Take Cleaner**: AGV 與清潔機間的雙向載料
- **Put/Take Soaker**: AGV 與浸潤機間的雙向載料
- **Put Pre-dryer**: AGV 到預乾燥機的單向載料
- **視覺定位**: 精確定位和載料操作

### 實際技術特點
- **完整實作**: 具備完整的測試套件和實作
- **多工位支援**: 支援 4 種不同工位的載料操作
- **端口管理**: 自動 AGV 端口狀態管理和分配
- **參數化控制**: LoaderRobotParameter 提供靈活配置

### 實際控制邏輯特性
```python
# Loader AGV 實際控制 (基於 loader_agv 代碼)
class LoaderRobotContext:
    # Robot 狀態機步驟 (來自 robot_context.py)
    IDLE = 0
    CHECK_IDLE = 1
    WRITE_CHG_PARA = 2
    WRITE_PGNO = 4
    CHECK_PGNO = 5
    ACTING = 6
    FINISH = 99
    
    # 端口狀態管理
    agv_port1: bool             # AGV Port 1 狀態
    agv_port2: bool             # AGV Port 2 狀態
    boxin_port1-4: bool         # Transfer 端口狀態
    cleaner_port1-4: bool       # Cleaner 端口狀態
    soaker_port1-6: bool        # Soaker 端口狀態
    pre_dryer_port1-8: bool     # Pre-dryer 端口狀態
```

### 實際機械臂控制
```python
# 實際機械臂控制 (基於 loader_agv/robot_context.py)
class RobotContext(BaseContext):
    def __init__(self, initial_state):
        # 實際 Robot 初始化
        self.robot_parameter = LoaderRobotParameter()
        self.robot = Robot(self.node, self.robot_parameter)
    
    def update_port_parameters(self):
        # 實際端口參數更新
        self.robot_parameter.loader_agv_port_front = self.get_loader_agv_port_front
        self.robot_parameter.boxin_port = self.get_boxin_port
        self.robot_parameter.cleaner_port = self.get_cleaner_port
        # 同步更新參數到 PLC
        self.robot_parameter.calculate_parameter()
        self.robot.update_parameter()
```

## 🏭 Unloader AGV (卸載車)

### 基本特性 (基於實際代碼)
- **主要功能**: 預乾燥機、烤箱、出口箱卸料操作
- **機械臂**: Robot 控制系統 (共用 agv_base.robot.Robot)
- **工作流程**: Take/Put Pre Dryer, Take/Put Oven, Take/Put Boxout Transfer
- **完整度**: 40% 實作，基礎架構完整
- **分揀功能**: 自動分揀和處理邏輯

### 實際機械結構
```
Unloader AGV 結構 (基於 unloader_agv 代碼)
├── 移動底盤
│   ├── 麥克納姆輪系統 (全向移動)
│   ├── 3層狀態機控制
│   └── 精密定位系統
├── 機械臂系統
│   ├── Robot 控制器 (agv_base.robot.Robot)
│   ├── UnloaderRobotParameter 參數管理
│   ├── PLC 通訊介面
│   └── PGNO 指令系統
├── 工位管理系統
│   ├── Pre-dryer 端口管理
│   ├── Oven 端口管理
│   ├── Boxout Transfer 端口管理
│   └── 分揀邏輯控制
├── 光通訊系統
│   ├── Hokuyo 8bit 光通訊模組 (前方)
│   └── PLC 資料通訊
└── 控制系統
    ├── UnloaderContext (AGV 層狀態)
    ├── RobotContext (Robot 層狀態)
    └── 基礎測試框架
```

### 實際應用場景
- **Take/Put Pre Dryer**: 預乾燥機的雙向卸料操作
- **Take/Put Oven**: 烤箱的雙向卸料操作
- **Take/Put Boxout Transfer**: 出口傳送箱的雙向操作
- **分揀功能**: 自動分揀和處理邏輯
- **工作 ID 範圍**: 1-40 (4個主要流程)

### 實際技術特點
- **基礎架構完整**: 繼承完整的 3層狀態機架構
- **工位特化**: 專門針對後段工位的卸料操作
- **麥克納姆輪**: 全向移動能力，與其他 AGV 一致
- **40% 實作**: 基礎框架完整，部分功能待開發

### 實際控制邏輯特性
```python
# Unloader AGV 實際控制 (基於 unloader_agv 架構)
class UnloaderRobotContext:
    # 繼承通用 Robot 狀態機步驟
    IDLE = 0
    CHECK_IDLE = 1
    WRITE_CHG_PARA = 2
    WRITE_PGNO = 4
    CHECK_PGNO = 5
    ACTING = 6
    FINISH = 99
    
    # Unloader 專用工位管理
    pre_dryer_ports: dict       # 預乾燥機端口狀態
    oven_ports: dict           # 烤箱端口狀態
    boxout_ports: dict         # 出口箱端口狀態
    sorting_logic: object      # 分揀邏輯控制
```

## 🔧 車型選擇指導

### 實際選擇標準
```
應用需求分析 (基於實際代碼)
├── 工作流程
│   ├── 搬運類型 (架台、工位、分揀)
│   ├── 操作複雜度
│   └── 端口數量需求
├── 實作狀態
│   ├── 開發完整度
│   ├── 測試覆蓋率
│   └── 穩定性
├── 技術需求
│   ├── 麥克納姆輪移動能力
│   ├── 機械臂精度要求
│   └── 3層狀態機複雜度
└── 部署考量
    ├── 維護複雜度
    ├── 系統整合度
    └── 擴展性
```

### 實際車型對比 (基於代碼分析)
| 特性 | Cargo Mover | Loader | Unloader |
|------|-------------|---------|----------|
| **實作完整度** | 60% | 100% | 40% |
| **機械臂** | 有 | 有 | 有 |
| **移動系統** | 麥克納姆輪 | 麥克納姆輪 | 麥克納姆輪 |
| **主要工位** | 架台+傳送箱 | 4個工位 | 3個工位 |
| **測試覆蓋** | 完整 | 完整 | 基礎 |
| **狀態機** | 3層架構 | 3層架構 | 3層架構 |

### 實際應用場景匹配
```
架台搬運 → Cargo Mover
├── ENTRANCE 流程 (架台→傳送箱)
├── EXIT 流程 (傳送箱→架台)
└── Hokuyo 光通訊模組 (左右側)

多工位載料 → Loader
├── Take Transfer (傳送箱取料)
├── Put/Take Cleaner (清潔機)
├── Put/Take Soaker (浸潤機)
└── Put Pre-dryer (預乾燥機)

後段卸料 → Unloader
├── Take/Put Pre Dryer
├── Take/Put Oven
└── Take/Put Boxout Transfer
```

## 🚀 技術發展現狀

### 共同技術基礎
- **麥克納姆輪**: 所有 AGV 都使用全向移動系統
- **機械臂整合**: 所有 AGV 都具備 Robot 控制系統
- **3層狀態機**: 統一的 Base-AGV-Robot 架構
- **Zenoh RMW**: 統一的 ROS 2 通訊機制

### 實作狀態分析
- **Loader AGV**: 完整實作，具備完整測試套件
- **Cargo Mover**: 60% 完成，Hokuyo 系統完整
- **Unloader**: 40% 完成，基礎架構完整

### 開發優先級
- **維護優先**: Loader AGV (生產就緒)
- **完善開發**: Cargo Mover AGV (核心功能完整)
- **持續開發**: Unloader AGV (基礎架構完整)

## 📋 開發考量

### 基於實際代碼的開發
- **統一架構**: 所有車型共用 agv_base 基礎框架
- **參數化設計**: 每種車型有專用的 RobotParameter
- **測試驅動**: 完整的測試套件確保品質
- **實際驗證**: 基於實際硬體配置和 PLC 通訊

### 實際技術限制
- **麥克納姆輪**: 所有車型必須考慮全向移動特性
- **PLC 通訊**: 統一的 PGNO 指令系統
- **容器環境**: 必須在 Docker 容器內開發和測試
- **ROS 2 Jazzy**: 統一的 ROS 2 發行版和 Zenoh RMW

## 🔗 交叉引用
- AGV 工作空間: docs-ai/context/workspaces/agv-workspaces.md
- ROS 2 開發: docs-ai/operations/development/ros2/ros2-development.md
- 系統診斷: docs-ai/operations/guides/system-diagnostics.md
- 容器開發: docs-ai/operations/development/docker-development.md
- 技術棧: docs-ai/context/system/technology-stack.md