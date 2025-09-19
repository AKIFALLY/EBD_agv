# 核心概念解釋

## 🎯 重要概念和術語

理解這些核心概念將幫助您更好地掌握 RosAGV 系統的設計理念和實作方式。

## 🏗️ 架構概念

### 雙環境架構 (Dual Environment)

**定義**：將 AGV 控制功能分為兩個獨立的環境，各自負責不同的職責。

```
🚗 AGV 車載環境              🖥️ AGVC 管理環境
├─ 即時控制                  ├─ 車隊管理
├─ 硬體整合                  ├─ 任務調度  
├─ 本地決策                  ├─ 資料管理
└─ 安全監控                  └─ 系統整合
```

**為什麼這樣設計？**
- **關注點分離**：即時控制與管理功能分離
- **獨立性**：車載系統可獨立運行，不依賴網路
- **可擴展性**：一個管理系統可管理多台 AGV
- **維護性**：可以獨立更新和維護

### 工作空間 (Workspace)

**定義**：ROS 2 中組織相關功能的邏輯單位，包含一個或多個套件（Package）。

```
agv_ws/                     # AGV 核心控制工作空間
├── src/                    # 原始碼目錄
│   ├── agv_base/          # 基礎狀態機套件
│   ├── cargo_mover_agv/   # Cargo Mover 車型套件
│   └── loader_agv/        # Loader 車型套件
├── build/                 # 建置輸出
├── install/               # 安裝檔案
└── log/                   # 建置日誌
```

**工作空間類型**：
- **AGV 專用**：9 個專用於車載系統
- **AGVC 專用**：11 個專用於管理系統
- **共用**：3 個兩環境共用（PLC 通訊、路徑規劃等）

## 🤖 ROS 2 概念

### 節點 (Node)

**定義**：ROS 2 系統中的基本執行單位，負責特定功能。

```python
# 典型的 ROS 2 節點結構
class AGVControlNode(Node):
    def __init__(self):
        super().__init__('agv_control_node')
        
        # 發布者：發送 AGV 狀態
        self.status_publisher = self.create_publisher(
            AGVStatus, '/agv_status', 10)
        
        # 訂閱者：接收移動指令
        self.cmd_subscriber = self.create_subscription(
            MoveCommand, '/move_cmd', self.move_callback, 10)
        
        # 服務：提供 AGV 控制服務
        self.control_service = self.create_service(
            AGVControl, 'agv_control', self.control_callback)
```

### 主題 (Topic)

**定義**：節點間異步通訊的管道，採用發布/訂閱模式。

```
發布者節點 → 主題 (/agv_status) → 訂閱者節點(們)
```

**常用主題**：
- `/agv_status`：AGV 狀態資訊
- `/move_cmd`：移動指令
- `/sensor_data`：感測器資料
- `/system_events`：系統事件

### 服務 (Service)

**定義**：節點間同步通訊機制，採用請求/回應模式。

```
客戶端節點 → 服務請求 → 伺服器節點 → 服務回應 → 客戶端節點
```

**常用服務**：
- `/plc_read`：讀取 PLC 資料
- `/agv_control`：AGV 控制指令
- `/path_planning`：路徑規劃服務

### Zenoh RMW

**定義**：ROS 2 的高效能中間件實作，提供跨網路通訊能力。

**核心特性**：
- **高效能**：低延遲（< 10ms）、高吞吐量
- **網路透明**：無縫跨網路通訊
- **自動發現**：節點和服務自動發現
- **QoS 保證**：服務品質保證

```json5
// Zenoh 配置範例
{
  mode: "router",
  listen: {
    endpoints: ["tcp/0.0.0.0:7447"]
  },
  transport: {
    unicast: {
      lowlatency: true
    }
  }
}
```

## 🚗 AGV 相關概念

### 狀態機 (State Machine)

**定義**：描述 AGV 不同狀態和狀態間轉換的控制邏輯。

**3 層狀態機架構**：
```
Base 層狀態機      # 基礎生命週期管理
├── CONFIGURING   # 配置中
├── INACTIVE      # 非活動
├── ACTIVE        # 活動
└── FINALIZED     # 已結束

AGV 層狀態機       # AGV 專用狀態
├── IDLE          # 閒置等待
├── MOVING        # 移動中
├── LOADING       # 裝載中
├── UNLOADING     # 卸載中
└── ERROR         # 錯誤狀態

Robot 層狀態機     # 機械臂專用狀態  
├── HOMING        # 歸零中
├── READY         # 準備就緒
├── WORKING       # 工作中
└── MAINTENANCE   # 維護模式
```

### PGNO (Program Number)

**定義**：PLC 程式編號，用於控制機械臂執行特定動作。

```python
# Cargo Mover Robot PGNO 定義
class CargoRobotStates:
    CHG_PARA = "40000"        # 參數變更
    PHOTO_RACK_UP = "40001"   # 架台上層拍照
    PHOTO_RACK_DOWN = "40002" # 架台下層拍照
    PHOTO_BOX_IN = "40003"    # 入口箱拍照
    PHOTO_BOX_OUT = "40004"   # 出口箱拍照
    IDLE = "50000"            # 機械臂閒置
```

### 麥克納姆輪 (Mecanum Wheel)

**定義**：一種特殊設計的輪子，使 AGV 具備全向移動能力。

**特性**：
- **全向移動**：前後、左右、斜向、原地旋轉
- **精確定位**：可以毫米級精度移動
- **靈活性**：在狹窄空間中靈活操作

```
AGV 移動方向：
├── 前進/後退 ←→
├── 左移/右移 ↕
├── 斜向移動  ↗↖↙↘
└── 原地旋轉  ↻↺
```

## 🏭 業務概念

### 工作 ID (Work ID)

**定義**：標識特定業務流程的唯一編號。

**範圍分配**：
- **Cargo Mover**：1-20（ENTRANCE/EXIT 流程）
- **Loader**：21-30（多工位載料流程）
- **Unloader**：31-40（後段卸料流程）

### 端口管理 (Port Management)

**定義**：管理 AGV 和工作站之間的物料交換接口。

```
Loader AGV 端口管理：
├── AGV 端口 (port1-4)      # AGV 車上的載料位置
├── Transfer 端口 (port1-4)  # 傳送箱接口
├── Cleaner 端口 (port1-4)   # 清潔機接口
├── Soaker 端口 (port1-6)    # 浸潤機接口
└── Pre-dryer 端口 (port1-8) # 預乾燥機接口
```

### WCS (Warehouse Control System)

**定義**：倉庫控制系統，負責高層次的任務分配和資源協調。

**RosAGV 中的 WCS**：
- **TAFL WCS**：使用 YAML 配置的流程執行引擎
- **RCS**：簡化的機器人控制系統（1秒定時器）

## 🔧 技術概念

### PLC (Programmable Logic Controller)

**定義**：可程式邏輯控制器，用於工業自動化控制。

**在 RosAGV 中的作用**：
- **設備控制**：控制機械臂、傳送帶等設備
- **感測器讀取**：讀取位置、壓力、溫度等感測器資料
- **安全監控**：監控設備狀態，確保安全運行

```python
# PLC 通訊範例
plc_client.read_data("DM", "2990")  # 讀取資料記憶體
plc_client.write_data("Y", "200", 1) # 控制輸出繼電器
```

### Docker 容器化

**定義**：將應用程式和其運行環境打包成輕量級、可移植的容器。

**RosAGV 容器架構**：
```
docker-compose.yml         # AGV 車載容器配置
├── rosagv                 # AGV 控制容器
└── host 網路模式          # 直接硬體存取

docker-compose.agvc.yml    # AGVC 管理容器配置  
├── agvc_server           # 管理服務容器
├── postgres              # 資料庫容器
├── nginx                 # Web 伺服器容器
└── bridge 網路模式       # 服務隔離
```

### 統一工具系統 (Unified Tools)

**定義**：RosAGV 提供的統一管理和診斷工具集。

```bash
# 統一入口：r 命令
r agvc-check              # AGVC 健康檢查
r containers-status       # 容器狀態
r network-check          # 網路診斷
r quick-diag             # 快速診斷

# 專業工具集
source scripts/docker-tools/docker-tools.sh
agvc_start               # 啟動 AGVC 系統
agvc_health              # 健康檢查
```

## 📊 監控概念

### 健康檢查 (Health Check)

**定義**：定期檢查系統各組件的運行狀態。

**檢查層次**：
- **容器層次**：檢查 Docker 容器運行狀態
- **服務層次**：檢查 ROS 2 節點和服務狀態
- **業務層次**：檢查 AGV 任務執行狀態
- **硬體層次**：檢查 PLC、感測器等硬體狀態

### 日誌分析 (Log Analysis)

**定義**：分析系統日誌以發現問題和效能瓶頸。

**日誌類型**：
- **ROS 2 日誌**：節點運行日誌
- **容器日誌**：Docker 容器運行日誌
- **應用日誌**：業務邏輯執行日誌
- **系統日誌**：作業系統層級日誌

## 🔗 概念關聯圖

```
RosAGV 核心概念關聯：

業務需求 → 工作 ID → 狀態機 → PGNO → PLC 控制 → AGV 動作
    ↓         ↓        ↓       ↓        ↓         ↓
  WCS 決策 → 任務分配 → 節點通訊 → 主題/服務 → Zenoh RMW → 跨環境協調
    ↓         ↓        ↓       ↓        ↓         ↓
  監控系統 → 健康檢查 → 日誌分析 → 故障診斷 → 統一工具 → 運維管理
```

## 🎯 學習建議

### 概念學習順序

1. **基礎概念**：雙環境架構 → 工作空間 → 節點/主題/服務
2. **ROS 2 概念**：Zenoh RMW → 狀態機 → 跨環境通訊
3. **AGV 概念**：車型特性 → PGNO → 端口管理
4. **業務概念**：工作 ID → WCS → 業務流程
5. **技術概念**：PLC 通訊 → 容器化 → 統一工具

### 實踐建議

- **動手實驗**：使用統一工具親自操作系統
- **閱讀程式碼**：查看實際的節點實作和狀態機定義
- **跟蹤日誌**：觀察系統運行時的日誌輸出
- **模擬場景**：在測試環境中模擬實際業務流程

## 🚀 下一步

理解了核心概念後，建議繼續探索：

- [快速上手指導](quick-start-guide.md) - 實際操作系統
- [業務流程詳解](../business-processes/eyewear-production.md) - 看概念如何應用
- [技術架構深入](../system-architecture/dual-environment.md) - 了解設計細節
- [操作維護指導](../operations/development.md) - 學習日常操作

---

💡 **提示**：這些概念在實際使用中會更加清晰，建議結合實際操作來加深理解！