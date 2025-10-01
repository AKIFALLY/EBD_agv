# 系統整體概覽

## 🎯 5 分鐘理解 RosAGV

這個概覽將幫助您快速理解 RosAGV 的整體架構、核心組件和運作流程。

## 🏗️ 系統架構

### 雙環境設計理念

RosAGV 採用**雙環境分離架構**，將不同職責的功能分別部署：

<div class="dual-column">
<div class="column">

### 🚗 AGV 車載系統

**部署位置**：AGV 車輛上  
**網路模式**：Host（直接硬體存取）  
**核心職責**：

- ⚡ 即時控制和狀態管理
- 🔌 PLC 設備直接通訊
- 📡 感測器資料處理
- 🕹️ 手動控制（搖桿）支援
- 🗺️ 路徑規劃和導航

</div>
<div class="column">

### 🖥️ AGVC 管理系統

**部署位置**：中央管理伺服器  
**網路模式**：Bridge（隔離安全）  
**核心職責**：

- 🚛 車隊管理和任務調度
- 💾 資料庫管理和資料持久化
- 🌐 Web 管理介面
- 🔗 外部系統整合（KUKA Fleet）
- 📊 系統監控和日誌管理

</div>
</div>

### 為什麼這樣設計？

1. **高可用性**：車載系統獨立運行，不依賴中央系統
2. **可擴展性**：管理系統可以管理多台 AGV
3. **維護性**：各環境可以獨立更新和維護
4. **安全性**：車載系統與管理系統網路隔離

## 🚗 AGV 車型和應用

### 三種專業車型

| 車型 | 主要用途 | 核心工作流程 | 完成度 |
|------|----------|-------------|--------|
| **🚛 Cargo Mover** | 架台與傳送箱搬運 | ENTRANCE/EXIT 流程控制 | 60% |
| **🏗️ Loader** | 多工位載料操作 | Take Transfer → Put/Take 各工位 | 100% ✅ |
| **🏭 Unloader** | 後段工位卸料 | Take/Put Pre-dryer, Oven, Boxout | 40% |

### 🔧 共同技術特性
- **麥克納姆輪**：全向移動能力，精確定位
- **機械臂系統**：Robot 控制器整合
- **3層狀態機**：Base-AGV-Robot 架構
- **Zenoh RMW 通訊**：統一的跨環境通訊

## 💻 技術棧概覽

### 🤖 ROS 2 生態系統
```yaml
ROS 2 Jazzy (LTS) + Zenoh RMW:
  架構: 模組化設計，每個功能獨立 Package
  效能: Zenoh 高效能通訊（< 10ms 延遲）
  開發: Python 3.12 主要語言
  建置: Colcon 系統，支援增量建置
```

### 🐳 容器化技術
```yaml
Docker Compose V2:
  AGV 容器: Host 網路，直接硬體存取
  AGVC 容器: Bridge 網路，服務隔離
  PostgreSQL: 企業級資料庫
  Nginx: 反向代理和靜態服務
```

### 🌐 Web 技術
```yaml
FastAPI + Socket.IO + PostgreSQL:
  AGVCUI: 管理員介面 (Port 8001)
  OPUI: 操作員介面 (Port 8002)
  API Server: 核心 API (Port 8000)
  pgAdmin: 資料庫管理 (Port 5050)
```

## 🔄 工作空間結構

### AGV 車載工作空間（9個）
```yaml
AGV 車載工作空間:
  專注: 即時控制和硬體整合
  工作空間:
    - agv_ws: 核心 AGV 控制和車型實作
    - agv_cmd_service_ws: 手動指令服務
    - joystick_ws: 搖桿控制整合
    - sensorpart_ws: 感測器資料處理
    - 共用模組: keyence_plc_ws, plc_proxy_ws, path_algorithm
```

### AGVC 管理工作空間（11個）
```yaml
AGVC 管理工作空間:
  專注: 車隊管理和系統整合
  工作空間:
    - web_api_ws: Web API 和 Socket.IO
    - db_proxy_ws: 資料庫代理服務
    - ecs_ws: 設備控制系統
    - rcs_ws: 機器人控制系統
    - tafl_wcs_ws: TAFL 流程執行引擎
    - kuka_fleet_ws: KUKA Fleet 整合
    - 共用模組: keyence_plc_ws, plc_proxy_ws, path_algorithm
```

## 🌐 通訊機制

### Zenoh RMW 跨環境通訊
```yaml
AGV 車載系統:
  網路模式: Host 網路
  Zenoh Router: 0.0.0.0:7447
  功能: ROS 2 節點自動發現

跨網路通訊: ↕️ 自動建立連接 ↕️

AGVC 管理系統:
  網路模式: Bridge 網路
  Zenoh Router: 192.168.100.100:7447
  功能: ROS 2 節點自動發現
```

### 通訊特性
- **自動服務發現**：節點和服務自動發現，無需手動配置
- **高效能**：本地 < 100μs，跨網路 < 10ms
- **可靠性**：支援 QoS 保證，自動重連機制
- **透明性**：開發者無需關心網路細節

## 🏭 業務流程示例

### 眼鏡生產完整流程
```yaml
眼鏡生產流程:
  步驟1: 射出機完成 → OPUI 叫車 → Cargo Mover AGV 前往
  步驟2: AGV 載入眼鏡框 → 運送至清潔工站 → Loader AGV 接手
  步驟3: 清潔完成 → Loader AGV 運送至烘乾工站
  步驟4: 烘乾完成 → Unloader AGV 運送至包裝區
  步驟5: KUKA 機器人協作包裝 → AGV 運送至出貨區
```

### 系統協調機制
- **TAFL WCS 流程執行**：YAML 配置驅動的任務建立和執行
- **RCS 簡化調度**：1秒定時器的基本任務分派
- **ECS 設備控制**：門控、電梯等設備協調
- **實時監控**：Web 介面實時顯示所有 AGV 狀態

## 📊 系統監控

### Web 管理介面
- **AGVCUI**：管理員專用，完整系統控制
- **OPUI**：操作員專用，簡化的叫車和監控介面
- **實時狀態**：Socket.IO 推送，即時更新
- **歷史數據**：PostgreSQL 存儲，支援報表和分析

### 診斷工具
```bash
# 統一診斷工具
r agvc-check           # AGVC 系統健康檢查
r containers-status    # 容器狀態檢查
r network-check        # 網路連接檢查
r quick-diag          # 快速綜合診斷
```

## 🔧 開發和部署

### 開發環境
- **必須在容器內開發**：宿主機無 ROS 2 環境
- **統一工具支援**：`r` 命令集提供便捷操作
- **自動載入**：`all_source` 自動檢測並載入對應工作空間

### 部署架構
- **AGV 部署**：每台 AGV 的邊緣計算設備
- **AGVC 部署**：中央管理伺服器或雲端平台
- **網路連接**：工業 WiFi 或 5G 網路連接

## 🎯 核心優勢

### 📈 企業級特性
- **高可用性**：99.9% 系統可用性
- **可擴展性**：支援 50+ AGV 同時運行
- **安全性**：網路隔離、權限控制、資料加密
- **維護性**：模組化設計，獨立更新部署

### 🚀 技術特色
- **最新技術棧**：ROS 2 Jazzy + Zenoh RMW
- **現代化架構**：微服務、容器化、API 優先
- **簡化設計**：務實的配置驅動架構
- **系統整合**：多品牌 AGV 和設備的統一管理

### 💼 業務價值
- **效率提升**：自動化替代人工，24/7 運行
- **成本控制**：減少人力成本，提高設備利用率
- **品質保證**：精確控制，減少人為錯誤
- **數據洞察**：實時數據收集，支援決策優化

## 🚀 下一步探索

根據您的角色和興趣，建議的學習路徑：

### 🏭 了解業務應用
- [眼鏡生產流程詳解](../business-processes/eyewear-production.md)
- [AGV 車型特色介紹](../agv-vehicles/vehicle-types.md)

### 🏛️ 深入技術架構
- [雙環境架構詳解](../system-architecture/dual-environment.md)
- [技術棧詳細說明](../system-architecture/technology-stack.md)

### 🔧 開始實際操作
- [部署指導](../operations/deployment.md)
- [開發環境設定](../operations/development.md)

### 🛠️ 維護和排障
- [系統診斷工具](../operations/maintenance.md)
- [故障排除指南](../operations/troubleshooting.md)

---

💡 **提示**：每個章節都包含實際的程式碼範例、配置檔案和操作指令，您可以按需深入學習。