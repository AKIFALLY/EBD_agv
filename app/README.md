# Loader AGV 專案

## 專案概述

Loader AGV 是一個基於 ROS 2 的自動導引車（AGV）控制系統，專門設計用於工業自動化環境中的物料裝載作業。該系統採用狀態機架構，能夠自動執行複雜的機器人操作任務，包括物料搬運、視覺定位和設備交互。

## 系統架構

### 核心組件

```
loader_agv/
├── agv_core_node.py          # 主要節點，系統入口點
├── loader_context.py         # Loader AGV 狀態機管理
├── robot_context.py          # 機器人狀態機管理
└── robot_states/             # 機器人狀態實現
    ├── base_robot_state.py   # 機器人狀態基礎類
    ├── idle_state.py         # 空閒狀態
    ├── loader_robot_parameter.py # 機器人參數配置
    ├── put_cleaner/          # 放置清洗機狀態
    ├── put_pre_dryer/        # 放置預烘乾機狀態
    ├── put_soaker/           # 放置浸泡機狀態
    ├── take_cleaner/         # 取出清洗機狀態
    ├── take_soaker/          # 取出浸泡機狀態
    └── take_transfer/        # 取出傳送箱狀態
```

### 狀態機架構

系統採用三層狀態機設計：

1. **Base Context** - 基礎 AGV 狀態管理
   - Idle State（空閒狀態）
   - Manual State（手動狀態）
   - Auto State（自動狀態）
   - Error State（錯誤狀態）

2. **Loader Context** - Loader AGV 特定狀態管理
   - Mission Select State（任務選擇狀態）
   - Running State（運行狀態）
   - Wait Robot State（等待機器人狀態）

3. **Robot Context** - 機器人操作狀態管理
   - 各種設備操作狀態（清洗機、浸泡機、預烘乾機等）

## 主要功能

### 1. 自動化物料處理
- **取出操作**：從傳送箱、清洗機、浸泡機取出物料
- **放置操作**：將物料放置到清洗機、浸泡機、預烘乾機
- **視覺定位**：使用視覺系統進行精確定位

### 2. 設備交互
- **PLC 通訊**：與工廠 PLC 系統進行數據交換
- **Hokuyo 雷射感測器**：用於物料檢測和定位
- **機器人控制**：控制工業機器人執行精確操作

### 3. 任務管理
- **任務接收**：從數據庫接收任務指令
- **路徑規劃**：自動規劃 AGV 移動路徑
- **狀態監控**：實時監控系統運行狀態

## 技術特點

### 狀態機設計模式
- 使用狀態機模式管理複雜的業務邏輯
- 清晰的狀態轉換和事件處理
- 易於擴展和維護

### 多線程執行
- 使用 ROS 2 MultiThreadedExecutor
- 支持並發處理多個任務
- 提高系統響應性能

### 模組化架構
- 高度模組化的代碼結構
- 可重用的基礎組件
- 易於客製化和擴展

## 系統配置

### 參數配置
- **room_id**: 房間識別碼
- **agv_id**: AGV 識別碼（預設：cargo02）
- **PLC IP**: PLC 通訊地址（192.168.2.100）

### 設備配置
- **機器人參數**：端口配置、位置參數
- **感測器配置**：Hokuyo 雷射感測器設定
- **通訊配置**：PLC 和數據庫連接設定

## 安裝與部署

### 系統需求
- ROS 2 (Humble/Foxy)
- Python 3.8+
- Ubuntu 20.04/22.04

### 建置步驟
```bash
# 進入工作空間
cd agv_ws

# 建置專案
colcon build --packages-select loader_agv

# 設定環境
source install/setup.bash
```

### 啟動系統
```bash
# 使用 launch 文件啟動
ros2 launch loader_agv launch.py

# 或直接啟動節點
ros2 run loader_agv agv_core_node
```

## 依賴套件

### ROS 2 套件
- `agv_base`: AGV 基礎功能
- `agv_interfaces`: AGV 介面定義
- `plc_proxy`: PLC 通訊代理
- `db_proxy_interfaces`: 數據庫介面

### Python 套件
- `rclpy`: ROS 2 Python 客戶端
- `yaml`: YAML 配置文件處理
- `setuptools`: 套件管理

## 開發指南

### 添加新狀態
1. 繼承 `BaseRobotState` 或 `State` 類
2. 實現 `enter()`, `handle()`, `leave()` 方法
3. 在適當的 Context 中註冊新狀態

### 自定義機器人參數
1. 修改 `LoaderRobotParameter` 類
2. 添加新的參數屬性
3. 更新 `values()` 方法返回參數列表

### 擴展設備支持
1. 在 `robot_states` 目錄下創建新的狀態目錄
2. 實現設備特定的操作狀態
3. 更新狀態轉換邏輯

## 故障排除

### 常見問題
1. **PLC 連接失敗**：檢查網路連接和 IP 配置
2. **機器人無響應**：確認機器人狀態和 PGNO 設定
3. **狀態轉換異常**：檢查狀態機邏輯和條件判斷

### 日誌分析
系統提供詳細的日誌輸出，包括：
- 狀態轉換信息
- 設備通訊狀態
- 錯誤和警告信息

## 維護與支持

### 版本信息
- 版本：0.0.0
- 維護者：root <yaze.lin.j303@gmail.com>
- 授權：待定

### 技術支持
如有技術問題或建議，請聯繫開發團隊。

---

*本文檔最後更新：2025-06-26*
