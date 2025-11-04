# loader_agv - 裝載車AGV控制系統

## 📚 Context Loading
../../../../CLAUDE.md  # 引用根目錄系統文档
../../CLAUDE.md  # 引用上層 agv_ws 工作空間文档

# Loader 專屬知識（套件特有）
@docs-ai/knowledge/agv-domain/write-path-state-analysis.md  # 路徑狀態分析

## 📋 專案概述
loader_agv 實現 Loader AGV 的完整控制邏輯，支援從傳送箱取料、多工位操作（清潔機、浸潤機、預乾燥機）、AGV端口管理等複雜載料流程。負責眼鏡生產房間內前段製程的精密載料操作，包含 4 種設備類型共 7 種業務流程。

## 🏭 核心業務流程 (基於眼鏡生產系統)

### 主要職責
- **工作區域**: 眼鏡房間內部
- **核心任務**: 前段製程的精密載料操作
- **關鍵設備**: Hokuyo 8bit 光通訊模組 (前方配置)
- **工作模式**: 一次1格精密操作，確保製程精度

### Work ID 自動路由系統
Loader AGV 採用自動 work_id 路由系統，格式：`room_id + equipment_type + station_number + action_type`

**設備類型編碼**：
- `01` = TRANSFER (入口傳送箱)
- `03` = CLEANER (清潔機) 
- `04` = SOAKER (浸潤機)
- `05` = PRE_DRYER (預乾燥機)

**動作類型編碼**：
- `01` = TAKE (取料)
- `02` = PUT (放料)

### 7種核心業務流程

#### 取料操作 (TAKE Operations)
**1. TAKE_TRANSFER - 從入口傳送箱取料**
- Work ID: `room_id + "01" + "01" + "01"`
- 功能: 從房間入口的4格傳送箱取得 Cargo AGV 送來的待處理產品
- 設備: 入口傳送箱 (4格: 上2下2)

**2. TAKE_CLEANER - 從清潔機取料**
- Work ID 範圍: `room_id + "03" + "01-02" + "01"`  
- 功能: 從2台清潔機取得清潔完成的產品
- 設備: 清潔機站點1-2 (2個工位)

**3. TAKE_SOAKER - 從浸潤機取料** 
- Work ID 範圍: `room_id + "04" + "01-06" + "01"`
- 功能: 從6台浸潤機取得處理完成的產品
- 設備: 浸潤機站點1-6 (6個工位)

**4. TAKE_PRE_DRYER - 從預乾燥機取料**
- Work ID 範圍: `room_id + "05" + "01-08" + "01"`
- 功能: 從8台預乾燥機取得預乾完成的產品  
- 設備: 預乾燥機站點1-8 (8個工位)

#### 放料操作 (PUT Operations)
**5. PUT_CLEANER - 放料到清潔機**
- Work ID 範圍: `room_id + "03" + "01-02" + "02"`
- 功能: 將產品放入2台清潔機進行清潔處理
- 設備: 清潔機站點1-2 (2個工位)

**6. PUT_SOAKER - 放料到浸潤機**
- Work ID 範圍: `room_id + "04" + "01-06" + "02"`
- 功能: 將產品放入6台浸潤機進行藥水處理
- 設備: 浸潤機站點1-6 (6個工位)

**7. PUT_PRE_DRYER - 放料到預乾燥機**
- Work ID 範圍: `room_id + "05" + "01-08" + "02"`
- 功能: 將產品放入8台預乾燥機進行預乾處理
- 設備: 預乾燥機站點1-8 (8個工位)

### 典型製程流程範例
**製程1 (泡藥1次) 流程**:
1. TAKE_TRANSFER → 從入口傳送箱取料
2. PUT_CLEANER → 放入清潔機清潔  
3. TAKE_CLEANER → 從清潔機取出
4. PUT_SOAKER → 放入浸潤機泡藥
5. TAKE_SOAKER → 從浸潤機取出
6. PUT_PRE_DRYER → 放入預乾燥機預乾

**製程2 (泡藥2次) 流程**:
1. TAKE_TRANSFER → 從入口傳送箱取料
2. PUT_CLEANER → 放入清潔機清潔
3. TAKE_CLEANER → 從清潔機取出  
4. PUT_SOAKER → 第1次放入浸潤機泡藥
5. TAKE_SOAKER → 第1次從浸潤機取出
6. PUT_SOAKER → 第2次放入浸潤機泡藥 
7. TAKE_SOAKER → 第2次從浸潤機取出
8. PUT_PRE_DRYER → 放入預乾燥機預乾

### 車載料架配置
- **物理結構**: 4層垂直排列 (第1層到第4層)
- **S尺寸產品配置**: 可使用全部4層 (1、2、3、4層)
- **L尺寸產品配置**: 只使用第1層和第3層 (2、4層掛勾由人工解下)
- **載重適配**: 根據產品尺寸調整車載料架配置，最佳化載運效率

### 技術實作特點
- **精密操作邏輯**: 所有操作都採用 **一次1格** 的精密處理方式（機械臂能力限制）
- **任務與執行動作的對應關係** ⚠️ 重要設計邏輯：
  - **核心設計原則**: `batch_size = Station 的 port 數量`
  - **標準設備** (入口箱、清潔機、預乾機等):
    - TAFL 任務定義: `batch_size = 2` (每個 Station 有2個 port)
    - 機械臂能力: 一次取放動作處理 **1格** Carrier
    - 執行邏輯: **一次任務執行兩次取放動作** (1任務 = 2動作)
    ```
    TAFL 任務: batch_size=2, ports=[2011, 2012]
      ↓
    第1次動作: 從 port 2011 取1格 → 放到 AGV 車指定層
      ↓
    第2次動作: 從 port 2012 取1格 → 放到 AGV 車另一層
      ↓
    任務完成: 總共處理 2格 Carrier
    ```
  - **特殊設備** (Soaker 浸潤機/泡藥機):
    - TAFL 任務定義: `batch_size = 1` (每個 Station 只有1個 port)
    - 機械臂能力: 一次取放動作處理 **1格** Carrier
    - 執行邏輯: **一次任務執行一次取放動作** (1任務 = 1動作)
    - 原因: 6台泡藥機（A-F）各自獨立，每台只有1個 port
    ```
    TAFL 任務: batch_size=1, ports=[2041]
      ↓
    第1次動作: 從 port 2041 取1格 → 放到 AGV 車指定層
      ↓
    任務完成: 總共處理 1格 Carrier
    ```
  - **與 Unloader AGV 對比**:
    - Loader AGV 標準設備: 一次任務 = 兩次動作 (batch_size=2, 機械臂1格×2次=2格)
    - Loader AGV 特殊設備: 一次任務 = 一次動作 (batch_size=1, 機械臂1格×1次=1格)
    - Unloader AGV: 一次任務 = 兩次動作 (batch_size=4, 機械臂2格×2次=4格)
  - **設計優勢**:
    - 靈活的 batch_size 設計，適應不同設備的 port 配置
    - 前段製程精密操作，機械臂一次只能處理1格
    - 一次任務處理一個 Station 的所有 port
    - 支援複雜的製程流程（清潔→浸潤→預乾燥，可能多次浸潤）
- **3層狀態機**: Base → AGV → Robot 層級控制
- **PGNO系統**: 使用程式編號(Program Number)控制機械臂精確動作
- **自動端口管理**: 動態AGV端口狀態管理和分配
- **參數化控制**: LoaderRobotParameter 提供靈活的設備配置
- **完整測試覆蓋**: 具備完整的測試套件和測試報告

## 📂 關鍵檔案位置

### 核心控制
```
loader_agv/
├─ agv_core_node.py          # Loader AGV 核心控制節點
├─ loader_context.py         # Loader 狀態管理上下文 (AGV層)
├─ robot_context.py          # 機器人狀態控制 (Robot層)
└─ launch/launch.py          # ROS 2 啟動配置
```

### 完整測試套件
```
test/
├─ TEST_REPORT.md                          # 詳細測試報告
├─ conftest.py                             # 測試配置和fixtures
├─ run_tests.py                            # 測試運行器
├─ test_take_transfer_integration.py       # Take Transfer完整流程測試
├─ test_agv_port_check_empty_state.py      # AGV端口檢查測試
├─ test_transfer_check_have_state.py       # 傳送箱載具檢查測試
├─ test_transfer_vision_position_state.py  # 傳送箱視覺定位測試
├─ test_take_transfer_state.py             # 取料轉移測試
└─ test_put_agv_state.py                   # AGV放料測試
```

## 🚀 Loader AGV 專用測試

### 車型特定測試
```bash
# 【推薦方式】透過上層工作空間工具  
# 參考: ../CLAUDE.md 開發環境設定

# 【直接測試】Loader AGV 功能
cd /app/agv_ws/src/loader_agv
python3 test/run_tests.py    # 完整測試套件運行器
```

## 🧪 Loader AGV 專項功能測試

### Work ID 路由系統測試
```bash
# 7種業務流程完整測試
python3 -m pytest test/test_take_transfer_integration.py -v      # Take Transfer 完整流程
python3 -m pytest test/test_agv_port_check_empty_state.py -v     # AGV 端口狀態檢查  
python3 -m pytest test/test_transfer_vision_position_state.py -v # 視覺定位精度測試
python3 -m pytest test/test_take_transfer_state.py -v            # 取料轉移邏輯
python3 -m pytest test/test_put_agv_state.py -v                  # AGV 放料邏輯

# 查看完整測試報告
cat test/TEST_REPORT.md

# Demo 快速驗證
python3 test/test_demo.py
python3 test/simple_test_runner.py
```

## 📊 配置設定

### AGV 配置檔案
- `/app/config/agv/loader01_config.yaml` - Loader01 配置
- `/app/config/agv/loader02_config.yaml` - Loader02 配置

### 關鍵配置參數
```yaml
agv_id: "loader01"
agv_type: "loader"
robot_arm_enabled: true
vision_system_enabled: true

# 端口配置
agv_ports:
  port_1: {x: 0.5, y: 0.3, z: 0.1}
  port_2: {x: 0.5, y: -0.3, z: 0.1}

# 工位配置
stations:
  transfer_box: {x: 5.0, y: 2.0, approach_angle: 0.0}
  cleaner: {x: 8.0, y: 1.5, approach_angle: 1.57}
  soaker: {x: 11.0, y: 1.5, approach_angle: 1.57}
  pre_dryer: {x: 14.0, y: 1.5, approach_angle: 1.57}
```

## 🔗 系統整合

### ROS 2 通訊
```bash
# 發布話題
/<agv_id>/status              # Loader AGV 狀態
/<agv_id>/robot_state         # 機器人狀態
/<agv_id>/vision_result       # 視覺定位結果
/<agv_id>/loading_status      # 載料過程狀態

# 訂閱話題
/<agv_id>/cmd                 # 任務指令
/system/stations_status       # 工位狀態
```

### 外部整合
- **agv_base**: 繼承 3層狀態機架構
- **plc_proxy_ws**: 機器人手臂 PLC 控制
- **sensorpart_ws**: 視覺定位和感測器整合

## 🚨 Loader AGV 專項故障排除

**⚠️ 通用故障排除請參考**: ../CLAUDE.md 故障排除章節

### Loader AGV 特有問題
- **Work ID 路由異常**: 檢查 7種業務流程的 work_id 格式和路由邏輯
- **AGV 端口管理錯誤**: 驗證 port1-4 的動態狀態管理和分配
- **多工位操作問題**: 檢查 Transfer、Cleaner、Soaker、Pre-dryer 工位整合
- **視覺定位精度問題**: 驗證視覺定位系統和載料精度