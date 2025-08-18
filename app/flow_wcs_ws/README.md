# Flow WCS v2 - Linear Flow-based WCS System

## 概述

Flow WCS v2 是一個基於線性流程的倉庫控制系統（WCS），取代了傳統的節點圖形式。新系統採用更直觀的線性工作流格式，提供更好的可讀性、可維護性和執行效率。

## 主要特點

### 線性流程格式 v2
- **Section-based Structure**: 以區段為基礎的工作流組織
- **Sequential Execution**: 清晰的順序執行邏輯
- **Variable Resolution**: 強大的變數解析系統 (`${variable_name}`)
- **Conditional Execution**: 支援 `skip_if` 和 `skip_if_not` 條件執行
- **Loop Support**: 支援 `foreach` 迴圈處理
- **Parallel Branches**: 支援平行分支執行

### 與舊系統的差異
| 特性 | 舊版 (Node-based) | 新版 (Linear Flow v2) |
|------|------------------|---------------------|
| 結構 | 節點和連接的圖形 | 線性的區段和步驟 |
| 可讀性 | 需要理解圖形結構 | 直觀的順序流程 |
| 維護性 | 複雜的連接管理 | 簡單的步驟管理 |
| 執行邏輯 | 基於圖遍歷 | 基於順序執行 |
| 條件控制 | 通過連接條件 | 內建 skip_if 指令 |

## 系統架構

```
flow_wcs_ws/
├── src/
│   └── flow_wcs/
│       ├── flow_wcs/
│       │   ├── flow_wcs_node.py      # 主節點
│       │   ├── flow_executor.py      # 執行引擎
│       │   ├── flow_monitor.py       # 監控服務
│       │   ├── flow_validator.py     # 驗證器
│       │   └── functions/             # 內建函數
│       ├── launch/
│       │   └── flow_wcs_launch.py    # Launch 文件
│       └── test/
│           └── test_flow_executor.py  # 測試套件
├── deploy.sh                          # 部署腳本
└── test_integration.py                # 整合測試
```

## 流程檔案格式

### 基本結構
```yaml
meta:
  system: linear_flow_v2
  version: "2.0.0"
  author: "Flow Designer"
  description: "流程描述"

flow:
  id: "flow_id"
  name: "流程名稱"
  work_id: "220001"
  enabled: true
  priority: 100

workflow:
  - section: "區段名稱"
    description: "區段描述"
    steps:
      - id: "step_id"
        exec: "function.name"
        params:
          key: value
        store: "variable_name"
        skip_if: "${condition}"
```

### 實際範例
參考 `/app/config/wcs/flows/` 目錄下的範例檔案：
- `rack_rotation_room_inlet.yaml` - 房間入口架台輪轉流程 (work_id: 220001)
- `rack_rotation_room_outlet.yaml` - 房間出口架台輪轉流程 (work_id: 220002)

## 快速開始

### 1. 部署系統

```bash
# 進入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 執行完整部署
cd /app/flow_wcs_ws
./deploy.sh full

# 或分步執行
./deploy.sh build    # 建置
./deploy.sh test     # 測試
./deploy.sh deploy   # 部署流程檔案
./deploy.sh start    # 啟動系統
```

### 2. 檢查狀態

```bash
# 檢查系統狀態
./deploy.sh status

# 查看 ROS 2 主題
ros2 topic list | grep flow_wcs

# 監控流程執行
ros2 topic echo /flow_wcs/status
```

### 3. 使用 Linear Flow Designer

開啟瀏覽器訪問 Linear Flow Designer：
```
http://localhost:8001/linear-flow/designer
```

功能：
- 視覺化編輯流程
- YAML 編輯器
- 流程驗證
- 匯入/匯出功能

## 內建函數庫

### Query Functions
- `query.locations` - 查詢位置
- `query.racks` - 查詢架台
- `query.tasks` - 查詢任務
- `query.agvs` - 查詢 AGV

### Check Functions
- `check.empty` - 檢查資料是否為空
- `check.task_exists` - 檢查任務是否存在
- `check.rack_status` - 檢查架台狀態
- `check.system_ready` - 檢查系統就緒

### Task Functions
- `task.create` - 創建任務
- `task.update` - 更新任務
- `task.assign` - 分配任務
- `task.cancel` - 取消任務

### Action Functions
- `action.log` - 記錄日誌
- `action.notify` - 發送通知
- `action.rotate_rack` - 旋轉架台
- `action.optimize_batch` - 優化批次

### Control Functions
- `control.wait` - 等待
- `control.stop` - 停止流程
- `control.count` - 計數
- `control.switch` - 分支控制

### Special Functions
- `foreach` - 迴圈處理
- `parallel` - 平行執行

## 測試

### 單元測試
```bash
cd /app/flow_wcs_ws
python3 -m pytest src/flow_wcs/test/ -v
```

### 整合測試
```bash
cd /app/flow_wcs_ws
python3 test_integration.py
```

## 開發指南

### 新增自定義函數

1. 在 `src/flow_wcs/flow_wcs/functions/` 建立新函數模組
2. 實作函數邏輯
3. 在 `flow_executor.py` 中註冊函數

```python
# functions/custom.py
async def custom_function(context, params):
    """自定義函數實作"""
    # 實作邏輯
    return result

# 在 flow_executor.py 註冊
self.functions['custom.function'] = custom_function
```

### 建立新流程

1. 使用 Linear Flow Designer 建立流程
2. 或手動建立 YAML 檔案到 `/app/config/wcs/flows/`
3. 確保設定正確的 `work_id`
4. 重啟 Flow WCS 以載入新流程

## 故障排除

### 流程未執行
- 檢查流程檔案的 `enabled` 欄位是否為 `true`
- 確認 `work_id` 設定正確
- 查看日誌：`ros2 topic echo /flow_wcs/events`

### 變數解析錯誤
- 確認變數名稱正確（使用 `${variable_name}` 格式）
- 檢查變數是否在之前的步驟中已儲存
- 使用 Flow Monitor 檢查執行上下文

### 系統未啟動
- 確認在 AGVC 容器內執行
- 檢查 ROS 2 環境是否正確載入
- 使用 `./deploy.sh status` 檢查系統狀態

## API 整合

Flow WCS 提供 ROS 2 服務介面：

```python
# 執行流程
ros2 service call /flow_wcs/execute_flow flow_interfaces/srv/ExecuteFlow "{flow_id: 'rack_rotation_inlet'}"

# 停止流程
ros2 service call /flow_wcs/stop_flow flow_interfaces/srv/StopFlow "{flow_id: 'rack_rotation_inlet'}"

# 查詢狀態
ros2 service call /flow_wcs/get_status flow_interfaces/srv/GetFlowStatus "{flow_id: 'rack_rotation_inlet'}"
```

## 許可證

本專案為擎添工業專有軟體。

## 支援

如有問題，請聯繫開發團隊或查閱內部文檔。