# CT Manager 智能任務分派機制實作總結

## 概述

本文件總結了 `ct_manager.py` 中實作的基於房間和車型的智能任務分派機制。該系統能夠根據任務的房間位置和所需車型，自動選擇合適的 AGV 進行任務派發。

## 系統架構

### 支援的 AGV 車型
- **Cargo**：負責房外作業（運輸、搬運等）
- **Loader**：負責房內裝載作業
- **Unloader**：負責房內卸載作業

### 資料庫結構
- **task 表**：包含 `room_id`（區分房間）和 `parameters['model']`（指定所需車型）
- **AGV 表**：車輛命名規則為 `{車型}{房間編號}`（例：Loader02、Unloader01、Cargo02）
- **狀態定義**：
  - 任務狀態：`status_id=1`（待執行）、`status_id=2`（執行中）、`status_id=3`（已完成）
  - AGV 狀態：`status_id=3`（閒置）、`status_id=4`（任務中）

## 分派邏輯規則

### 1. 房內任務分派（room_id 不為 None）
- `room_id=2` 且 `parameters['model']='Loader'` → 派給 `Loader02`
- `room_id=1` 且 `parameters['model']='Unloader'` → 派給 `Unloader01`
- 命名規則：`{車型}{房間編號:02d}`

### 2. 房外任務分派（room_id 為 None）
- `room_id=None` 且 `parameters['model']='Cargo'` → 派給 `Cargo02`
- 目前規劃：1 台 Cargo 負責 1 個房間的房外任務

## 核心方法說明

### `dispatch()`
主要的任務派發方法，執行以下步驟：
1. 查詢待執行的 CT 任務（按優先級降序排列）
2. 逐一處理每個任務
3. 驗證任務參數
4. 選擇合適的 AGV
5. 檢查 AGV 閒置狀態
6. 執行任務派發
7. 提交資料庫變更

### `_validate_task_parameters(task)`
驗證任務參數是否完整：
- 檢查 `parameters` 欄位存在
- 檢查 `model` 參數存在且有效
- 支援的車型：Cargo、Loader、Unloader

### `_select_agv_for_task(session, task)`
根據房間和車型選擇合適的 AGV：
- 呼叫 `_determine_target_agv_name()` 決定目標 AGV 名稱
- 查詢資料庫中對應的 AGV
- 確保 AGV 已啟用（`enable=1`）

### `_determine_target_agv_name(model, room_id)`
根據車型和房間 ID 決定目標 AGV 名稱：
- 房外任務（`room_id=None`）：只允許 Cargo 車型
- 房內任務（`room_id≠None`）：允許 Loader 和 Unloader 車型
- 返回格式化的 AGV 名稱

### `_assign_task_to_agv(session, task, agv)`
執行實際的任務派發：
- 生成唯一的任務代碼（UUID）
- 更新任務資訊（`agv_id`、`mission_code`、`status_id`、`updated_at`）
- 更新 AGV 狀態為任務中（`status_id=4`）
- 記錄派發詳情

## 異常處理機制

### 前置檢查
- 資料庫連線池可用性檢查
- 任務參數完整性驗證
- AGV 存在性和狀態檢查

### 錯誤處理
- 找不到對應名稱的 AGV
- 目標 AGV 非閒置狀態
- 任務參數不完整或格式錯誤
- 資料庫操作異常

### 日誌記錄
- 詳細的派發過程記錄
- 錯誤和警告訊息
- 任務派發成功/失敗狀態

## 擴展性設計

### 車輛數量擴充
- 支援多台 Cargo 車輛（原則：1 台 Cargo 負責 1 個房間）
- 房間編號格式化為兩位數，支援最多 99 個房間
- 車型命名規則可彈性調整

### 新增車型支援
- 在 `_validate_task_parameters()` 中添加新車型
- 在 `_determine_target_agv_name()` 中添加對應邏輯
- 更新資料庫初始化資料

## 輔助功能

### `get_available_agvs()`
取得可用的 CT AGV 列表：
- 查詢閒置且啟用的非 KUKA AGV
- 返回可派發的 AGV 清單

### `update_agv_status(agv_id, status)`
更新 AGV 狀態：
- 支援手動狀態更新
- 包含狀態變更日誌記錄

### `get_task_statistics()`
取得任務統計資訊：
- 統計各狀態的任務數量
- 提供系統運行狀況概覽

## 程式碼品質

### 可讀性
- 詳細的方法註解和文件字串
- 清晰的變數命名和邏輯結構
- 分層的方法設計

### 可維護性
- 模組化的方法設計
- 統一的錯誤處理機制
- 完整的日誌記錄

### 可測試性
- 獨立的驗證方法
- 模擬測試環境支援
- 詳細的測試覆蓋

## 部署注意事項

1. **資料庫準備**：確保 AGV 和任務資料已正確初始化
2. **狀態同步**：確保 AGV 實際狀態與資料庫狀態一致
3. **監控機制**：建議添加任務派發監控和告警
4. **效能考量**：大量任務時考慮批次處理和索引優化

## 未來改進方向

1. **動態負載平衡**：根據 AGV 工作負載動態調整派發策略
2. **優先級算法**：更複雜的任務優先級計算邏輯
3. **故障恢復**：AGV 故障時的任務重新分派機制
4. **效能監控**：任務執行時間和效率統計分析
