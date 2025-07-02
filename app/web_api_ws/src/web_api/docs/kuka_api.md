# Kuka API 說明文件

## 概述

本文件說明為 Kuka 系統提供的 API 端點，用於接收 Kuka 系統的任務狀態回報並更新資料庫中的任務資訊。

## API 端點

### POST /interfaces/api/amr/missionStateCallback

接收 Kuka 系統的任務狀態回報。

**URL**: `http://[IP:Port]/interfaces/api/amr/missionStateCallback`

**HTTP 方法**: `POST`

**Content-Type**: `application/json`

## 請求參數

| 參數名稱         | 類型    | 必填 | 最大長度 | 說明 |
|------------------|---------|------|-----------|------|
| `missionCode`    | String  | 是   | 32        | 任務代碼 ID |
| `viewBoardType`  | String  | 否   | -         | 任務類型 |
| `containerCode`  | String  | 否   | -         | 容器代碼 |
| `currentPosition`| String  | 否   | -         | 容器當前位置 |
| `slotCode`       | String  | 否   | -         | 所在槽位 |
| `robotId`        | String  | 否   | -         | 執行任務的機器人 ID |
| `missionStatus`  | String  | 是   | -         | 任務狀態（見下方說明） |
| `message`        | String  | 否   | -         | 補充說明 |
| `missionData`    | Object  | 否   | -         | 任務自訂資料（可為空物件） |

## 任務狀態說明

以下為任務狀態（`missionStatus`）可能值及其對應的資料庫狀態更新：

| Kuka 狀態 | 說明 | 資料庫狀態 ID | 資料庫狀態名稱 |
|-----------|------|---------------|----------------|
| `MOVE_BEGIN` | 開始移動 | 2 | 執行中 |
| `ARRIVED` | 到達任務節點 | 2 | 執行中 |
| `UP_CONTAINER` | 升箱完成 | 2 | 執行中 |
| `DOWN_CONTAINER` | 放下完成 | 2 | 執行中 |
| `ROLLER_RECEIVE` | 滾筒上料完成 | 2 | 執行中 |
| `ROLLER_SEND` | 滾筒下料完成 | 2 | 執行中 |
| `PICKER_RECEIVE` | 料箱取料完成 | 2 | 執行中 |
| `PICKER_SEND` | 料箱下料完成 | 2 | 執行中 |
| `FORK_UP` | 叉車叉取完成 | 2 | 執行中 |
| `FORK_DOWN` | 叉車放下完成 | 2 | 執行中 |
| `COMPLETED` | 任務完成 | 3 | 已完成 |
| `CANCELED` | 任務取消完成 | 1 | 待執行 |

## 請求範例

```json
{
  "missionCode": "mission202309250005",
  "viewBoardType": "",
  "slotCode": "",
  "robotId": "14",
  "containerCode": "1000002",
  "currentPosition": "M001-A001-31",
  "missionStatus": "MOVE_BEGIN",
  "message": "",
  "missionData": {}
}
```

## 回應格式

### 成功回應

```json
{
  "success": true,
  "message": "Mission state callback processed successfully",
  "task_id": 123,
  "mission_code": "mission202309250005",
  "mission_status": "MOVE_BEGIN"
}
```

### 錯誤回應

#### 任務不存在 (404)

```json
{
  "detail": "Task not found for missionCode: mission202309250005"
}
```

#### 伺服器錯誤 (500)

```json
{
  "detail": "Error processing mission state callback: [錯誤詳情]"
}
```

## 資料庫更新說明

當 API 接收到 Kuka 的狀態回報時，會執行以下操作：

1. **查找任務**: 根據 `missionCode` 查找對應的任務
   - 使用 `Task.mission_code` 欄位進行精確匹配查找

2. **更新任務參數**: 將 Kuka 回報的所有資訊存入 `Task.parameters` 欄位，包括：
   - `kuka_mission_status`: Kuka 任務狀態
   - `kuka_robot_id`: 機器人 ID
   - `kuka_container_code`: 容器代碼
   - `kuka_current_position`: 當前位置
   - `kuka_slot_code`: 槽位代碼
   - `kuka_view_board_type`: 任務類型
   - `kuka_message`: 補充說明
   - `kuka_mission_data`: 任務自訂資料
   - `kuka_last_update`: 最後更新時間

3. **更新任務狀態**: 根據 `missionStatus` 更新 `Task.status_id`

4. **更新時間戳**: 更新 `Task.updated_at` 為當前時間

## 資料庫結構更新

為了支援 Kuka API，Task 資料表已新增以下欄位：

- `mission_code` (VARCHAR): 用於存儲 Kuka 系統的任務代碼，對應 API 中的 `missionCode` 參數

## 測試

可以使用提供的測試腳本來測試 API：

```bash
# 創建測試任務
python tests/create_test_task.py

# 測試 API
python tests/test_kuka_api.py
```

## 注意事項

1. 確保 web_api 伺服器正在運行（預設端口 8000）
2. 確保資料庫連接正常
3. 確保 `missionCode` 對應的任務在資料庫中存在，且 `Task.mission_code` 欄位已正確設定
4. 所有 Kuka 回報的資訊都會保存在任務的 `parameters` 欄位中，以 JSON 格式存儲
5. 在創建任務時，請務必設定 `mission_code` 欄位，以便 Kuka 系統能夠正確回報狀態
