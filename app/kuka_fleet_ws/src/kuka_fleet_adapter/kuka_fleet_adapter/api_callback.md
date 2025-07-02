# API 規格說明：missionStateCallback

## 說明

此 API 用於將任務狀態從 Kuka 系統回報給上游系統，包含任務代碼、容器資訊、當前位置與任務狀態等。

---

## 基本資訊

- **API 名稱**：`missionStateCallback`
- **API URL**：`http://[IP:Port]/interfaces/api/amr/missionStateCallback`
- **HTTP 方法**：`POST`
- **Content-Type**：`application/json`

---

## 請求參數（JSON 格式）

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

---

## missionStatus 狀態說明

以下為任務狀態（`missionStatus`）可能值：

- `"MOVE_BEGIN"`：開始移動
- `"ARRIVED"`：到達任務節點
- `"UP_CONTAINER"`：升箱完成
- `"DOWN_CONTAINER"`：放下完成
- `"ROLLER_RECEIVE"`：滾筒上料完成
- `"ROLLER_SEND"`：滾筒下料完成
- `"PICKER_RECEIVE"`：料箱取料完成
- `"PICKER_SEND"`：料箱下料完成
- `"FORK_UP"`：叉車叉取完成
- `"FORK_DOWN"`：叉車放下完成
- `"COMPLETED"`：任務完成
- `"CANCELED"`：任務取消完成

---

## 範例請求 JSON

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
