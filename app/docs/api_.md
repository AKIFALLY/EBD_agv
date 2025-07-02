# KUKA 標準接口 API 文件（整理版）

---

## 1. submitMission - 任務下發

### 說明
下發 AMR 任務到 Kuka 系統，支援多種任務類型（RACK_MOVE、ROLLER_MOVE、PICKER_MOVE、FORKLIFT_MOVE、MOVE、ROBOTICS_MOVE）。

### 基本資訊
- **API 名稱**：submitMission
- **API URL**：`http://[IP:Port]/interfaces/api/amr/submitMission`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱               | 類型      | 必填 | 最大長度 | 說明                         |
|------------------------|-----------|------|----------|------------------------------|
| orgId                  | String    | 是   | -        | 組織 ID                      |
| requestId              | String    | 是   | 32       | 請求唯一識別碼               |
| missionCode            | String    | 是   | 32       | 任務代碼                     |
| missionType            | String    | 是   | -        | 任務類型（見下方說明）       |
| viewBoardType          | String    | 否   | -        | 任務類型                     |
| robotModels            | List      | 是   | -        | 機器人型號列表               |
| robotIds               | List      | 是   | -        | 機器人 ID 列表               |
| robotType              | String    | 是   | -        | 機器人類型                   |
| priority               | Integer   | 否   | -        | 優先級（1-99）               |
| containerModelCode     | String    | 否   | -        | 容器模型編碼                 |
| containerCode          | String    | 否   | -        | 容器編碼                     |
| templateCode           | String    | 否   | -        | 流程模板編碼                 |
| lockRobotAfterFinish   | Boolean   | 否   | -        | 任務結束後是否鎖定機器人     |
| unlockRobotId          | String    | 否   | -        | 解鎖機器人 ID                |
| unlockMissionCode      | String    | 否   | -        | 解鎖任務代碼                 |
| idleNode               | String    | 否   | -        | 指定機器人停放區域/點        |
| missionData            | List      | 是   | -        | 任務流程節點詳情（見下方）   |

#### missionType 任務類型說明
- RACK_MOVE：貨架搬運
- ROLLER_MOVE：滾筒車搬運
- PICKER_MOVE：料箱搬運
- FORKLIFT_MOVE：叉車搬運
- MOVE：機器人移動
- ROBOTICS_MOVE：複合機器人任務

#### missionData 子欄位（依 missionType 不同）
- **RACK_MOVE**：
  | 參數名稱      | 類型    | 必填 | 說明                 |
  |---------------|---------|------|----------------------|
  | sequence      | Integer | 是   | 節點序號             |
  | position      | String  | 是   | 作業路徑位置         |
  | type         | String  | 是   | 位置型別（NODE_POINT/NODE_AREA）|
  | putDown      | Boolean | 否   | 是否需放下貨架        |
  | passStrategy | String  | 是   | 行策略（AUTO/MANUAL）|
  | waitingMillis| Integer | 否   | 等待時間（ms）        |
- **ROLLER_MOVE**：
  | 參數名稱      | 類型    | 必填 | 說明                 |
  |---------------|---------|------|----------------------|
  | sequence      | Integer | 是   | 節點序號             |
  | position      | String  | 是   | 作業路徑位置         |
  | type         | String  | 是   | 位置型別             |
  | actionType   | String  | 是   | 動作型別（ROLLER_RECEIVE/ROLLER_SEND）|
  | binCode      | String  | 否   | 料箱編碼             |
  | rollerLevel  | Integer | 否   | 滾筒層級             |
  | deviceCode   | String  | 否   | 設備編碼             |
  | actionConfirm| Boolean | 否   | 上下料前是否需確認    |
  | actionInform | Boolean | 否   | 上下料後是否需通知    |
- **PICKER_MOVE**：
  | 參數名稱         | 類型    | 必填 | 說明                 |
  |------------------|---------|------|----------------------|
  | sequence         | Integer | 是   | 節點序號             |
  | binCode          | String  | 是   | 料箱編碼             |
  | startPosition    | String  | 是   | 起始位置             |
  | startSlotCode    | String  | 否   | 起始槽位             |
  | takeActionConfirm| Boolean | 否   | 取料前需確認         |
  | takeActionInform | Boolean | 否   | 取料後需通知         |
  | endPosition      | String  | 是   | 結束位置             |
  | endSlotCode      | String  | 否   | 結束槽位             |
  | putActionConfirm | Boolean | 否   | 放料前需確認         |
  | putActionInform  | Boolean | 否   | 放料後需通知         |
- **FORKLIFT_MOVE**：
  | 參數名稱      | 類型    | 必填 | 說明                 |
  |---------------|---------|------|----------------------|
  | sequence      | Integer | 是   | 節點序號             |
  | position      | String  | 是   | 作業路徑位置         |
  | stackNumber  | Integer | 否   | 堆疊數               |
  | actionConfirm| Boolean | 否   | 動作前需確認         |
- **MOVE**：
  | 參數名稱      | 類型    | 必填 | 說明                 |
  |---------------|---------|------|----------------------|
  | sequence      | Integer | 是   | 節點序號             |
  | position      | String  | 是   | 作業路徑位置         |
  | type         | String  | 是   | 位置型別             |
  | passStrategy | String  | 是   | 行策略（AUTO/MANUAL）|
  | waitingMillis| Integer | 否   | 等待時間（ms）        |
- **ROBOTICS_MOVE**：
  | 參數名稱         | 類型    | 必填 | 說明                 |
  |------------------|---------|------|----------------------|
  | sequence         | Integer | 是   | 節點序號             |
  | position         | String  | 是   | 作業路徑位置         |
  | type             | String  | 是   | 位置型別             |
  | applicationName  | String  | 是   | 應用名稱             |
  | params           | String  | 否   | 參數（JSON 字串）    |
  | passStrategy     | String  | 是   | 行策略（AUTO/MANUAL）|
  | waitingMillis    | Integer | 否   | 等待時間（ms）        |

### 請求範例
```json
{
  "orgid": "UNIVERSAL",
  "requestid": "request202309250001",
  "missionCode": "mission202309250001",
  "missionType": "RACK_MOVE",
  "viewBoardType": "",
  "robotModels": ["KMP600I"],
  "robotlds": ["44"],
  "robotType": "LIFT",
  "priority": 1,
  "containerModelCode": "10001",
  "containerCode": "1000002",
  "templateCode": "",
  "lockRobotAfterFinish": false,
  "unlockRobotld": "",
  "unlockMissionCode": "",
  "idleNode": "A000000013",
  "missionData": [
    {
      "sequence": 1,
      "position": "M001-A001-45",
      "type": "NODE_POINT",
      "putDown": false,
      "passStrategy": "AUTO",
      "waitingMillis": 0
    },
    {
      "sequence": 2,
      "position": "M001-A001-40",
      "type": "NODE_POINT",
      "putDown": true,
      "passStrategy": "AUTO",
      "waitingMillis": 0
    }
  ]
}
```

### 回應範例
```json
{
  "data": null,
  "code": "0",
  "message": null,
  "success": true
}
```

---

## 2. missionCancel - 任務取消

### 說明
取消已下發的 AMR 任務。

### 基本資訊
- **API 名稱**：missionCancel
- **API URL**：`http://[IP:Port]/interfaces/api/amr/missionCancel`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱     | 類型   | 必填 | 說明                       |
|--------------|--------|------|----------------------------|
| requestId    | String | 是   | 請求唯一識別碼             |
| missionCode  | String | 是   | 任務代碼                   |
| containerCode| String | 否   | 容器編碼                   |
| position     | String | 否   | 當前位置                   |
| cancelMode   | String | 是   | FORCE/NORMAL/REDIRECT_END/REDIRECT_START |
| reason       | String | 否   | 取消原因                   |

### 請求範例
```json
{
  "requestid": "request202309250006",
  "missionCode": "mission202309250004",
  "containerCode": "",
  "position": "",
  "cancelMode": "FORCE",
  "reason": ""
}
```

### 回應範例
```json
{
  "data": null,
  "code": "0",
  "message": null,
  "success": true
}
```

---

## 3. operationFeedback - 任務放行

### 說明
節點任務完成後，上游對任務完成信息的反饋。

### 基本資訊
- **API 名稱**：operationFeedback
- **API URL**：`http://[IP:Port]/interfaces/api/amr/operationFeedback`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱     | 類型   | 必填 | 說明                       |
|--------------|--------|------|----------------------------|
| requestId    | String | 是   | 請求唯一識別碼             |
| missionCode  | String | 是   | 任務代碼                   |
| containerCode| String | 否   | 容器編碼                   |
| position     | String | 否   | 當前執行作業的節點         |

### 請求範例
```json
{
  "requestid": "request202309250007",
  "containerCode": "",
  "missionCode": "mission202309250005",
  "position": ""
}
```

### 回應範例
```json
{
  "data": null,
  "code": "0",
  "message": null,
  "success": true
}
```

---

## 4. jobQuery - 作業看板查詢

### 說明
查詢作業看板上的任務。

### 基本資訊
- **API 名稱**：jobQuery
- **API URL**：`http://[IP:Port]/interfaces/api/amr/jobQuery`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱     | 類型   | 必填 | 說明                       |
|--------------|--------|------|----------------------------|
| containerCode| String | 否   | 容器編碼                   |
| createUsername| String| 否   | 建立者                     |
| jobCode      | String | 否   | 任務編碼                   |
| limit        | Int    | 否   | 返回數量                   |
| maps         | List   | 否   | 地圖列表                   |
| robotId      | String | 否   | 機器人 ID                  |
| sourceValue  | Int    | 否   | 來源                       |
| status       | Int    | 否   | 狀態碼                     |
| targetCellCode|String | 否   | 目標點位                   |
| workflowCode | String | 否   | 流程編碼                   |
| workflowId   | Int    | 否   | 流程 ID                    |
| workflowName | String | 否   | 流程名稱                   |

### 請求範例
```json
{
  "containerCode": "C001",
  "createUsername": "admin",
  "jobCode": "T000096284",
  "limit": 10,
  "maps": ["TEST"],
  "robotld": "1",
  "sourceValue": 6,
  "status": 20,
  "targetCellCode": "TEST-1-90",
  "workflowCode": "W000000587",
  "workflowld": 100218,
  "workflowName": "Carry01"
}
```

### 回應範例
```json
{
  "data": [
    {
      "jobCode": "T000096284",
      "workflowld": 100218,
      "containerCode": "C001",
      "robotld": "1",
      "status": 20,
      "workflowName": "Carry01",
      "workflowCode": "W000000587",
      "workflowPriority": 1,
      "mapCode": "TEST",
      "targetCellCode": "TEST-1-90",
      "beginCellCode": "TEST-1-80",
      "targetCellCodeForeign": "DROPPOINT",
      "beginCellCodeForeign": "PICKPOINT",
      "finalNodeCode": "TEST-1-90",
      "warnFlag": 0,
      "warnCode": null,
      "completeTime": null,
      "spendTime": null,
      "createUsername": "admin",
      "createTime": "2025-01-10 16:01:42",
      "source": "SELF",
      "materialsinfo": "-"
    }
  ],
  "code": "0",
  "message": null,
  "success": true
}
```

---

## 5. containerin - 容器入場

### 說明
將容器入場到 Kuka 系統。

### 基本資訊
- **API 名稱**：containerin
- **API URL**：`http://[IP:Port]/interfaces/api/amr/containerin`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱     | 類型   | 必填 | 說明                       |
|--------------|--------|------|----------------------------|
| requestId    | String | 是   | 請求唯一識別碼             |
| containerType| String | 是   | 容器類型（RACK/BIN）       |
| containerCode| String | 是   | 容器編碼                   |
| position     | String | 否   | 位置                       |
| containerModelCode|String|否  | 容器模型編碼               |
| enterOrientation|String|否   | 入場方向                   |
| isNew        | Boolean| 否   | 是否為新增容器             |
| containerValidationCode|String|否| 容器校驗碼                |
| withDefaultValidationCode|Boolean|否| 配置容器預設校驗碼      |

### 請求範例
```json
{
  "requestid": "request202309250008",
  "containerType": "",
  "containerCode": "10",
  "position": "M001-A001-31",
  "containerModelCode": "",
  "enterOrientation": "",
  "isNew": false
}
```

### 回應範例
```json
{
  "data": null,
  "code": "0",
  "message": null,
  "success": true
}
```

---

## 6. containerout - 容器出場

### 說明
將容器從 Kuka 系統出場。

### 基本資訊
- **API 名稱**：containerout
- **API URL**：`http://[IP:Port]/interfaces/api/amr/containerout`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱     | 類型   | 必填 | 說明                       |
|--------------|--------|------|----------------------------|
| requestId    | String | 是   | 請求唯一識別碼             |
| containerCode| String | 是   | 容器編碼                   |
| position     | String | 否   | 位置                       |
| isDelete     | Boolean| 否   | 是否刪除容器資料           |

### 請求範例
```json
{
  "requestid": "request202309250009",
  "containerType": "",
  "containerCode": "10",
  "position": "M001-A001-31",
  "isDelete": false
}
```

### 回應範例
```json
{
  "data": null,
  "code": "0",
  "message": null,
  "success": true
}
```

---

## 7. updateContainer - 容器信息更新

### 說明
更新容器位置和空滿狀態。

### 基本資訊
- **API 名稱**：updateContainer
- **API URL**：`http://[IP:Port]/interfaces/api/amr/updateContainer`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱     | 類型   | 必填 | 說明                       |
|--------------|--------|------|----------------------------|
| requestId    | String | 是   | 請求唯一識別碼             |
| containerCode| String | 是   | 容器編碼                   |
| containerType| String | 否   | 容器類型                   |
| originPosition|String | 否   | 原始位置                   |
| targetPosition|String | 否   | 目標位置                   |
| emptyStatus  | String | 否   | 空滿狀態（EMPTY/FULL）     |
| reason       | String | 否   | 更新原因                   |
| position     | String | 否   | 位置（兼容舊欄位）         |
| isFull       | Boolean| 否   | 是否滿（兼容舊欄位）       |

### 請求範例
```json
{
  "requestid": "request202309250010",
  "containerType": "BUCKET",
  "containerCode": "10",
  "originPosition": "M001-A001-31",
  "targetPosition": "M001-A001-30",
  "emptyStatus": "EMPTY",
  "reason": ""
}
```

### 回應範例
```json
{
  "data": null,
  "code": "0",
  "message": null,
  "success": true
}
```

---

## 8. missionStateCallback - 任務狀態回調

### 說明
Kuka 系統回報任務狀態給上游系統。

### 基本資訊
- **API 名稱**：missionStateCallback
- **API URL**：`http://[IP:Port]/interfaces/api/amr/missionStateCallback`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱         | 類型    | 必填 | 最大長度 | 說明 |
|------------------|---------|------|-----------|------|
| missionCode      | String  | 是   | 32        | 任務代碼 ID |
| viewBoardType    | String  | 否   | -         | 任務類型 |
| containerCode    | String  | 否   | -         | 容器代碼 |
| currentPosition  | String  | 否   | -         | 容器當前位置 |
| slotCode         | String  | 否   | -         | 所在槽位 |
| robotId          | String  | 否   | -         | 執行任務的機器人 ID |
| missionStatus    | String  | 是   | -         | 任務狀態（MOVE_BEGIN、ARRIVED、UP_CONTAINER、DOWN_CONTAINER、ROLLER_RECEIVE、ROLLER_SEND、PICKER_RECEIVE、PICKER_SEND、FORK_UP、FORK_DOWN、COMPLETED、CANCELED） |
| message          | String  | 否   | -         | 補充說明 |
| missionData      | Object  | 否   | -         | 任務自訂資料（可為空物件） |

### 請求範例
```json
{
  "missionCode": "mission202309250005",
  "viewBoardType": "",
  "slotCode": "",
  "robotld": "14",
  "containerCode": "1000002",
  "currentPosition": "M001-A001-31",
  "missionStatus": "MOVE_BEGIN",
  "message": "",
  "missionData": {}
}
```

### 回應範例
```json
{
  "data": null,
  "code": "0",
  "message": null,
  "success": true
}
```

---

## 9. queryAllContainerModelCode - 容器模型查詢

### 基本資訊
- **API 名稱**：queryAllContainerModelCode
- **API URL**：`http://[IP:Port]/interfaces/api/amr/queryAllContainerModelCode`
- **HTTP 方法**：GET
- **Content-Type**：application/json

### 請求參數
| 參數名稱 | 類型  | 必填 | 最大長度 | 說明         |
|----------|-------|------|----------|--------------|
| 無       | -     | -    | -        | 無           |

### 請求範例
```json
{}
```

### 回應範例
```json
{
  "code": "0",
  "message": null,
  "success": true,
  "data": [
    "10001"
  ]
}
```

---

## 10. queryAreaCodeForContainerModel - 查詢容器模型推薦可存放區域

### 基本資訊
- **API 名稱**：queryAreaCodeForContainerModel
- **API URL**：`http://[IP:Port]/interfaces/api/amr/queryAreaCodeForContainerModel`
- **HTTP 方法**：GET
- **Content-Type**：application/json

### 請求參數
| 參數名稱         | 類型    | 必填 | 最大長度 | 說明         |
|------------------|---------|------|----------|--------------|
| containerModelCode | String | 是   | -        | 容器模型編碼 |
| noContainerFirst  | Boolean| 否   | -        | 是否優先無容器區域 |

### 請求範例
```
GET /interfaces/api/amr/queryAreaCodeForContainerModel?containerModelCode=10001&noContainerFirst=false
```

### 回應範例
```json
{
  "code": "0",
  "message": "",
  "success": true,
  "data": [
    "areaCode1",
    "areaCode2",
    "areaCode3"
  ]
}
```

---

## 11. containerQuery - 容器信息查詢（僅查入場）

### 基本資訊
- **API 名稱**：containerQuery
- **API URL**：`http://[IP:Port]/interfaces/api/amr/containerQuery`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱           | 類型    | 必填 | 最大長度 | 說明                 |
|--------------------|---------|------|----------|----------------------|
| nodeCode           | String  | 否   | -        | 點位編碼             |
| containerModelCode | String  | 否   | -        | 容器模型編碼         |
| containerCode      | String  | 否   | -        | 容器編碼             |
| areaCode           | String  | 否   | -        | 區域編碼             |
| emptyFullStatus    | Integer | 否   | -        | 空滿狀態(0空/1滿/2全部) |
| isCarry            | Integer | 否   | -        | 搬運狀態(0靜止/1搬運中) |
| orientation        | String  | 否   | -        | 容器朝向             |
| containerCheckCode | String  | 否   | -        | 容器校驗碼           |
| mapCode            | String  | 否   | -        | 地圖編碼             |
| districtCode       | String  | 否   | -        | 地區編碼             |

### 請求範例
```json
{
  "nodeCode": "M001-A001-30",
  "containerModelCode": "10001",
  "containerCode": "10",
  "areaCode": "A000000014"
}
```

### 回應範例
```json
{
  "data": [
    {
      "containerCode": "10",
      "nodeCode": "",
      "orientation": "0.0"
    }
  ],
  "code": "0",
  "message": null,
  "success": true
}
```

---

## 12. containerQueryAll - 容器信息查詢（入場+離場）

### 基本資訊
- **API 名稱**：containerQueryAll
- **API URL**：`http://[IP:Port]/interfaces/api/amr/containerQueryAll`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱           | 類型    | 必填 | 最大長度 | 說明                 |
|--------------------|---------|------|----------|----------------------|
| mapCode            | String  | 否   | -        | 地圖編碼             |
| containerCode      | String  | 否   | -        | 容器編碼             |
| containerModelCode | String  | 否   | -        | 容器模型編碼         |
| emptyFullStatus    | Integer | 否   | -        | 空滿狀態(0空/1滿/2全部) |
| isCarry            | Integer | 否   | -        | 搬運狀態(0靜止/1搬運中) |
| orientation        | String  | 否   | -        | 容器朝向             |
| containerCheckCode | String  | 否   | -        | 容器校驗碼           |
| districtCode       | String  | 否   | -        | 地區編碼             |
| inMapStatus        | Integer | 否   | -        | 入場/離場狀態(0離場/1入場) |

### 請求範例
```json
{
  "mapCode": "M001",
  "containerCode": "10",
  "containerModelCode": "10001",
  "emptyFullStatus": 2,
  "isCarry": 0,
  "orientation": "0.0",
  "containerCheckCode": "CHK001",
  "districtCode": "D01",
  "inMapStatus": 1
}
```

### 回應範例
```json
{
  "data": [
    {
      "containerCode": "10",
      "nodeCode": "",
      "orientation": "0.0"
    }
  ],
  "code": "0",
  "message": null,
  "success": true
}
```

---

## 13. queryRobByNodeUuidOrForeignCode - 依點位查詢機器人

### 基本資訊
- **API 名稱**：queryRobByNodeUuidOrForeignCode
- **API URL**：`http://[IP:Port]/interfaces/api/amr/queryRobByNodeUuidOrForeignCode`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱         | 類型    | 必填 | 最大長度 | 說明         |
|------------------|---------|------|----------|--------------|
| nodeCode         | String  | 是   | -        | 點位編碼     |

### 請求範例
```json
{
  "nodeCode": "M001-A001-30"
}
```

### 回應範例
```json
{
  "code": "0",
  "message": null,
  "success": true,
  "data": [
    {
      "robotId": "R001",
      "robotType": "LIFT"
    }
  ]
}
```

---

## 14. robotQuery - 機器人信息查詢

### 基本資訊
- **API 名稱**：robotQuery
- **API URL**：`http://[IP:Port]/interfaces/api/amr/robotQuery`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱      | 類型   | 必填 | 最大長度 | 說明         |
|---------------|--------|------|----------|--------------|
| robotId       | String | 否   | -        | 機器人 ID    |
| mapCode       | String | 否   | -        | 地圖編碼     |
| floorNumber   | String | 否   | -        | 樓層編號     |
| status        | Integer| 否   | -        | 狀態碼       |
| occupyStatus  | Integer| 否   | -        | 是否占用     |
| batteryLevel  | Double | 否   | -        | 電量百分比   |
| nodeCode      | String | 否   | -        | 當前點位     |
| x             | String | 否   | -        | X座標        |
| y             | String | 否   | -        | Y座標        |
| robotOrientation | String | 否 | -        | 當前角度     |
| missionCode   | String | 否   | -        | 當前任務號   |
| liftStatus    | Integer| 否   | -        | 項升狀態     |
| reliability   | Integer| 否   | -        | 定位置信度   |
| runTime       | String | 否   | -        | 運行時長     |
| karOsVersion  | String | 否   | -        | 軟體版本     |
| mileage       | String | 否   | -        | 里程         |
| leftMotorTemperature | String | 否 | -    | 左電機溫度   |
| rightMotorTemperature| String | 否 | -    | 右電機溫度   |
| liftMotorTemperature | String | 否 | -    | 頂升電機溫度 |
| rotateMotorTemperature| String| 否 | -    | 旋轉電機溫度 |
| rotateTimes   | Integer| 否   | -        | 托盤旋轉次數 |
| liftTimes     | Integer| 否   | -        | 托盤頂升次數 |
| nodeForeignCode | String | 否 | -        | 外部編碼     |

### 請求範例
```json
{
  "robotId": "R001",
  "mapCode": "M001"
}
```

### 回應範例
```json
{
  "code": "0",
  "message": null,
  "success": true,
  "data": [
    {
      "robotId": "R001",
      "robotType": "LIFT"
    }
  ]
}
```

---

## 15. robotMoveCarry - 下發機器人移動搬運任務

### 基本資訊
- **API 名稱**：robotMoveCarry
- **API URL**：`http://[IP:Port]/interfaces/api/amr/robotMoveCarry`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱         | 類型    | 必填 | 最大長度 | 說明         |
|------------------|---------|------|----------|--------------|
| robotId          | String  | 是   | -        | 機器人 ID    |
| containerCode    | String  | 否   | -        | 容器編碼     |
| targetNodeCode   | String  | 是   | -        | 目標點位     |
| missionCode      | String  | 否   | -        | 任務編碼     |

### 請求範例
```json
{
  "robotId": "R001",
  "containerCode": "10",
  "targetNodeCode": "N001"
}
```

### 回應範例
```json
{
  "code": "0",
  "message": null,
  "success": true,
  "data": "mission202309250011"
}
```

---

## 16. chargeRobot - 機器人充電

### 基本資訊
- **API 名稱**：chargeRobot
- **API URL**：`http://[IP:Port]/interfaces/api/amr/chargeRobot`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱         | 類型    | 必填 | 最大長度 | 說明         |
|------------------|---------|------|----------|--------------|
| robotId          | String  | 是   | -        | 機器人 ID    |
| lowestLevel      | Integer | 是   | -        | 最低電量百分比 |
| necessary        | Integer | 是   | -        | 是否強制充電（1=是, 0=否） |
| targetLevel      | Integer | 是   | -        | 目標電量百分比 |

### 請求範例
```json
{
  "robotId": "R001",
  "lowestLevel": 20,
  "necessary": 1,
  "targetLevel": 80
}
```

### 回應範例
```json
{
  "code": "0",
  "message": null,
  "success": true,
  "data": null
}
```

---

## 17. insertRobot - 入場機器人

### 基本資訊
- **API 名稱**：insertRobot
- **API URL**：`http://[IP:Port]/interfaces/api/amr/insertRobot`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱         | 類型    | 必填 | 最大長度 | 說明         |
|------------------|---------|------|----------|--------------|
| cellCode         | String  | 是   | -        | 點位編碼     |
| robotId          | String  | 是   | -        | 機器人 ID    |
| synchroContainer | Integer | 否   | -        | 是否同步容器（1=是, 0=否） |

### 請求範例
```json
{
  "cellCode": "N001",
  "robotId": "R001",
  "synchroContainer": 1
}
```

### 回應範例
```json
{
  "code": "0",
  "message": null,
  "success": true,
  "data": null
}
```

---

## 18. removeRobot - 離場機器人

### 基本資訊
- **API 名稱**：removeRobot
- **API URL**：`http://[IP:Port]/interfaces/api/amr/removeRobot`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱         | 類型    | 必填 | 最大長度 | 說明         |
|------------------|---------|------|----------|--------------|
| robotId          | String  | 是   | -        | 機器人 ID    |
| withContainer    | Integer | 否   | -        | 是否帶容器離場（1=是, 0=否） |

### 請求範例
```json
{
  "robotId": "R001",
  "withContainer": 1
}
```

### 回應範例
```json
{
  "code": "0",
  "message": null,
  "success": true,
  "data": null
}
```

---

## 19. areaQuery - 查詢所有 WCS 區域

### 基本資訊
- **API 名稱**：areaQuery
- **API URL**：`http://[IP:Port]/interfaces/api/amr/areaQuery`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱 | 類型    | 必填 | 最大長度 | 說明         |
|----------|---------|------|----------|--------------|
| areaType | Integer | 否   | -        | 區域類型     |
| areaCode | String  | 否   | -        | 區域編碼     |

### 請求範例
```json
{
  "areaType": 1,
  "areaCode": "A001"
}
```

### 回應範例
```json
{
  "code": "0",
  "message": null,
  "success": true,
  "data": [
    {
      "areaCode": "A001",
      "areaName": "區域1"
    }
  ]
}
```

---

## 20. areaNodesQuery - 區域內點位查詢

### 基本資訊
- **API 名稱**：areaNodesQuery
- **API URL**：`http://[IP:Port]/interfaces/api/amr/areaNodesQuery`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱  | 類型         | 必填 | 最大長度 | 說明         |
|-----------|--------------|------|----------|--------------|
| areaCodes | List[String] | 是   | -        | 區域編碼列表 |

### 請求範例
```json
{
  "areaCodes": ["A001", "A002"]
}
```

### 回應範例
```json
{
  "code": "0",
  "message": null,
  "success": true,
  "data": [
    {
      "areaCode": "A001",
      "nodes": ["N001", "N002"]
    },
    {
      "areaCode": "A002",
      "nodes": ["N003"]
    }
  ]
}
```

---

## 21. queryWCSAreaByMapNode - 查詢點位所屬區域

### 基本資訊
- **API 名稱**：queryWCSAreaByMapNode
- **API URL**：`http://[IP:Port]/interfaces/api/amr/queryWCSAreaByMapNode`
- **HTTP 方法**：GET
- **Content-Type**：application/json

### 請求參數
| 參數名稱         | 類型    | 必填 | 最大長度 | 說明         |
|------------------|---------|------|----------|--------------|
| nodeUuid         | String  | 是   | -        | 點位 UUID    |

### 請求範例
```json
{
  "nodeUuid": "N001"
}
```

### 回應範例
```json
{
  "code": "0",
  "message": null,
  "success": true,
  "data": {
    "areaCode": "A001",
    "areaName": "區域1"
  }
}
```

---

## 22. queryAllForbiddenAreas - 查詢所有禁行區

### 基本資訊
- **API 名稱**：queryAllForbiddenAreas
- **API URL**：`http://[IP:Port]/interfaces/api/amr/queryAllForbiddenAreas`
- **HTTP 方法**：GET
- **Content-Type**：application/json

### 請求參數
| 參數名稱 | 類型  | 必填 | 最大長度 | 說明         |
|----------|-------|------|----------|--------------|
| 無       | -     | -    | -        | 無           |

### 請求範例
```json
{}
```

### 回應範例
```json
{
  "code": "0",
  "message": null,
  "success": true,
  "data": [
    {
      "forbiddenAreaId": 1,
      "forbiddenAreaCode": "F001",
      "status": "ACTIVE"
    }
  ]
}
```

---

## 23. queryOneForbiddenArea - 查詢指定禁行區

### 基本資訊
- **API 名稱**：queryOneForbiddenArea
- **API URL**：`http://[IP:Port]/interfaces/api/amr/queryOneForbiddenArea`
- **HTTP 方法**：GET
- **Content-Type**：application/json

### 請求參數
| 參數名稱         | 類型    | 必填 | 最大長度 | 說明         |
|------------------|---------|------|----------|--------------|
| forbiddenAreaId  | Integer | 是   | -        | 禁行區 ID    |

### 請求範例
```json
{
  "forbiddenAreaId": 1
}
```

### 回應範例
```json
{
  "code": "0",
  "message": null,
  "success": true,
  "data": {
    "forbiddenAreaId": 1,
    "forbiddenAreaCode": "F001",
    "status": "ACTIVE"
  }
}
```

---

## 24. updateForbiddenAreaStatus - 更新禁行區狀態

### 基本資訊
- **API 名稱**：updateForbiddenAreaStatus
- **API URL**：`http://[IP:Port]/interfaces/api/amr/updateForbiddenAreaStatus`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱           | 類型    | 必填 | 最大長度 | 說明         |
|--------------------|---------|------|----------|--------------|
| forbiddenAreaId    | Integer | 否   | -        | 禁行區 ID    |
| forbiddenAreaCode  | String  | 否   | -        | 禁行區編碼   |
| status             | String  | 是   | -        | 狀態         |

### 請求範例
```json
{
  "forbiddenAreaId": 1,
  "status": "INACTIVE"
}
```

### 回應範例
```json
{
  "code": "0",
  "message": null,
  "success": true,
  "data": null
}
```

---

## 25. queryFunctionNode - 查詢功能點位

### 基本資訊
- **API 名稱**：queryFunctionNode
- **API URL**：`http://[IP:Port]/interfaces/api/amr/queryFunctionNode`
- **HTTP 方法**：POST
- **Content-Type**：application/json

### 請求參數
| 參數名稱         | 類型    | 必填 | 最大長度 | 說明         |
|------------------|---------|------|----------|--------------|
| functionType     | Integer | 是   | -        | 功能點類型   |
| robotTypeClass   | String  | 否   | -        | 機器人類型   |
| containerModel   | String  | 否   | -        | 容器模型     |
| floorNumber      | String  | 否   | -        | 樓層編號     |
| mapCode          | String  | 否   | -        | 地圖編碼     |

### 請求範例
```json
{
  "functionType": 1,
  "robotTypeClass": "LIFT",
  "containerModel": "10001",
  "floorNumber": "1F",
  "mapCode": "M001"
}
```

### 回應範例
```json
{
  "code": "0",
  "message": null,
  "success": true,
  "data": [
    {
      "functionNodeId": "FN001",
      "functionType": 1
    }
  ]
}
```

---

## 26. API 通用回應格式

- 成功：
```json
{
  "code": "0",
  "message": "OK",
  "success": true,
  "data": ...
}
```
- 失敗：
```json
{
  "code": "100001",
  "message": "錯誤訊息...",
  "success": false,
  "data": null
}
```

---

> 本文件由 OCR 轉換並人工整理，若有遺漏請參考原始 KUKA API 文件補充。
