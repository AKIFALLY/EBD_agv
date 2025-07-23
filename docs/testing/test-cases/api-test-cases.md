# RosAGV API 測試用例規格

## 📋 概述

本文檔詳細描述 RosAGV 專案的 API 測試用例，基於 `web_api_ws` 中的實際 API 端點實作，確保所有 API 功能的正確性和可靠性。

## 🎯 測試範圍

### API 服務分類
- **核心 Web API (port 8000)**: PLC 控制、門控制、交通管制、KUKA Fleet API、地圖匯入
- **AGVCUI API (port 8001)**: 車隊管理、使用者認證、管理頁面、Socket.IO 車隊通訊
- **OPUI API (port 8002)**: 機台操作、設備授權、AGV 控制、Socket.IO 操作通訊

## 🌐 核心 Web API 測試用例 (port 8000)

### TC-CORE-001: PLC 控制 API 測試

#### TC-CORE-001-01: PLC 讀取操作測試
```yaml
測試目標: 驗證 PLC 資料讀取功能
API 端點: POST /plc/read
HTTP 方法: POST
認證要求: 無

測試案例:
  - 請求體: {"address": "DM1000", "count": 1}
    預期回應: HTTP 200, {"success": true, "data": [value]}
  - 請求體: {"address": "DM2000", "count": 5}
    預期回應: HTTP 200, {"success": true, "data": [v1, v2, v3, v4, v5]}
  - 請求體: {"address": "INVALID", "count": 1}
    預期回應: HTTP 400, {"success": false, "error": "Invalid address"}
  - 請求體: {"count": 1}  # 缺少 address
    預期回應: HTTP 422, 驗證錯誤

驗收標準:
  - 有效地址讀取成功
  - 無效地址適當錯誤處理
  - 請求驗證邏輯正確
  - 回應格式一致
```

#### TC-CORE-001-02: PLC 寫入操作測試
```yaml
測試目標: 驗證 PLC 資料寫入功能
API 端點: POST /plc/write
HTTP 方法: POST
認證要求: 無

測試案例:
  - 請求體: {"address": "DM1000", "value": 100}
    預期回應: HTTP 200, {"success": true, "message": "Write successful"}
  - 請求體: {"address": "DM2000", "values": [1, 2, 3]}
    預期回應: HTTP 200, {"success": true, "message": "Batch write successful"}
  - 請求體: {"address": "READONLY", "value": 100}
    預期回應: HTTP 403, {"success": false, "error": "Address is read-only"}
  - 請求體: {"value": 100}  # 缺少 address
    預期回應: HTTP 422, 驗證錯誤

驗收標準:
  - 寫入操作成功執行
  - 批次寫入功能正常
  - 唯讀地址保護有效
  - 錯誤處理機制完整
```

### TC-CORE-002: 門控制 API 測試

#### TC-CORE-002-01: 門狀態查詢測試
```yaml
測試目標: 驗證門狀態查詢功能
API 端點: POST /door/state
HTTP 方法: POST
認證要求: 無

測試案例:
  - 請求體: {"doorId": "1"}
    預期回應: HTTP 200, {"doorId": "1", "state": "open|closed", "timestamp": "..."}
  - 請求體: {"doorId": "2"}
    預期回應: HTTP 200, {"doorId": "2", "state": "open|closed", "timestamp": "..."}
  - 請求體: {"doorId": "999"}
    預期回應: HTTP 404, {"error": "Door not found"}
  - 請求體: {}  # 缺少 doorId
    預期回應: HTTP 422, 驗證錯誤

驗收標準:
  - 有效門 ID 查詢成功
  - 無效門 ID 適當錯誤處理
  - 狀態資訊準確完整
  - 時間戳格式正確
```

#### TC-CORE-002-02: 門控制操作測試
```yaml
測試目標: 驗證門開關控制功能
API 端點: POST /door/control
HTTP 方法: POST
認證要求: 無

測試案例:
  - 請求體: {"doorId": "1", "action": "open"}
    預期回應: HTTP 200, {"success": true, "doorId": "1", "action": "open"}
  - 請求體: {"doorId": "1", "action": "close"}
    預期回應: HTTP 200, {"success": true, "doorId": "1", "action": "close"}
  - 請求體: {"doorId": "1", "action": "invalid"}
    預期回應: HTTP 400, {"error": "Invalid action"}
  - 請求體: {"doorId": "999", "action": "open"}
    預期回應: HTTP 404, {"error": "Door not found"}

驗收標準:
  - 門控制指令正確執行
  - 無效操作適當拒絕
  - 操作結果正確回報
  - 錯誤處理邏輯完整
```

### TC-CORE-003: 交通管制 API 測試

#### TC-CORE-003-01: 交管區域使用權申請測試
```yaml
測試目標: 驗證交管區域使用權申請功能
API 端點: POST /traffic/request
HTTP 方法: POST
認證要求: 無

測試案例:
  - 請求體: {"agvId": "AGV_001", "zoneId": "ZONE_A", "priority": 1}
    預期回應: HTTP 200, {"success": true, "granted": true, "token": "..."}
  - 請求體: {"agvId": "AGV_002", "zoneId": "ZONE_A", "priority": 2}
    預期回應: HTTP 200, {"success": true, "granted": false, "waitTime": 30}
  - 請求體: {"agvId": "AGV_001", "zoneId": "INVALID"}
    預期回應: HTTP 404, {"error": "Zone not found"}
  - 請求體: {"zoneId": "ZONE_A"}  # 缺少 agvId
    預期回應: HTTP 422, 驗證錯誤

驗收標準:
  - 使用權申請邏輯正確
  - 優先級處理機制有效
  - 衝突檢測功能正常
  - 等待時間估算準確
```

#### TC-CORE-003-02: 交管區域使用權釋放測試
```yaml
測試目標: 驗證交管區域使用權釋放功能
API 端點: POST /traffic/release
HTTP 方法: POST
認證要求: 無

測試案例:
  - 請求體: {"agvId": "AGV_001", "zoneId": "ZONE_A", "token": "valid_token"}
    預期回應: HTTP 200, {"success": true, "released": true}
  - 請求體: {"agvId": "AGV_001", "zoneId": "ZONE_A", "token": "invalid_token"}
    預期回應: HTTP 403, {"error": "Invalid token"}
  - 請求體: {"agvId": "AGV_999", "zoneId": "ZONE_A", "token": "valid_token"}
    預期回應: HTTP 404, {"error": "AGV not found in zone"}
  - 請求體: {"agvId": "AGV_001", "zoneId": "ZONE_A"}  # 缺少 token
    預期回應: HTTP 422, 驗證錯誤

驗收標準:
  - 使用權釋放邏輯正確
  - Token 驗證機制有效
  - 釋放後狀態更新正確
  - 等待佇列處理正常
```

### TC-CORE-004: KUKA Fleet API 測試

#### TC-CORE-004-01: 任務提交測試 (主動請求)
```yaml
測試目標: 驗證向 KUKA Fleet 提交任務的功能
API 端點: POST /kuka/submit_mission
HTTP 方法: POST
認證要求: 無
通訊方向: RosAGV → KUKA Fleet

測試案例:
  - 請求體: {"missionCode": "MISSION_001", "robotId": "1", "sourceLocation": "A1", "targetLocation": "B2"}
    預期回應: HTTP 200, {"success": true, "missionId": "...", "status": "submitted"}
  - 請求體: {"missionCode": "MISSION_002", "robotId": "999"}  # 無效機器人
    預期回應: HTTP 400, {"error": "Invalid robot ID"}
  - 請求體: {"robotId": "1"}  # 缺少 missionCode
    預期回應: HTTP 422, 驗證錯誤

驗收標準:
  - 任務提交成功
  - KUKA Fleet 回應正確處理
  - 錯誤情況適當處理
  - 任務狀態正確記錄
```

#### TC-CORE-004-02: 任務狀態回調測試 (被動接收)
```yaml
測試目標: 驗證接收 KUKA Fleet 任務狀態回報的功能
API 端點: POST /interfaces/api/amr/missionStateCallback
HTTP 方法: POST
認證要求: 無
通訊方向: KUKA Fleet → RosAGV

測試案例:
  - 請求體: {"missionCode": "MISSION_001", "missionStatus": "COMPLETED", "robotId": "1", "currentPosition": "B2"}
    預期回應: HTTP 200, {"success": true, "task_id": 123, "status_updated": true}
  - 請求體: {"missionCode": "INVALID_MISSION", "missionStatus": "COMPLETED"}
    預期回應: HTTP 404, {"error": "Mission not found"}
  - 請求體: {"missionCode": "MISSION_001", "missionStatus": "INVALID_STATUS"}
    預期回應: HTTP 400, {"error": "Invalid mission status"}
  - 請求體: {"missionStatus": "COMPLETED"}  # 缺少 missionCode
    預期回應: HTTP 422, 驗證錯誤

驗收標準:
  - 狀態回調正確處理
  - 任務狀態正確更新
  - 無效任務適當處理
  - 狀態驗證邏輯正確
```

### TC-CORE-005: 地圖匯入 API 測試

#### TC-CORE-005-01: KUKA 地圖上傳測試
```yaml
測試目標: 驗證 KUKA 地圖檔案上傳和處理功能
API 端點: POST /map_importer/upload-kuka-map/
HTTP 方法: POST
認證要求: 無
Content-Type: multipart/form-data

測試案例:
  - 檔案: 有效的 kuka_map.json
    預期回應: HTTP 200, {"success": true, "nodes_imported": 100, "edges_imported": 200}
  - 檔案: 格式錯誤的 JSON
    預期回應: HTTP 400, {"error": "Invalid JSON format"}
  - 檔案: 缺少必要欄位的 JSON
    預期回應: HTTP 400, {"error": "Missing required fields"}
  - 無檔案上傳
    預期回應: HTTP 422, {"error": "No file provided"}

驗收標準:
  - 有效地圖檔案正確處理
  - 地圖資料正確匯入資料庫
  - 檔案格式驗證有效
  - 錯誤處理機制完整
```

## 🖥️ AGVCUI API 測試用例 (port 8001)

### TC-AGVCUI-001: 使用者認證 API 測試

#### TC-AGVCUI-001-01: 登入頁面測試
```yaml
測試目標: 驗證登入頁面的載入和顯示
API 端點: GET /login
HTTP 方法: GET
認證要求: 無

測試案例:
  - 請求: GET /login
    預期回應: HTTP 200, HTML 登入頁面
  - 請求: GET /login?redirect=/tasks
    預期回應: HTTP 200, 包含重定向參數的登入頁面

驗收標準:
  - 登入頁面正確載入
  - 重定向參數正確處理
  - 頁面內容完整顯示
```

#### TC-AGVCUI-001-02: 登入處理測試
```yaml
測試目標: 驗證使用者登入處理邏輯
API 端點: POST /login
HTTP 方法: POST
認證要求: 無

測試案例:
  - 請求體: {"username": "admin", "password": "admin123"}
    預期回應: HTTP 302, 重定向到首頁，設定 access_token cookie
  - 請求體: {"username": "admin", "password": "wrong_password"}
    預期回應: HTTP 200, 登入頁面含錯誤訊息
  - 請求體: {"username": "invalid_user", "password": "password"}
    預期回應: HTTP 200, 登入頁面含錯誤訊息
  - 請求體: {"username": "admin"}  # 缺少密碼
    預期回應: HTTP 200, 登入頁面含驗證錯誤

驗收標準:
  - 有效憑證登入成功
  - 無效憑證適當拒絕
  - JWT Token 正確生成
  - 錯誤訊息清晰明確
```

### TC-AGVCUI-002: 管理頁面 API 測試

#### TC-AGVCUI-002-01: 首頁存取測試
```yaml
測試目標: 驗證系統首頁的存取控制
API 端點: GET /
HTTP 方法: GET
認證要求: 需要登入

測試案例:
  - 請求: GET / (已登入)
    預期回應: HTTP 200, 系統儀表板頁面
  - 請求: GET / (未登入)
    預期回應: HTTP 302, 重定向到 /login?redirect=/
  - 請求: GET / (無效 token)
    預期回應: HTTP 302, 重定向到登入頁面，清除 cookie

驗收標準:
  - 認證使用者正常存取
  - 未認證使用者正確重定向
  - Token 驗證邏輯正確
```

#### TC-AGVCUI-002-02: AGV 管理頁面測試
```yaml
測試目標: 驗證 AGV 管理頁面功能
API 端點: GET /agvs
HTTP 方法: GET
認證要求: 需要登入

測試案例:
  - 請求: GET /agvs (已登入)
    預期回應: HTTP 200, AGV 管理頁面
  - 請求: GET /agvs (未登入)
    預期回應: HTTP 302, 重定向到登入頁面

驗收標準:
  - AGV 列表正確顯示
  - 頁面功能完整載入
  - 認證控制有效
```

### TC-AGVCUI-003: Socket.IO 車隊管理測試

#### TC-AGVCUI-003-01: 使用者登入事件測試
```yaml
測試目標: 驗證 Socket.IO 使用者登入事件
事件名稱: user_login
通訊方向: 客戶端 → 伺服器

測試案例:
  - 事件資料: {"username": "admin", "password": "admin123"}
    預期回應: {"success": true, "message": "登入成功，歡迎 管理員", "user": {...}, "access_token": "..."}
  - 事件資料: {"username": "admin", "password": "wrong"}
    預期回應: {"success": false, "message": "密碼錯誤"}
  - 事件資料: {"username": "invalid", "password": "password"}
    預期回應: {"success": false, "message": "使用者不存在"}

驗收標準:
  - 登入驗證邏輯正確
  - JWT Token 正確生成
  - 錯誤處理機制完整
```

#### TC-AGVCUI-003-02: 系統資料廣播測試
```yaml
測試目標: 驗證系統資料的自動廣播功能
事件名稱: map_info, agv_list, task_list, rack_list
通訊方向: 伺服器 → 客戶端

測試案例:
  - 觸發: 客戶端連線
    預期廣播: map_info (地圖資訊)
  - 觸發: AGV 資料變更
    預期廣播: agv_list (AGV 列表更新)
  - 觸發: 任務狀態變更
    預期廣播: task_list (任務列表更新)
  - 觸發: 料架位置變更
    預期廣播: rack_list (料架列表更新)

驗收標準:
  - 資料變更自動檢測
  - 廣播事件及時觸發
  - 資料格式正確完整
  - 100ms 間隔同步正常
```

## 📱 OPUI API 測試用例 (port 8002)

### TC-OPUI-001: 設備授權 API 測試

#### TC-OPUI-001-01: 首頁設備授權測試
```yaml
測試目標: 驗證首頁的設備授權機制
API 端點: GET /home
HTTP 方法: GET
認證要求: deviceId 參數

測試案例:
  - 請求: GET /home?deviceId=device_001 (有效設備)
    預期回應: HTTP 200, 首頁 HTML 內容
  - 請求: GET /home?deviceId=invalid_device (無效設備)
    預期回應: HTTP 403, 授權失敗錯誤頁面
  - 請求: GET /home (缺少 deviceId)
    預期回應: HTTP 400, 缺少參數錯誤頁面

驗收標準:
  - 有效設備正常存取
  - 無效設備適當拒絕
  - 參數驗證邏輯正確
```

#### TC-OPUI-001-02: 設定頁面授權測試
```yaml
測試目標: 驗證設定頁面的設備授權機制
API 端點: GET /setting
HTTP 方法: GET
認證要求: deviceId 參數

測試案例:
  - 請求: GET /setting?deviceId=device_001
    預期回應: HTTP 200, 設定頁面 HTML 內容
  - 請求: GET /setting?deviceId=unauthorized_device
    預期回應: HTTP 403, 授權失敗錯誤頁面

驗收標準:
  - 設備授權檢查一致
  - 頁面內容正確載入
  - 錯誤處理統一
```

### TC-OPUI-002: REST API 端點測試

#### TC-OPUI-002-01: 產品管理 API 測試
```yaml
測試目標: 驗證產品管理的 REST API 功能
API 端點: GET /products, POST /products
HTTP 方法: GET, POST
認證要求: 無

測試案例:
  - 請求: GET /products
    預期回應: HTTP 200, [{"id": 1, "name": "PRODUCT_A", "size": "S"}, ...]
  - 請求: POST /products {"name": "NEW_PRODUCT", "size": "L", "process_settings_id": 1}
    預期回應: HTTP 201, {"id": 3, "name": "NEW_PRODUCT", "size": "L", ...}
  - 請求: POST /products {"name": ""}  # 無效名稱
    預期回應: HTTP 422, 驗證錯誤

驗收標準:
  - 產品列表正確返回
  - 產品建立功能正常
  - 資料驗證邏輯正確
```

#### TC-OPUI-002-02: AGV 任務 API 測試
```yaml
測試目標: 驗證 AGV 任務相關的 API 功能
API 端點: GET /api/tasks/{task_id}/status
HTTP 方法: GET
認證要求: 無

測試案例:
  - 請求: GET /api/tasks/123/status (有效任務)
    預期回應: HTTP 200, {"success": true, "task_id": 123, "status": 3, "status_name": "執行中"}
  - 請求: GET /api/tasks/999/status (無效任務)
    預期回應: HTTP 404, {"success": false, "error": "Task not found"}

驗收標準:
  - 任務狀態查詢正確
  - 無效任務適當處理
  - 狀態資訊準確完整
```

### TC-OPUI-003: Socket.IO 操作通訊測試

#### TC-OPUI-003-01: 客戶端登入事件測試
```yaml
測試目標: 驗證客戶端登入和狀態管理
事件名稱: login
通訊方向: 客戶端 → 伺服器

測試案例:
  - 事件資料: {"deviceId": "device_001", "machineId": 1, "userAgent": "..."}
    預期回應: {"success": true, "message": "登入成功，clientId: device_001", "client": {...}}
  - 事件資料: {"deviceId": "invalid_device", "machineId": 1}
    預期回應: {"success": false, "message": "設備授權失敗"}

驗收標準:
  - 客戶端登入邏輯正確
  - clientId 映射建立成功
  - 設備授權檢查有效
```

#### TC-OPUI-003-02: AGV 操作事件測試
```yaml
測試目標: 驗證 AGV 操作相關的 Socket.IO 事件
事件名稱: call_empty, dispatch_full, cancel_task, confirm_delivery
通訊方向: 客戶端 → 伺服器

測試案例:
  - 事件: call_empty {"side": "left"}
    預期回應: {"success": true, "message": "叫車成功，任務 ID: 123"}
  - 事件: dispatch_full {"side": "right", "rack": "RACK_001"}
    預期回應: {"success": true, "message": "派車成功，任務 ID: 124"}
  - 事件: cancel_task {"side": "left"}
    預期回應: {"success": true, "message": "已取消停車位 [101] 的任務"}
  - 事件: confirm_delivery {"side": "right"}
    預期回應: {"success": true, "message": "已確認停車位 [102] 的rack架已搬移至作業區"}

驗收標準:
  - 所有 AGV 操作正常執行
  - 任務狀態正確更新
  - 機台狀態同步及時
  - 錯誤處理機制完整
```

---

**最後更新**: 2025-01-23  
**維護責任**: API 測試工程師、後端開發團隊  
**版本**: v1.0.0 (基於實際 API 端點分析)
