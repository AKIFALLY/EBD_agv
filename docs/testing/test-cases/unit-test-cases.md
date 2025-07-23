# RosAGV 單元測試用例規格

## 📋 概述

本文檔詳細描述 RosAGV 專案的單元測試用例，基於現有程式碼實作和測試檔案分析，確保所有核心功能的正確性。

## 🎯 測試範圍

### 測試分類
- **AGV 狀態機測試**: 各種 AGV 類型的狀態轉換和業務邏輯
- **資料庫操作測試**: CRUD 操作、連線管理、資料完整性
- **Web API 測試**: HTTP 端點、Socket.IO 事件、認證授權
- **計算邏輯測試**: 演算法、數學計算、參數處理
- **整合邏輯測試**: 模組間互動、資料流轉、狀態同步

## 🤖 AGV 狀態機測試用例

### TC-AGV-001: Loader AGV Take Transfer 流程測試

#### TC-AGV-001-01: AGV Port 動態參數計算
```yaml
測試目標: 驗證 AGV Port 的動態參數計算邏輯
測試檔案: agv_ws/src/loader_agv/test/test_agv_port_check_empty_state.py
測試方法: test_agv_port_dynamic_calculations

測試案例:
  - 輸入: room_id=1
    預期: port_address=1100, eqp_id=110
  - 輸入: room_id=2  
    預期: port_address=2100, eqp_id=210
  - 輸入: room_id=3
    預期: port_address=3100, eqp_id=310

驗收標準:
  - 所有 room_id 計算結果正確
  - 參數格式符合系統要求
  - 計算效能在可接受範圍內
```

#### TC-AGV-001-02: Port 選擇算法測試
```yaml
測試目標: 驗證 15 種 Port 狀態組合的選擇邏輯
測試檔案: agv_ws/src/loader_agv/test/test_agv_port_check_empty_state.py
測試方法: test_agv_port_selection_logic

測試案例:
  - 輸入: (port1=0, port2=0, port3=0, port4=0)
    預期: SELECT_PORT01 (全空選第一個)
  - 輸入: (port1=1, port2=0, port3=0, port4=0)
    預期: SELECT_PORT02 (第一個有貨選第二個)
  - 輸入: (port1=1, port2=1, port3=0, port4=0)
    預期: SELECT_PORT03 (前兩個有貨選第三個)
  - 輸入: (port1=1, port2=1, port3=1, port4=0)
    預期: SELECT_PORT04 (前三個有貨選第四個)
  - 輸入: (port1=1, port2=1, port3=1, port4=1)
    預期: 錯誤處理 (所有 port 滿載)

驗收標準:
  - 所有 15 種組合邏輯正確
  - 邊界條件處理正確
  - 錯誤情況適當處理
```

#### TC-AGV-001-03: Transfer Continuation Logic 測試
```yaml
測試目標: 驗證 take_transfer 流程的核心決策邏輯
測試檔案: agv_ws/src/loader_agv/test/test_demo.py
測試方法: test_transfer_continuation_logic

測試案例:
  - 輸入: select_boxin_port=1, boxin_port2=True
    預期: continue=True
  - 輸入: select_boxin_port=3, boxin_port4=True
    預期: continue=True
  - 輸入: select_boxin_port=1, boxin_port2=False
    預期: continue=False
  - 輸入: select_boxin_port=2, boxin_port4=True
    預期: continue=False

驗收標準:
  - 核心決策邏輯 100% 正確
  - 所有條件組合測試通過
  - 邏輯一致性驗證通過
```

### TC-AGV-002: Cargo Mover AGV Hokuyo 設備測試

#### TC-AGV-002-01: Idle State Hokuyo 初始化測試
```yaml
測試目標: 驗證 Idle 狀態下 Hokuyo 設備的初始化和寫入操作
測試檔案: agv_ws/src/cargo_mover_agv/test/test_idle_state_hokuyo.py
測試方法: test_hokuyo_write_initialization

測試案例:
  - 場景: Hokuyo 設備初始化
    預期: 設備狀態正確設定
  - 場景: 寫入操作執行
    預期: 寫入指令正確發送
  - 場景: 狀態轉換觸發
    預期: 狀態機正確轉換

驗收標準:
  - Hokuyo 設備初始化成功
  - 寫入操作無錯誤
  - 狀態轉換邏輯正確
```

#### TC-AGV-002-02: Complete State 延遲重置測試
```yaml
測試目標: 驗證任務完成後的延遲重置邏輯
測試檔案: agv_ws/src/cargo_mover_agv/test/test_complete_state_delayed_reset.py
測試方法: test_delayed_reset_mechanism

測試案例:
  - 場景: 任務完成觸發
    預期: 延遲重置計時器啟動
  - 場景: 延遲時間到達
    預期: 狀態重置執行
  - 場景: 延遲期間中斷
    預期: 重置邏輯正確處理

驗收標準:
  - 延遲重置時間準確
  - 重置邏輯執行正確
  - 中斷處理機制有效
```

### TC-AGV-003: Unloader AGV 計算邏輯測試

#### TC-AGV-003-01: Pre Dryer Port 計算測試
```yaml
測試目標: 驗證 pre_dryer_port 的 row 和 column 計算邏輯
測試檔案: agv_ws/src/unloader_agv/test/test_pre_dryer_calculation.py
測試方法: test_port_1_to_4_calculation, test_port_5_to_8_calculation

測試案例:
  - 輸入: port 1-4
    預期: row=1, column=0
  - 輸入: port 5-8
    預期: row=2, column=0
  - 輸入: 邊界值 (port 1, port 8)
    預期: 計算結果正確

驗收標準:
  - 所有 port 計算結果正確
  - 邊界條件處理正確
  - 計算邏輯一致性驗證
```

#### TC-AGV-003-02: Take Quantity 邏輯測試
```yaml
測試目標: 驗證 take_quantity 的計算邏輯和參數整合處理
測試檔案: agv_ws/src/unloader_agv/test/test_take_quantity.py
測試方法: test_take_quantity_calculation

測試案例:
  - 輸入: carrier_id_min=None, carrier_id_max=None
    預期: take_quantity=0 (兩個 port 都沒有貨物)
  - 輸入: carrier_id_min="123", carrier_id_max=None
    預期: take_quantity=1 (只有第一個 port 有貨物)
  - 輸入: carrier_id_min=None, carrier_id_max="456"
    預期: take_quantity=1 (只有第二個 port 有貨物)
  - 輸入: carrier_id_min="123", carrier_id_max="456"
    預期: take_quantity=2 (兩個 port 都有貨物)

驗收標準:
  - 所有計算邏輯正確
  - 參數類型轉換正確
  - 邊界條件處理完整
```

## 🗄️ 資料庫操作測試用例

### TC-DB-001: 基礎 CRUD 操作測試

#### TC-DB-001-01: 基礎 CRUD 功能測試
```yaml
測試目標: 驗證 BaseCRUD 的基本 CRUD 操作
測試檔案: db_proxy_ws/src/db_proxy/test/test_base_crud.py
測試方法: test_create_and_get_by_id

測試案例:
  - 操作: 建立記錄
    預期: 記錄成功建立，ID 自動生成
  - 操作: 查詢記錄
    預期: 記錄正確返回，資料完整
  - 操作: 更新記錄
    預期: 記錄成功更新，時間戳更新
  - 操作: 刪除記錄
    預期: 記錄成功刪除，查詢返回空

驗收標準:
  - 所有 CRUD 操作成功
  - 資料完整性保持
  - 時間戳自動管理
```

#### TC-DB-001-02: 連線池管理測試
```yaml
測試目標: 驗證資料庫連線池的管理功能
測試檔案: db_proxy_ws/src/db_proxy/test/test_connection_pool_manager.py
測試方法: test_create_tables, test_pool_status_logging

測試案例:
  - 場景: 連線池初始化
    預期: 連線池正確建立
  - 場景: 多個連線請求
    預期: 連線池正確分配連線
  - 場景: 連線釋放
    預期: 連線正確返回池中
  - 場景: 連線池狀態監控
    預期: 狀態資訊準確記錄

驗收標準:
  - 連線池功能正常
  - 連線分配和釋放正確
  - 狀態監控資訊準確
```

### TC-DB-002: 地圖資料測試

#### TC-DB-002-01: KUKA 地圖匯入測試
```yaml
測試目標: 驗證 KUKA 地圖資料的匯入功能
測試檔案: db_proxy_ws/src/db_proxy/docs/testing/test_kuka_map_import.py
測試方法: test_kuka_map_import_complete

測試案例:
  - 場景: 地圖檔案讀取
    預期: JSON 檔案正確解析
  - 場景: 節點資料匯入
    預期: 所有節點正確匯入
  - 場景: 邊資料匯入
    預期: 所有邊正確匯入，外鍵關係正確
  - 場景: 資料完整性檢查
    預期: 無孤立邊，資料一致性驗證通過

驗收標準:
  - 地圖資料完整匯入
  - 外鍵約束正確
  - 資料完整性驗證通過
```

#### TC-DB-002-02: 智能清除功能測試
```yaml
測試目標: 驗證地圖資料的智能清除功能
測試檔案: db_proxy_ws/src/db_proxy/docs/testing/test_smart_clear.py
測試方法: test_smart_clear_kuka_map

測試案例:
  - 場景: 清除前資料統計
    預期: 正確統計現有資料
  - 場景: 智能清除執行
    預期: 按依賴關係正確清除
  - 場景: 清除後驗證
    預期: 目標資料已清除，相關資料保持完整
  - 場景: 清除結果報告
    預期: 清除統計資訊準確

驗收標準:
  - 清除邏輯正確執行
  - 資料依賴關係正確處理
  - 清除統計資訊準確
```

### TC-DB-003: License 管理測試

#### TC-DB-003-01: License CRUD 操作測試
```yaml
測試目標: 驗證 License 模型的 CRUD 操作
測試檔案: db_proxy_ws/src/db_proxy/test/test_license.py
測試方法: test_license_crud_operations

測試案例:
  - 操作: 建立 License
    預期: License 記錄成功建立
  - 操作: 根據 device_id 查詢
    預期: 正確返回對應 License
  - 操作: 更新 License 狀態
    預期: 狀態成功更新
  - 操作: 刪除 License
    預期: 記錄成功刪除

驗收標準:
  - 所有 CRUD 操作正常
  - device_id 唯一性約束有效
  - 狀態欄位驗證正確
```

#### TC-DB-003-02: License 初始化測試
```yaml
測試目標: 驗證 License 初始化功能的冪等性
測試檔案: db_proxy_ws/src/db_proxy/test/test_license.py
測試方法: test_initialize_license_idempotent

測試案例:
  - 場景: 首次初始化
    預期: 預設 License 記錄建立
  - 場景: 重複初始化
    預期: 不會重複建立記錄
  - 場景: 初始化後查詢
    預期: 預設記錄存在且正確

驗收標準:
  - 初始化功能冪等性正確
  - 預設資料正確建立
  - 重複執行無副作用
```

## 🌐 Web API 測試用例

### TC-API-001: KUKA API 測試

#### TC-API-001-01: Mission State Callback 測試
```yaml
測試目標: 驗證 KUKA missionStateCallback API 功能
測試檔案: web_api_ws/src/web_api/tests/test_kuka_api.py
測試方法: test_kuka_mission_state_callback

測試案例:
  - 輸入: 有效的 mission 資料
    預期: HTTP 200, 任務狀態正確更新
  - 輸入: 無效的 mission_code
    預期: HTTP 404, 適當錯誤訊息
  - 輸入: 格式錯誤的資料
    預期: HTTP 400, 驗證錯誤訊息
  - 輸入: 缺少必要欄位
    預期: HTTP 422, 欄位驗證錯誤

驗收標準:
  - 所有 HTTP 狀態碼正確
  - 錯誤訊息清晰明確
  - 資料驗證邏輯正確
```

### TC-API-002: OPUI Socket.IO 測試

#### TC-API-002-01: Socket.IO 連線測試
```yaml
測試目標: 驗證 OPUI Socket.IO 連線和基本事件
測試檔案: web_api_ws/src/opui/tests/validation/test_socket_functions.py
測試方法: test_socket_connection

測試案例:
  - 場景: 客戶端連線
    預期: 連線成功建立
  - 場景: 登入事件
    預期: 認證成功，會話建立
  - 場景: 資料同步事件
    預期: 基礎資料正確推送
  - 場景: 客戶端斷線
    預期: 連線正確清理

驗收標準:
  - 連線建立和斷開正常
  - 事件處理邏輯正確
  - 會話管理功能正常
```

#### TC-API-002-02: AGV 操作事件測試
```yaml
測試目標: 驗證 AGV 操作相關的 Socket.IO 事件
測試檔案: web_api_ws/src/opui/tests/validation/test_socket_functions.py
測試方法: test_agv_operations

測試案例:
  - 事件: call_empty (叫空車)
    預期: 任務建立成功，狀態正確更新
  - 事件: dispatch_full (派車)
    預期: 派車任務建立，料架狀態更新
  - 事件: cancel_task (取消任務)
    預期: 任務取消，狀態重置
  - 事件: confirm_delivery (確認送達)
    預期: 任務完成，狀態同步

驗收標準:
  - 所有 AGV 操作事件正常
  - 任務狀態轉換正確
  - 資料同步及時準確
```

---

**最後更新**: 2025-01-23  
**維護責任**: 測試工程師、開發團隊  
**版本**: v1.0.0 (基於實際測試檔案分析)
