# Loader AGV Take Transfer 流程測試

## 快速開始

### 運行 Demo 測試
```bash
cd agv_ws/src/loader_agv/test
python3 test_demo.py
```

這將運行一個完整的 demo 測試，展示 take_transfer 流程的核心功能測試。

## 測試內容

### 核心測試功能

1. **AGV Port 檢查邏輯**
   - 動態參數計算 (`port_address`, `eqp_id`)
   - Port 選擇算法 (15 種狀態組合)
   - EQP 信號查詢和 Carrier 查詢

2. **Transfer Continuation Logic** ⭐ **核心邏輯**
   ```python
   # 繼續條件
   select_boxin_port=1 AND boxin_port2=True → continue=True
   select_boxin_port=3 AND boxin_port4=True → continue=True
   # 其他情況 → continue=False
   ```

3. **機器人動作控制**
   - 取傳送箱動作 (TakeTransferState)
   - 放置 AGV 動作 (PutAgvState)
   - 狀態機步驟轉換

4. **資料庫操作**
   - Carrier 資料更新
   - 狀態同步

5. **整合流程**
   - 完整的狀態轉換流程
   - 數據流驗證
   - 錯誤處理

## 測試架構

### Mock 策略
- **ROS2 依賴**: 完全模擬，無需實際 ROS2 環境
- **硬體依賴**: 模擬 PLC、Hokuyo 等設備
- **資料庫依賴**: 模擬所有資料庫操作

### 測試文件
- `test_demo.py` - 可運行的 demo 測試
- `test_agv_port_check_empty_state.py` - AGV port 檢查測試
- `test_transfer_check_have_state.py` - 傳送箱檢查測試
- `test_take_transfer_state.py` - 取傳送箱動作測試
- `test_put_agv_state.py` - 放置 AGV 動作測試
- `test_transfer_vision_position_state.py` - 視覺定位測試
- `test_take_transfer_integration.py` - 整合測試

## 運行方式

### 1. Demo 測試 (推薦)
```bash
python3 test_demo.py
```

### 2. 單個測試方法
```bash
python3 -m unittest test_demo.TestTakeTransferDemo.test_transfer_continuation_logic -v
```

### 3. 所有 Demo 測試
```bash
python3 -m unittest test_demo -v
```

## 測試結果示例

```
🚀 運行 loader_agv take_transfer 流程 Demo 測試...
test_agv_port_dynamic_calculations ... ok
test_agv_port_selection_logic ... ok
test_complete_flow_simulation ... ok
test_error_handling ... ok
test_transfer_continuation_logic ... ok

----------------------------------------------------------------------
Ran 5 tests in 0.001s

OK
```

## 核心測試案例

### 1. 動態參數計算
```python
# room_id=1 → port_address=1100, eqp_id=110
# room_id=2 → port_address=2100, eqp_id=210
```

### 2. Port 選擇邏輯
```python
# (port1, port2, port3, port4) → 選擇結果
(0,0,0,0) → SELECT_PORT01  # 全空選第一個
(1,0,0,0) → SELECT_PORT02  # 第一個有貨選第二個
(1,1,0,0) → SELECT_PORT03  # 前兩個有貨選第三個
(1,1,1,0) → SELECT_PORT04  # 前三個有貨選第四個
```

### 3. Transfer Continuation 決策
```python
# 繼續處理傳送箱的條件
port1_selected + port2_has_cargo = True
port3_selected + port4_has_cargo = True
# 其他情況 = False
```

## 注意事項

1. **依賴問題**: 測試使用完整的 mock 策略，不需要實際的 ROS2 或硬體環境
2. **測試隔離**: 每個測試都是獨立的，不會互相影響
3. **擴展性**: 可以輕鬆添加新的測試案例
4. **可維護性**: 使用清晰的命名和文檔化的測試結構

## 詳細報告

查看 `TEST_REPORT.md` 獲取完整的測試報告和技術細節。
