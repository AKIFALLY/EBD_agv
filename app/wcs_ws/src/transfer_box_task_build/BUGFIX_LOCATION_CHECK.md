# Bug 修正：添加 Location Rack 檢查

## 🐛 問題描述

**報告日期**: 2025-11-10 19:44

**問題**：即使 Rack 不在設定的 location_id（26, 27），系統仍然會從 PLC 讀取並寫入 DM2010/DM2020。

### 問題原因
原始邏輯流程：
```
Timer 每 3 秒 → 遍歷所有傳送箱 → 從 PLC 讀取 DM → 判斷 → 寫入 PLC
```

**漏洞**：
- ❌ 沒有檢查資料庫中是否有 Rack 在指定 location_id
- ❌ 即使 Rack 已離開該位置，仍會讀取 PLC
- ❌ PLC DM 可能保留舊的或無效的資料
- ❌ 基於錯誤資料進行判斷和寫入

## ✅ 修正方案

### 新的邏輯流程
```
Timer → 檢查資料庫 Rack 是否在 location_id → 如果有 → 從 PLC 讀取 → 判斷 → 寫入
                                            → 如果沒有 → 跳過
```

### 修改內容

#### 1. `_check_and_write_single_transfer_box()` 方法

**修改前**：
```python
def _check_and_write_single_transfer_box(self, transfer_box: dict):
    try:
        # 1. 從 PLC 讀取 Rack 資訊（異步）
        self.plc_client.async_read_continuous_data(
            device_type="DM",
            start_address=transfer_box["dm_read_rack_start"],
            count=config.DM_READ_RACK_COUNT,
            callback=lambda response: self._handle_rack_read_response(
                response, transfer_box
            )
        )
```

**修改後**：
```python
def _check_and_write_single_transfer_box(self, transfer_box: dict):
    try:
        # 1. 先檢查資料庫中是否有 Rack 在指定的 location_id
        rack = self.db_helper.get_rack_by_location(transfer_box["location_id"])

        if not rack:
            self.get_logger().debug(
                f"{transfer_box['name']} Location {transfer_box['location_id']} "
                f"沒有 Rack，跳過 PLC 讀取"
            )
            return

        # 2. 確認有 Rack 後，從 PLC 讀取 Rack 資訊（異步）
        self.plc_client.async_read_continuous_data(
            device_type="DM",
            start_address=transfer_box["dm_read_rack_start"],
            count=config.DM_READ_RACK_COUNT,
            callback=lambda response: self._handle_rack_read_response(
                response, transfer_box, rack  # 傳遞 rack 物件
            )
        )
```

#### 2. `_handle_rack_read_response()` 方法

**改進**：
- 新增 `rack` 參數（由上游傳入）
- 移除重複的資料庫查詢
- 直接使用傳入的 rack.id

**修改前**：
```python
def _handle_rack_read_response(self, response, transfer_box: dict):
    # ... 處理邏輯 ...
    
    # 查詢資料庫 Rack（重複查詢）
    rack = self.db_helper.get_rack_by_location(transfer_box["location_id"])
    rack_id = rack.id if rack else None
```

**修改後**：
```python
def _handle_rack_read_response(self, response, transfer_box: dict, rack):
    # ... 處理邏輯 ...
    
    # 使用已查詢的 Rack（避免重複查詢）
    rack_id = rack.id
```

#### 3. 日誌輸出優化

**修改前**：
```python
rack_info = f"Rack ID={rack_id}" if rack_id else "無 Rack ID"
```

**修改後**：
```python
# rack_id 現在一定存在，簡化日誌
f"Rack ID={rack_id}"
```

## 🎯 修正效果

### Before (有問題)
```
Timer → 從 PLC 讀取 DM2010 → 解析資料 → 判斷 → 寫入
        ↑
        即使 location_id 26/27 沒有 Rack，仍會執行
```

### After (已修正)
```
Timer → 檢查 location_id 26/27 是否有 Rack?
        ├─ 有 → 從 PLC 讀取 → 解析 → 判斷 → 寫入
        └─ 沒有 → 跳過（記錄 debug 日誌）
```

## 📊 驗證方法

### 1. 正常情況（有 Rack）
**預期日誌**：
```
✅ 入口傳送箱 寫入 PLC 成功: A面=0xffff, B面=0x0000, 確認值=1 (Rack ID=123)
```

### 2. 異常情況（無 Rack）
**預期日誌**：
```
[DEBUG] 入口傳送箱 Location 27 沒有 Rack，跳過 PLC 讀取
```

### 3. 測試步驟
1. 將 Rack 移動到 location_id 26 或 27
2. 觀察日誌應顯示正常寫入
3. 將 Rack 移開
4. 觀察日誌應顯示「跳過 PLC 讀取」

## 📝 其他改進

### 效能優化
- ✅ 避免重複查詢資料庫
- ✅ 提早返回（沒有 Rack 時直接跳過）
- ✅ 減少不必要的 PLC 通訊

### 資料一致性
- ✅ 確保 Rack 確實在指定位置才進行操作
- ✅ 避免基於過時的 PLC 資料進行判斷

### 程式碼品質
- ✅ 邏輯更清晰
- ✅ 參數傳遞更合理
- ✅ 日誌輸出更準確

## 🔄 部署資訊

**修改檔案**：
- `transfer_box_task_build_node.py:105-136` - 主要修正
- `transfer_box_task_build_node.py:138-229` - 參數調整
- `transfer_box_task_build_node.py:334-352` - 日誌優化

**部署時間**: 2025-11-10 19:44:24

**測試狀態**:
- ✅ Python 語法檢查通過
- ✅ ROS2 建置成功
- ✅ 服務正常啟動
- ⏳ 實際環境測試中

## 📚 相關文檔
- `CHANGELOG.md` - 完整變更記錄
- `README_PLC_READ.md` - PLC 讀取功能說明
- `config.py` - 傳送箱配置

---

**修正人員**: Claude AI Assistant  
**問題回報**: 用戶  
**嚴重程度**: 中 (邏輯錯誤，但不會造成系統崩潰)  
**影響範圍**: 入口/出口傳送箱 PLC 寫入判斷
