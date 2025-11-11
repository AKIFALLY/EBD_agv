# Transfer Box Task Build - PLC 讀取 Rack 資訊說明

## 🎯 核心功能
從 PLC 讀取入口/出口傳送箱的 Rack 資訊，取代原本的資料庫查詢方式。

## 📊 PLC DM 配置

### 入口傳送箱 (DM2010~2014)
```
DM2010~2011 (32-bit): carrier_bitmap      - 載具在席值
DM2012~2013 (32-bit): carrier_enable_bitmap - 載具啟用 bitmap
DM2014      (16-bit): direction            - 方向角度
```

### 出口傳送箱 (DM2020~2024)
```
DM2020~2021 (32-bit): carrier_bitmap
DM2022~2023 (32-bit): carrier_enable_bitmap
DM2024      (16-bit): direction
```

## 🔄 資料格式說明

### carrier_bitmap (32-bit)
- **高 16-bit**: A面載具 bitmap
- **低 16-bit**: B面載具 bitmap
- **小端序**: PLC DM[0]=低位, DM[1]=高位

範例：
```python
DM[0] = 0x0001  # B面 (低16位)
DM[1] = 0xFFFF  # A面 (高16位)
→ carrier_bitmap = 0xFFFF0001
→ A面 = 0xFFFF, B面 = 0x0001
```

### direction 角度轉換
```python
0° ~ 179°   → direction = +1  (正向)
180° ~ 359° → direction = -1  (反向)
```

## 🔧 關鍵方法

### 1. `_check_and_write_single_transfer_box()`
**原功能**: 查詢資料庫 → 判斷 → 寫入 PLC
**新功能**: 從 PLC 讀取 → 判斷 → 寫入 PLC

```python
# 從 PLC 讀取 Rack 資訊（異步）
self.plc_client.async_read_continuous_data(
    device_type="DM",
    start_address=transfer_box["dm_read_rack_start"],
    count=config.DM_READ_RACK_COUNT,
    callback=lambda response: self._handle_rack_read_response(
        response, transfer_box
    )
)
```

### 2. `_handle_rack_read_response()`
**功能**: 處理 PLC 讀取回應
**流程**:
1. 解析 PLC 資料（carrier_bitmap, carrier_enable_bitmap, direction）
2. 轉換 direction 角度
3. 判斷入口/出口寫入條件
4. 檢查 Task 重複
5. 寫入 PLC

```python
# 解析 32-bit carrier_bitmap
carrier_bitmap_full = self._combine_32bit(
    carrier_bitmap_low, carrier_bitmap_high
)
b_side = carrier_bitmap_full & 0xFFFF
a_side = (carrier_bitmap_full >> 16) & 0xFFFF

# 轉換 direction
direction_converted = self._convert_direction_angle(direction_angle)
```

### 3. `_convert_direction_angle()`
**功能**: 將 PLC 角度轉換為 direction 值

```python
def _convert_direction_angle(self, angle: int) -> int:
    normalized_angle = angle % 360
    if 0 <= normalized_angle < 180:
        return 1  # 正向
    else:
        return -1  # 反向
```

## ⚡ 寫入判斷邏輯

### 入口傳送箱 (entrance)
```python
has_material = (a_side > 0) or (b_side > 0)
if not has_material:
    return  # 無料跳過
# 有料才寫入 PLC
```

### 出口傳送箱 (exit)
```python
has_material = (a_side > 0) or (b_side > 0)
if has_material:
    return  # 有料跳過
# 無料才寫入 PLC
```

## 📝 配置說明

### config.py 新增項目
```python
# DM 讀取 Rack 資訊參數
DM_READ_RACK_COUNT = 5  # 5個 words

TRANSFER_BOXES = [
    {
        "name": "入口傳送箱",
        "dm_read_rack_start": "2010",    # 新增
        "dm_write_start": "2010",
        "dm_feedback_start": "3012",
    },
    {
        "name": "出口傳送箱",
        "dm_read_rack_start": "2020",    # 新增
        "dm_write_start": "2020",
        "dm_feedback_start": "3014",
    },
]
```

## 🧪 測試方法

### 1. 啟動節點
```bash
# [容器內]
ros2 run transfer_box_task_build transfer_box_task_build_node
```

### 2. 觀察日誌
```
📥 入口傳送箱 PLC Rack 資訊: A面=0xffff, B面=0x0001, Direction=90° → 1
✅ 入口傳送箱 寫入 PLC 成功: A面=0xffff, B面=0x0001, 確認值=1 (Rack ID=123)
```

### 3. 檢查點
- [ ] PLC 讀取是否成功
- [ ] carrier_bitmap 解析正確
- [ ] direction 轉換符合預期
- [ ] 入口/出口判斷正確
- [ ] PLC 寫入成功

## ⚠️ 重要提醒

1. **異步讀取**: PLC 讀取為異步操作，不阻塞主執行緒
2. **小端序**: PLC 資料為小端序，需注意位元組順序
3. **錯誤處理**: 所有方法都包含異常捕獲
4. **資料庫查詢**: 僅用於獲取 `rack_id`，不影響主邏輯
5. **回饋機制**: Timer 4 的回饋更新機制保持不變

## 🔗 相關檔案
- `config.py` - 配置管理
- `transfer_box_task_build_node.py` - 主節點邏輯
- `transfer_box_manager.py` - 傳送箱管理器
- `database_helper.py` - 資料庫操作
