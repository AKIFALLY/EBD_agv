# check_room_have_carrier() 方法使用指南

## 🎯 **功能概述**

`check_room_have_carrier()` 方法用於檢查系統中是否存在以下三個條件同時滿足的情況：

1. **房間內有載具** - Carrier 表中存在 room_id 不為空的記錄
2. **載具沒有對應任務** - Task 表中沒有與該載具相關的執行中任務
3. **出口傳送箱位置無空 rack** - 所有出口傳送箱位置都被佔用

## 📋 **詳細邏輯說明**

### **步驟 1：查詢房間內載具**
```sql
SELECT * FROM carrier 
WHERE room_id IS NOT NULL 
AND status_id IS NOT NULL
```

- 查詢所有在房間內的載具（room_id 不為空）
- 確保載具有有效狀態

### **步驟 2：檢查任務關聯**
對每個房間內的載具：
```python
related_tasks = [
    task for task in task_table 
    if (task.get('room_id') == carrier.room_id and 
        task.get('status_id') in [0, 1, 2])  # 未執行、已選擇、執行中
]
```

- 查詢與載具同房間的執行中任務
- 任務狀態：0=未執行, 1=已選擇, 2=執行中

### **步驟 3：檢查出口傳送箱狀態**
檢查以下位置的 rack 狀態：
- `10201` - room01 Unloader Box
- `20002` - room02 Unloader Box  
- `20301` - 其他出口傳送箱

```sql
SELECT * FROM rack WHERE location_id = {location_id}
```

判斷邏輯：
- 沒有 rack → 位置空的
- 有 rack 且 status_id = 1 → 有空架（位置可用）
- 有 rack 且 status_id ≠ 1 → 位置被佔用

## 🔧 **資料表結構參考**

### **Carrier 表**
```python
class Carrier(SQLModel, table=True):
    id: Optional[int]           # 載具ID
    room_id: Optional[int]      # 房間ID (NULL表示不在房間內)
    rack_id: Optional[int]      # 貨架ID
    port_id: Optional[int]      # 端口ID
    rack_index: Optional[int]   # 貨架索引
    status_id: Optional[int]    # 載具狀態
```

### **Task 表**
```python
class Task(SQLModel, table=True):
    id: Optional[int]           # 任務ID
    room_id: Optional[int]      # 房間ID
    status_id: Optional[int]    # 任務狀態
    agv_id: Optional[int]       # AGV ID
    work_id: Optional[int]      # 工作ID
```

### **Rack 表**
```python
class Rack(SQLModel, table=True):
    id: Optional[int]           # 貨架ID
    location_id: Optional[int]  # 位置ID
    status_id: Optional[int]    # 貨架狀態 (1=空架, 2=滿料架等)
    direction: int              # 方向
```

## 📊 **狀態定義**

### **任務狀態 (Task.status_id)**
- `0` - 未執行
- `1` - 已選擇
- `2` - 執行中
- `3` - 已完成

### **貨架狀態 (Rack.status_id)**
- `1` - 空架
- `2` - 滿料架-32
- `3` - 滿料架-16
- `4` - 未滿架-32
- `5` - 未滿架-16

### **出口傳送箱位置 ID**
- `10201` - room01 Unloader Box
- `20002` - room02 Unloader Box
- `20301` - 其他出口傳送箱

## 🎯 **使用範例**

### **在 WCS Base Node 中使用**
```python
def cycle_process(self):
    """定時處理任務"""
    if self.have_data:
        # 檢查房間載具狀況
        need_action = self.kuka_wcs_handler.check_room_have_carrier()
        
        if need_action:
            self.get_logger().info("🚨 需要處理房間載具：有載具無任務且出口傳送箱滿載")
            # 在這裡添加相應的處理邏輯
            # 例如：創建新任務、發送警告等
        else:
            self.get_logger().debug("✅ 房間載具狀況正常")
```

### **獨立調用**
```python
# 創建 handler
handler = KukaWCSHandler(node)

# 檢查房間載具狀況
result = handler.check_room_have_carrier()

if result:
    print("需要處理：房間內有無任務載具且出口傳送箱都被佔用")
else:
    print("狀況正常：載具都有任務或出口傳送箱有空位")
```

## 🔍 **返回值說明**

| 返回值 | 含義 | 處理建議 |
|--------|------|----------|
| `True` | 滿足所有條件 | 需要創建新任務或採取行動 |
| `False` | 不滿足條件 | 系統狀況正常，無需特殊處理 |

### **返回 True 的情況**
- ✅ 房間內有載具沒有對應任務
- ✅ 所有出口傳送箱位置都被佔用

### **返回 False 的情況**
- ❌ 房間內沒有載具
- ❌ 所有載具都有對應任務
- ❌ 有空的出口傳送箱位置

## 🚨 **注意事項**

### **1. 資料同步**
- 確保調用前資料已載入完成
- 使用最新的任務和載具資料

### **2. 錯誤處理**
- 方法內部已包含異常處理
- 發生錯誤時返回 `False` 並記錄錯誤日誌

### **3. 效能考慮**
- 方法會執行多次資料庫查詢
- 建議不要過於頻繁調用
- 可以結合快取機制優化

### **4. 業務邏輯**
- 此方法只負責狀態檢查
- 具體的處理邏輯需要在調用方實現

## 🔧 **測試方法**

### **運行測試腳本**
```bash
cd /app
python3 wcs_ws/src/kuka_wcs/tests/test_check_room_carrier.py
```

### **手動測試步驟**
1. 確保資料庫中有測試資料
2. 創建不同的載具和任務組合
3. 觀察方法返回值是否符合預期
4. 檢查日誌輸出是否正確

## 📈 **擴展建議**

### **1. 增加更多檢查條件**
```python
# 可以添加載具狀態檢查
if carrier.status_id not in [1, 2]:  # 只檢查正常狀態的載具
    continue

# 可以添加時間條件
if task_age > threshold:  # 只考慮超過一定時間的任務
    continue
```

### **2. 優化查詢效能**
```python
# 使用 JOIN 查詢減少資料庫訪問次數
query = select(Carrier, Task).join(Task, Carrier.room_id == Task.room_id)
```

### **3. 增加配置選項**
```python
# 可配置的出口傳送箱位置
UNLOADER_BOX_LOCATIONS = config.get('unloader_locations', [10201, 20002])
```

這個方法為您的 WCS 系統提供了重要的狀態監控功能，幫助及時發現需要處理的載具和任務狀況。
