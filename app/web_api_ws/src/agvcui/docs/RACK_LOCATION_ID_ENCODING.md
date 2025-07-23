# 貨架 Location ID 編碼規則

## 概述

本文檔定義了 Dashboard 儀表板中房間貨架狀態檢測使用的 location_id 編碼規則，確保系統能夠準確識別和監控各房間的入口和出口貨架狀態。

## 編碼規則

### 基本格式

```
location_id = {房間ID}{位置類型}00
```

### 位置類型定義

- **01**: 入口 (Entrance)
- **02**: 出口 (Exit)

### 編碼示例

| 房間ID | 位置 | Location ID | 說明 |
|--------|------|-------------|------|
| 1 | 入口 | 10100 | 房間1入口 |
| 1 | 出口 | 10200 | 房間1出口 |
| 2 | 入口 | 20100 | 房間2入口 |
| 2 | 出口 | 20200 | 房間2出口 |
| 3 | 入口 | 30100 | 房間3入口 |
| 3 | 出口 | 30200 | 房間3出口 |
| 4 | 入口 | 40100 | 房間4入口 |
| 4 | 出口 | 40200 | 房間4出口 |
| 5 | 入口 | 50100 | 房間5入口 |
| 5 | 出口 | 50200 | 房間5出口 |

## 技術實現

### JavaScript 編碼邏輯

```javascript
/**
 * 計算房間入口和出口的 location_id
 * @param {number} roomId - 房間 ID
 * @returns {Object} 包含入口和出口 location_id 的對象
 */
function calculateRoomLocationIds(roomId) {
    return {
        entrance: roomId * 10000 + 100,  // 入口
        exit: roomId * 10000 + 200       // 出口
    };
}

// 使用示例
const room2Locations = calculateRoomLocationIds(2);
console.log(room2Locations); 
// 輸出: { entrance: 20100, exit: 20200 }
```

### 貨架狀態檢測邏輯

```javascript
/**
 * 檢查房間的貨架狀態
 * @param {number} roomId - 房間 ID
 * @param {Array} racks - 貨架列表
 */
function checkRoomRackStatus(roomId, racks) {
    // 計算精確的 location_id
    const entranceLocationId = roomId * 10000 + 100;
    const exitLocationId = roomId * 10000 + 200;
    
    // 精確匹配檢查
    const entranceRacks = racks.filter(rack =>
        rack.location_id === entranceLocationId && rack.count > 0
    );
    
    const exitRacks = racks.filter(rack =>
        rack.location_id === exitLocationId && rack.count > 0
    );
    
    return {
        entrance: {
            hasRack: entranceRacks.length > 0,
            racks: entranceRacks,
            status: entranceRacks.length > 0 
                ? `貨架 ${entranceRacks[0].name || entranceRacks[0].id}` 
                : '無貨架'
        },
        exit: {
            hasRack: exitRacks.length > 0,
            racks: exitRacks,
            status: exitRacks.length > 0 
                ? `貨架 ${exitRacks[0].name || exitRacks[0].id}` 
                : '無貨架'
        }
    };
}
```

## 修正歷史

### 修正前的問題

**錯誤的範圍檢查邏輯：**
```javascript
// ❌ 錯誤：使用範圍陣列檢查
const entranceLocationIds = Array.from({length: 11}, (_, i) => roomId * 100 + i);
const exitLocationIds = Array.from({length: 10}, (_, i) => roomId * 100 + 90 + i);

// 這會產生錯誤的 location_id 範圍：
// 房間2入口：200-210 (錯誤)
// 房間2出口：290-299 (錯誤)
```

**模糊的狀態顯示：**
```javascript
// ❌ 錯誤：只顯示「有貨架」或「無貨架」
const entranceStatus = rackStatus.entranceHasRack ? '有貨架' : '無貨架';
```

### 修正後的改善

**精確的 location_id 匹配：**
```javascript
// ✅ 正確：使用精確的 location_id 匹配
const entranceLocationId = roomId * 10000 + 100; // 房間2入口：20100
const exitLocationId = roomId * 10000 + 200;     // 房間2出口：20200
```

**具體的貨架資訊顯示：**
```javascript
// ✅ 正確：顯示具體的貨架 ID 或名稱
const entranceStatus = entranceRacks.length > 0 
    ? `貨架 ${entranceRacks[0].name || entranceRacks[0].id}` 
    : '無貨架';
```

## 測試驗證

### 編碼規則測試

```python
def test_room_location_id_encoding():
    """測試房間 location_id 編碼規則"""
    test_cases = [
        {'room_id': 1, 'entrance': 10100, 'exit': 10200},
        {'room_id': 2, 'entrance': 20100, 'exit': 20200},
        {'room_id': 3, 'entrance': 30100, 'exit': 30200},
        {'room_id': 4, 'entrance': 40100, 'exit': 40200},
        {'room_id': 5, 'entrance': 50100, 'exit': 50200},
    ]
    
    for case in test_cases:
        room_id = case['room_id']
        actual_entrance = room_id * 10000 + 100
        actual_exit = room_id * 10000 + 200
        
        assert actual_entrance == case['entrance']
        assert actual_exit == case['exit']
```

### 貨架狀態檢測測試

```python
def test_room2_rack_status():
    """測試房間2貨架狀態檢測"""
    racks = [
        {"id": 101, "name": "Rack-R2-IN-01", "location_id": 20100, "count": 16},
        {"id": 102, "name": "Rack-R2-OUT-01", "location_id": 20200, "count": 8}
    ]
    
    # 使用新的編碼規則檢測
    entrance_racks = [r for r in racks if r['location_id'] == 20100 and r['count'] > 0]
    exit_racks = [r for r in racks if r['location_id'] == 20200 and r['count'] > 0]
    
    assert len(entrance_racks) == 1
    assert len(exit_racks) == 1
    assert entrance_racks[0]['name'] == 'Rack-R2-IN-01'
    assert exit_racks[0]['name'] == 'Rack-R2-OUT-01'
```

## 資料庫配置

### 貨架資料表結構

確保資料庫中的貨架資料使用正確的 location_id：

```sql
-- 房間2貨架配置示例
INSERT INTO rack (id, name, location_id, total, count) VALUES
(101, 'Rack-R2-IN-01', 20100, 32, 16),   -- 房間2入口
(102, 'Rack-R2-OUT-01', 20200, 32, 8);   -- 房間2出口

-- 房間1貨架配置示例
INSERT INTO rack (id, name, location_id, total, count) VALUES
(201, 'Rack-R1-IN-01', 10100, 32, 12),   -- 房間1入口
(202, 'Rack-R1-OUT-01', 10200, 32, 0);   -- 房間1出口
```

## 向後兼容性

### 遷移指南

如果系統中存在使用舊編碼規則的資料，需要進行資料遷移：

```sql
-- 資料遷移腳本示例
-- 將舊的 location_id 轉換為新的編碼格式

-- 房間1入口：101 -> 10100
UPDATE rack SET location_id = 10100 WHERE location_id = 101;

-- 房間1出口：191 -> 10200  
UPDATE rack SET location_id = 10200 WHERE location_id = 191;

-- 房間2入口：201 -> 20100
UPDATE rack SET location_id = 20100 WHERE location_id = 201;

-- 房間2出口：291 -> 20200
UPDATE rack SET location_id = 20200 WHERE location_id = 291;
```

### 驗證腳本

```sql
-- 驗證新編碼規則的資料完整性
SELECT 
    FLOOR(location_id / 10000) as room_id,
    CASE 
        WHEN location_id % 10000 = 100 THEN '入口'
        WHEN location_id % 10000 = 200 THEN '出口'
        ELSE '未知'
    END as position_type,
    location_id,
    name,
    count
FROM rack 
WHERE location_id >= 10000
ORDER BY room_id, position_type;
```

## 最佳實踐

1. **一致性**: 所有房間都應使用相同的編碼規則
2. **可擴展性**: 編碼格式支援最多99個房間
3. **可讀性**: location_id 本身就包含了房間和位置資訊
4. **精確性**: 使用精確匹配而非範圍檢查，避免誤判
5. **可維護性**: 集中管理編碼邏輯，便於修改和擴展

## 故障排除

### 常見問題

1. **貨架狀態不更新**
   - 檢查 location_id 是否使用正確的編碼格式
   - 確認 WebSocket 事件中的資料結構

2. **顯示「無貨架」但實際有貨架**
   - 檢查 `rack.count > 0` 條件
   - 驗證 location_id 匹配邏輯

3. **多個房間顯示相同貨架**
   - 確認 location_id 的唯一性
   - 檢查編碼計算邏輯

### 調試工具

```javascript
// 調試函數：顯示房間的 location_id 資訊
function debugRoomLocationIds(roomId) {
    const entrance = roomId * 10000 + 100;
    const exit = roomId * 10000 + 200;
    
    console.log(`房間${roomId}:`);
    console.log(`  入口 location_id: ${entrance}`);
    console.log(`  出口 location_id: ${exit}`);
    
    return { entrance, exit };
}

// 使用示例
debugRoomLocationIds(2); // 輸出房間2的 location_id 資訊
```

這個新的編碼規則確保了系統的準確性、可維護性和可擴展性，為 Dashboard 儀表板提供了可靠的房間貨架狀態監控功能。
