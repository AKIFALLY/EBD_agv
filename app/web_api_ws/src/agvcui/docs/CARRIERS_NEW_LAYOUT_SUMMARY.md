# 📦 Carriers 頁面新版排版設計完成

## 🎯 設計目標

根據用戶需求重新設計 carriers 頁面排版，分為「貨架上」和「房間內」兩大類別，並根據設備的 `location_id` 來判斷房間歸屬。

## ✅ 新版排版結構

### 1. 🏠 房間內載具 (按 location_id 分組)

#### 房間判斷邏輯
- **location_id 1xx (100-199)** → 房間 1
- **location_id 2xx (200-299)** → 房間 2  
- **location_id 3xx (300-399)** → 房間 3
- **location_id 4xx (400-499)** → 房間 4
- **location_id 5xx (500-599)** → 房間 5

#### 顯示結構
```
房間 2 (3個載具) [可收合]
├── 設備: LoaderAGV (Location 210) [可收合]
│   └── 端口: LoaderAGV_Port01 → 載具 #12
├── 設備: Room2_BoxOut (Location 202) [可收合]
│   └── 端口: Room2_BoxOut_Port03 → 載具 #11
└── 設備: Room2_Cleaner (Location 203) [可收合]
    └── 端口: Room2_Cleaner_Port02 → 載具 #10
```

#### 特色功能
- **三層收合**: 房間 → 設備 → 端口載具
- **設備資訊**: 顯示設備名稱和 location_id
- **端口詳細**: 表格顯示端口名稱、載具和狀態
- **載具統計**: 顯示每個房間和設備的載具數量

### 2. 📦 貨架上載具 (保持原有設計)

#### 顯示結構
```
貨架上載具 (9個載具) [可收合]
├── 貨架 123 (7個載具) [可收合]
│   ├── 格位視覺化 (A面/B面)
│   └── 載具詳細列表
├── 貨架 2 (1個載具) [可收合]
└── 貨架 3 (1個載具) [可收合]
```

#### 特色功能
- **格位視覺化**: S產品32格，L產品16格
- **A/B面顯示**: 清楚標示格位佔用狀況
- **載具列表**: 表格顯示載具ID、格位、狀態

### 3. ❓ 未分配載具

顯示沒有分配到貨架或設備端口的載具。

## 🔧 技術實現

### 資料庫查詢邏輯

```python
def get_carriers_grouped():
    # 1. 獲取貨架上的載具
    rack_carriers = select(Carrier, Rack.name).join(Rack).where(Carrier.rack_id.is_not(None))
    
    # 2. 獲取設備端口上的載具，按 location_id 分房間
    port_carriers = select(
        Carrier, EqpPort.name, Eqp.name, Eqp.id, Eqp.location_id
    ).join(EqpPort).join(Eqp).where(Carrier.port_id.is_not(None))
    
    # 3. 根據 location_id 判斷房間
    room_number = location_id // 100  # 200-299 -> 2, 100-199 -> 1
    
    # 4. 按房間 → 設備 → 端口三層結構分組
    return {
        'rack_groups': [...],     # 貨架分組
        'room_groups': [...],     # 房間分組
        'unassigned_carriers': [...] # 未分配載具
    }
```

### 前端模板結構

```html
<!-- 房間內載具 -->
{% for room_group in grouped_carriers.room_groups %}
<div class="box mb-5">
    <h2 onclick="toggleSection('room-{{ room_group.room_number }}')">
        {{ room_group.room_name }} ({{ total_carriers }} 個載具)
    </h2>
    <div id="room-{{ room_group.room_number }}-content">
        <!-- 房間內的設備 -->
        {% for equipment in room_group.equipment %}
        <div class="box is-light mb-4">
            <h3 onclick="toggleSection('eqp-{{ equipment.eqp_id }}')">
                {{ equipment.eqp_name }} (Location {{ equipment.location_id }})
            </h3>
            <div id="eqp-{{ equipment.eqp_id }}-content">
                <!-- 設備端口表格 -->
                <table class="table">
                    {% for port in equipment.ports %}
                    <tr>
                        <td>{{ port.port_name }}</td>
                        <td>載具 #{{ carrier.id }}</td>
                        <td>{{ carrier.status }}</td>
                    </tr>
                    {% endfor %}
                </table>
            </div>
        </div>
        {% endfor %}
    </div>
</div>
{% endfor %}
```

## 📊 實際資料驗證

### 當前系統資料
- **房間 2**: 3個設備，3個載具
  - LoaderAGV (Location 210): 1個載具
  - Room2_BoxOut (Location 202): 1個載具  
  - Room2_Cleaner (Location 203): 1個載具
- **貨架**: 3個貨架，9個載具
  - 貨架 123: 7個載具
  - 貨架 2: 1個載具
  - 貨架 3: 1個載具
- **未分配**: 1個載具

### 顯示效果
- ✅ 房間按 location_id 正確分組
- ✅ 設備按房間正確歸類
- ✅ 載具按端口正確顯示
- ✅ 三層收合展開功能正常

## 🎨 用戶體驗

### 視覺層次
1. **第一層**: 房間分組 (房間1-5)
2. **第二層**: 設備分組 (按 location_id)
3. **第三層**: 端口載具 (表格顯示)

### 互動功能
- **收合展開**: 三層獨立收合控制
- **視覺指示**: 箭頭圖示和 hover 效果
- **載具統計**: 即時顯示各層級載具數量
- **狀態顏色**: 載具狀態的顏色編碼

### 資訊密度
- **概覽模式**: 收合狀態下快速查看統計
- **詳細模式**: 展開狀態下查看完整資訊
- **靈活切換**: 用戶可自由控制顯示層級

## 🔄 與舊版對比

| 項目 | 舊版設計 | 新版設計 |
|------|----------|----------|
| 分組方式 | 房間/貨架/端口/未分配 | 房間(按location_id)/貨架 |
| 房間判斷 | 載具的 room_id | 設備的 location_id |
| 設備顯示 | 分散在端口分組 | 集中在房間內 |
| 層級結構 | 2層 (分組→載具) | 3層 (房間→設備→載具) |
| 收合功能 | 2層收合 | 3層收合 |
| 資訊完整性 | 端口資訊分散 | 設備端口集中顯示 |

## 🚀 業務價值

### 1. 符合實際作業流程
- **房間管理**: 按實際房間1-5分組管理
- **設備維護**: 清楚顯示每個房間的設備狀況
- **端口監控**: 詳細顯示載具在哪個端口

### 2. 提升管理效率
- **快速定位**: 房間→設備→端口的清晰路徑
- **狀態監控**: 一目了然的載具分佈狀況
- **問題排查**: 快速找到特定設備的載具問題

### 3. 優化用戶體驗
- **邏輯清晰**: 符合用戶的認知模式
- **操作便利**: 三層收合提供靈活的檢視方式
- **資訊完整**: 設備和端口資訊完整呈現

## 📋 使用指南

### 查看房間內載具
1. 點擊房間標題查看該房間的所有設備
2. 點擊設備標題查看該設備的所有端口
3. 表格中顯示端口名稱、載具ID和狀態

### 查看貨架載具
1. 點擊貨架標題查看格位視覺化
2. A面和B面清楚顯示載具佔用狀況
3. 表格列出所有載具的詳細資訊

### 管理載具
- **編輯載具**: 點擊載具旁的編輯按鈕
- **查看狀態**: 載具狀態用顏色標籤顯示
- **快速統計**: 標題顯示各層級的載具數量

---

**🎉 新版 Carriers 頁面排版設計完成！**

現在載具管理更符合實際作業流程，用戶可以按房間查看設備和載具狀況，同時保持貨架管理的視覺化優勢。三層收合展開功能讓用戶可以靈活控制資訊顯示層級，大幅提升管理效率！
