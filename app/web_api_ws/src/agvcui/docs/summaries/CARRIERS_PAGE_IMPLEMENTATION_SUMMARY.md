# 📦 AGVCUI Carriers 頁面分組功能實現總結

## 🎯 實現目標

為 AGVCUI 中的 carriers 頁面實現分組顯示功能，讓管理員能夠按照載具的實際位置（房間/貨架/端口）進行管理，並提供貨架格位視覺化和完整的載具狀態管理。

## ✅ 已完成的功能

### 1. 載具狀態管理系統

#### CarrierStatus 模型
```python
class CarrierStatus(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    description: Optional[str] = None
    color: Optional[str] = Field(default="is-light")  # Bulma CSS 顏色類別
```

#### 8種預設狀態
| ID | 狀態名稱 | 描述 | 顏色類別 |
|----|---------|------|----------|
| 1 | 空閒 | 載具空閒，可以使用 | `is-success` |
| 2 | 使用中 | 載具正在使用中 | `is-warning` |
| 3 | 故障 | 載具發生故障 | `is-danger` |
| 4 | 待處理 | 載具等待處理 | `is-info` |
| 5 | 處理中 | 載具正在處理製程 | `is-primary` |
| 6 | NG | 載具處理結果不良 | `is-dark` |
| 7 | 維護中 | 載具正在維護 | `is-light` |
| 8 | 已完成 | 載具處理完成 | `is-link` |

### 2. 分組顯示功能

#### 四大分組類別
```python
def get_carriers_grouped():
    return {
        'room_carriers': [],      # 房間內載具
        'rack_carriers': [],      # 貨架上載具  
        'port_carriers': [],      # 設備端口載具
        'unassigned_carriers': [] # 未分配載具
    }
```

#### 分組顯示特色
- 🏠 **房間內載具**: 卡片式顯示，顯示房間名稱和載具狀態
- 📦 **貨架上載具**: 按貨架分組，包含格位視覺化
- 🔌 **設備端口載具**: 顯示設備名稱和端口資訊
- ❓ **未分配載具**: 顯示沒有指定位置的載具

### 3. 貨架格位視覺化

#### S 產品配置 (32格)
```
A面 (格位 1-16)     B面 (格位 17-32)
┌─┬─┬─┬─┐         ┌─┬─┬─┬─┐
│1│2│3│4│         │17│18│19│20│
├─┼─┼─┼─┤         ├─┼─┼─┼─┤
│5│6│7│8│         │21│22│23│24│
├─┼─┼─┼─┤         ├─┼─┼─┼─┤
│9│10│11│12│      │25│26│27│28│
├─┼─┼─┼─┤         ├─┼─┼─┼─┤
│13│14│15│16│     │29│30│31│32│
└─┴─┴─┴─┘         └─┴─┴─┴─┘
```

#### L 產品配置 (16格)
```
A面 (格位 1-8)      B面 (格位 9-16)
┌─┬─┬─┬─┐         ┌─┬─┬─┬─┐
│1│2│3│4│         │9│10│11│12│
├─┼─┼─┼─┤         ├─┼─┼─┼─┤
│5│6│7│8│         │13│14│15│16│
└─┴─┴─┴─┘         └─┴─┴─┴─┘
```

### 4. 完整的 CRUD 功能

#### 後端路由
```python
GET  /carriers                    # 分組列表頁面
GET  /carriers/create             # 創建表單
POST /carriers/create             # 處理創建
GET  /carriers/{id}/edit          # 編輯表單
POST /carriers/{id}/edit          # 處理編輯
POST /carriers/{id}/delete        # 刪除載具
```

#### 資料庫函數
```python
get_carriers_grouped()            # 獲取分組資料
get_carrier_status_list()         # 獲取狀態列表
create_carrier()                  # 創建載具
update_carrier()                  # 更新載具
delete_carrier()                  # 刪除載具
get_rack_grid_info()              # 獲取貨架格位資訊
```

### 5. 載具位置管理

#### 位置類型
- **房間 (Room)**: 載具在房間內自由移動
- **貨架 (Rack)**: 載具固定在特定格位
- **端口 (Port)**: 載具在設備端口進行處理
- **未分配**: 載具沒有指定位置

#### 位置切換
載具編輯表單提供標籤式介面，讓用戶選擇位置類型：
- 🏠 房間標籤: 選擇房間
- 📦 貨架標籤: 選擇貨架和格位索引
- 🔌 端口標籤: 選擇設備端口
- ❓ 未分配標籤: 清除所有位置

### 6. 雙檢視模式

#### 分組檢視 (預設)
- 按位置類型分組顯示
- 貨架格位視覺化
- 卡片式載具資訊

#### 列表檢視
- 傳統表格顯示
- 完整載具資訊
- 便於批量操作

### 7. 前端互動功能

#### 檢視切換
```javascript
document.getElementById('view-mode').addEventListener('change', function() {
    const mode = this.value;
    if (mode === 'grouped') {
        // 顯示分組檢視
    } else {
        // 顯示列表檢視
    }
});
```

#### 貨架格位樣式
```css
.rack-slot {
    width: 40px;
    height: 40px;
    border: 1px solid #dbdbdb;
    display: flex;
    align-items: center;
    justify-content: center;
    cursor: pointer;
    transition: all 0.2s;
}

.rack-slot.occupied {
    background: #3273dc;
    color: white;
    font-weight: bold;
}
```

## 🔧 技術特點

### 1. 資料庫設計
- **CarrierStatus 表**: 管理載具狀態
- **外鍵關聯**: carrier.status_id → carrier_status.id
- **位置管理**: room_id, rack_id, port_id 互斥設計

### 2. 分組查詢優化
```python
# 使用 JOIN 查詢減少資料庫請求
room_carriers = select(Carrier, Room.name).join(Room, ...)
rack_carriers = select(Carrier, Rack.name).join(Rack, ...)
port_carriers = select(Carrier, EqpPort.name, Eqp.name).join(...)
```

### 3. 前端響應式設計
- Bulma CSS 框架
- 卡片式佈局
- 格位視覺化
- 行動裝置友善

### 4. 權限控制
- 創建: `operator` 權限
- 編輯: `operator` 權限  
- 刪除: `admin` 權限

## 📊 測試驗證

### 測試覆蓋率: 8/8 (100%)
- ✅ 模組導入測試
- ✅ 載具狀態函數測試
- ✅ 載具分組功能測試
- ✅ CRUD 功能測試
- ✅ 模板文件測試
- ✅ 貨架格位視覺化測試
- ✅ 狀態顏色映射測試
- ✅ 位置類型處理測試

### 實際資料驗證
- 📦 12個載具在貨架上
- 📋 8種載具狀態已創建
- 🏗️ 資料庫表格和序列正確設置

## 🎯 業務價值

### 1. 提升管理效率
- 按位置分組，快速定位載具
- 視覺化格位狀態，直觀管理
- 狀態顏色編碼，快速識別問題

### 2. 優化操作流程
- 載具位置一目了然
- 貨架格位佔用狀況清楚
- 設備端口載具狀態透明

### 3. 增強資料完整性
- 載具狀態標準化管理
- 位置資訊準確記錄
- 製程狀態追蹤完整

## 🚀 使用指南

### 1. 查看載具分組
- 訪問 `/carriers` 頁面
- 預設顯示分組檢視
- 可切換到列表檢視

### 2. 管理載具位置
- 點擊 "編輯" 按鈕
- 選擇位置類型標籤
- 設定具體位置資訊

### 3. 監控載具狀態
- 查看狀態顏色標籤
- 了解載具當前狀況
- 追蹤製程進度

### 4. 貨架格位管理
- 查看格位視覺化圖
- 識別空閒和佔用格位
- 規劃載具擺放位置

**Carriers 頁面現在已經從簡單的列表升級為功能完整的載具管理系統！** 🎉
