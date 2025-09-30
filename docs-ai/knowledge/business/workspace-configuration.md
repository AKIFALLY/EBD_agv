# 工作區配置功能設計

## 🎯 適用場景
- 理解工作區與停車格分離設計的業務背景和技術實作
- 為 OPUI 系統的料架管理功能提供技術指導
- 解決工作區配置相關的問題和優化

## 📋 業務背景

### 問題起源
在原始設計中，料架的存放位置只有停車格（parking space），這導致了以下問題：
1. **容量限制**：每個操作員只有一個停車格，限制了料架暫存容量
2. **職責混淆**：停車格既用於暫存也用於派送，職責不明確
3. **效率問題**：當停車格被佔用時，無法加入新的料架

### 解決方案
引入**工作區（workspace）**概念，將料架存放區域分為兩個獨立部分：
- **工作區**：用於暫時存放料架，容量可配置
- **停車格**：專門用於 AGV 派送任務

## 🏗️ 架構設計

### 資料結構設計
```
Machine (射出機)
├── workspace_1: INTEGER[]      # 左側工作區位置陣列
├── workspace_2: INTEGER[]      # 右側工作區位置陣列
├── parking_space_1: INTEGER    # 左側停車格位置
└── parking_space_2: INTEGER    # 右側停車格位置
```

### 職責分離原則
| 區域 | 用途 | 容量 | 使用場景 |
|------|------|------|----------|
| **工作區** | 暫時存放 | 可配置多個 | add_rack 操作 |
| **停車格** | AGV 派送 | 固定1個 | dispatch_full 操作 |

## 💻 技術實作

### 資料庫設計
```sql
-- PostgreSQL 支援 INTEGER[] 陣列類型
ALTER TABLE machine
ADD COLUMN workspace_1 INTEGER[],
ADD COLUMN workspace_2 INTEGER[];

-- 範例資料
UPDATE machine SET
  workspace_1 = ARRAY[101, 102, 103],
  workspace_2 = ARRAY[104, 105, 106]
WHERE id = 1;
```

### SQLModel 模型定義
```python
from sqlalchemy import Column, Integer, ARRAY
from sqlalchemy.ext.mutable import MutableList
from sqlmodel import Field
from typing import Optional, List

class Machine(SQLModel, table=True):
    # 新增工作區陣列欄位
    workspace_1: Optional[List[int]] = Field(
        default=None,
        sa_column=Column(
            MutableList.as_mutable(ARRAY(Integer)),
            nullable=True
        ),
        description="作業員1(左側)的工作區location ID陣列"
    )
    workspace_2: Optional[List[int]] = Field(
        default=None,
        sa_column=Column(
            MutableList.as_mutable(ARRAY(Integer)),
            nullable=True
        ),
        description="作業員2(右側)的工作區location ID陣列"
    )
```

### 加入料架邏輯實作
```python
async def add_rack(self, sid, data):
    """新增料架到工作區（自動選擇第一個可用位置）"""
    # 1. 根據操作員側選擇工作區
    if side == "left":
        workspace_locations = machine.workspace_1 or []
    else:
        workspace_locations = machine.workspace_2 or []

    # 2. 查詢可用的工作區位置
    available_location = None
    for location_id in workspace_locations:
        existing_rack = rack_crud.get_by_field(
            session, "location_id", location_id
        )
        if not existing_rack:  # 位置可用
            available_location = location_id
            break

    # 3. 檢查是否有可用位置
    if not available_location:
        return {
            "success": False,
            "message": f"{side_name} 工作區已滿，請等待料架派送完成"
        }

    # 4. 分配料架到工作區
    rack.location_id = available_location
    rack_crud.update(session, rack.id, rack)
```

### 派車邏輯實作
```python
async def dispatch_full(self, sid, data):
    """派滿車任務（從工作區移動到停車格）"""
    # 1. 檢查料架是否在工作區中
    if side == "left":
        workspace_locations = machine.workspace_1 or []
        parking_space = machine.parking_space_1
    else:
        workspace_locations = machine.workspace_2 or []
        parking_space = machine.parking_space_2

    # 2. 確認料架在工作區中（可選檢查）
    if rack.location_id not in workspace_locations:
        print(f"⚠️ 料架不在工作區中")

    # 3. 移動料架到停車格
    rack.location_id = parking_space
    rack_crud.update(session, rack.id, rack)

    # 4. 創建派送任務
    task = create_dispatch_task(...)
```

## 🧪 測試案例

### 測試腳本
```python
#!/usr/bin/env python3
# agents/test_workspace_config.py

def test_workspace_configuration():
    """測試工作區配置"""
    # 查詢機器工作區配置
    machines = session.query(Machine).all()
    for machine in machines:
        print(f"{machine.name}:")
        print(f"  工作區1: {machine.workspace_1}")
        print(f"  工作區2: {machine.workspace_2}")

def test_add_rack_logic():
    """測試 add_rack 邏輯"""
    # 模擬 add_rack 邏輯
    if machine.workspace_1:
        selected_location = machine.workspace_1[0]
        print(f"選擇工作區位置: {selected_location}")

def test_workspace_full():
    """測試工作區滿載情況"""
    # 填滿工作區
    for location_id in machine.workspace_1:
        rack = Rack(name=f"RACK-{location_id}",
                   location_id=location_id)
        session.add(rack)

    # 嘗試再加入料架（應該失敗）
    # 預期結果：返回「工作區已滿」錯誤
```

### 邊界情況測試
1. **工作區未配置**：workspace_1 = NULL
   - 預期：返回「工作區未配置」錯誤

2. **工作區已滿**：所有位置都有料架
   - 預期：返回「工作區已滿」錯誤

3. **料架 location 為 NULL**：料架未分配位置
   - 預期：可以成功加入到工作區

## 🔍 常見問題

### Q1：為什麼使用 PostgreSQL 陣列而不是關聯表？
**A**：考慮到工作區位置數量有限且固定，使用陣列更簡單直接：
- 減少表關聯查詢
- 簡化資料結構
- 提高查詢效率
- 易於配置和維護

### Q2：工作區位置如何確定？
**A**：工作區位置對應實際的 Location 記錄：
- 每個 location_id 對應 location 表中的一筆記錄
- 位置應該預先在系統中定義
- 通常與實體空間的料架位置對應

### Q3：如何處理向後相容性？
**A**：保持與舊版本的相容：
- 保留原有的 parking_space_1/2 欄位
- dispatch_full 仍然支援從停車格派送
- 如果 workspace 為 NULL，使用停車格作為後備方案

## 📊 配置範例

### 機台配置範例
```python
# 射出機1配置
machine_1 = {
    "id": 1,
    "name": "射出機1",
    "workspace_1": [101, 102, 103],  # 左側3個工作區
    "workspace_2": [104, 105, 106],  # 右側3個工作區
    "parking_space_1": 95,           # 左側停車格
    "parking_space_2": 96,           # 右側停車格
}

# 射出機2配置
machine_2 = {
    "id": 2,
    "name": "射出機2",
    "workspace_1": [201, 202, 203],  # 左側3個工作區
    "workspace_2": [204, 205, 206],  # 右側3個工作區
    "parking_space_1": 97,           # 左側停車格
    "parking_space_2": 98,           # 右側停車格
}
```

### 容量配置建議
- **小型生產線**：每側 2-3 個工作區位置
- **中型生產線**：每側 4-6 個工作區位置
- **大型生產線**：每側 8-10 個工作區位置

## 🚀 未來優化方向

### 短期優化
1. **UI 顯示優化**：限制前端顯示的料架數量與實際容量匹配
2. **狀態追蹤**：新增工作區使用率統計
3. **優先級管理**：為工作區位置設定優先級

### 長期規劃
1. **動態配置**：支援透過 UI 動態調整工作區容量
2. **智能分配**：基於歷史資料優化位置分配策略
3. **預測功能**：預測工作區使用模式，提前準備

## 🔗 相關文檔
- OPUI 系統文檔：/app/web_api_ws/src/opui/CLAUDE.md
- 資料庫設計：/app/db_proxy_ws/CLAUDE.md
- 測試腳本：/home/ct/RosAGV/agents/test_workspace_config.py

---

*文檔版本：2025-09*
*最後更新：2025-09-23*
*狀態：已實作並測試*