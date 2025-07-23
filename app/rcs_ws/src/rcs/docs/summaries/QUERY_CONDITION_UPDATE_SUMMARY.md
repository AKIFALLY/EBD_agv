# 查詢條件更新總結

## 修改概述

根據要求，已將所有查詢條件中的 `!= "KUKA400i"` 改為明確指定支援的 CT 車型：
```sql
(model == "Cargo") OR (model == "Loader") OR (model == "Unloader")
```

## 修改的檔案和位置

### 1. `rcs_ws/src/rcs/rcs/ct_manager.py`

#### 1.1 `_load_ct_agvs()` 方法 (第124-133行)
**修改前：**
```python
agvs = session.exec(
    select(AGV).where(
        AGV.enable == 1,
        AGV.model != "KUKA400i"
    )
).all()
```

**修改後：**
```python
agvs = session.exec(
    select(AGV).where(
        AGV.enable == 1,
        # 明確指定支援的 CT 車型
        (AGV.model == "Cargo") |
        (AGV.model == "Loader") |
        (AGV.model == "Unloader")
    )
).all()
```

#### 1.2 `dispatch()` 方法 (第157-165行)
**修改前：**
```python
ct_tasks = session.exec(
    select(Task).where(
        Task.status_id == 1,
        Task.mission_code == None,
        Task.parameters["model"].as_string() != "KUKA400i"
    ).order_by(Task.priority.desc())
).all()
```

**修改後：**
```python
ct_tasks = session.exec(
    select(Task).where(
        Task.status_id == 1,
        Task.mission_code == None,
        # 明確指定支援的 CT 車型
        (Task.parameters["model"].as_string() == "Cargo") |
        (Task.parameters["model"].as_string() == "Loader") |
        (Task.parameters["model"].as_string() == "Unloader")
    ).order_by(Task.priority.desc())
).all()
```

#### 1.3 `get_available_agvs()` 方法 (第384-394行)
**修改前：**
```python
available_agvs = session.exec(
    select(AGV).where(
        AGV.enable == 1,
        AGV.model != "KUKA400i",
        AGV.status_id == 3
    )
).all()
```

**修改後：**
```python
available_agvs = session.exec(
    select(AGV).where(
        AGV.enable == 1,
        # 明確指定支援的 CT 車型
        (AGV.model == "Cargo") |
        (AGV.model == "Loader") |
        (AGV.model == "Unloader"),
        AGV.status_id == 3
    )
).all()
```

#### 1.4 `get_task_statistics()` 方法 (第468-502行)
所有統計查詢都已更新為明確指定車型條件。

## 修改的優點

1. **明確性**：明確指定支援的車型，避免意外包含其他車型
2. **可維護性**：當新增車型時，需要明確添加到查詢條件中
3. **安全性**：避免因為車型命名變更而導致的邏輯錯誤
4. **可讀性**：程式碼意圖更加清晰，容易理解

## 支援的車型

目前系統支援的 CT 車型：
- **Cargo**：負責房外作業
- **Loader**：負責房內裝載作業  
- **Unloader**：負責房內卸載作業

## 驗證

所有修改已完成，可以通過以下方式驗證：

1. **語法檢查**：程式碼無語法錯誤
2. **功能測試**：執行測試腳本進行功能驗證
3. **邏輯驗證**：確保查詢條件符合業務邏輯

## 注意事項

如果未來需要新增新的車型（如 Transporter、Conveyor 等），需要在以下位置添加對應的查詢條件：
1. `ct_manager.py` 中的所有查詢方法
2. 測試腳本中的測試查詢
3. `_validate_task_parameters()` 方法中的車型驗證邏輯

## 影響範圍

- **主要模組**：ct_manager.py
- **測試檔案**：test_ct_dispatch.py
- **查詢方法**：4個主要查詢方法
- **統計功能**：3個統計查詢

## 後續維護

1. **新增車型時**：記得更新所有相關查詢條件
2. **測試覆蓋**：確保新車型的測試案例完整
3. **文件更新**：同步更新相關技術文件
