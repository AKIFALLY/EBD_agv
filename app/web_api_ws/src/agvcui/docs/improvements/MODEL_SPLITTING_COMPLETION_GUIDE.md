# 📁 Model 拆分完成指南

## ✅ 已完成的拆分

### 1. agvc_wcs.py → 4 個獨立檔案
- ✅ `room.py` - Room model
- ✅ `rack_status.py` - RackStatus model  
- ✅ `rack.py` - Rack model
- ✅ `carrier.py` - Carrier model

### 2. agvc_base.py → 9 個獨立檔案
- ✅ `log_level.py` - LogLevel enum
- ✅ `rosout_log.py` - RosoutLog model
- ✅ `runtime_log.py` - RuntimeLog model
- ✅ `modify_log.py` - ModifyLog model
- ✅ `node.py` - Node model
- ✅ `edge.py` - Edge model
- ✅ `node_type.py` - NodeType model

### 3. agvc_client.py → 3 個獨立檔案
- ✅ `machine.py` - Machine model
- ✅ `client.py` - Client model
- ✅ `user.py` - User model

## 🔄 需要完成的拆分

### 4. agvc_eqp.py → 3 個檔案
```python
# eqp.py
from typing import Optional, List
from sqlmodel import SQLModel, Field, Relationship
from pydantic import ConfigDict

class Eqp(SQLModel, table=True):
    __tablename__ = "eqp"
    id: Optional[int] = Field(default=None, primary_key=True)
    location_id: Optional[int] = None
    name: str
    description: Optional[str] = None
    
    # Relationships
    signals: List["EqpSignal"] = Relationship(back_populates="eqp")
    ports: List["EqpPort"] = Relationship(back_populates="eqp")
    
    model_config = ConfigDict(from_attributes=True)

# eqp_port.py
from typing import Optional
from sqlmodel import SQLModel, Field, Relationship
from pydantic import ConfigDict

class EqpPort(SQLModel, table=True):
    __tablename__ = "eqp_port"
    id: Optional[int] = Field(default=None, primary_key=True)
    eqp_id: int = Field(foreign_key="eqp.id")
    name: str
    description: Optional[str] = None
    
    eqp: Optional["Eqp"] = Relationship(back_populates="ports")
    
    model_config = ConfigDict(from_attributes=True)

# eqp_signal.py
from typing import Optional
from sqlmodel import SQLModel, Field, Relationship
from pydantic import ConfigDict

class EqpSignal(SQLModel, table=True):
    __tablename__ = "eqp_signal"
    id: Optional[int] = Field(default=None, primary_key=True)
    eqp_id: int = Field(foreign_key="eqp.id")
    eqp_port_id: Optional[int] = Field(default=None, foreign_key="eqp_port.id")
    name: str
    description: Optional[str] = None
    value: str
    type_of_value: str
    dm_address: Optional[str] = None
    
    eqp: Optional["Eqp"] = Relationship(back_populates="signals")
    
    model_config = ConfigDict(from_attributes=True)
```

### 5. agvc_location.py → 2 個檔案
```python
# location.py
# location_status.py
```

### 6. agvc_product.py → 2 個檔案
```python
# process_settings.py
# product.py
```

### 7. agvc_rcs.py → 3 個檔案
```python
# agv.py
# agv_context.py
# traffic_zone.py
```

### 8. agvc_task.py → 3 個檔案
```python
# task.py
# task_status.py
# work.py
```

### 9. agvc_kuka.py → 2 個檔案
```python
# kuka_node.py
# kuka_edge.py
```

## 🔧 拆分步驟

### 對於每個要拆分的檔案：

1. **創建新的單獨檔案**
   ```bash
   # 例如拆分 agvc_eqp.py
   touch eqp.py eqp_port.py eqp_signal.py
   ```

2. **複製對應的 model 到新檔案**
   - 包含必要的 imports
   - 保持 model 定義完整
   - 保留 relationships（如果有）

3. **更新 __init__.py**
   ```python
   # 替換
   from db_proxy.models.agvc_eqp import Eqp, EqpPort, EqpSignal
   
   # 為
   from db_proxy.models.eqp import Eqp
   from db_proxy.models.eqp_port import EqpPort
   from db_proxy.models.eqp_signal import EqpSignal
   ```

4. **刪除原檔案**
   ```bash
   rm agvc_eqp.py
   ```

5. **測試引用**
   - 確認所有引用都正常工作
   - 運行測試確保沒有破壞

## 📋 檢查清單

### 拆分前檢查
- [ ] 確認檔案中有哪些 models
- [ ] 檢查是否有 relationships
- [ ] 查看是否有特殊的 imports

### 拆分後檢查
- [ ] 新檔案包含正確的 imports
- [ ] Model 定義完整
- [ ] Relationships 正確設置
- [ ] __init__.py 已更新
- [ ] 原檔案已刪除
- [ ] 所有引用都正常工作

## 🎯 最終目標結構

```
models/
├── __init__.py
├── # Base models
├── log_level.py
├── rosout_log.py
├── runtime_log.py
├── modify_log.py
├── node.py
├── edge.py
├── node_type.py
├── ct_node.py
├── ct_edge.py
├── # Client models
├── client.py
├── machine.py
├── user.py
├── # Equipment models
├── eqp.py
├── eqp_port.py
├── eqp_signal.py
├── # Location models
├── location.py
├── location_status.py
├── # Product models
├── process_settings.py
├── product.py
├── # RCS models
├── agv.py
├── agv_context.py
├── traffic_zone.py
├── # Task models
├── task.py
├── task_status.py
├── work.py
├── # WCS models
├── room.py
├── rack_status.py
├── rack.py
├── carrier.py
├── # Kuka models
├── kuka_node.py
└── kuka_edge.py
```

## 🚀 完成後的好處

1. **更好的組織結構**
   - 每個 model 都有自己的檔案
   - 更容易找到和維護

2. **更清晰的依賴關係**
   - 可以清楚看到每個 model 的依賴
   - 避免循環引用

3. **更好的可維護性**
   - 修改單個 model 不會影響其他
   - 更容易進行版本控制

4. **更好的可讀性**
   - 檔案更小，更容易理解
   - 符合單一職責原則

---

**進度**: 3/9 個檔案已完成拆分 (33%)
**下一步**: 繼續拆分剩餘的 6 個檔案
