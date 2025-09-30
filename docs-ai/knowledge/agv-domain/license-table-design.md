# License 資料表設計文檔

## 📋 文件資訊
- **版本**: 1.0
- **建立日期**: 2025-09-19
- **適用對象**: 系統架構師、資料庫管理員、後端開發人員
- **系統版本**: RosAGV 2.0

## 🎯 適用場景
本文檔詳細說明 License 資料表的設計理念、結構定義、關聯關係和使用方式，為資料庫設計和 API 開發提供技術指導。

## 📖 設計理念

### 核心目標
License 資料表是 RosAGV 系統的設備授權管理核心，實現以下目標：
1. **安全控制**：確保只有授權設備能存取系統
2. **權限管理**：精細化控制不同設備的操作權限
3. **擴展性**：支援未來新增設備類型和權限
4. **稽核追蹤**：記錄設備操作歷史

### 設計原則
- **單一職責**：專注於設備授權管理
- **靈活配置**：使用 JSON 欄位支援動態權限配置
- **向後相容**：新增權限不影響現有設備
- **簡潔明瞭**：欄位設計直觀易懂

## 🗄️ 資料表結構

### SQLModel 定義
```python
from sqlmodel import SQLModel, Field, Column
from sqlalchemy import JSON
from typing import Optional

class License(SQLModel, table=True):
    """設備授權資料表"""
    __tablename__ = "license"

    # 主鍵
    id: Optional[int] = Field(default=None, primary_key=True)

    # 設備唯一識別碼（16位字元）
    device_id: str = Field(index=True, unique=True)

    # 授權狀態（1=啟用, 0=停用）
    active: int = Field(default=1)

    # 設備類型（op_station, hmi_terminal, 或其他擴展類型）
    device_type: str = Field(default="op_station")

    # 設備描述
    description: Optional[str] = None

    # 權限配置（JSON格式，支援動態擴展）
    permissions: Optional[dict] = Field(default=None, sa_column=Column(JSON))
```

### PostgreSQL DDL
```sql
CREATE TABLE license (
    id SERIAL PRIMARY KEY,
    device_id VARCHAR(16) NOT NULL UNIQUE,
    active INTEGER DEFAULT 1,
    device_type VARCHAR(50) DEFAULT 'op_station',
    description TEXT,
    permissions JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- 索引
CREATE INDEX idx_license_device_id ON license(device_id);
CREATE INDEX idx_license_active ON license(active);
CREATE INDEX idx_license_device_type ON license(device_type);

-- 觸發器：自動更新 updated_at
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ language 'plpgsql';

CREATE TRIGGER update_license_updated_at BEFORE UPDATE
ON license FOR EACH ROW EXECUTE PROCEDURE update_updated_at_column();
```

## 📊 欄位詳解

### id (Primary Key)
- **類型**: INTEGER (自動遞增)
- **用途**: 資料表主鍵，唯一識別每筆授權記錄
- **特性**: 系統自動產生，不可修改

### device_id (Unique Key)
- **類型**: VARCHAR(16)
- **用途**: 設備的唯一識別碼
- **規範**:
  - 長度必須為 16 個字元
  - 只能包含英文字母（大小寫）和數字
  - 系統中必須唯一
  - 建議使用 MAC 地址或硬體序號
- **範例**: `ca08777c72096c51`, `HMI0000000000001`

### active
- **類型**: INTEGER
- **用途**: 控制設備授權狀態
- **值定義**:
  - `1`: 啟用（設備可正常使用）
  - `0`: 停用（設備被暫時禁止存取）
- **預設值**: `1`
- **使用場景**: 臨時停用設備、設備維護期間

### device_type
- **類型**: VARCHAR(50)
- **用途**: 定義設備類型，決定路由和介面
- **預定義類型**:
  - `op_station`: 操作員工作站（標準 OPUI 介面）
  - `hmi_terminal`: HMI 終端（大按鈕觸控介面）
- **預設值**: `op_station`
- **擴展性**: 可新增其他類型如 `warehouse_operator`, `supervisor_station`

### description
- **類型**: TEXT (可選)
- **用途**: 設備的描述性資訊
- **內容建議**:
  - 設備位置（如「射出機區 #1」）
  - 負責人資訊
  - 用途說明
- **範例**: `人工收料區 HMI 終端`, `控制室操作員工作站 #2`

### permissions
- **類型**: JSONB
- **用途**: 儲存設備的權限配置
- **特性**:
  - 動態擴展，無需修改資料表結構
  - 支援巢狀結構
  - 可查詢和索引

#### op_station 權限結構
```json
{
    "can_call_agv": true,      // 可以呼叫 AGV
    "can_view_tasks": true,     // 可以查看任務
    "can_create_tasks": true,   // 可以建立任務
    "can_modify_tasks": false,  // 可以修改任務（擴展）
    "can_delete_tasks": false   // 可以刪除任務（擴展）
}
```

#### hmi_terminal 權限結構
```json
{
    "locations": [              // 可管理的位置列表
        "ManualReceiveArea01",
        "ManualReceiveArea02"
    ],
    "layout": "2x2",           // 按鈕排版方式
    "can_remove_rack": true,   // 可以移出 Rack
    "can_add_rack": false,     // 可以加入 Rack（擴展）
    "max_operations_per_hour": 100  // 每小時操作次數限制（擴展）
}
```

## 🔗 關聯關係

### 與其他資料表的關係

#### 1. 與 Client 表的潛在關聯
雖然目前沒有外鍵關係，但 device_id 可能對應到 client 表的記錄：
```sql
-- 查詢設備對應的客戶端資訊
SELECT
    l.*,
    c.ip_address,
    c.last_connected
FROM license l
LEFT JOIN client c ON l.device_id = c.device_id;
```

#### 2. 與操作日誌的關聯
設備的操作會記錄在日誌表中：
```sql
-- 假設的操作日誌表結構
CREATE TABLE operation_log (
    id SERIAL PRIMARY KEY,
    device_id VARCHAR(16) REFERENCES license(device_id),
    operation_type VARCHAR(50),
    operation_data JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

#### 3. 與 Location 表的業務關聯
HMI 終端的 permissions.locations 對應到 location 表：
```sql
-- 查詢 HMI 終端可管理的位置詳情
SELECT
    l.device_id,
    l.description,
    loc.name as location_name,
    loc.rack_id
FROM license l
CROSS JOIN LATERAL jsonb_array_elements_text(l.permissions->'locations') AS perm_loc(location)
LEFT JOIN location loc ON loc.name = perm_loc.location
WHERE l.device_type = 'hmi_terminal';
```

## 🚀 API 整合

### RESTful API 端點
```python
# FastAPI 路由定義
from fastapi import APIRouter, HTTPException
from typing import List

router = APIRouter(prefix="/api/license", tags=["License"])

@router.get("/", response_model=List[License])
async def list_licenses():
    """取得所有授權記錄"""
    pass

@router.get("/{device_id}")
async def get_license_by_device(device_id: str):
    """根據設備 ID 查詢授權"""
    pass

@router.post("/")
async def create_license(license: License):
    """建立新授權"""
    pass

@router.put("/{license_id}")
async def update_license(license_id: int, license: License):
    """更新授權"""
    pass

@router.delete("/{license_id}")
async def delete_license(license_id: int):
    """刪除授權"""
    pass

@router.post("/{device_id}/activate")
async def activate_device(device_id: str):
    """啟用設備"""
    pass

@router.post("/{device_id}/deactivate")
async def deactivate_device(device_id: str):
    """停用設備"""
    pass
```

### 認證流程整合
```python
from fastapi import Request, HTTPException

async def verify_device_authorization(request: Request):
    """驗證設備授權的中間件"""
    device_id = request.query_params.get("deviceId")

    if not device_id:
        raise HTTPException(status_code=401, detail="Device ID required")

    # 查詢授權
    license = await get_license_by_device_id(device_id)

    if not license:
        raise HTTPException(status_code=403, detail="Device not authorized")

    if license.active != 1:
        raise HTTPException(status_code=403, detail="Device is deactivated")

    # 將授權資訊加入請求上下文
    request.state.license = license
    request.state.permissions = license.permissions
```

## 📊 查詢範例

### 常用查詢

#### 查詢所有啟用的 HMI 終端
```sql
SELECT
    device_id,
    description,
    permissions->'locations' as managed_locations
FROM license
WHERE device_type = 'hmi_terminal'
AND active = 1;
```

#### 查詢可管理特定位置的設備
```sql
SELECT
    device_id,
    description
FROM license
WHERE device_type = 'hmi_terminal'
AND permissions->'locations' @> '["ManualReceiveArea01"]'::jsonb;
```

#### 統計各類型設備數量
```sql
SELECT
    device_type,
    COUNT(*) as total,
    COUNT(*) FILTER (WHERE active = 1) as active_count
FROM license
GROUP BY device_type;
```

#### 查詢權限配置
```sql
-- 查詢所有可以建立任務的設備
SELECT
    device_id,
    description
FROM license
WHERE device_type = 'op_station'
AND (permissions->>'can_create_tasks')::boolean = true;
```

## 🔒 安全考量

### 存取控制
1. **最小權限原則**：每個設備只授予必要的權限
2. **定期審核**：定期檢查和清理不再使用的授權
3. **即時停用**：可立即停用可疑或異常的設備

### 資料保護
1. **device_id 加密**：考慮對 device_id 進行加密儲存
2. **權限加密**：敏感權限資訊可加密後儲存在 permissions
3. **稽核日誌**：所有授權變更都應記錄在稽核日誌

### SQL 注入防護
```python
# ✅ 安全：使用參數化查詢
device_id = request.query_params.get("deviceId")
query = "SELECT * FROM license WHERE device_id = %s"
result = await database.fetch_one(query, device_id)

# ❌ 危險：直接字串組合
query = f"SELECT * FROM license WHERE device_id = '{device_id}'"
```

## 🚀 擴展性設計

### 新增設備類型
無需修改資料表結構，只需：
1. 定義新的 device_type 值
2. 定義對應的 permissions 結構
3. 實作對應的路由處理

範例：新增倉庫操作員類型
```json
{
    "device_type": "warehouse_operator",
    "permissions": {
        "can_manage_inventory": true,
        "warehouse_sections": ["A", "B", "C"],
        "max_daily_operations": 500
    }
}
```

### 權限擴展
利用 JSONB 的靈活性，可隨時新增權限項目：
```sql
-- 為所有 op_station 新增報表權限
UPDATE license
SET permissions = permissions || '{"can_view_reports": true}'::jsonb
WHERE device_type = 'op_station';
```

## 📈 效能優化

### 索引策略
```sql
-- 針對常用查詢建立索引
CREATE INDEX idx_license_permissions_locations
ON license USING GIN ((permissions->'locations'));

CREATE INDEX idx_license_permissions_can_create_tasks
ON license ((permissions->>'can_create_tasks'))
WHERE device_type = 'op_station';
```

### 查詢優化
```sql
-- 使用 EXISTS 代替 COUNT
-- ✅ 優化後
SELECT EXISTS (
    SELECT 1 FROM license
    WHERE device_id = 'DEVICE_ID' AND active = 1
);

-- ❌ 原始查詢
SELECT COUNT(*) FROM license
WHERE device_id = 'DEVICE_ID' AND active = 1;
```

## 🔍 監控指標

### 關鍵指標
1. **授權使用率**：活躍設備數 / 總授權數
2. **設備活躍度**：最近 N 天有操作的設備比例
3. **權限使用分布**：各種權限的使用頻率
4. **異常檢測**：長期未使用的授權、異常操作頻率

### 監控查詢
```sql
-- 設備活躍度報表
WITH device_activity AS (
    SELECT
        l.device_id,
        l.device_type,
        COUNT(ol.id) as operation_count,
        MAX(ol.created_at) as last_activity
    FROM license l
    LEFT JOIN operation_log ol ON l.device_id = ol.device_id
    WHERE ol.created_at > NOW() - INTERVAL '30 days'
    GROUP BY l.device_id, l.device_type
)
SELECT
    device_type,
    COUNT(*) as total_devices,
    COUNT(operation_count) as active_devices,
    AVG(operation_count) as avg_operations
FROM device_activity
GROUP BY device_type;
```

## 🔗 相關文檔
- HMI 系統設計：docs-ai/knowledge/system/hmi-system-design.md
- Rack 管理操作：docs-ai/operations/guides/rack-management-guide.md
- 資料庫操作：docs-ai/operations/development/database-operations.md

## 📝 更新記錄
| 版本 | 日期 | 更新內容 | 更新者 |
|------|------|----------|--------|
| 1.0 | 2025-09-19 | 初版發布 | AI Agent |