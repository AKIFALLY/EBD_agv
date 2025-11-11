# KUKA Container 同步工具說明

## 📋 工具概覽

提供兩個方向的同步工具：

### 1. 查詢工具 (`query_kuka_containers.py`)
**用途**: 查詢 KUKA Fleet Manager 中的容器資訊

**功能**:
- 查詢在地圖中的容器（預設）
- 查詢所有容器（使用 `--all`）
- 支援多種輸出格式（表格、JSON、簡單列表）
- 按狀態和位置過濾

### 2. 反向同步工具 (`sync_from_kuka_to_rack.py`)
**用途**: 從 KUKA Fleet Manager 同步容器資訊回 Rack 表

**同步內容**:
- `inMapStatus` → `Rack.is_in_map`
- `isCarry` → `Rack.is_carry`
- `nodeCode` → `Rack.location_id`（透過 KukaNode 表映射）

---

## 🔍 查詢工具使用方式

### 基本查詢
```bash
# 查詢在地圖中的容器（預設）
python3 query_kuka_containers.py

# 查詢所有容器（包括不在地圖中的）
python3 query_kuka_containers.py --all
```

### JSON 格式輸出
```bash
# 在地圖中的容器（JSON）
python3 query_kuka_containers.py --json

# 所有容器（JSON）
python3 query_kuka_containers.py --all --json
```

### 過濾查詢
```bash
# 只顯示在地圖中的容器
python3 query_kuka_containers.py --status in

# 只顯示不在地圖中的容器
python3 query_kuka_containers.py --status out --all

# 查詢特定位置的容器
python3 query_kuka_containers.py --position AlanACT-AlanSec1-3

# 查詢特定容器
python3 query_kuka_containers.py --code 001
```

### 簡單列表輸出
```bash
# 只顯示容器編號
python3 query_kuka_containers.py --simple

# 所有容器編號
python3 query_kuka_containers.py --all --simple
```

---

## 🔄 反向同步工具使用方式

### 基本同步

```bash
# 同步在地圖中的容器（預設）
python3 sync_from_kuka_to_rack.py

# 同步所有容器（包括不在地圖中的）
python3 sync_from_kuka_to_rack.py --all
```

### 預覽模式（推薦先使用）
```bash
# 預覽將要進行的變更（不實際更新資料庫）
python3 sync_from_kuka_to_rack.py --dry-run

# 預覽所有容器的變更
python3 sync_from_kuka_to_rack.py --all --dry-run
```

### 同步特定容器
```bash
# 只同步特定容器
python3 sync_from_kuka_to_rack.py --container 001

# 預覽特定容器
python3 sync_from_kuka_to_rack.py --container 001 --dry-run
```

### 完整工作流程範例
```bash
# 步驟1: 先查詢 KUKA 容器狀態
python3 query_kuka_containers.py --json

# 步驟2: 預覽將要同步的變更
python3 sync_from_kuka_to_rack.py --dry-run

# 步驟3: 確認無誤後，執行實際同步
python3 sync_from_kuka_to_rack.py

# 步驟4: 驗證同步結果
# 透過 AGVCUI 查看 Rack 表，或使用 psql 查詢
```

---

## 📊 同步結果說明

### 同步摘要統計
```
總計: X 個容器
已更新: Y 個      # 實際更新的容器數量
無需更新: Z 個    # 資料一致，無需更新
找不到: N 個      # KUKA 容器在 Rack 表中找不到對應記錄
錯誤: M 個        # 發生錯誤的容器數量
```

### 更新項目範例
```
容器: 001
   ✅ 已更新:
      - is_in_map: 0 → 1
      - is_carry: 0 → 1
      - location_id: 52 → 6 (nodeCode: AlanACT-AlanSec1-6)
```

### 退出碼
- `0`: 成功完成，無錯誤
- `1`: 發生錯誤
- `2`: 有容器找不到對應的 Rack
- `130`: 用戶中斷操作（Ctrl+C）

---

## 🔧 進階配置

### 自訂 KUKA Fleet Manager 連接
```bash
python3 sync_from_kuka_to_rack.py \
  --kuka-url http://192.168.10.3:10870 \
  --kuka-username admin \
  --kuka-password Admin
```

### 自訂資料庫連接
```bash
python3 sync_from_kuka_to_rack.py \
  --db-url "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
```

---

## 📝 nodeCode 映射邏輯

KUKA 的 `nodeCode` 會透過以下步驟映射到 `location_id`:

1. **查詢 KukaNode 表**:
   ```sql
   SELECT id FROM kuka_node WHERE uuid = 'AlanACT-AlanSec1-3'
   ```

2. **查詢 Location 表**:
   ```sql
   SELECT id FROM location WHERE node_id = <kuka_node.id>
   ```

3. **更新 Rack 表**:
   ```sql
   UPDATE rack SET location_id = <location.id> WHERE name = '001'
   ```

**注意**: 如果找不到對應的 KukaNode 或 Location，該容器的 `location_id` 不會更新。

---

## ⚠️ 注意事項

### 容器匹配
- KUKA `containerCode` 必須與 Rack `name` 完全匹配
- 如果 KUKA 容器在 Rack 表中不存在，會顯示「找不到」

### nodeCode 映射
- 必須先在 `kuka_node` 表中建立 KUKA Node 的映射關係
- 必須先在 `location` 表中建立 Location 與 Node 的關聯

### 資料一致性
- 同步工具會覆蓋 Rack 表中的 `is_in_map`, `is_carry`, `location_id`
- 建議先使用 `--dry-run` 預覽變更

### 自動同步機制
- RCS 服務中的 `simple_kuka_manager.py` 已經自動同步 `is_in_map` 和 `is_carry`
- 每 0.05 秒查詢一次（20 Hz）
- 但 **不同步** `location_id`（位置資訊）

---

## 🛠️ 故障排除

### 錯誤: 找不到 KukaNode
```
⚠️ 找不到 KUKA Node: AlanACT-AlanSec1-3
```
**解決方式**: 確保 `kuka_node` 表中有對應的記錄，uuid 欄位要匹配

### 錯誤: 找不到 Location
```
⚠️ 找不到 Location for node_id=123
```
**解決方式**: 確保 `location` 表中有對應的記錄，node_id 要匹配

### 錯誤: Rack not found
```
⚠️ 找不到對應的 Rack
```
**解決方式**:
1. 確認 KUKA containerCode 是否正確
2. 確認 Rack 表中是否有對應的記錄（name 欄位）

### 連接失敗
```
❌ 無法匯入 KukaApiClient
```
**解決方式**: 確保在 AGVC 容器內執行，並載入環境：
```bash
source /app/setup.bash
agvc_source
```

---

## 📚 相關文檔

- KUKA Fleet API 規格: `docs-ai/knowledge/protocols/kuka-fleet-api.md`
- 測試總結: `test/TEST_SUMMARY.md`
- KUKA 查詢工具說明: `README_KUKA_QUERY.md`
- RCS 工作空間: `../../rcs_ws/CLAUDE.md`

---

## 🔗 相關工具

### Web API
- AGVCUI Rack 管理: http://localhost:8001/racks
- pgAdmin 資料庫管理: http://agvc.ui/pgadmin/

### ROS 2 服務
```bash
# 查詢 Rack 資料
ros2 service call /rack_query db_proxy_interfaces/srv/RackQuery "query_type: 'get_all'"

# 更新 Rack
ros2 service call /update_rack db_proxy_interfaces/srv/UpdateRack "..."
```

### 資料庫直接查詢
```bash
# 連接資料庫
PGPASSWORD=password psql -h 192.168.100.254 -U agvc -d agvc

# 查詢 Rack 表
SELECT id, name, is_in_map, is_carry, location_id FROM rack;

# 查詢 KukaNode 表
SELECT id, uuid FROM kuka_node;

# 查詢 Location 表
SELECT id, node_id FROM location;
```
