# TAFL Flow 測試指南

## 📋 測試位置與架構

### 測試分層架構

RosAGV 的 TAFL 測試分為兩個層級：

#### 1️⃣ TAFL 語言核心測試
**位置**: `/app/tafl_ws/src/tafl/test/`

```
tafl_ws/src/tafl/test/
├── test_parser.py        # TAFL 解析器測試
├── test_executor.py      # TAFL 執行器測試
├── test_validator.py     # TAFL 驗證器測試
├── test_verbs.py         # TAFL 動詞測試
└── test_strict_v112.py   # TAFL v1.1.2 合規性測試
```

**測試範圍**: TAFL 語言本身的正確性（語法、解析、執行）

#### 2️⃣ TAFL 業務流程測試
**位置**: `/app/tafl_wcs_ws/src/tafl_wcs/test/`

```
tafl_wcs_ws/src/tafl_wcs/test/
├── run_all_tests.py                    # ⭐ 統一執行入口
│
├── # 核心業務流程測試（8個流程）
├── test_parking_flows.py               # 空料架停車區管理（3個流程）
├── test_machine_to_prepare.py          # 射出機停車格→準備區
├── test_full_rack_to_collection.py     # 完成料架→收料區
├── test_rack_rotation.py               # 架台翻轉（入口+出口）
├── test_room_dispatch.py               # 房間投料調度
├── test_duplicate_prevention.py        # 重複執行防護
│
├── # Loader/Unloader AGV 流程測試（10個流程）
├── test_loader_flows.py                # Loader AGV 6 個流程
├── test_unloader_flows.py              # Unloader AGV 4 個流程
│
└── README.md                           # 詳細測試說明
```

**測試範圍**: 實際業務流程的完整執行驗證

---

## 📊 測試覆蓋範圍

### Loader AGV 流程（6個）- 100% 覆蓋

| Flow 檔案 | 測試函數 | 操作模式 |
|-----------|---------|---------|
| `loader_take_boxin_transfer.yaml` | `test_1_loader_take_boxin_transfer()` | 1格操作 |
| `loader_put_cleaner.yaml` | `test_2_loader_put_cleaner()` | 1格操作 |
| `loader_take_cleaner.yaml` | `test_3_loader_take_cleaner()` | 1格操作 |
| `loader_put_soaker.yaml` | `test_4_loader_put_soaker()` | 1格處理 |
| `loader_take_soaker.yaml` | `test_5_loader_take_soaker()` | 1格處理 |
| `loader_put_pre_dryer.yaml` | `test_6_loader_put_pre_dryer()` | 1格操作 |

**測試檔案**: `tafl_wcs_ws/src/tafl_wcs/test/test_loader_flows.py`

### Unloader AGV 流程（4個）- 100% 覆蓋

| Flow 檔案 | 測試函數 | 操作模式 |
|-----------|---------|---------|
| `unloader_take_pre_dryer.yaml` | `test_1_unloader_take_pre_dryer()` | 批量4格 |
| `unloader_put_oven.yaml` | `test_2_unloader_put_oven()` | 批量4格 |
| `unloader_take_oven.yaml` | `test_3_unloader_take_oven()` | 批量4格 |
| `unloader_put_boxout_transfer.yaml` | `test_4_unloader_put_boxout_transfer()` | 批量4格 |

**測試檔案**: `tafl_wcs_ws/src/tafl_wcs/test/test_unloader_flows.py`

### 核心業務流程（8個）- 100% 覆蓋

1. **空料架停車區管理**（3個流程）
   - 入口→出口（優先路徑）
   - 入口→停車區（備選路徑）
   - 停車區→出口（需求調度）

2. **射出機停車格→系統準備區**
   - 已派車料架移動
   - 未派車料架過濾

3. **完成料架出口→人工收料區**
   - 滿載情況
   - 尾批情況

4. **架台翻轉**（2個流程）
   - 房間入口（A空B工作）
   - 房間出口（A滿B空）

5. **房間投料調度**
   - 準備區→房間入口

6. **重複執行防護**
   - 防止重複創建任務

**總計**: 18 個測試場景，100% 覆蓋所有核心流程 🎉

---

## 🚀 執行測試

### 方法 1: 執行所有業務流程測試（推薦）

```bash
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c \
  "source /app/setup.bash && agvc_source && \
   cd /app/tafl_wcs_ws/src/tafl_wcs/test && python3 run_all_tests.py"
```

**預期輸出**:
```
RosAGV TAFL 業務流程完整測試套件
開始時間: 2025-10-16 22:00:00

[1/6] 執行測試: 空料架停車區管理（3個流程）
✅ 空料架停車區管理（3個流程） - 通過
...
測試總結
總測試數: 6
✅ 通過: 6
❌ 失敗: 0
```

### 方法 2: 執行特定類別測試

```bash
# Loader AGV 流程測試（6個）
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c \
  "source /app/setup.bash && agvc_source && \
   python3 /app/tafl_wcs_ws/src/tafl_wcs/test/test_loader_flows.py"

# Unloader AGV 流程測試（4個）
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c \
  "source /app/setup.bash && agvc_source && \
   python3 /app/tafl_wcs_ws/src/tafl_wcs/test/test_unloader_flows.py"

# 停車區管理流程測試（3個）
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c \
  "source /app/setup.bash && agvc_source && \
   python3 /app/tafl_wcs_ws/src/tafl_wcs/test/test_parking_flows.py"
```

### 方法 3: 在容器內直接執行

```bash
# 進入容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 載入環境
source /app/setup.bash && agvc_source

# 進入測試目錄
cd /app/tafl_wcs_ws/src/tafl_wcs/test

# 執行測試
python3 run_all_tests.py                # 所有測試
python3 test_loader_flows.py            # Loader 測試
python3 test_unloader_flows.py          # Unloader 測試
```

---

## ✅ 測試驗證內容

每個 TAFL 流程測試都包含完整的驗證週期：

### 1. 測試前準備
- ✅ 清理舊測試資料
- ✅ 創建測試實體（Room, AGV, Rack, Eqp, Carrier, Work 等）
- ✅ 驗證測試資料正確創建

### 2. 執行流程
- ✅ 載入 TAFL 流程 YAML
- ✅ 執行流程（real mode）
- ✅ 記錄執行步驟和結果

### 3. 結果驗證
- ✅ 驗證任務創建（預期數量）
- ✅ 驗證任務參數正確性
- ✅ 驗證資料庫狀態變化

### 4. 測試後清理
- ✅ 刪除測試創建的所有實體
- ✅ 驗證清理完成

### 測試輸出範例

```
======================================================================
🧪 Loader AGV 從清洗機取料流程測試
======================================================================

🧹 清理舊測試資料...

============================================================
📊 資料庫狀態檢查 - 清理後
============================================================
Work 2030101: ❌ 不存在
Room 995: ❌ 不存在
AGV 9005: ❌ 不存在
...

📝 創建測試資料...

============================================================
📊 資料庫狀態檢查 - 準備完成
============================================================
Work 2030101: ✅ 存在
Room 995: ✅ 存在
AGV 9005: ✅ 存在
Carriers (清洗完成): 1 個
...

🚀 執行 Loader 從清洗機取料流程...

📊 執行結果: completed
   總步驟: 47

✅ 數值變化驗證:
   AGV 上 Carriers: 0 → 0 → 0
   Port 上 Carriers (清洗完成): 0 → 1 → 1
   Tasks 數量: 0 → 0 → 1

🎯 測試結果判定:
   ✅ Loader 從清洗機取料任務創建成功！
   ✅ 數值變化符合預期：0 → 0 → 1
```

---

## 🔧 測試覆蓋的關鍵機制

| 機制 | 說明 | 驗證方式 |
|------|------|---------|
| **重複任務防護** | 同一 work_id 不重複創建未完成任務 | 查詢 `status_id_in: [0,1,2,3]` |
| **條件判斷邏輯** | 滿載、尾批、空閒狀態檢查 | 計數驗證、狀態驗證 |
| **產品與房間匹配** | `process_settings_id` 一致性 | 查詢驗證 |
| **派車狀態檢查** | `room_id != null` | 條件過濾驗證 |
| **位置佔用管理** | `location_status_id` 正確更新 | 狀態檢查 |
| **載具計數** | Carrier 數量計算正確性 | `.length` 屬性驗證 |
| **AGV 空位計算** | 可用空間計算 | `max_capacity - agv_carrier_count` |

---

## 🐛 已修正的問題記錄

### 1. NoneType 減法錯誤（2025-10-16）

**問題**: 所有 "take" 流程在計算 AGV 空位時出現錯誤
```
Cannot perform subtraction: <class 'NoneType'> - <class 'int'>
```

**原因**: TAFL `set` 語句中多個變數同時定義時，變數 `max_capacity` 在被引用時尚未定義

**修復前**:
```yaml
- set:
    agv_carrier_count: "${agv_carriers.length}"
    max_capacity: 4
    required_space: 1
    available_space: "${max_capacity - agv_carrier_count}"  # ❌ max_capacity 未定義
    agv_has_space: "${available_space >= required_space}"
```

**修復後**:
```yaml
- set:
    agv_carrier_count: "${agv_carriers.length}"
- set:
    max_capacity: 4
    required_space: 1
- set:
    available_space: "${max_capacity - agv_carrier_count}"  # ✅ max_capacity 已定義
- set:
    agv_has_space: "${available_space >= required_space}"
```

**影響流程**:
- ✅ loader_take_cleaner.yaml
- ✅ loader_take_soaker.yaml
- ✅ loader_take_boxin_transfer.yaml
- ✅ loader_put_cleaner.yaml
- ✅ loader_put_soaker.yaml
- ✅ loader_put_pre_dryer.yaml
- ✅ unloader_take_oven.yaml
- ✅ unloader_take_pre_dryer.yaml
- ✅ unloader_put_oven.yaml
- ✅ unloader_put_boxout_transfer.yaml

### 2. 其他已修正問題

詳細記錄請參考: `tafl_wcs_ws/src/tafl_wcs/test/README.md`

---

## 💡 開發工作流程

### 修改 TAFL 流程後的驗證流程

1. **修改流程檔案**
   ```bash
   # 編輯 TAFL 流程
   vim app/config/tafl/flows/loader_take_cleaner.yaml
   ```

2. **執行對應測試**
   ```bash
   # 驗證修改
   docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c \
     "source /app/setup.bash && agvc_source && \
      python3 /app/tafl_wcs_ws/src/tafl_wcs/test/test_loader_flows.py"
   ```

3. **執行完整測試套件**
   ```bash
   # 確保無回歸
   docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c \
     "source /app/setup.bash && agvc_source && \
      cd /app/tafl_wcs_ws/src/tafl_wcs/test && python3 run_all_tests.py"
   ```

4. **Commit 前必須通過所有測試** ⚠️

---

## 📝 新增測試

### 1. 在正確位置創建測試

```bash
# 業務流程測試
/app/tafl_wcs_ws/src/tafl_wcs/test/test_new_flow.py

# TAFL 語言測試
/app/tafl_ws/src/tafl/test/test_new_feature.py
```

### 2. 參考現有測試結構

```python
#!/usr/bin/env python3
"""新流程測試"""
import sys
import asyncio

# 設定正確的 import 路徑
sys.path.insert(0, '/app/tafl_wcs_ws/install/tafl_wcs/lib/python3.12/site-packages')
sys.path.insert(0, '/app/db_proxy_ws/install/db_proxy/lib/python3.12/site-packages')

from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
from db_proxy.connection_pool_manager import ConnectionPoolManager

async def test_new_flow():
    """測試新流程"""
    pool_manager = ConnectionPoolManager(db_url)
    flow_executor = TAFLExecutorWrapper(database_url=db_url)

    try:
        # 1. 清理舊資料
        # 2. 創建測試資料
        # 3. 執行流程
        # 4. 驗證結果
        # 5. 清理測試資料
        pass
    finally:
        flow_executor.shutdown()
        pool_manager.shutdown()

if __name__ == '__main__':
    asyncio.run(test_new_flow())
```

### 3. 添加到測試套件

編輯 `run_all_tests.py`:
```python
TEST_SCRIPTS = [
    ...
    ("新流程測試", "test_new_flow.py"),  # 添加這行
]
```

---

## ⚠️ 重要提醒

### agents/ 目錄不是測試位置

`/home/ct/RosAGV/agents/` 目錄用於**暫時性測試和實驗代碼**，不是正式測試套件。

正式測試必須：
1. ✅ 放在對應的工作空間 `test/` 目錄
2. ✅ 整合到測試套件系統（`run_all_tests.py`）
3. ✅ 有完整的文檔說明
4. ✅ 包含完整的清理邏輯

---

## 🔗 相關文檔

- **TAFL 語言規格**: `docs-ai/knowledge/system/tafl/tafl-language-specification.md`
- **TAFL API 參考**: `docs-ai/knowledge/system/tafl/tafl-api-reference.md`
- **TAFL 使用指南**: `docs-ai/knowledge/system/tafl/tafl-user-guide.md`
- **ROS2 測試結構**: `docs-ai/operations/development/testing/ros2-workspace-test-structure.md`
- **測試標準**: `docs-ai/operations/development/testing/testing-standards.md`
- **TAFL WCS README**: `/app/tafl_wcs_ws/src/tafl_wcs/test/README.md`

---

## 📞 CI/CD 整合

```bash
# 在 CI pipeline 中執行
docker compose -f docker-compose.agvc.yml exec -T agvc_server bash -c \
  "source /app/setup.bash && agvc_source && \
   cd /app/tafl_wcs_ws/src/tafl_wcs/test && python3 run_all_tests.py"
```

---

**最後更新**: 2025-10-16
**維護者**: RosAGV Development Team
