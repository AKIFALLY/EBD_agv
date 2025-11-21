# ⚠️ 已棄用並歸檔 (DEPRECATED & ARCHIVED)

**棄用日期**: 2025-11-18
**歸檔原因**: TAFL WCS 系統已被 KUKA WCS 完全取代
**替代方案**: 使用 `kuka_wcs_ws` 進行倉儲控制系統開發
**遷移指南**: 參見 /home/ct/RosAGV/docs-ai/guides/migration-from-tafl-to-kuka-wcs.md

本文檔已移至 archived 目錄，僅供歷史參考。不應再用於新的開發工作。

---

# AI Agent TAFL 語言使用原則

## TAFL 概述
**Task Automation Flow Language** - WCS/AGV 任務自動化流程語言
- 10個核心動詞涵蓋所有功能
- YAML 格式，簡單易讀
- 支援複雜表達式和邏輯控制

## 核心動詞 (10個)
| 動詞 | 用途 | 範例 |
|------|------|------|
| **query** | 查詢資料 | `query: {target: locations, limit: 5}` |
| **check** | 檢查條件 | `check: {condition: expr, as: result}` |
| **create** | 創建資源 | `create: {target: task, data: {...}}` |
| **update** | 更新資料 | `update: {target: rack, id: 1, data: {...}}` |
| **if** | 條件判斷 | `if: {condition: expr, then: [...]}` |
| **for** | 迴圈處理 | `for: {each: item, in: collection, do: [...]}` |
| **switch** | 多分支 | `switch: {expression: var, cases: [...]}` |
| **set** | 設定變數 | `set: {count: 0}` |
| **stop** | 停止流程 | `stop: {reason: "完成"}` |
| **notify** | 發送通知 | `notify: {message: "訊息"}` |

## TAFL 程式結構
```yaml
metadata:           # 元資料（可選）
  id: "flow_001"
  name: "流程名稱"
  enabled: true

preload:           # 資料預載（可選）
  active_rooms:
    query:
      target: rooms
      where:
        status: active

variables:         # 變數初始化（可選）
  room_id: 1
  task_count: 0

flow:             # 主流程（必要）
  - query:
      target: locations
      as: locations
  - set:
      task_count: "${task_count + 1}"
```

## 常用查詢範例
```yaml
# 簡單查詢
- query:
    target: locations
    limit: 5

# 條件查詢
- query:
    target: agvs
    where:
      status: idle
    as: available_agvs

# 排序查詢
- query:
    target: tasks
    order_by: priority desc
    limit: 10
```

## 條件控制
```yaml
# IF 判斷
- if:
    condition: "${count > 0}"
    then:
      - notify:
          message: "有任務待處理"
    else:
      - stop:
          reason: "無任務"

# FOR 迴圈
- for:
    each: agv
    in: "${agvs}"
    do:
      - update:
          target: agv
          id: "${agv.id}"
          data:
            status: "busy"

# SWITCH 多分支
- switch:
    expression: "${task.type}"
    cases:
      - when: "move"
        do:
          - create:
              target: movement_task
      - when: "load"
        do:
          - create:
              target: loading_task
      - when: "default"
        do:
          - notify:
              message: "未知任務類型"
```

## 變數使用
```yaml
# 設定變數
- set:
    count: 0
    status: "idle"

# 使用變數
- query:
    target: locations
    where:
      room_id: "${room_id}"

# 運算表達式
- set:
    count: "${count + 1}"
    total: "${price * quantity}"
```

## 檔案位置
```bash
# TAFL 流程檔案
/home/ct/RosAGV/app/config/tafl/flows/

# 測試檔案
test_simple_query.yaml
test_real_execution.yaml
test_comprehensive_demo.yaml
```

## 測試方法

### UI 測試
```
http://localhost:8001/tafl/editor
```

### API 測試
```bash
# 檢查狀態
curl http://localhost:8001/tafl/status | jq .

# 執行流程
curl -X POST http://localhost:8001/tafl/execute \
  -H "Content-Type: application/json" \
  -d '{"flow":[{"query":{"target":"locations","limit":2}}],"mode":"real"}' \
  | jq .

# 查看動詞
curl http://localhost:8001/tafl/verbs | jq .
```

### Python 測試
```python
import requests

# 執行 TAFL 流程
response = requests.post(
    "http://localhost:8001/tafl/execute",
    json={
        "flow": [
            {"query": {"target": "locations", "limit": 5}}
        ],
        "mode": "real"
    }
)
print(response.json())
```

## 驗證工具
```bash
# 驗證 TAFL 檔案格式
r tafl-validate my_flow.yaml

# 驗證所有 TAFL 檔案
r tafl-validate all

# 列出所有 TAFL 檔案
r tafl-validate list
```

## 常見錯誤
```yaml
# ❌ 錯誤：SET 不支援簡化格式
- set: count = 0

# ✅ 正確：使用結構化格式
- set:
    count: 0

# ❌ 錯誤：CHECK 缺少 as
- check:
    condition: "${count > 0}"

# ✅ 正確：必須包含 as
- check:
    condition: "${count > 0}"
    as: has_items
```

## 關鍵規則
1. **10個動詞**: 只使用標準動詞，不要發明新動詞
2. **YAML 格式**: 注意縮排和語法
3. **變數引用**: 使用 `${}` 包裹變數
4. **必要參數**: query要target, check要as
5. **執行模式**: real=真實執行, dry_run=模擬