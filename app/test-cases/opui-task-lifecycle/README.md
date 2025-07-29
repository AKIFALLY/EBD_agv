# OPUI 任務生命週期測試案例

## 📋 測試概述

這個測試案例用於驗證 **OPUI 叫空車和派滿車任務** 從創建到完成的完整生命週期，包含 OPUI → AI WCS → RCS → KUKA Fleet 的完整整合流程。

### 🎯 測試目標
- **叫空車任務** (work_id: 100001): 驗證機台叫空車流程
- **派滿車任務** (work_id: 100002): 驗證派送滿車流程  
- **AI WCS 整合**: 確認 AI WCS 能正確更新現有 OPUI 任務 (不創建新任務)
- **RCS 協調**: 觀察 RCS 的任務分配和 KUKA Fleet 執行過程
- **完整生命週期**: 監控任務狀態從 0→1→2→3→4 的完整變化流程

### 🔄 OPUI 任務完整生命週期

#### 📊 狀態轉換圖
```
OPUI 創建任務 → [status_id: 0] REQUESTING
        ↓ (AI WCS 更新現有任務)
    [status_id: 1] ASSIGNED (設置 model='KUKA400i')
        ↓ (RCS 分配)
    [status_id: 2] EXECUTING (分配 mission_code)
        ↓ (KUKA 開始執行)
    [status_id: 3] DISPATCHED (任務派送給 KUKA Fleet)
        ↓ (KUKA 完成任務)
    [status_id: 4] COMPLETED (AI WCS 檢測到 COMPLETED 狀態)
```

#### 🎯 關鍵狀態說明
| Status ID | 狀態名稱 | 說明 | 負責組件 | 觸發條件 |
|-----------|----------|------|----------|----------|
| **0** | **REQUESTING** | OPUI 任務創建，等待處理 | OPUI | 用戶創建叫空車/派滿車任務 |
| **1** | **ASSIGNED** | 任務已分配，設置執行參數 | AI WCS | AI WCS **更新現有任務** (不創建新任務) |
| **2** | **EXECUTING** | 任務執行中，已分配 mission_code | RCS | RCS 分配 KUKA AGV 和 mission_code |
| **3** | **DISPATCHED** | 任務已派送給 KUKA Fleet | RCS | 任務發送給 KUKA Fleet |
| **4** | **COMPLETED** | 任務成功完成 | AI WCS | KUKA 回調 "COMPLETED" 後 AI WCS 檢測

## 📂 測試檔案結構

```
test-cases/opui-task-lifecycle/
├── README.md                           # 本文檔 (包含完整說明和 KUKA 狀態詳解)
└── test_complete_opui_lifecycle.py     # 完整整合測試程式 (唯一測試檔案)
```

## 🚀 快速執行

### 統一完整測試
```bash
# 進入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 執行完整整合測試 (包含所有組件)
cd /app/test-cases/opui-task-lifecycle
python3 test_complete_opui_lifecycle.py
```

**⚡ 這個測試涵蓋所有功能：**
- OPUI 任務創建和驗證
- 資料庫操作驗證
- RCS 任務處理邏輯
- AI WCS 監聽機制
- 完整流程整合測試
- 機台狀態檢查和清理

## 📊 測試流程詳解

**完整的 7 步驟整合測試流程：**

### 步驟 1: 設置測試環境
- 導入必要模組 (OPUI、AI WCS、RCS、資料庫)
- 建立資料庫連接
- 驗證系統組件可用性

### 步驟 2: 測試 OPUI 任務創建
- 創建叫空車任務 (work_id: 100001) 
- 創建派滿車任務 (work_id: 100002)
- 驗證任務已進入資料庫

### 步驟 3: 驗證資料庫任務狀態
- 查詢測試任務在資料庫中的狀態
- 驗證任務屬性 (work_id、status_id、parameters)
- 確認初始狀態為 0

### 步驟 4: 測試 RCS 任務處理邏輯
- KUKA 任務查詢邏輯測試
- CT 任務查詢邏輯測試
- 任務路由邏輯驗證

### 步驟 5: 測試 AI WCS 監聽機制
- 驗證 AI WCS OPUI 任務查詢
- 測試 WCS 決策引擎處理邏輯
- 檢查 WCS 任務創建機制

### 步驟 6: 完整流程整合測試
- 等待系統處理任務
- 檢查任務狀態變化
- 分析完整流程架構

### 步驟 7: 機台狀態檢查和清理
- 檢查 machine 表停車位狀態
- 分析 AI WCS 決策條件
- 自動清理測試資料

## 🔍 關鍵發現

### AI WCS 處理條件
根據測試和代碼分析發現，AI WCS 處理 OPUI 任務需要滿足以下條件：
```python
# 在 unified_decision_engine.py 中的邏輯
space_1_status = request.get('parking_space_1_status', 0)
space_2_status = request.get('parking_space_2_status', 0)
if space_1_status != 0 or space_2_status != 0:
    # 只有當停車位狀態非 0 時才處理
```

**重要**: 如果所有機台的 `parking_space_1_status` 和 `parking_space_2_status` 都是 0，AI WCS 不會處理任何 OPUI 任務。

### 實際系統流程
```
OPUI 創建任務 (status=0)
        ↓
AI WCS 監聽 (每 8 秒檢查)
        ↓
檢查停車位狀態條件
        ↓
生成派車任務決策
        ↓
更新任務狀態 (status=1) + 設置 model='KUKA400i'
        ↓
RCS 查詢待執行任務
        ↓
分配 mission_code (status=2)
        ↓
AGV 執行任務 (status=3)
        ↓
任務完成 (status=4)
```

## 📋 測試結果分析

### 成功指標
- ✅ 任務狀態從 0 → 1: AI WCS 成功處理
- ✅ 任務狀態從 1 → 2: RCS 成功分配
- ✅ 任務狀態從 2 → 3: AGV 開始執行
- ✅ 任務狀態變為 4: 任務成功完成

### 常見問題
- ❌ 任務停留在狀態 0: 檢查機台停車位狀態
- ❌ 任務停留在狀態 1: 檢查 RCS 服務狀態
- ❌ 任務狀態變為 5+: 檢查任務參數和 AGV 狀態

## 🛠️ 故障排除

### 統一測試故障排除
```bash
# 1. 檢查整個系統狀態
r agvc-check

# 2. 檢查容器狀態
r containers-status

# 3. 檢查網路連接  
r network-check

# 4. 重新執行完整測試
cd /app/test-cases/opui-task-lifecycle
python3 test_complete_opui_lifecycle.py
```

### 測試失敗分析
- **步驟 1-2 失敗**: 檢查資料庫連接和 OPUI 模組
- **步驟 3-4 失敗**: 檢查 ROS 2 環境和 RCS 服務
- **步驟 5 失敗**: 檢查 AI WCS 節點狀態
- **步驟 6-7 失敗**: 檢查系統整體運行狀況

## 📝 測試報告範本

### 基本資訊
- 測試時間: YYYY-MM-DD HH:MM:SS
- 測試環境: AGVC Docker 容器
- 測試程式: test_complete_opui_lifecycle.py
- 涵蓋組件: OPUI + AI WCS + RCS + Web API

### 7步驟測試結果
| 步驟 | 測試項目 | 狀態 | 備註 |
|------|----------|------|------|
| 1 | 設置測試環境 | ✅ | 所有模組正常導入 |
| 2 | OPUI 任務創建 | ✅ | 成功創建 2 個測試任務 |
| 3 | 資料庫任務驗證 | ✅ | 任務狀態和屬性正確 |
| 4 | RCS 任務處理邏輯 | ✅ | KUKA/CT 查詢和路由正常 |
| 5 | AI WCS 監聽機制 | ✅ | WCS 查詢和處理邏輯正常 |
| 6 | 完整流程整合 | ✅ | 流程架構分析完成 |
| 7 | 機台狀態和清理 | ✅ | 環境恢復成功 |

### 測試任務追蹤
- 叫空車任務 (work_id: 100001): 初始狀態 → 最終狀態
- 派滿車任務 (work_id: 100002): 初始狀態 → 最終狀態
- 自動清理: ✅ 所有測試資料已清除

## 🎯 KUKA Fleet 整合狀態轉變詳解

**⚠️ 重要**: OPUI 任務使用與 KUKA 任務相同的狀態管理機制

### 🔄 KUKA 狀態轉換核心機制

#### 📡 KUKA Fleet 回調機制核心原理
KUKA Fleet Manager 透過 Web API 回調更新任務狀態：

```
KUKA Fleet 完成任務 → POST /missionStateCallback
                   ↓
Web API 更新 task.parameters.kuka_mission_status = "COMPLETED"
                   ↓  
AI WCS 8秒輪詢檢測 → 發現 "COMPLETED" 狀態
                   ↓
AI WCS 更新 task.status_id = 4 (COMPLETED)
```

#### 🔧 Web API 回調處理
**端點**: `POST /interfaces/api/amr/missionStateCallback`  
**實現**: `app/web_api_ws/src/web_api/web_api/routers/kuka.py`

**KUKA 狀態類型** (儲存在 `task.parameters.kuka_mission_status`):
```python
KUKA_STATUS_TYPES = {
    "MOVE_BEGIN": "開始移動",
    "ARRIVED": "到達任務節點", 
    "UP_CONTAINER": "升箱完成",
    "DOWN_CONTAINER": "放下完成",
    "COMPLETED": "任務完成",      # ← 關鍵完成狀態
    "CANCELED": "任務取消完成"
    # ... 其他狀態
}
```

#### 🤖 AI WCS 狀態判斷邏輯
**實現**: `app/ai_wcs_ws/src/ai_wcs/ai_wcs/ai_wcs_node.py`  
**決策週期**: 8 秒輪詢

```python
# AI WCS 任務完成檢測邏輯
def check_task_completion(task):
    if task.parameters:
        kuka_status = task.parameters.get("kuka_mission_status")
        if kuka_status == "COMPLETED":
            task.status_id = 4  # 更新任務狀態為完成
            return True
    return False
```

### ⏱️ 時間特性分析

#### 典型時間週期
- **AI WCS 處理**: 0-8 秒 (取決於決策週期時機)
- **RCS 分配**: 0-1 秒 (取決於 RCS 週期時機)  
- **KUKA 執行**: 30-300 秒 (取決於任務複雜度)
- **完成檢測**: 0-8 秒 (AI WCS 輪詢週期)

#### 狀態停留時間分佈
```
Status 0: 平均 4 秒 (0-8 秒範圍)
Status 1: 平均 0.5 秒 (0-1 秒範圍)  
Status 2: 瞬間轉換 (< 0.1 秒)
Status 3: 60-180 秒 (任務執行時間)
Status 4: 永久狀態 (任務完成)
```

### 🔍 詳細狀態轉換分析

#### 狀態 0 → 1: AI WCS 處理階段
**觸發時機**: AI WCS 8秒決策週期  
**OPUI 特殊處理**: AI WCS 更新現有任務 (不創建新任務)

```python
# OPUI 叫空車任務特殊處理
task.status_id = 1  # ASSIGNED
task.parameters["model"] = "KUKA400i"  # 設置 AGV 型號
```

#### 狀態 1 → 2: RCS 分配階段  
**觸發時機**: RCS 1秒週期檢查  
**條件**: 任務有 `model` 參數且有可用的 KUKA AGV

```python
task.status_id = 2  # EXECUTING
task.agv_id = "kuka01"  # 分配 AGV
task.mission_code = f"KUKA_{timestamp}"  # 生成任務代碼
```

#### 狀態 2 → 3: KUKA 派送階段
**觸發**: RCS 向 KUKA Fleet 發送任務成功

#### 狀態 3 → 4: 任務完成階段
**完成流程**:
1. KUKA Fleet 完成任務 → 發送 `"COMPLETED"` 回調
2. Web API 更新 `task.parameters.kuka_mission_status = "COMPLETED"`
3. AI WCS 8秒輪詢檢測 → `task.status_id = 4`

### 🚨 異常狀態處理

| Status ID | 狀態名稱 | 說明 | 觸發條件 |
|-----------|----------|------|----------|
| **5** | **FAILED** | 任務執行失敗 | KUKA Fleet 回調錯誤狀態 |
| **6** | **CANCELLED** | 任務被取消 | 用戶手動取消或系統取消 |
| **51-54** | **取消流程** | 擴展取消狀態 | 分階段取消處理 |

## 🔗 相關文檔

- **[完整整合測試程式](./test_complete_opui_lifecycle.py)** - 唯一的統一測試檔案
- [AI WCS 系統設計](../../ai_wcs_ws/CLAUDE.md) - AI WCS 統一決策引擎詳解
- [OPUI 操作界面](../../web_api_ws/src/opui/CLAUDE.md) - OPUI 任務創建功能
- [RCS 機器人控制](../../rcs_ws/CLAUDE.md) - RCS 任務分配和執行
- [KUKA Fleet API](../../docs-ai/knowledge/protocols/kuka-fleet-api.md) - KUKA Fleet 整合協議
- [系統診斷工具](../../docs-ai/operations/maintenance/system-diagnostics.md) - 系統故障排除

## 📧 問題回報

如果測試過程中遇到問題，請記錄：
1. 錯誤訊息和堆疊追蹤
2. 任務 ID 和狀態變化
3. 系統日誌相關片段
4. 測試環境配置資訊