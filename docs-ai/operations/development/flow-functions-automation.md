# Flow Functions 自動化開發工具

## 🎯 適用場景
- Flow WCS 函數開發的自動化工作流程
- @flow_function 裝飾器函數的同步管理
- 簡化從 Python 程式碼到所有相關檔案的更新流程
- Git 整合的自動同步機制

## 📋 自動化工具體系概覽

### 工具架構
當您在 `flow_executor.py` 中使用 `@flow_function` 裝飾器添加新函數後，系統提供完整的自動化工具鏈來同步更新所有相關檔案。

```
開發流程：
1. 編輯 flow_executor.py → 添加 @flow_function
2. 執行自動化工具 → 驗證、測試、同步
3. 所有相關檔案自動更新：
   - /app/config/wcs/flow_functions_cache.yaml (動態快取)
   - /app/config/wcs/flow_functions.yaml (參考文檔)
   - linear_flow_designer.py (內嵌靜態函數)
```

## 🛠️ 核心工具介紹

### 1. dev-flow-functions.sh - 開發輔助工具
**位置**: `/home/ct/RosAGV/scripts/flow-tools/dev-flow-functions.sh`

**功能**：
- `edit` - 編輯 flow_executor.py
- `validate` - 驗證 Python 語法
- `test` - 測試函數載入
- `sync` - 自動同步所有檔案
- `preview` - 預覽函數變更
- `workflow` - 完整工作流程（驗證→測試→同步）

**使用範例**：
```bash
# 編輯函數
./scripts/flow-tools/dev-flow-functions.sh edit

# 執行完整更新流程
./scripts/flow-tools/dev-flow-functions.sh workflow
```

### 2. auto-sync-functions.sh - 自動同步工具
**位置**: `/home/ct/RosAGV/scripts/flow-tools/auto-sync-functions.sh`

**功能**：
1. 重啟 Flow WCS 服務載入新裝飾器
2. 從 API 生成新的快取
3. 更新手動維護的 flow_functions.yaml
4. 更新 Linear Flow Designer 內嵌函數
5. 顯示更新結果統計

**使用範例**：
```bash
# 一鍵同步所有檔案
./scripts/flow-tools/auto-sync-functions.sh
```

### 3. flow-functions-manager.sh - 綜合管理工具
**位置**: `/home/ct/RosAGV/scripts/flow-tools/flow-functions-manager.sh`

**功能**：
- `status` - 顯示系統狀態
- `refresh` - 從 API 重新生成快取
- `update` - 更新手動維護檔案
- `static` - 更新靜態備援
- `compare` - 比較檔案差異
- `full` - 執行完整更新流程

**使用範例**：
```bash
# 查看系統狀態
./scripts/flow-tools/flow-functions-manager.sh status

# 執行完整更新
./scripts/flow-tools/flow-functions-manager.sh full
```

### 4. quick-update-functions.sh - 超簡化更新工具
**位置**: `/home/ct/RosAGV/scripts/flow-tools/quick-update-functions.sh`

**功能**：
- 直接從快取複製到手動維護檔案
- 自動備份原檔案
- 顯示函數統計

**使用範例**：
```bash
# 快速更新（因為兩個檔案在同一目錄，超簡單！）
./scripts/flow-tools/quick-update-functions.sh
```

### 5. Git Hook 自動同步
**安裝腳本**: `/home/ct/RosAGV/scripts/flow-tools/install-git-hook.sh`

**功能**：
- 在 git commit 時自動檢測 flow_executor.py 變更
- 自動執行同步腳本
- 將更新的檔案加入 commit

**安裝方法**：
```bash
# 安裝 Git Hook（只需執行一次）
./scripts/flow-tools/install-git-hook.sh

# 移除 Git Hook
rm /home/ct/RosAGV/.git/hooks/pre-commit
```

## 📝 開發工作流程

### 📋 重要檔案位置說明

由於 **flow_functions_cache.yaml** 和 **flow_functions.yaml** 都在同一個目錄 `/app/config/wcs/` 下，更新變得非常簡單！

- **快取檔案**: `/app/config/wcs/flow_functions_cache.yaml` (Linear Flow Designer 自動生成)
- **手動檔案**: `/app/config/wcs/flow_functions.yaml` (參考文檔)
- **同目錄優勢**: 直接 `cp` 即可更新，無需複雜路徑處理

### 標準開發流程

#### 步驟 1：添加新函數
在 `flow_executor.py` 中使用 @flow_function 裝飾器：

```python
@flow_function("action", "發送郵件通知", ["email", "subject", "body"], "boolean",
               defaults={"email": "admin@example.com", "subject": "通知", "body": ""})
def send_email_notification(self, params: Dict) -> bool:
    """發送郵件通知"""
    email = params.get('email')
    subject = params.get('subject')
    body = params.get('body')
    
    # 實作郵件發送邏輯
    self.logger.info(f"發送郵件到 {email}: {subject}")
    
    return True
```

#### 步驟 2：執行自動化更新

**方法 A - 最簡單快速更新**：
```bash
# 因為檔案在同一目錄，直接複製即可！
./scripts/flow-tools/quick-update-functions.sh
```

**方法 B - 使用開發工具（完整流程）**：
```bash
# 驗證、測試並同步
./scripts/flow-tools/dev-flow-functions.sh workflow
```

**方法 C - 直接同步**：
```bash
# 只執行同步
./scripts/flow-tools/auto-sync-functions.sh
```

**方法 D - Git 自動同步**：
```bash
# 如果已安裝 Git Hook
git add -A
git commit -m "feat: 添加郵件通知函數"
# 自動同步會在 commit 時執行
```

## 🔧 @flow_function 裝飾器規範

### 裝飾器參數說明
```python
@flow_function(
    category: str,           # 函數分類：query, check, task, action, control, special
    description: str,        # 函數描述
    params: List[str],       # 參數列表
    returns: str,           # 返回類型：boolean, string, number, object, array, any
    defaults: Dict = None,   # 參數預設值（可選）
    also_register_as: str = None  # 額外註冊名稱（可選）
)
```

### 函數分類指南
- **query**: 查詢類函數（查詢資料庫、獲取狀態）
- **check**: 檢查類函數（條件判斷、狀態檢查）
- **task**: 任務類函數（創建、更新、分配任務）
- **action**: 動作類函數（執行操作、發送通知）
- **control**: 控制類函數（流程控制、變數操作）
- **special**: 特殊函數（foreach 等特殊控制結構）

### 實作範例
```python
# 查詢類函數
@flow_function("query", "查詢設備狀態", ["device_id"], "object",
               defaults={"device_id": "device001"})
def query_device_status(self, params: Dict) -> Dict:
    device_id = params.get('device_id')
    # 查詢邏輯
    return {"device_id": device_id, "status": "online"}

# 檢查類函數
@flow_function("check", "檢查設備在線", ["device_id"], "boolean",
               defaults={"device_id": "device001"})
def check_device_online(self, params: Dict) -> bool:
    device_id = params.get('device_id')
    # 檢查邏輯
    return True

# 動作類函數
@flow_function("action", "重啟設備", ["device_id", "force"], "boolean",
               defaults={"device_id": "device001", "force": False})
def restart_device(self, params: Dict) -> bool:
    device_id = params.get('device_id')
    force = params.get('force', False)
    # 重啟邏輯
    return True
```

## 📊 檔案同步機制

### 三層備援系統
```
Layer 1: Live API (動態)
├── 來源：@flow_function 裝飾器（運行時註冊）
├── 位置：Flow WCS API (http://localhost:8000/api/flow/functions)
└── 特性：最新、最準確

Layer 2: Cache (快取)
├── 來源：從 Live API 生成
├── 位置：/app/config/wcs/flow_functions_cache.yaml
└── 特性：API 失敗時的備援

Layer 3: Static (內嵌)
├── 來源：寫死在程式碼中
├── 位置：linear_flow_designer.py (第 480-741 行)
└── 特性：最終備援，確保系統永不失效

參考文檔: Manual (手動維護)
├── 來源：從快取更新
├── 位置：/app/config/wcs/flow_functions.yaml
└── 特性：開發參考、文檔用途
```

### 更新優先級
1. **開發時**：修改 flow_executor.py → 執行自動化工具
2. **運行時**：API 優先 → 快取次之 → 靜態備援
3. **維護時**：定期執行 full update 確保同步

## 🧪 測試和驗證

### 自動化測試
```bash
# 執行完整測試套件
./scripts/flow-tools/test-automation.sh
```

### 測試項目
- ✅ 腳本可執行性檢查
- ✅ 關鍵檔案存在性檢查
- ✅ Python 語法驗證
- ✅ @flow_function 裝飾器統計
- ✅ Git Hook 安裝狀態
- ✅ 工具 help 功能測試

### 手動驗證
```bash
# 檢查裝飾器數量
grep -c "@flow_function" app/flow_wcs_ws/src/flow_wcs/flow_wcs/flow_executor.py

# 驗證語法
python3 -m py_compile app/flow_wcs_ws/src/flow_wcs/flow_wcs/flow_executor.py

# 查看函數統計
./scripts/flow-tools/flow-functions-manager.sh status
```

## ⚠️ 注意事項

### 環境要求
- **容器內執行**：某些功能（如 API 同步）需要在 AGVC 容器內執行
- **宿主機功能**：語法驗證、檔案檢查、Git Hook 可在宿主機執行

### 常見問題

#### 問題：快取無法生成
**原因**：Flow WCS API 未運行
**解決**：
```bash
# 在 AGVC 容器內啟動 Flow WCS
docker compose -f docker-compose.agvc.yml exec agvc_server bash
cd /app/flow_wcs_ws
python3 -m flow_wcs.flow_executor
```

#### 問題：Git Hook 不執行
**原因**：Hook 檔案權限問題
**解決**：
```bash
chmod +x /home/ct/RosAGV/.git/hooks/pre-commit
```

#### 問題：函數未出現在 Linear Flow Designer
**原因**：同步未完成
**解決**：
```bash
# 執行完整同步
./scripts/flow-tools/flow-functions-manager.sh full
```

## 💡 最佳實踐

### 開發建議
1. **命名規範**：函數名使用 snake_case，清晰描述功能
2. **分類準確**：選擇正確的 category 便於管理
3. **預設值完整**：提供合理的 defaults 方便測試
4. **文檔完善**：在 description 中清楚說明函數用途

### 維護建議
1. **定期同步**：每週執行一次 full update
2. **版本控制**：重要更新前備份 flow_functions.yaml
3. **測試驗證**：新增函數後執行測試確保正常
4. **監控日誌**：檢查同步過程的錯誤訊息

## 🔗 交叉引用
- Flow WCS 函數系統: @docs-ai/knowledge/system/flow-wcs-function-system.md
- Flow WCS 開發指導: @docs-ai/operations/development/flow-wcs-development.md
- Linear Flow Designer: @docs-ai/knowledge/system/flow-wcs-system.md
- 統一工具系統: @docs-ai/operations/tools/unified-tools.md
- 核心開發原則: @docs-ai/operations/development/core-principles.md