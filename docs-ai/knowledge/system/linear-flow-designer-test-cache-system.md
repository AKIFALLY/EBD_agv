# Linear Flow Designer v2 測試模式與快取系統設計

## 🎯 適用場景
- 理解 Linear Flow Designer v2 的三種測試模式運作機制
- 了解函數庫快取系統的設計和實作
- 為開發和故障排除提供架構參考

## 📋 系統概述

Linear Flow Designer v2 實現了一個精巧的三層測試架構，配合智能快取系統，提供高效能、高可靠性的流程設計和測試環境。

## 🧪 三種測試模式

### 1. 模擬模式 (Simulation Mode) - 預設
- **用途**: 前端純模擬執行，適合快速測試流程邏輯
- **source**: `'config'` 
- **format**: `'categorized'`
- **函數來源**: 優先使用靜態配置（不需要 API）
- **特點**: 
  - 不需要後端 API 支援
  - 快速響應
  - 適合離線開發

### 2. 驗證模式 (Validation Mode)
- **用途**: 檢查函數是否存在、參數是否正確
- **source**: `'flow_wcs'`
- **format**: `'flat'` (關鍵差異！)
- **函數來源**: 使用 flat 格式進行 O(1) 快速查詢
- **特點**:
  - 使用 `window.functionLibraryFlat` 儲存
  - O(1) 時間複雜度查詢
  - 完整的參數驗證和預設值檢查

### 3. API 測試模式 (API Test Mode)
- **用途**: 實際調用後端測試 API，驗證真實執行結果
- **source**: `'flow_wcs'`
- **format**: `'categorized'`
- **函數來源**: 使用真實的 flow_wcs API
- **特點**:
  - 調用 `/api/flow/execute` 端點
  - 真實的後端執行
  - 可以測試資料庫操作等副作用

## 📦 快取系統架構

### 快取檔案位置
- **路徑**: `/app/config/wcs/flow_functions_cache.yaml`
- **格式**: YAML
- **更新時間**: 記錄在 `meta.updated_at` 欄位

### 三層載入策略

#### Layer 1: Live API (`flow_wcs_live`)
- **優先級**: 最高
- **適用模式**: API 模式、驗證模式
- **端點**: 
  - Categorized: `/api/flow/functions`
  - Flat: `/api/flow/functions/flat`
- **成功時行為**: 更新快取（僅 categorized 格式）

#### Layer 2: Cache YAML (`cache`)
- **優先級**: 中等
- **觸發條件**: API 呼叫失敗
- **檔案**: `/app/config/wcs/flow_functions_cache.yaml`
- **特性**: 包含完整的函數定義和預設值

#### Layer 3: Static Fallback (`static`)
- **優先級**: 最低
- **觸發條件**: 快取也不可用
- **來源**: 硬編碼在程式中的基本函數集
- **用途**: 確保系統最基本可用性

## 🔄 格式差異設計

### Categorized Format（分類格式）
```javascript
{
  "query": [
    {
      "name": "query.locations",
      "params": ["type", "rooms", "has_rack"],
      "defaults": {...}
    }
  ],
  "action": [...],
  "flow": [...]
}
```
- **用途**: UI 顯示、函數庫瀏覽
- **使用場景**: 模擬模式、API 模式
- **儲存位置**: `window.functionLibrary`（轉換為陣列）

### Flat Format（扁平格式）
```javascript
{
  "query.locations": {
    "category": "query",
    "params": ["type", "rooms", "has_rack"],
    "defaults": {...}
  },
  "action.log_message": {...},
  "foreach": {...}
}
```
- **用途**: 快速函數查詢和驗證
- **使用場景**: 驗證模式專用
- **儲存位置**: `window.functionLibraryFlat`
- **優勢**: O(1) 查詢時間複雜度

## 🔧 實作細節

### 前端實作檔案
- **主檔案**: `/app/web_api_ws/src/agvcui/agvcui/static/js/linearFlowDesigner.js`
- **關鍵函數**:
  - `loadAvailableFunctions()`: 根據測試模式載入對應格式的函數庫
  - `simulateFunctionExecution()`: 執行測試邏輯
  - `validateFunctionCall()`: 驗證函數呼叫

### 後端實作檔案
- **主檔案**: `/app/web_api_ws/src/agvcui/agvcui/routers/linear_flow_designer.py`
- **關鍵函數**:
  - `get_available_functions()`: API 端點，處理不同 source 和 format
  - `update_functions_cache()`: 更新快取檔案
  - `load_functions_cache()`: 載入快取

### 快取更新策略
```python
# 快取更新條件
if not _cache_updated_this_session and format != "flat":
    update_functions_cache(functions_data)
    _cache_updated_this_session = True
```
- **只在 categorized 格式時更新**: 避免 flat 格式覆蓋原始結構
- **每個 session 只更新一次**: 避免頻繁寫入
- **保留完整預設值**: 確保離線時也能正常使用

## 🔍 運作流程範例

### 驗證模式流程
1. 使用者選擇「驗證模式」
2. 前端請求: `GET /api/functions?source=flow_wcs&format=flat`
3. 後端嘗試從 API 取得 flat 格式
4. 如果 API 失敗，從快取轉換成 flat 格式
5. 前端儲存到 `window.functionLibraryFlat`
6. 執行時使用 `functionLibraryFlat[funcName]` 進行 O(1) 查詢
7. 驗證函數存在性和參數正確性

### API 模式流程
1. 使用者選擇「API 測試模式」
2. 前端請求: `GET /api/functions?source=flow_wcs&format=categorized`
3. 後端嘗試從 flow_wcs API 取得最新函數
4. **成功時更新快取檔案**
5. 前端執行時調用 `POST /api/flow/execute`
6. 後端實際執行函數並返回結果

### 模擬模式流程
1. 使用者選擇「模擬模式」（預設）
2. 前端請求: `GET /api/functions?source=config&format=categorized`
3. 後端返回靜態配置或快取
4. 前端純本地模擬執行
5. 不調用任何後端 API

## 💡 設計優點

### 效能優化
- **Flat 格式**: O(1) 查詢時間，適合頻繁的驗證操作
- **快取機制**: 減少 API 呼叫，提升響應速度
- **Session 級更新**: 避免頻繁的檔案 I/O

### 可靠性保證
- **三層備援**: API → Cache → Static，確保系統可用性
- **離線支援**: 快取和靜態配置支援離線開發
- **格式分離**: 不同用途使用不同格式，避免相互干擾

### 開發友好
- **模式切換**: 一鍵切換測試模式，適應不同需求
- **完整預設值**: 所有函數都有預設參數，降低使用門檻
- **清晰的錯誤提示**: 各模式都有對應的錯誤處理和提示

## 🚨 故障排除

### 常見問題

#### 驗證模式顯示「函數不存在」
- **原因**: 函數庫未正確載入或格式錯誤
- **解決**: 檢查 `window.functionLibraryFlat` 是否存在
- **驗證**: 在瀏覽器控制台執行 `Object.keys(window.functionLibraryFlat)`

#### API 模式執行失敗
- **原因**: flow_wcs 服務未啟動或網路問題
- **解決**: 檢查 flow_wcs 服務狀態
- **備援**: 自動降級到快取模式

#### 快取未更新
- **原因**: 只在 categorized 格式且 session 首次成功時更新
- **解決**: 重啟服務或手動刪除 `_cache_updated_this_session` flag

## 🔗 相關文檔
- Flow Designer 開發文檔: @docs-ai/operations/development/flow-designer-developer-documentation.md
- Simple WCS 系統: @docs-ai/knowledge/system/flow-wcs-system.md
- WCS 系統設計: @docs-ai/knowledge/agv-domain/wcs-system-design.md
- Web API 開發: @docs-ai/operations/development/web-development.md

## 📝 版本歷史
- **2025-01-13**: 初始文檔建立，記錄測試模式和快取系統設計
- **設計者**: Claude AI 與開發團隊協作完成
- **關鍵創新**: Flat 格式 O(1) 查詢、三層載入策略、智能快取更新