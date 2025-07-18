# OPUI 文檔導覽

本目錄包含 OPUI 專案的所有技術文檔，按類別組織以便查閱。

## 📋 核心文檔

### [ARCHITECTURE.md](ARCHITECTURE.md)
**系統架構說明**
- 整體架構設計
- 模組化結構說明
- 技術棧介紹
- 目錄結構說明

### [FRONTEND_ARCHITECTURE.md](FRONTEND_ARCHITECTURE.md)
**前端架構說明**
- 模組化 JavaScript 架構
- 狀態管理系統
- 頁面功能分離設計
- 前端開發指南

### [SOCKET_API_FORMAT_GUIDE.md](SOCKET_API_FORMAT_GUIDE.md)
**Socket.IO API 格式指南**
- API 資料格式說明
- 事件類型定義
- 前後端通訊協定
- 資料結構範例

### [MAINTENANCE.md](MAINTENANCE.md)
**維護指南**
- 日常維護操作
- 故障排除指南
- 效能監控
- 備份和恢復

## 📚 歷史記錄

### [archives/](archives/)
**重要的修復和變更記錄**

#### 架構變更記錄
- **FLAT_FORMAT_MIGRATION_SUMMARY.md** - Socket.IO API 扁平化格式遷移
- **TASK_STATUS_REFACTOR_SUMMARY.md** - 任務狀態重構記錄

#### 重要修復記錄
- **HOME_PAGE_DISPLAY_FIX_SUMMARY.md** - Home 頁面顯示問題修復
- **HOME_PAGE_PRODUCTS_ARRAY_FIX.md** - Products 陣列為空問題修復

#### 效能優化記錄
- **PERFORMANCE_OPTIMIZATION_SUMMARY.md** - 系統效能優化總結

#### 功能實作記錄
- **LICENSE_IMPLEMENTATION_SUMMARY.md** - 授權系統實作記錄

## 🧪 測試文檔

### [../tests/](../tests/)
**測試相關文檔**
- 測試框架說明
- 測試執行指南
- 測試案例文檔

## 📖 文檔使用指南

### 新開發者入門
1. 先閱讀 [ARCHITECTURE.md](ARCHITECTURE.md) 了解整體架構
2. 閱讀 [FRONTEND_ARCHITECTURE.md](FRONTEND_ARCHITECTURE.md) 了解前端設計
3. 參考 [SOCKET_API_FORMAT_GUIDE.md](SOCKET_API_FORMAT_GUIDE.md) 了解 API 格式
4. 查看 [MAINTENANCE.md](MAINTENANCE.md) 了解維護操作

### 問題排除
1. 查看 [MAINTENANCE.md](MAINTENANCE.md) 的故障排除章節
2. 參考 [archives/](archives/) 中的相關修復記錄
3. 檢查測試文檔中的相關測試案例

### 架構變更
1. 參考 [archives/](archives/) 中的歷史變更記錄
2. 了解之前的設計決策和實作細節
3. 確保新變更與現有架構一致

## 🔄 文檔維護

### 文檔更新原則
- **核心文檔**：隨專案變更即時更新
- **歷史記錄**：保持不變，作為參考資料
- **測試文檔**：隨測試框架變更更新

### 新增文檔指南
- 核心功能文檔放在 `docs/` 根目錄
- 歷史記錄放在 `docs/archives/`
- 測試相關文檔放在 `tests/`
- 臨時修復指南完成後移至 `archives/` 或刪除

### 文檔品質標準
- 使用繁體中文撰寫
- 包含清晰的標題和結構
- 提供具體的程式碼範例
- 包含適當的圖表和流程圖
- 定期檢查連結有效性

## 📞 支援

如有文檔相關問題或建議，請：
1. 檢查現有文檔是否已涵蓋相關內容
2. 查看歷史記錄中是否有類似問題的解決方案
3. 參考測試文檔中的相關測試案例
4. 聯繫開發團隊進行進一步協助
