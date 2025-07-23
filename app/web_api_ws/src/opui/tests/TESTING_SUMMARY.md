# OPUI 測試程式摘要

## 專案架構分析

### 核心模組
1. **op_ui_server.py** - FastAPI 伺服器主程式
2. **op_ui_socket.py** - Socket.IO 事件處理
3. **db.py** - 資料庫操作層
4. **routers/** - API 路由模組

### 主要功能
- 使用者連線和會話管理
- 產品和機台資料管理
- AGV 任務創建（叫空車、派滿車）
- 料架管理
- 即時通訊和通知

## 測試覆蓋範圍

### 1. 單元測試 (test_db.py)
**測試類別**: `TestClientOperations`, `TestProductOperations`, `TestMachineOperations`, `TestRoomOperations`, `TestTaskOperations`, `TestWorkAndStatusOperations`

**覆蓋功能**:
- ✅ 客戶端 CRUD 操作
- ✅ 產品管理功能
- ✅ 機台和房間資料獲取
- ✅ 任務創建和刪除
- ✅ 工作類型和狀態管理
- ✅ 資料欄位映射和預設值處理
- ✅ 錯誤處理和異常情況

**測試數量**: 25+ 個測試函數

### 2. Socket.IO 測試 (test_op_ui_socket.py)
**測試類別**: `TestOpUiSocketConnection`, `TestOpUiSocketLogin`, `TestOpUiSocketClientUpdate`, `TestOpUiSocketTaskOperations`, `TestOpUiSocketRackOperations`, `TestOpUiSocketNotifications`

**覆蓋功能**:
- ✅ 連線和斷線處理
- ✅ 使用者登入和驗證
- ✅ 客戶端設定更新
- ✅ 叫空車和派滿車操作
- ✅ 任務取消功能
- ✅ 料架新增和刪除
- ✅ 即時通知功能
- ✅ 錯誤處理和驗證

**測試數量**: 30+ 個測試函數

### 3. 伺服器測試 (test_op_ui_server.py)
**測試類別**: `TestOpUiServer`, `TestOpUiServerErrorHandling`, `TestOpUiServerMain`, `TestOpUiServerConfiguration`, `TestOpUiServerIntegration`

**覆蓋功能**:
- ✅ 伺服器初始化和配置
- ✅ 路由註冊和處理
- ✅ CORS 中介軟體配置
- ✅ 靜態檔案服務
- ✅ 模板渲染
- ✅ Socket.IO 整合
- ✅ 錯誤處理機制

**測試數量**: 20+ 個測試函數

### 4. API 路由測試 (test_routers.py)
**測試類別**: `TestProductRouter`, `TestProcessSettingsRouter`, `TestRouterIntegration`

**覆蓋功能**:
- ✅ 產品 API 端點 (CRUD)
- ✅ 製程設定 API 端點 (CRUD)
- ✅ 請求驗證和錯誤處理
- ✅ 資料庫整合
- ✅ 多路由器整合

**測試數量**: 25+ 個測試函數

### 5. 整合測試 (test_integration.py)
**測試類別**: `TestServerSocketIntegration`, `TestDatabaseSocketIntegration`, `TestAPISocketIntegration`, `TestFullSystemIntegration`

**覆蓋功能**:
- ✅ 伺服器和 Socket 整合
- ✅ 資料庫和 Socket 整合
- ✅ 完整使用者工作流程
- ✅ 多使用者並發處理
- ✅ 錯誤恢復機制
- ✅ 資源清理

**測試數量**: 15+ 個測試函數

### 6. 效能測試 (test_performance.py)
**測試類別**: `TestPerformance`, `TestStress`, `TestScalability`

**覆蓋功能**:
- ✅ Socket 連線效能
- ✅ 登入和任務創建效能
- ✅ API 回應時間測試
- ✅ 記憶體使用穩定性
- ✅ 並發操作處理
- ✅ 高頻率請求處理
- ✅ 壓力測試和可擴展性

**測試數量**: 15+ 個測試函數

## 測試技術和工具

### 測試框架
- **pytest**: 主要測試框架
- **pytest-asyncio**: 異步測試支援
- **pytest-mock**: Mock 和 patch 功能

### Mock 策略
- **資料庫操作**: 使用 mock 隔離真實資料庫
- **Socket.IO**: 模擬 Socket 伺服器和事件
- **外部依賴**: Mock 所有外部服務調用

### 測試資料管理
- **Fixtures**: 提供一致的測試資料
- **工廠函數**: 動態創建測試物件
- **範例資料**: 預定義的測試資料集

### 異步測試
- 正確處理 `async/await` 語法
- 使用 `AsyncMock` 模擬異步函數
- 並發測試和競爭條件處理

## 測試品質指標

### 覆蓋率目標
- **目標覆蓋率**: 85%+
- **核心業務邏輯**: 95%+
- **錯誤處理路徑**: 80%+

### 測試類型分布
- **單元測試**: 70% (快速、隔離)
- **整合測試**: 20% (模組間互動)
- **效能測試**: 10% (效能和壓力)

### 測試執行時間
- **單元測試**: < 30 秒
- **整合測試**: < 60 秒
- **效能測試**: < 120 秒
- **完整測試套件**: < 5 分鐘

## 測試執行指南

### 基本測試命令
```bash
# ROS2 標準測試流程
cd /app/web_api_ws
all_source  # 設定 ROS2 環境
colcon test --packages-select opui --event-handlers console_direct+

# 查看測試結果
colcon test-result --all --verbose

# 運行特定類型測試（需要先設定環境）
cd src/opui/test
python run_tests.py unit
python run_tests.py integration
python run_tests.py performance
```

### 測試覆蓋率
```bash
# 生成覆蓋率報告
python test/run_tests.py coverage

# 查看 HTML 報告
open htmlcov/index.html
```

### 持續整合
- 所有測試必須在 CI/CD 管道中通過
- 新功能必須包含對應的測試
- 測試覆蓋率不得低於既定標準

## 測試最佳實踐

### 1. 測試設計原則
- **獨立性**: 每個測試都是獨立的
- **可重複性**: 測試結果一致且可重複
- **快速執行**: 單元測試應該快速執行
- **清晰命名**: 測試名稱應該描述測試目的

### 2. Mock 使用原則
- **隔離外部依賴**: 資料庫、網路、檔案系統
- **保持真實性**: Mock 行為應該接近真實情況
- **適度使用**: 避免過度 Mock 導致測試失去意義

### 3. 異步測試注意事項
- **正確標記**: 使用 `@pytest.mark.asyncio`
- **資源清理**: 確保異步資源正確清理
- **超時處理**: 設定合理的超時時間

### 4. 錯誤測試策略
- **正常路徑**: 測試預期的成功情況
- **異常路徑**: 測試錯誤和異常情況
- **邊界條件**: 測試極限值和邊界情況

## 維護和擴展

### 新增測試
1. 識別需要測試的功能
2. 選擇適當的測試類型和檔案
3. 編寫測試函數和必要的 fixtures
4. 確保測試通過並更新文檔

### 測試重構
- 定期檢查和更新測試程式碼
- 移除過時或重複的測試
- 改善測試效能和可讀性

### 測試監控
- 監控測試執行時間
- 追蹤測試覆蓋率變化
- 分析測試失敗模式

## 已知限制和改進建議

### 當前限制
1. **真實資料庫測試**: 目前主要使用 Mock，可考慮增加真實資料庫測試
2. **前端測試**: 缺少 JavaScript 前端測試
3. **端到端測試**: 可增加完整的端到端測試

### 改進建議
1. **增加基準測試**: 建立效能基準線
2. **測試資料生成**: 使用 Faker 生成更真實的測試資料
3. **並行測試**: 使用 pytest-xdist 加速測試執行
4. **測試報告**: 增強測試報告和分析功能

## 總結

OPUI 專案的測試程式提供了全面的測試覆蓋，包括：

- **130+ 個測試函數**涵蓋所有主要功能
- **完整的 Mock 策略**確保測試隔離和穩定性
- **多層次測試**從單元到整合到效能測試
- **詳細的文檔**和執行指南
- **自動化工具**簡化測試執行和報告生成

這個測試套件為 OPUI 專案提供了可靠的品質保證，支援持續開發和維護。
