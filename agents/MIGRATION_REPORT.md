# 測試檔案遷移報告

## 執行日期
2025-09-09

## 背景
根據 `@docs-ai/operations/development/test-file-management.md` 規範，所有臨時性測試檔案必須存放在 `~/RosAGV/agents/` 目錄下。

## 違規情況調查結果

### 1. app/ 根目錄違規檔案
發現 19 個 `test_*.py` 檔案直接放在 `/home/ct/RosAGV/app/` 根目錄下，違反了測試檔案管理規範。

### 2. 其他違規位置
- `app/web_api_ws/src/agvcui/testing/` - 包含多個測試檔案
- `app/web_api_ws/src/agvcui/tests/` - 包含測試 HTML 檔案
- `app/web_api_ws/src/agvcui/agvcui/static/js/` - 包含 test-*.html 檔案
- `app/db_proxy_ws/` - 多處散落的測試檔案

## 遷移行動

### 第一階段：已完成 ✅
已將 `app/` 根目錄下的 19 個測試檔案遷移到 `agents/legacy_tests/` 目錄：

1. test_control_switch.py
2. test_direct_integration.py
3. test_flow_execution.py
4. test_flow_logic.py
5. test_flow_log_integration.py
6. test_flow_wcs_node.py
7. test_flow_wcs_queries.py
8. test_foreach_function.py
9. test_location_query.py
10. test_new_functions.py
11. test_phase2_complete.py
12. test_phase3_integration.py
13. test_phase3_simple.py
14. test_rack_status_logic.py
15. test_rotation_conditions.py
16. test_switch_function.py
17. test_task_exists.py
18. test_task_name_fix.py
19. test_variable_resolution.py

### 第二階段：待評估
以下目錄的檔案可能是正式的單元測試套件，需要進一步評估：

- `agvcui/testing/` - 看起來是 AGVCUI 的正式測試套件
- `db_proxy_ws/test/` - 看起來是資料庫代理的正式測試
- `kuka_fleet_ws/src/kuka_fleet_adapter/test/` - KUKA Fleet 的正式測試

## 建議

### 立即行動
1. ✅ 已完成：遷移 app/ 根目錄的臨時測試檔案
2. 待確認：評估其他目錄中的測試檔案性質

### 長期改進
1. **明確區分**：
   - 正式單元測試：保留在各模組的 `test/` 或 `tests/` 目錄
   - 臨時測試檔案：統一放在 `agents/` 目錄

2. **命名規範**：
   - 正式測試：`test_<module>_<feature>.py`
   - 臨時測試：`temp_test_*.py` 或日期標記

3. **定期清理**：
   - 每月檢查 `agents/` 目錄
   - 清理超過 30 天的臨時測試檔案

## 影響評估
- **正面影響**：專案結構更清晰，符合管理規範
- **風險**：已遷移的檔案如果有被其他腳本引用，可能需要更新路徑
- **建議**：觀察幾天，確認沒有功能受影響

## 後續追蹤
- [ ] 確認遷移的檔案沒有被其他程式引用
- [ ] 評估 `agvcui/testing/` 目錄的檔案性質
- [ ] 制定正式測試與臨時測試的明確區分標準
- [ ] 通知團隊關於測試檔案管理規範

---
*此報告由 AI Agent 自動生成*