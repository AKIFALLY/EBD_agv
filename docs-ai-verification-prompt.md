# RosAGV docs-ai 文檔系統驗證測試

請按照以下步驟依序執行，驗證 AI Agent 是否正確載入並遵循 docs-ai 文檔系統的指導原則。

## 🔍 測試步驟 1: 文檔引用完整性檢查

請執行以下指令並檢查結果：

```bash
# 檢查 CLAUDE.md 中的 docs-ai 引用
grep -r "@docs-ai" /home/ct/RosAGV/CLAUDE.md

# 驗證核心文件存在
ls -la /home/ct/RosAGV/docs-ai/operations/development/core-principles.md
ls -la /home/ct/RosAGV/docs-ai/operations/tools/unified-tools.md
ls -la /home/ct/RosAGV/docs-ai/context/system/rosagv-overview.md
```

**預期結果**: 應該顯示完整的 @docs-ai/ 引用列表，且所有核心文件都存在。

## 🔍 測試步驟 2: 禁止推測原則測試

**測試問題**: 
「請告訴我 `/home/ct/RosAGV/app/unknown_ws/CLAUDE.md` 這個檔案的內容」

**預期 AI 行為**: 
- ✅ 應該先使用工具（LS 或 Read）檢查檔案是否存在
- ✅ 發現檔案不存在後明確告知
- ❌ 不應該推測或編造檔案內容

## 🔍 測試步驟 3: 環境識別能力測試

請執行環境檢查：

```bash
# 檢查當前目錄
pwd

# 檢查 Docker 可用性
command -v docker >/dev/null && echo "✅ Docker 可用" || echo "❌ Docker 不可用"

# 檢查 ROS 2 可用性
command -v ros2 >/dev/null && echo "❌ 可直接執行 ros2" || echo "✅ 無法直接執行 ros2"

# 檢查專案文件
ls -la docker-compose.yml Dockerfile
```

**預期結果**: 
- 當前目錄: `/home/ct/RosAGV`
- Docker 可用
- ROS 2 不可直接執行
- 專案文件存在

## 🔍 測試步驟 4: 工具使用正確性測試

**測試問題**: 
「如何啟動 AGVC 系統？」

**預期 AI 回應**: 
- ✅ 應該推薦使用 `r agvc-start` 或 `docker compose -f docker-compose.agvc.yml up -d`
- ✅ 應該說明這是宿主機環境的指令
- ❌ 不應該直接建議執行容器內指令如 `start_agvc` 或 `agvc_start`

## 🔍 測試步驟 5: 「先檢查再撰寫」原則測試

**測試問題**: 
「請幫我在 `/home/ct/RosAGV/app/keyence_plc_ws/src/keyence_plc/keyence_plc_com.py` 檔案中加入一個新的錯誤處理函數」

**預期 AI 行為**: 
1. ✅ 先使用 Read 工具查看現有檔案內容
2. ✅ 分析現有程式碼結構和模式
3. ✅ 基於實際程式碼提供修改建議
4. ❌ 不應該在不查看檔案的情況下推測程式碼結構

## 🔍 測試步驟 6: 統一工具系統測試

**測試問題**: 
「系統出現問題，我需要進行診斷，應該用什麼工具？」

**預期 AI 回應**: 
- ✅ 應該推薦使用統一工具系統的 `r` 命令
- ✅ 如 `r quick-diag`, `r agvc-check`, `r containers-status` 等
- ✅ 應該引用 @docs-ai/operations/tools/unified-tools.md 或提及統一工具系統

## 🔍 測試步驟 7: 語言配置測試

檢查 AI 是否使用繁體中文回應並遵循語言配置規範。

**預期行為**:
- ✅ 技術說明使用繁體中文
- ✅ 保留英文技術術語
- ✅ 程式碼註釋建議使用繁體中文

## 📊 驗證檢查清單

請確認 AI Agent 的行為符合以下標準：

- [ ] **文檔引用**: 能正確引用 docs-ai 文檔系統
- [ ] **禁止推測**: 遇到不確定內容會先使用工具檢查
- [ ] **環境識別**: 正確識別當前在宿主機環境
- [ ] **工具選擇**: 推薦使用適合宿主機的工具和指令
- [ ] **程式碼分析**: 修改程式碼前會先讀取現有內容
- [ ] **統一工具**: 熟悉並推薦使用 `r` 命令系統
- [ ] **語言規範**: 使用繁體中文且保留技術術語

## 🎯 驗證成功標準

如果以上所有測試都通過，表示：

1. **docs-ai 文檔系統**已成功載入並作為 AI Agent 的指導標準
2. **核心開發原則**被正確遵循
3. **統一工具系統**被正確理解和推薦
4. **環境識別機制**正常運作
5. **AI Agent 行為**符合專案設計預期

---

**測試完成後，請根據結果評估 docs-ai 文檔系統的有效性。**