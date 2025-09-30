# RosAGV TAFL 業務流程測試套件

## 📋 測試總覽

本測試套件涵蓋 **8 個核心 TAFL 業務流程** + **1 個重複執行防護測試**，確保 WCS 自動化調度系統的正確性和穩定性。

### 測試執行時間
- **日期**: 2025-09-30
- **總測試時間**: ~3秒
- **測試結果**: ✅ 所有測試通過（6/6 測試腳本，10個業務場景）

---

## 🚀 快速開始

### 執行所有測試

```bash
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c \
  "source /app/setup.bash && agvc_source && cd /app/tafl_wcs_ws/src/tafl_wcs/test && python3 run_all_tests.py"
```

### 執行單個測試

```bash
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c \
  "source /app/setup.bash && agvc_source && cd /app/tafl_wcs_ws/src/tafl_wcs/test && python3 test_parking_flows.py"
```

---

## 📂 測試檔案結構

```
/home/ct/RosAGV/app/tafl_wcs_ws/src/tafl_wcs/test/
├── run_all_tests.py                    # 統一測試套件入口
├── test_parking_flows.py               # 測試1-3：空料架停車區管理
├── test_machine_to_prepare.py          # 測試4：射出機停車格→準備區
├── test_full_rack_to_collection.py     # 測試5：完成料架→收料區
├── test_rack_rotation.py               # 測試6-7：架台翻轉
├── test_room_dispatch.py               # 測試8：房間投料調度
├── test_duplicate_prevention.py        # 重複執行防護驗證
└── README.md                           # 本文件（TAFL 業務流程測試）
```

---

## ✅ 測試場景詳解

### 1. 空料架入口→出口（優先路徑）
- **檔案**: `test_parking_flows.py` - 測試1
- **流程**: `empty_rack_inlet_to_outlet.yaml`
- **場景**: A/B面都為空 + 出口空閒
- **期望**: 創建搬運任務到出口
- **結果**: ✅ 通過

### 2. 空料架入口→停車區（備選路徑）
- **檔案**: `test_parking_flows.py` - 測試2
- **流程**: `empty_rack_inlet_to_parking.yaml`
- **場景**: A/B面都為空 + 出口被佔用
- **期望**: 創建搬運任務到停車區（位置31-34）
- **結果**: ✅ 通過

### 3. 停車區→出口（需求調度）
- **檔案**: `test_parking_flows.py` - 測試3
- **流程**: `parking_to_outlet.yaml`
- **場景**: 房間有已完成carrier等待 + 出口缺rack
- **期望**: 從停車區調配rack到出口
- **結果**: ✅ 通過

### 4. 射出機停車格→系統準備區
- **檔案**: `test_machine_to_prepare.py`
- **流程**: `machine_to_prepare.yaml`
- **場景1**: 機台停車格有已派車料架 + 準備區有空位
- **期望**: 創建搬運任務到準備區（位置11-18）
- **場景2**: 料架未派車（room_id=null）
- **期望**: 不創建任務
- **結果**: ✅ 兩個場景都通過

### 5. 完成料架出口→人工收料區（滿載）
- **檔案**: `test_full_rack_to_collection.py` - 測試1
- **流程**: `full_rack_outlet_to_manual_collection.yaml`
- **場景**: rack滿載（S尺寸32個，L尺寸16個carrier）
- **期望**: 創建搬運任務到收料區（位置51-55）
- **結果**: ✅ 通過

### 6. 完成料架出口→人工收料區（尾批）
- **檔案**: `test_full_rack_to_collection.py` - 測試2
- **流程**: `full_rack_outlet_to_manual_collection.yaml`
- **場景**: rack未滿但房間無可放carrier（尾批）
- **期望**: 創建搬運任務到收料區
- **結果**: ✅ 通過

### 7. 房間入口架台翻轉（A空B工作）
- **檔案**: `test_rack_rotation.py` - 測試1
- **流程**: `rack_rotation_room_inlet_aempty_bwork.yaml`
- **場景**: A面空（載具已取走） + B面有待作業載具
- **期望**: 創建180度翻轉任務
- **結果**: ✅ 通過

### 8. 房間出口架台翻轉（A滿B空）
- **檔案**: `test_rack_rotation.py` - 測試2
- **流程**: `rack_rotation_room_outlet_afull_bempty.yaml`
- **場景**: A面滿16個carrier + B面空 + 房間有載具待處理
- **期望**: 創建180度翻轉任務
- **結果**: ✅ 通過

### 9. 房間投料調度（準備區→房間入口）
- **檔案**: `test_room_dispatch.py`
- **流程**: `room_dispatch_simple.yaml`
- **場景1**: 準備區有已派車料架 + 房間入口空閒 + 產品匹配
- **期望**: 創建搬運任務到房間入口
- **場景2**: 料架未派車（room_id=null）
- **期望**: 不創建任務
- **結果**: ✅ 兩個場景都通過

### 10. 重複執行防護機制
- **檔案**: `test_duplicate_prevention.py`
- **流程**: `empty_rack_inlet_to_outlet.yaml`
- **場景**: 連續執行同一流程3次
- **期望**: 只創建1個任務（不重複）
- **結果**: ✅ 通過

---

## 🔧 測試覆蓋的關鍵機制

| 機制 | 說明 | 驗證方式 |
|------|------|---------|
| **重複任務防護** | 同一rack不重複創建未完成任務 | 查詢 status_id_in: [0,1,2,3] |
| **條件判斷邏輯** | 滿載、尾批、空閒狀態檢查 | 計數驗證、狀態驗證 |
| **產品與房間匹配** | process_settings_id 一致性 | 查詢驗證 |
| **派車狀態檢查** | room_id != null | 條件過濾驗證 |
| **位置佔用管理** | location_status_id 正確更新 | 狀態檢查 |
| **載具側面判斷** | A面(1-16)、B面(17-32) 計數 | 範圍查詢驗證 |

---

## 🐛 已修正的問題

### 1. `is_docked` 誤用
- **問題**: 流程中使用 `is_docked: 1` 作為業務邏輯條件
- **原因**: `is_docked` 僅用於 AGVCUI 地圖顯示，不應用於流程邏輯
- **修正**: 移除所有流程中的 `is_docked` 條件
- **影響檔案**:
  - `empty_rack_inlet_to_outlet.yaml`
  - `empty_rack_inlet_to_parking.yaml`
  - `parking_to_outlet.yaml`

### 2. `room_id_not_null` 語法不支援
- **問題**: `machine_to_prepare.yaml` 使用 `room_id_not_null: true` 條件
- **原因**: TAFL query 不支援此語法
- **修正**: 改用 `if: condition: "${rack.room_id != null}"` 檢查
- **影響檔案**: `machine_to_prepare.yaml`

### 3. YAML 中文字串被誤認為函數
- **問題**: `rack_rotation_room_outlet_afull_bempty.yaml` 錯誤 "Unknown function: A面已滿"
- **原因**: 中文字串中的逗號、括號等特殊字符未加引號
- **修正**:
  1. 所有中文字串加引號
  2. 移除字串中的逗號和括號
- **影響檔案**: `rack_rotation_room_outlet_afull_bempty.yaml`

---

## 📊 測試數據清理

所有測試都包含完整的清理邏輯：
1. **測試前清理**: 刪除舊測試數據
2. **測試後清理**: 刪除新創建的數據
3. **清理順序**: Tasks → Locations → Racks → Carriers
4. **確保隔離**: 使用唯一的測試 ID（501-902）

---

## 🎯 測試覆蓋率

- **流程覆蓋**: 8/8 (100%)
- **場景覆蓋**: 10/10 (100%)
- **關鍵機制**: 6/6 (100%)

---

## 📝 使用建議

### 開發流程
1. 修改 TAFL 流程檔案
2. 執行對應的測試腳本驗證
3. 執行完整測試套件確保無回歸
4. Commit 前必須通過所有測試

### 新增測試
1. 在 `/app/tafl_wcs_ws/src/tafl_wcs/test/` 創建測試檔案
2. 參考現有測試的結構和清理邏輯
3. 添加到 `run_all_tests.py` 的 `TEST_SCRIPTS` 列表
4. 執行驗證

### CI/CD 整合
```bash
# 在 CI pipeline 中執行
docker compose -f docker-compose.agvc.yml exec -T agvc_server bash -c \
  "source /app/setup.bash && agvc_source && cd /app/tafl_wcs_ws/src/tafl_wcs/test && python3 run_all_tests.py"
```

---

## 📞 問題回報

如遇測試失敗：
1. 檢查資料庫狀態（是否有殘留測試數據）
2. 檢查流程檔案是否被修改
3. 執行單個測試定位問題
4. 查看詳細錯誤日誌

---

**最後更新**: 2025-09-30
**維護者**: TAFL System Team