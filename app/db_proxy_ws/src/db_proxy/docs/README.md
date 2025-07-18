# 📚 DB Proxy SQL 模組文檔

這個資料夾包含了 DB Proxy SQL 模組的所有文檔、測試腳本和初始化功能。

## 📁 資料夾結構

```
docs/
├── README.md                    # 本文檔
├── testing/                     # 測試腳本和調試工具
│   ├── test_*.py               # 功能測試腳本
│   ├── debug_*.py              # 調試工具
│   └── simple_*.py             # 簡單測試工具
└── summaries/                   # 功能說明文檔
    ├── README_KUKA_MAP.md      # KUKA 地圖匯入功能說明
    ├── README_CT_MAP.md        # CT 地圖匯入功能說明
    └── ...
```

## 🎯 主要功能模組

### 1. 資料庫初始化系統
- **位置**: `init_data/`
- **功能**: 自動化資料庫初始化和資料匯入
- **狀態**: ✅ 完成

#### 初始化模組列表
1. `01_node_types.py` - 節點類型初始化
2. `02_location_status.py` - 位置狀態初始化
3. `03_rack_status.py` - 貨架狀態初始化
4. `04_process_settings.py` - 流程設定初始化
5. `05_nodes.py` - 節點資料初始化
6. `06_machines.py` - 機器資料初始化
7. `07_rooms.py` - 房間資料初始化
8. `08_locations.py` - 位置資料初始化
9. `09_products.py` - 產品資料初始化
10. `10_agvs.py` - AGV 資料初始化
11. `11_racks.py` - 貨架資料初始化
12. `12_equipment.py` - 設備資料初始化
13. `13_works_tasks.py` - 工作任務初始化
14. `14_carriers.py` - 載具資料初始化
15. `15_carrier_status.py` - 載具狀態初始化
16. `16_traffic_zone.py` - 交通區域初始化
17. `17_agv_status.py` - AGV 狀態初始化
18. `18_kuka_map.py` - KUKA 地圖匯入
19. `19_ct_map.py` - CT 地圖匯入

### 2. KUKA 地圖自動匯入系統
- **文檔**: `summaries/README_KUKA_MAP.md`
- **功能**: 從 `kuka_map.json` 自動匯入 KUKA 地圖資料
- **特點**: 
  - 兩階段處理（節點→邊）
  - 座標轉換 (m to px)
  - 智能重複執行
  - 完整錯誤處理
- **狀態**: ✅ 完成

### 3. CT 地圖自動匯入系統
- **文檔**: `summaries/README_CT_MAP.md`
- **功能**: 從 `20250509_pathtest.json` 自動匯入 CT 地圖資料
- **特點**:
  - 兩階段處理（節點→邊）
  - 座標轉換 (mm to px)
  - 智能重複執行
  - 完整錯誤處理
- **狀態**: ✅ 完成

### 4. 智能清除系統
- **功能**: 智能清除地圖資料，跳過有外鍵約束的節點
- **特點**:
  - 外鍵約束檢測
  - 詳細的跳過原因分析
  - 部分清除能力
  - 完整的統計報告
- **狀態**: ✅ 完成

## 🧪 測試工具

### 地圖匯入測試
- `testing/test_kuka_map_import.py` - KUKA 地圖匯入測試
- `testing/test_ct_map_import.py` - CT 地圖匯入測試
- `testing/test_both_maps.py` - 整合測試

### 調試工具
- `testing/debug_edge_update.py` - 邊更新調試
- `testing/simple_edge_test.py` - 簡單邊測試
- `testing/test_edge_fix.py` - 邊修復測試

## 🚀 使用指南

### 執行完整初始化
```bash
cd db_proxy_ws/src/db_proxy
python -m db_proxy.sql.db_install
```

### 單獨測試地圖匯入
```bash
# 測試 KUKA 地圖匯入
python docs/testing/test_kuka_map_import.py

# 測試 CT 地圖匯入
python docs/testing/test_ct_map_import.py

# 測試兩個地圖整合
python docs/testing/test_both_maps.py
```

### 智能清除地圖資料
```python
from db_proxy.sql.init_data.init_data.kuka_map import clear_kuka_map
from db_proxy.sql.init_data.init_data.ct_map import clear_ct_map

# 在 session 中使用
clear_kuka_map(session)  # 智能清除 KUKA 地圖
clear_ct_map(session)    # 智能清除 CT 地圖
```

## 📊 功能特點

### 自動化程度
- ✅ 完全自動化的初始化流程
- ✅ 無需手動干預的地圖匯入
- ✅ 智能錯誤處理和恢復

### 資料完整性
- ✅ 外鍵約束檢查
- ✅ 資料一致性驗證
- ✅ 孤立資料檢測

### 使用者體驗
- ✅ 詳細的進度顯示
- ✅ 清晰的錯誤訊息
- ✅ 完整的統計報告

### 維護性
- ✅ 模組化設計
- ✅ 清晰的文檔結構
- ✅ 完整的測試覆蓋

## 🔧 故障排除

### 常見問題

1. **地圖檔案找不到**
   - 確認 `kuka_map.json` 和 `20250509_pathtest.json` 在 `init_data/` 目錄中
   - 檢查檔案權限

2. **資料庫連接問題**
   - 檢查資料庫服務狀態
   - 確認連接設定正確

3. **外鍵約束錯誤**
   - 使用智能清除功能
   - 檢查資料依賴關係

4. **匯入資料不完整**
   - 查看詳細日誌
   - 使用測試腳本驗證

### 測試流程

1. **執行基本測試**
   ```bash
   python docs/testing/test_both_maps.py
   ```

2. **檢查資料完整性**
   - 查看測試輸出的統計資訊
   - 確認沒有孤立的邊

3. **驗證功能正常**
   - 執行完整的 db_install
   - 檢查日誌輸出

## 📈 未來改進

### 計劃功能
- [ ] 支援更多地圖格式
- [ ] 增加地圖驗證功能
- [ ] 實作增量更新機制
- [ ] 加入效能監控

### 優化方向
- [ ] 提升匯入速度
- [ ] 減少記憶體使用
- [ ] 增強錯誤恢復能力
- [ ] 改善使用者介面

## 📚 參考資源

- [SQLModel 官方文檔](https://sqlmodel.tiangolo.com/)
- [PostgreSQL 外鍵約束](https://www.postgresql.org/docs/current/ddl-constraints.html)
- [Python 資料庫最佳實踐](https://docs.python.org/3/library/sqlite3.html)

## 🤝 貢獻指南

### 新增測試
1. 在 `docs/testing/` 目錄中建立測試檔案
2. 使用描述性的檔案名稱 (`test_*.py`)
3. 包含完整的文檔字串
4. 遵循現有的測試模式

### 新增功能文檔
1. 在 `docs/summaries/` 目錄中建立說明文檔
2. 使用 Markdown 格式
3. 包含使用範例和故障排除
4. 更新本 README 文檔

### 程式碼風格
- 遵循 PEP 8 規範
- 使用有意義的變數名稱
- 加入適當的註解和文檔字串
- 實作完整的錯誤處理
