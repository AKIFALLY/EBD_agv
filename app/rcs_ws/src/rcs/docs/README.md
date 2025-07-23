# RCS 專案文件

## 資料夾結構

```
rcs_ws/src/rcs/
├── rcs/                           # 主程式模組
│   ├── ct_manager.py             # CT 車隊管理器（主要實作）
│   ├── kuka_manager.py           # KUKA 車隊管理器
│   └── rcs_core.py               # RCS 核心節點
├── docs/                         # 文件資料夾
│   ├── README.md                 # 本文件
│   ├── summaries/                # 總結文件
│   │   ├── CT_DISPATCH_IMPLEMENTATION_SUMMARY.md
│   │   └── QUERY_CONDITION_UPDATE_SUMMARY.md
│   └── testing/                  # 測試相關文件
├── test_ct_dispatch.py           # CT 任務派發測試腳本
├── package.xml                   # ROS 2 套件配置
└── setup.py                     # Python 套件設定
```

## 文件說明

### 總結文件 (summaries/)

#### CT_DISPATCH_IMPLEMENTATION_SUMMARY.md
- CT Manager 智能任務分派機制的完整實作說明
- 包含系統架構、分派邏輯、核心方法說明
- 異常處理機制和擴展性設計
- 程式碼品質和部署注意事項

#### QUERY_CONDITION_UPDATE_SUMMARY.md
- 查詢條件更新的詳細記錄
- 從 `!= "KUKA400i"` 改為明確指定車型的修改過程
- 修改位置、優點和注意事項

### 測試文件 (testing/)
- 預留給測試相關的文件和腳本
- 可放置測試指南、測試案例文件等

## 主要功能

### CT Manager (ct_manager.py)
基於房間和車型的智能任務分派機制：

- **支援車型**：Cargo、Loader、Unloader
- **分派邏輯**：
  - 房內任務：`Loader{房間編號:02d}`、`Unloader{房間編號:02d}`
  - 房外任務：`Cargo02`
- **狀態管理**：AGV 狀態檢查和任務狀態更新
- **異常處理**：完整的錯誤處理和日誌記錄

### 測試
```bash
# 執行 CT 派發測試
cd rcs_ws/src/rcs
python test_ct_dispatch.py
```

## 開發指南

### 新增車型
1. 更新 `ct_manager.py` 中的查詢條件
2. 修改 `_validate_task_parameters()` 方法
3. 更新 `_determine_target_agv_name()` 邏輯
4. 更新測試腳本和文件

### 文件管理
- 總結性文件放在 `docs/summaries/`
- 測試相關文件放在 `docs/testing/`
- 使用 Markdown 格式
- 保持文件與程式碼同步更新

### 測試開發
- 測試腳本放在專案根目錄
- 使用 `test_` 前綴命名
- 包含完整的功能驗證

## 建置與執行

```bash
# 建置專案
cd rcs_ws
colcon build

# 執行 RCS 核心
source install/setup.bash
ros2 run rcs rcs_core

# 執行測試
cd src/rcs
python test_ct_dispatch.py
```

## 注意事項

1. **路徑設定**：測試腳本已配置正確的 Python 路徑
2. **資料庫連線**：確保資料庫服務正常運行
3. **車型配置**：AGV 資料需要正確初始化
4. **狀態同步**：確保 AGV 實際狀態與資料庫一致

## 維護

- 定期更新文件
- 保持測試覆蓋率
- 監控系統效能
- 記錄重要變更

## 參考

- 參考 `agvcui` 專案的文件組織結構
- 遵循 ROS 2 套件開發規範
- 保持與其他專案的一致性
