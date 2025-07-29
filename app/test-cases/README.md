# RosAGV 測試案例集合

## 📋 概述

這個資料夾包含了 RosAGV 系統的各種測試案例，用於驗證系統功能、診斷問題和確保品質。

## 📂 測試案例結構

```
test-cases/
├── README.md                    # 本文檔
└── opui-task-lifecycle/         # OPUI 任務生命週期測試
    ├── README.md               # 測試情境說明
    ├── test_task_lifecycle.py  # 主要生命週期測試
    ├── test_opui_rcs_wcs_flow.py # 流程驗證測試
    ├── check_wcs_conditions.py # AI WCS 條件檢查
    └── check_machine_status.py # 機台狀態檢查
```

## 🧪 可用測試案例

### 1. OPUI 任務生命週期測試
**路徑**: `opui-task-lifecycle/`
**目的**: 驗證 OPUI 任務從創建到完成的完整流程
**涵蓋**: OPUI → AI WCS → RCS → AGV 的完整生命週期

**快速執行**:
```bash
cd /app/test-cases/opui-task-lifecycle
python3 test_task_lifecycle.py
```

**測試場景**:
- 叫空車任務處理流程
- 派滿車任務處理流程  
- AI WCS 決策條件驗證
- RCS 任務分配機制
- 任務狀態變化監控

## 🎯 測試分類

### 功能測試
- [x] OPUI 任務生命週期測試
- [ ] AGV 狀態機測試 (規劃中)
- [ ] PLC 通訊測試 (規劃中)
- [ ] 路徑規劃測試 (規劃中)

### 整合測試
- [x] OPUI-AI WCS-RCS 整合測試
- [ ] 多 AGV 協調測試 (規劃中)
- [ ] KUKA Fleet 整合測試 (規劃中)

### 效能測試
- [ ] 高並發任務處理測試 (規劃中)
- [ ] 系統負載測試 (規劃中)
- [ ] 網路延遲測試 (規劃中)

### 故障模擬測試
- [ ] 網路中斷模擬 (規劃中) 
- [ ] 服務異常模擬 (規劃中)
- [ ] 資料庫故障模擬 (規劃中)

## 🚀 執行環境要求

### 基本要求
- **執行環境**: AGVC Docker 容器內
- **資料庫**: PostgreSQL 服務運行
- **ROS 2**: Jazzy 版本，已載入工作空間
- **網路**: 容器間網路正常通訊

### 進入測試環境
```bash
# 進入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 載入 ROS 2 工作空間
all_source

# 進入測試案例目錄
cd /app/test-cases
```

## 📊 測試結果管理

### 日誌檔案
測試執行時會產生各種日誌和報告：
- 控制台輸出: 即時測試進度和結果
- 系統日誌: ROS 2 節點和服務的運行日誌
- 錯誤追蹤: 詳細的錯誤堆疊和診斷資訊

### 測試報告
每個測試案例都應該產生結構化的測試報告，包含：
- 測試執行時間和環境資訊
- 測試案例通過/失敗統計
- 關鍵效能指標
- 問題和建議

## 🛠️ 開發測試案例指導

### 測試案例結構
每個新的測試案例應該包含：
```
test-case-name/
├── README.md           # 測試情境說明
├── test_main.py        # 主要測試程式
├── test_utils.py       # 測試工具函數 (可選)
├── config.json         # 測試配置 (可選)
└── requirements.txt    # 額外依賴 (可選)
```

### 測試程式規範
1. **環境檢查**: 確認在正確的容器環境中執行
2. **前置設定**: 設置測試所需的環境狀態
3. **執行測試**: 按步驟執行測試案例
4. **結果驗證**: 驗證測試結果是否符合預期
5. **環境清理**: 恢復原始狀態，避免影響其他測試

### 命名規範
- 測試案例資料夾: `kebab-case` (如: `opui-task-lifecycle`)
- Python 檔案: `snake_case` (如: `test_task_lifecycle.py`)
- 函數和變數: `snake_case`
- 類別: `PascalCase`

## 🔍 故障排除

### 常見問題
1. **模組導入失敗**: 確認在 AGVC 容器內且已載入工作空間
2. **資料庫連接失敗**: 檢查 PostgreSQL 服務狀態
3. **ROS 2 節點無法啟動**: 檢查 RMW 設定和網路連接
4. **權限問題**: 確認檔案權限和容器掛載正確

### 診斷工具
```bash
# 檢查系統狀態
r agvc-check

# 檢查容器狀態  
r containers-status

# 檢查網路連接
r network-check

# 檢查 ROS 2 環境
echo $ROS_DISTRO
echo $RMW_IMPLEMENTATION
```

## 📝 貢獻指導

### 新增測試案例
1. 在對應分類下建立新資料夾
2. 按照結構規範建立檔案
3. 撰寫詳細的 README.md
4. 確保測試可以獨立執行
5. 更新本文檔的測試案例列表

### 測試品質要求
- 測試應該可重複執行
- 測試應該有明確的成功/失敗標準
- 測試應該包含適當的錯誤處理
- 測試應該不會對生產環境造成影響
- 測試應該有完整的文檔說明

## 🔗 相關資源

- [ROS 2 開發指導](../docs-ai/operations/development/ros2-development.md)
- [Docker 開發環境](../docs-ai/operations/development/docker-development.md)
- [系統診斷工具](../docs-ai/operations/maintenance/system-diagnostics.md)
- [故障排除流程](../docs-ai/operations/maintenance/troubleshooting.md)