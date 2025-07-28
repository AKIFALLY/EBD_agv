# RosAGV AI 學習指南

## 🎯 AI 助理學習目標
本指南專為 AI 助理設計，幫助快速掌握 RosAGV 工具系統的使用方法，提供準確的技術支援和故障排除建議。

## 📚 必讀 Prompts 清單

### 🔥 核心必讀 (優先級 1)
```markdown
@docs-ai/operations/maintenance/system-diagnostics.md  # 統一工具使用指南
@docs-ai/operations/development/docker-development.md  # Docker 容器管理
@docs-ai/operations/maintenance/troubleshooting.md     # 故障排除指導
```

### 🔧 專業工具 (優先級 2)
```markdown
@docs-ai/operations/maintenance/log-analysis.md        # 日誌分析
@docs-ai/operations/development/ros2-development.md    # ROS 2 開發建置
@docs-ai/operations/development/testing-procedures.md  # 測試程序
```

### 📖 系統架構 (優先級 3)
```markdown
@docs-ai/context/system/technology-stack.md         # ROS 2 + Zenoh 架構
@docs-ai/context/system/dual-environment.md         # 雙環境設計
```

## 🚀 AI 助理快速上手流程

### 第一步：掌握統一工具入口
**關鍵概念**: 只需要記住一個字母 `r`

```bash
# 最重要的 4 個命令
r                    # 顯示工具選單
r agvc-check         # 每日健康檢查
r containers-status  # 檢查容器狀態
r quick-diag         # 快速診斷
```

**AI 助理應該知道**:
- `r` 是 `rosagv-tools.sh` 的快捷方式
- 這是宿主機環境的統一工具入口
- 適用於日常運維和故障排除

### 第二步：理解雙環境架構
**關鍵概念**: AGV 車載系統 + AGVC 管理系統

```bash
# AGV 車載系統 (通常不在運行)
r agv-check          # AGV 健康檢查
r agv-start          # 啟動 AGV 容器

# AGVC 管理系統 (主要使用)
r agvc-check         # AGVC 健康檢查
r agvc-start         # 啟動 AGVC 系統
```

**AI 助理應該知道**:
- 大部分時候只有 AGVC 系統在運行
- AGV 容器通常在實際車輛上運行
- 容器名稱: `agvc_server`, `postgres_container`, `nginx`

### 第三步：掌握故障排除流程
**標準診斷流程**:

```bash
# 1. 快速狀態檢查
r containers-status  # 檢查容器是否運行

# 2. 健康檢查
r agvc-check        # 系統健康狀態

# 3. 深度診斷 (如果有問題)
r quick-diag        # 綜合診斷
r log-errors        # 日誌錯誤分析
r network-check     # 網路連接檢查
```

## 🔧 AI 助理常用回答模板

### 用戶問：系統狀態檢查
```markdown
請執行以下命令檢查系統狀態：

1. **快速狀態檢查**:
   ```bash
   r containers-status
   ```

2. **健康檢查**:
   ```bash
   r agvc-check
   ```

3. **如果發現問題，執行深度診斷**:
   ```bash
   r quick-diag
   ```
```

### 用戶問：容器啟動問題
```markdown
請按以下步驟排除容器啟動問題：

1. **檢查當前容器狀態**:
   ```bash
   r containers-status
   ```

2. **嘗試啟動 AGVC 系統**:
   ```bash
   r agvc-start
   ```

3. **檢查啟動後狀態**:
   ```bash
   r agvc-check
   ```

4. **如果仍有問題，檢查日誌**:
   ```bash
   r log-errors
   ```
```

### 用戶問：網路連接問題
```markdown
請執行以下網路診斷步驟：

1. **基礎網路檢查**:
   ```bash
   r network-check
   ```

2. **Zenoh 連接檢查**:
   ```bash
   r zenoh-check
   ```

3. **如果需要詳細分析**:
   ```bash
   r quick-diag
   ```
```

## 🚨 AI 助理故障排除決策樹

### 容器相關問題
```
用戶報告容器問題
├── 執行: r containers-status
├── 如果容器未運行 → r agvc-start
├── 如果啟動失敗 → r log-errors
└── 如果運行異常 → r agvc-check
```

### 網路相關問題
```
用戶報告網路問題
├── 執行: r network-check
├── 如果端口異常 → 檢查服務狀態
├── 執行: r zenoh-check
└── 如果 Zenoh 異常 → 建議重啟服務
```

### 系統效能問題
```
用戶報告效能問題
├── 執行: r agvc-check
├── 執行: r quick-diag
├── 檢查: r log-scan
└── 分析系統資源使用情況
```

## 💡 AI 助理最佳實踐

### 回答用戶問題時
1. **總是先提供最簡單的解決方案** (`r` 命令)
2. **按步驟提供診斷流程**，不要一次給太多命令
3. **解釋每個命令的作用**，幫助用戶理解
4. **提供預期的輸出結果**，幫助用戶判斷是否正常

### 故障排除時
1. **從基礎檢查開始** (`r containers-status`)
2. **逐步深入診斷** (`r agvc-check` → `r quick-diag`)
3. **根據症狀選擇對應工具** (網路問題用 `r network-check`)
4. **提供具體的解決建議**，不只是診斷

### 技術支援時
1. **確認用戶環境** (宿主機 vs 容器內)
2. **提供環境適配的命令** (宿主機用 `r`，容器內用專業工具)
3. **解釋工具的適用場景**
4. **提供後續步驟建議**

## 📋 AI 助理檢查清單

### 回答前自檢
- [ ] 是否提供了最簡單的 `r` 命令？
- [ ] 是否解釋了命令的作用？
- [ ] 是否提供了逐步診斷流程？
- [ ] 是否考慮了用戶的技術水平？

### 故障排除自檢
- [ ] 是否從基礎檢查開始？
- [ ] 是否提供了多個診斷選項？
- [ ] 是否解釋了可能的原因？
- [ ] 是否提供了具體解決方案？

### 技術準確性自檢
- [ ] 命令語法是否正確？
- [ ] 檔案路徑是否準確？
- [ ] 容器名稱是否正確？
- [ ] 是否符合實際的系統架構？

## 🔗 進階學習路徑

### 深入學習專業工具
當用戶需要更複雜的操作時，引導學習：
```markdown
@docs-ai/operations/development/docker-development.md  # Docker 專業操作
@docs-ai/operations/maintenance/system-diagnostics.md  # 系統深度診斷
@docs-ai/operations/maintenance/log-analysis.md        # 日誌深度分析
```

### 開發相關支援
當用戶涉及開發工作時：
```markdown
@docs-ai/operations/development/ros2-development.md    # ROS 2 開發工具
@docs-ai/operations/development/web-development.md     # Web 開發
```

### 系統架構理解
當需要解釋系統設計時：
```markdown
@docs-ai/context/system/technology-stack.md         # ROS 2 架構
@docs-ai/context/system/dual-environment.md         # 雙環境設計
```

## 🎯 AI 助理成功指標

### 用戶滿意度指標
- 用戶能夠快速解決問題
- 用戶理解了解決方案的原理
- 用戶學會了基本的診斷方法
- 用戶對工具使用更有信心

### 技術準確性指標
- 提供的命令能夠正確執行
- 診斷流程符合實際情況
- 解決方案有效且安全
- 技術解釋準確且易懂
