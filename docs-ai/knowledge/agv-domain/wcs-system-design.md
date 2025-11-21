# WCS 系統架構設計

> ⚠️ **重要說明：本文檔包含歷史內容**
>
> 本文檔原為 TAFL WCS 系統設計文檔，TAFL WCS 已於 2025 年棄用。
>
> **當前系統**：RosAGV 已改用 **KUKA WCS** 作為倉儲控制系統。
> - **當前 WCS**: `kuka_wcs_ws` - KUKA WCS 系統
> - **輔助模組**: `wcs_ws` - WCS 工作空間（流程控制邏輯）
>
> 本文檔保留僅供：
> 1. 理解系統演進歷史
> 2. TAFL 語言特性參考
> 3. 流程自動化設計思維
>
> 現行系統文檔請參考：
> - `kuka_wcs_ws/CLAUDE.md` - KUKA WCS 實作
> - `wcs_ws/CLAUDE.md` - WCS 流程控制邏輯

## 🎯 適用場景
- ~~理解 RosAGV 中以 TAFL WCS 為核心的任務自動化系統架構~~（歷史參考）
- ~~掌握 TAFL (Task Automation Flow Language) 的執行引擎和流程調度邏輯~~（歷史參考）
- ~~了解 TAFL Editor 視覺化設計與 TAFL WCS 的整合關係~~（歷史參考）
- 理解流程自動化設計思維（仍然適用）

## 📋 系統概述（歷史內容）

**WCS (Warehouse Control System)** 是 RosAGV 系統的核心決策引擎，~~以 **TAFL WCS** 為統一實作核心~~（**已棄用，當前使用 KUKA WCS**），負責管理 Rack 車的搬移需求決策和任務調度。

### 🎯 ~~TAFL WCS 核心定位~~（歷史內容）
- ~~**流程自動化**: 基於 TAFL 語言的任務自動化執行引擎~~（已棄用）
- ~~**視覺化設計**: TAFL Editor 提供拖拽式流程設計介面~~（已棄用）
- ~~**靈活調度**: 支援複雜的條件判斷、迴圈和多分支邏輯~~（已棄用）
- ~~**生產就緒**: 已完全實作並投入生產使用~~（已棄用，改用 KUKA WCS）

### 基本環境配置
- **房間數量**: 支援 1-10 個房間動態擴展
- **每個房間配置**: 1個入口 + 1個出口 (均可停靠 Rack 車)
- **Rack 位置管理**: 由 WCS 掌控並儲存在資料庫中
- **實體搬運**: 由 KUKA AGV 透過 KUKA API 執行

## 🏗️ 系統架構分工

### 🔬 ~~TAFL WCS (tafl_wcs_ws)~~ → KUKA WCS (kuka_wcs_ws) + WCS (wcs_ws)
> ⚠️ **已棄用**: `tafl_wcs_ws` 已於 2025 年停用
>
> **當前系統**:
> - `kuka_wcs_ws` - KUKA WCS 系統（當前使用）
> - `wcs_ws` - WCS 工作空間（流程控制邏輯）

**歷史核心職責**（TAFL WCS）:
- ~~**TAFL 執行引擎**: 解析和執行 TAFL 流程定義~~（已棄用）
- ~~**任務自動化**: 基於 TAFL 語言的複雜流程自動化~~（已棄用）
- **資料庫整合**: 與 db_proxy_ws 無縫整合，執行資料庫操作（仍然適用）
- **ROS 2 節點架構**: 提供完整的系統整合（仍然適用）

### 🎨 TAFL Editor - 視覺化流程編輯器
**核心職責**:
- **流程設計器**: 視覺化拖拽式流程設計 (/tafl/editor 路由)
- **TAFL 檔案產生**: 生成標準 TAFL YAML 檔案
- **即時驗證**: 流程語法驗證和錯誤提示
- **視覺化預覽**: 流程執行路徑預覽

### 🤖 RCS (rcs_ws) - 執行層
**支援職責**:
- 讀取 TAFL WCS 產生的任務
- 透過 KUKA API 向 KUKA Fleet 下達搬運指令
- 管理 AGV 調度和交通控制

### 🌐 Web API (web_api_ws) - 整合層
**支援職責**:
- 任務狀態監控和更新
- 與 KUKA Fleet 介面整合
- 外部系統 API Gateway

## 🚀 TAFL 語言特性

### 支援的 10 個核心動詞
1. **query** - 查詢資料
2. **check** - 條件檢查
3. **create** - 創建資源
4. **update** - 更新資源
5. **set** - 設定變數
6. **if** - 條件分支
7. **for** - 迴圈迭代
8. **switch** - 多分支選擇
9. **stop** - 停止執行
10. **notify** - 發送通知

### TAFL 流程範例

```yaml
metadata:
  id: rack_rotation_check
  name: Rack 旋轉檢查流程
  version: "1.1.2"

variables:
  min_battery: 30
  priority_threshold: 5

flow:
  # 查詢待處理任務
  - query:
      target: tasks
      where:
        status: "pending"
        priority: "> ${priority_threshold}"
      as: pending_tasks

  # 檢查是否有任務
  - check:
      condition: "${pending_tasks.length} > 0"
      as: has_tasks

  # 條件執行
  - if:
      condition: "${has_tasks}"
      then:
        - for:
            as: task
            in: "${pending_tasks}"
            do:
              - update:
                  target: task
                  where:
                    id: "${task.id}"
                  set:
                    status: "processing"
                    assigned_time: "${now()}"

              - notify:
                  level: info
                  message: "開始處理任務 ${task.id}"
```

## 🎯 TAFL WCS 技術特點

### 流程自動化引擎
- **TAFL 解析器**: 解析 TAFL v1.1.2 語法
- **執行器**: 執行 TAFL 定義的流程邏輯
- **變數管理**: 支援變數定義、引用和表達式計算
- **錯誤處理**: 結構化的錯誤處理和日誌記錄

### 與 TAFL Editor 整合
- **雙向同步**: 視覺化設計與程式碼雙向轉換
- **即時預覽**: 編輯時即時顯示流程邏輯
- **驗證功能**: 自動驗證 TAFL 語法正確性

## 📊 系統運作流程

### TAFL 流程設計階段
1. **開啟 TAFL Editor**: 存取 http://localhost:8001/tafl/editor
2. **拖拽設計**: 使用視覺化介面設計流程
3. **參數配置**: 設定各步驟的參數和條件
4. **語法驗證**: 自動驗證 TAFL 語法
5. **儲存流程**: 儲存為 TAFL YAML 檔案

### TAFL WCS 執行階段
1. **載入流程**: TAFL WCS 載入 YAML 流程檔案
2. **解析 AST**: 解析為抽象語法樹
3. **執行流程**: 按照定義順序執行各步驟
4. **資料操作**: 透過 db_proxy 執行資料庫操作
5. **任務創建**: 創建對應的 AGV 任務
6. **狀態更新**: 即時更新執行狀態

## 💡 最佳實踐

### 流程設計原則
- 使用描述性的 flow ID 和名稱
- 合理使用變數避免硬編碼
- 適當的錯誤處理和日誌記錄
- 避免過度巢狀的流程結構

### 效能優化
- 使用 query 的 limit 參數限制資料量
- 在 for 迴圈中使用 filter 減少處理量
- 合理使用 check 避免不必要的執行

## ⚠️ 系統限制

### TAFL v1.1.2 不支援
- ❌ delete 動詞（刪除操作）
- ❌ while 迴圈
- ❌ 函數呼叫
- ❌ 平行執行
- ❌ break/continue（使用 stop 替代）

## 🔗 交叉引用
- **當前 WCS 系統**:
  - `app/kuka_wcs_ws/CLAUDE.md` - KUKA WCS 系統（當前使用）
  - `app/wcs_ws/CLAUDE.md` - WCS 工作空間（流程控制邏輯）
- **歷史參考**:
  - ~~`app/tafl_wcs_ws/`~~ - ⚠️ 已棄用 TAFL 基礎 WCS 系統
  - ~~**TAFL Editor**: AGVCUI 中的 `/tafl/editor`~~ - ⚠️ 已棄用視覺化流程編輯器
  - ~~**TAFL 語言規範**: docs-ai/knowledge/system/tafl/tafl-language-specification.md~~ - 歷史參考
- **相關文檔**:
  - **資料庫設計**: docs-ai/knowledge/agv-domain/wcs-database-design.md
  - **Work ID 系統**: docs-ai/knowledge/agv-domain/wcs-workid-system.md
  - **業務流程**: docs-ai/knowledge/business/eyewear-production-process.md
  - **系統架構**: docs-ai/context/system/dual-environment.md