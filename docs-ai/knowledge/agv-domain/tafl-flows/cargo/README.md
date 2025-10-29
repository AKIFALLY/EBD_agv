# Cargo AGV TAFL Flows

## 📋 概述

Cargo AGV 負責 RosAGV 系統的**物料搬運**，從料架（Rack）取出載具卸載到入口傳送箱，以及從出口傳送箱取出載具裝載回料架，連接整個制程的起點和終點。

## 🔄 完整制程流程

```
料架（Rack儲存）
    ↓
┌─────────────────────────────────┐
│   Cargo AGV 物料搬運（2個 Flow） │
├─────────────────────────────────┤
│ Flow 1: ENTRANCE_UNLOAD         │ ← 從料架卸載到入口箱
└─────────────────────────────────┘
    ↓
入口傳送箱（BOX_IN_TRANSFER）
    ↓
Loader AGV（前段制程：入口 → 預烘機）
    ↓
Unloader AGV（後段制程：預烘機 → 烤箱 → 出口）
    ↓
出口傳送箱（BOX_OUT_TRANSFER）
    ↓
┌─────────────────────────────────┐
│   Cargo AGV 物料搬運（2個 Flow） │
├─────────────────────────────────┤
│ Flow 2: EXIT_LOAD               │ ← 從出口箱裝載回料架
└─────────────────────────────────┘
    ↓
料架（Rack儲存）
```

## 🎯 2 個 TAFL Flows

### Flow 1: [入口卸載](./01_entrance_unload.md)
- **Work ID**: 2000102
- **功能**: 從料架取出載具，卸載到入口傳送箱
- **觸發條件**: 入口傳送箱有空位，料架有待處理載具
- **KUKA API**: `rack_move` (RACK_MOVE)
- **執行結果**: 載具從料架移至入口傳送箱，等待 Loader AGV 取料

### Flow 2: [出口裝載](./02_exit_load.md)
- **Work ID**: 2000201
- **功能**: 從出口傳送箱取出載具，裝載回料架
- **觸發條件**: 出口傳送箱有完成制程的載具，料架有空位
- **KUKA API**: `rack_move` (RACK_MOVE)
- **執行結果**: 載具從出口傳送箱移回料架，完成整個制程循環

## 🔧 技術規格總覽

### Work ID 系統

| Work ID | 操作 | 起點 | 終點 | KUKA API | 說明 |
|---------|------|------|------|----------|------|
| 2000102 | 卸載 | Rack | 入口傳送箱 (Equipment 201) | rack_move | 啟動前段制程 |
| 2000201 | 裝載 | 出口傳送箱 (Equipment 202) | Rack | rack_move | 完成整體循環 |

**關鍵設計特點**：
- ✅ **2個 Work ID**（對應入口卸載和出口裝載）
- ✅ **KUKA Fleet 整合**：使用 KUKA Fleet Manager API
- ✅ **批量4格處理**：與 Unloader AGV 統一，一次處理4格載具
- ✅ **料架180度旋轉**：KUKA AGV 自動處理料架旋轉，對接傳送箱

### KUKA Fleet API 整合

Cargo AGV 透過 KUKA Fleet Manager 執行物料搬運任務：

```python
# Work ID 參數配置
{
    "function": "rack_move",           # KUKA Fleet 功能類型
    "api": "submit_mission",           # KUKA API 端點
    "missionType": "RACK_MOVE",        # KUKA 任務類型
    "nodes": []                        # 路徑節點（由系統動態生成）
}
```

**KUKA AGV 自動處理**：
- 導航到料架位置
- 舉起料架（Lift Rack）
- 旋轉料架 180 度（適配傳送箱方向）
- 移動到目標位置
- 放下料架（Drop Rack）

### Cargo AGV 狀態機架構

**入口流程狀態機** (`entrance/` 目錄，7個狀態）：
- `check_rack_side_state.py` - 檢查料架側邊
- `select_rack_port_state.py` - 選擇料架端口
- `take_rack_port_state.py` - 從料架取出載具
- `rack_vision_position_state.py` - 料架視覺定位
- `transfer_check_empty_state.py` - 檢查傳送箱空位
- `transfer_vision_position_state.py` - 傳送箱視覺定位
- `put_tranfer_state.py` - 放入傳送箱
- `wait_rotation_state.py` - 等待料架旋轉

**出口流程狀態機** (`exit/` 目錄，7個狀態）：
- 類似結構，反向操作（從出口傳送箱裝載回料架）

## 📊 Carrier 狀態轉換

```
料架儲存（在Rack）
    ↓
100（待入料）     ← Flow 1 ENTRANCE_UNLOAD
    ↓
201（入口箱）     ← Loader AGV 前段制程開始
    ↓
... Loader/Unloader 制程 ...
    ↓
202（出口箱）     ← Flow 2 EXIT_LOAD
    ↓
料架儲存（在Rack）
```

## 🎯 設計特點

### 整合 KUKA Fleet
- **KUKA Fleet Manager**: 統一的 KUKA AGV 調度介面
- **rack_move API**: 專門的料架搬運功能
- **自動導航**: KUKA AGV 自主規劃路徑
- **料架旋轉**: 180度旋轉適配傳送箱對接

### 批量處理一致性
- **4格批量**: 與 Unloader AGV 統一批量處理邏輯
- **統一批次**: 確保整個制程的批量一致性
- **效率優化**: 減少搬運次數，提升產能

### 系統銜接
- **前段銜接**: Flow 1 完成後，Loader AGV 接手處理
- **後段銜接**: Unloader AGV 完成後，Flow 2 接手回收
- **循環完整**: 料架 → 制程 → 料架的完整循環

## 🔗 相關文檔

### AGV 領域知識
- [WCS Work ID 系統](../../wcs-workid-system.md) - Work ID 編碼規則和系統設計
- [WCS 資料庫設計](../../wcs-database-design.md) - 資料表結構和關係
- [WCS 系統設計](../../wcs-system-design.md) - WCS 整體架構

### TAFL 系統
- [TAFL 語言規範](../../../system/tafl/tafl-language-specification.md)
- [TAFL 使用指南](../../../system/tafl/tafl-user-guide.md)
- [TAFL Editor 規範](../../../system/tafl/tafl-editor-specification.md)

### 實作代碼
- **Cargo AGV 狀態機**: `/app/agv_ws/src/cargo_mover_agv/cargo_mover_agv/robot_states/`
  - `entrance/` - 入口卸載流程（7個狀態）
  - `exit/` - 出口裝載流程（7個狀態）
- **KUKA Fleet Manager**: `/app/kuka_fleet_ws/src/kuka_fleet/kuka_fleet/kuka_fleet_manager.py`
- **Work ID 初始化**: `/app/db_proxy_ws/src/db_proxy/db_proxy/sql/init_data/13_works_tasks.py`

## 📅 版本歷史

- **2025-10-16**: 建立 Cargo AGV TAFL Flows 設計文檔
- **未來計劃**:
  - YAML 實作和測試
  - 與 KUKA Fleet 整合測試
  - 料架旋轉邏輯優化

## 💡 實施指南

### TAFL Flow 開發順序
1. ✅ 設計階段：完成所有 Flow 設計文檔（本目錄）
2. ⏳ 實作階段：編寫 YAML 配置文件
3. ⏳ 測試階段：單元測試、整合測試、邊界測試
4. ⏳ 部署階段：載入到 TAFL WCS 系統

### 關鍵注意事項
- **KUKA Fleet 依賴**: 確保 KUKA Fleet Manager 服務正常運行
- **料架旋轉**: KUKA AGV 自動處理，無需額外邏輯
- **批量一致性**: 與 Loader/Unloader 保持4格批量一致
- **狀態碼驗證**: 確保 Carrier status_id 轉換正確
