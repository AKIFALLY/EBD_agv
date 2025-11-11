# ROS2 服務管理函數開發標準

## 📋 概述

本文檔定義了 RosAGV 專案中所有 ROS2 服務管理函數的開發標準。所有 `manage_*` 函數都應遵循此標準，以確保系統的穩定性、可維護性和一致性。

## 🎯 標準化目標

### 為什麼需要標準化？

在標準化之前，各個服務管理函數的實現差異很大，導致：
- ❌ 某些服務啟動後無法完全停止，留下殘留進程
- ❌ 重啟服務時偶爾失敗，需要手動清理
- ❌ 缺少詳細的診斷資訊，故障排查困難
- ❌ 沒有統一的日誌查看方式

### 標準化後的優勢

採用統一標準後，所有服務管理函數都能：
- ✅ **可靠啟動**：每次都能正常啟動和停止
- ✅ **完全清理**：6 階段停止流程確保無殘留進程
- ✅ **幂等性**：重複執行 start 不會造成問題
- ✅ **診斷友好**：失敗時提供詳細的診斷建議
- ✅ **統一介面**：所有服務都支援 start/stop/restart/status/logs

## 📚 核心文檔

### 必讀文檔

1. **[manage-function-template.md](./manage-function-template.md)** - 詳細的實作模板
   - 完整的程式碼範例
   - 驗證方法決策樹
   - 常見陷阱與解決方案
   - 28 項核心檢查清單

2. **本文檔** - 開發標準與最佳實踐

## 🏗️ 架構設計

### 4 階段啟動流程

```bash
階段 1: 啟動前檢查
├─ 1.1 檢查是否已運行（幂等性）
└─ 1.2 清理過期的 PID 文件

階段 2: 依賴檢查
├─ 2.1 檢查上游服務依賴
├─ 2.2 檢查資料庫連接
└─ 2.3 檢查工作空間建置

階段 3: 啟動服務
├─ 3.1 創建日誌檔案
├─ 3.2 啟動服務進程
├─ 3.3 記錄父進程 PID
├─ 3.4 記錄子進程 PID
└─ 3.5 記錄服務進程 PID

階段 4: 驗證啟動
├─ 4.1 驗證父進程存活
├─ 4.2 驗證 ROS2 節點註冊 / 進程存在 / 端口監聽
├─ 4.3 最終進程驗證
└─ 4.4 顯示啟動摘要
```

### 6 階段停止流程

```bash
階段 1: 優雅停止 (SIGTERM)
└─ 按 PID 文件從後到前發送 SIGTERM，等待 3 秒

階段 2: 強制終止 (SIGKILL)
└─ 對仍在運行的進程發送 SIGKILL

階段 3: 備份清理
└─ 檢查並清理僵屍進程（zombie processes）

階段 4: 殘留進程清理
└─ 使用 pgrep 查找並清理所有殘留進程

階段 5: 端口資源釋放
└─ 清理佔用的端口（如果有）

階段 6: 清理臨時文件
├─ 清理 PID 文件
└─ 清理 launch 參數目錄
```

## 🔍 驗證方法決策樹

根據服務類型選擇正確的驗證方法：

```
服務類型判斷
│
├─ ROS2 Launch 服務？
│  └─ 是 → 使用 verify_ros2_node_startup()
│         (等待 ROS2 節點在網路中註冊)
│
├─ 單一 ROS2 節點？
│  └─ 是 → 使用 verify_ros2_node_startup()
│         (確認節點已註冊到 ROS2 網路)
│
├─ Web 服務（有端口）？
│  └─ 是 → 使用 wait_for_port_with_retry()
│         (等待端口開始監聽)
│
└─ 其他進程？
   └─ 使用 verify_process_startup()
      (檢查進程存在即可)
```

## 📝 已標準化的函數

### 核心服務 (3個)
- ✅ `manage_plc_service_agvc` - PLC 服務管理
- ✅ `manage_ecs_core` - ECS 核心服務管理
- ✅ `manage_rcs_core` - RCS 核心服務管理

### 資料服務 (3個)
- ✅ `manage_agvc_database_node` - 資料庫代理節點
- ✅ `manage_tafl_wcs` - TAFL WCS 流程控制
- ✅ `manage_room_task_build` - Room Task Build 節點

### Web 服務 (參考標準)
- ✅ `manage_web_api_launch` - Web API 服務群組（最佳實踐範例）

## 🚀 快速開始指南

### 1️⃣ 新增服務管理函數

當需要為新的 ROS2 服務創建管理函數時：

1. **選擇合適的模板**
   - ROS2 Launch 服務 → 使用完整模板
   - 單一 ROS2 節點 → 使用簡化模板
   - Web 服務 → 參考 `manage_web_api_launch`

2. **複製模板程式碼**
   - 從 `manage-function-template.md` 複製對應模板
   - 替換服務名稱、檔案路徑、節點名稱

3. **調整依賴檢查**
   - 根據服務實際依賴，調整階段 2 的依賴檢查
   - 例如：檢查資料庫、上游服務、配置檔案

4. **選擇驗證方法**
   - 參考決策樹選擇 `verify_ros2_node_startup` 或其他方法
   - 設定合適的超時時間（一般 10-15 秒）

5. **測試完整流程**
   ```bash
   # 測試啟動
   manage_your_service start

   # 測試幂等性（應該返回「已在運行」）
   manage_your_service start

   # 測試狀態檢查
   manage_your_service status

   # 測試日誌查看
   manage_your_service logs

   # 測試停止
   manage_your_service stop

   # 驗證完全清理（應該無殘留進程）
   ps aux | grep your_service
   ```

### 2️⃣ 改進現有函數

如果需要改進現有的服務管理函數：

1. **評估現有實作**
   - 是否有 PID 文件管理？
   - 是否有多層 PID 追蹤？
   - 停止流程是否完整？
   - 是否有 logs 子命令？

2. **對照檢查清單**
   - 使用模板文檔中的 28 項檢查清單
   - 逐項檢查並標記缺失功能

3. **逐步改進**
   - 優先改進停止流程（6 階段）
   - 然後改進啟動流程（4 階段）
   - 最後加強診斷資訊

## 💡 最佳實踐

### DO ✅

1. **使用 PID 文件追蹤所有進程**
   ```bash
   echo "$PARENT_PID" > "$PID_FILE"
   echo "$CHILD_PIDS" >> "$PID_FILE"
   echo "$SERVICE_PIDS" >> "$PID_FILE"
   ```

2. **從後到前停止進程（使用 tac）**
   ```bash
   while IFS= read -r pid; do
       kill -TERM "$pid" 2>/dev/null || true
   done < <(tac "$PID_FILE")
   ```

3. **提供詳細的診斷建議**
   ```bash
   echo "💡 診斷建議："
   echo "   1. 查看日誌: tail -f $LOG_FILE"
   echo "   2. 檢查環境: ros2 node list"
   ```

4. **實現幂等性**
   ```bash
   if [ -f "$PID_FILE" ]; then
       local all_running=true
       while IFS= read -r pid; do
           [ -z "$pid" ] && continue
           if ! kill -0 "$pid" 2>/dev/null; then
               all_running=false; break
           fi
       done < "$PID_FILE"
       if [ "$all_running" = true ]; then
           echo "✅ 服務已在運行中"
           return 0
       fi
   fi
   ```

5. **清理僵屍進程**
   ```bash
   local zombie_pids=$(pgrep -f "service_name" | while read p; do
       if [ -d "/proc/$p" ] && grep -q "Z (zombie)" "/proc/$p/status" 2>/dev/null; then
           echo $p
       fi
   done)
   ```

### DON'T ❌

1. **不要只記錄父進程 PID**
   ```bash
   # ❌ 錯誤：只記錄父進程
   echo "$PARENT_PID" > "$PID_FILE"

   # ✅ 正確：記錄所有相關進程
   echo "$PARENT_PID" > "$PID_FILE"
   echo "$CHILD_PIDS" >> "$PID_FILE"
   echo "$SERVICE_PIDS" >> "$PID_FILE"
   ```

2. **不要只用 SIGTERM 停止**
   ```bash
   # ❌ 錯誤：只發送 SIGTERM
   kill -TERM $pid

   # ✅ 正確：SIGTERM → 等待 → SIGKILL
   kill -TERM $pid
   sleep 3
   if kill -0 $pid 2>/dev/null; then
       kill -KILL $pid
   fi
   ```

3. **不要忽略停止失敗時的清理**
   ```bash
   # ❌ 錯誤：PID 文件中的進程停止失敗後就結束

   # ✅ 正確：繼續使用 pgrep 查找殘留進程
   local remaining_pids=$(pgrep -f "service_name")
   if [ -n "$remaining_pids" ]; then
       kill -KILL $remaining_pids
   fi
   ```

4. **不要使用簡單的 pgrep 檢查幂等性**
   ```bash
   # ❌ 錯誤：只檢查進程存在
   if pgrep -f "service" > /dev/null; then
       return 0
   fi

   # ✅ 正確：驗證 PID 文件中所有進程
   if [ -f "$PID_FILE" ]; then
       local all_running=true
       while IFS= read -r pid; do
           if ! kill -0 "$pid" 2>/dev/null; then
               all_running=false; break
           fi
       done < "$PID_FILE"
   fi
   ```

5. **不要省略診斷資訊**
   ```bash
   # ❌ 錯誤：只說失敗
   echo "❌ 啟動失敗"
   return 1

   # ✅ 正確：提供診斷步驟
   echo "❌ 啟動失敗"
   echo "💡 診斷建議："
   echo "   1. 查看日誌: tail -f $LOG_FILE"
   echo "   2. 檢查建置: ls /app/xxx_ws/install"
   ```

## 🔧 常見問題與解決方案

### Q1: 服務啟動後立即退出

**症狀**：父進程啟動成功，但驗證時發現已經退出

**可能原因**：
- 工作空間未建置或建置不完整
- 環境變數未正確載入
- Python 模組導入失敗
- 配置檔案路徑錯誤

**診斷步驟**：
```bash
# 1. 查看日誌
tail -f /tmp/service.log

# 2. 手動執行命令測試
source /app/setup.bash
agvc_source
ros2 run package_name node_name

# 3. 檢查建置
ls /app/xxx_ws/install/package_name
```

### Q2: 停止後仍有殘留進程

**症狀**：執行 stop 後，`ps aux | grep service` 仍顯示進程

**可能原因**：
- 只記錄了父進程 PID，未記錄子進程和服務進程
- 停止流程缺少階段 4（殘留進程清理）
- 僵屍進程需要清理父進程

**解決方案**：
```bash
# 確保記錄所有進程
echo "$PARENT_PID" > "$PID_FILE"
sleep 1
echo "$(pgrep -P $PARENT_PID)" >> "$PID_FILE"
sleep 1
echo "$(pgrep -f 'service_name')" >> "$PID_FILE"

# 添加殘留進程清理
local remaining_pids=$(pgrep -f "service_name")
if [ -n "$remaining_pids" ]; then
    kill -KILL $remaining_pids
fi
```

### Q3: ROS2 節點驗證失敗但進程存在

**症狀**：進程正在運行，但 `ros2 node list` 看不到節點

**可能原因**：
- Zenoh Router 未運行
- ROS_DOMAIN_ID 不匹配
- 網路連接問題
- 節點啟動延遲較長

**診斷步驟**：
```bash
# 1. 檢查 Zenoh Router
check_zenoh_status

# 2. 檢查 ROS2 網路
ros2 doctor --report

# 3. 延長超時時間
verify_ros2_node_startup "node_name" 20  # 從 10 秒改為 20 秒
```

### Q4: 重啟服務時失敗

**症狀**：第一次啟動成功，重啟時失敗

**可能原因**：
- 端口仍被佔用
- PID 文件未清理
- 臨時文件未清除
- 前一次未完全停止

**解決方案**：
```bash
# 確保 stop 完整執行 6 階段
# 特別是階段 5（端口釋放）和階段 6（臨時文件清理）

# 端口清理範例
if lsof -ti:$PORT > /dev/null 2>&1; then
    kill -KILL $(lsof -ti:$PORT)
fi

# 臨時文件清理
rm -f "$PID_FILE"
rm -rf /tmp/launch_params_*
```

## 📊 驗證檢查清單

完成函數開發後，請使用以下檢查清單驗證：

### 基本功能 (10項)
- [ ] 支援 start 子命令
- [ ] 支援 stop 子命令
- [ ] 支援 restart 子命令
- [ ] 支援 status 子命令
- [ ] 支援 logs 子命令
- [ ] 有 LOG_FILE 和 PID_FILE 定義
- [ ] 有環境檢測（is_agvc_environment）
- [ ] 有清晰的區段標題和註解
- [ ] 有使用說明（usage message）
- [ ] 有標準化的輸出格式（分隔線、emoji）

### 啟動流程 (8項)
- [ ] 階段 1: 幂等性檢查（PID 文件驗證）
- [ ] 階段 2: 依賴檢查（資料庫、上游服務等）
- [ ] 階段 3: 記錄父進程 PID
- [ ] 階段 3: 記錄子進程 PID
- [ ] 階段 3: 記錄服務進程 PID
- [ ] 階段 4: 父進程驗證
- [ ] 階段 4: 使用正確的驗證方法
- [ ] 階段 4: 啟動失敗時清理並提供診斷

### 停止流程 (6項)
- [ ] 階段 1: SIGTERM 優雅停止（使用 tac 倒序）
- [ ] 階段 2: SIGKILL 強制終止
- [ ] 階段 3: 僵屍進程清理
- [ ] 階段 4: 殘留進程清理（pgrep）
- [ ] 階段 5: 端口資源釋放
- [ ] 階段 6: 臨時文件清理

### 診斷與日誌 (4項)
- [ ] 失敗時提供診斷建議（💡 符號）
- [ ] status 顯示詳細資訊（PID、節點狀態、日誌）
- [ ] logs 使用 tail -f 持續顯示
- [ ] 所有關鍵步驟有清晰的輸出訊息

## 📖 參考資料

### 內部文檔
- [manage-function-template.md](./manage-function-template.md) - 詳細實作模板
- [common.bash](../../../../app/setup_modules/common.bash) - 驗證函數實作
- [node-management.bash](../../../../app/setup_modules/node-management.bash) - 所有管理函數

### 最佳實踐範例
- `manage_web_api_launch` - Web 服務管理（603 行，6階段停止 + 4階段啟動）
- `manage_plc_service_agvc` - PLC 服務管理（256 行，標準化實作）
- `manage_tafl_wcs` - Launch 服務管理（340 行，完整流程）

## 🎯 未來改進方向

### 短期計劃
- [ ] 標準化剩餘的服務管理函數
- [ ] 創建自動化測試腳本
- [ ] 統一日誌格式和級別

### 長期計劃
- [ ] 開發函數生成工具（基於模板自動生成）
- [ ] 整合監控系統（Prometheus/Grafana）
- [ ] 實現服務依賴圖自動檢查

## 📝 變更歷史

### 2025-11-07
- ✅ 建立標準化規範文檔
- ✅ 完成 6 個核心服務函數標準化
- ✅ 創建開發模板和檢查清單

---

**維護者**: AI Agent
**最後更新**: 2025-11-07
**文檔版本**: 1.0
