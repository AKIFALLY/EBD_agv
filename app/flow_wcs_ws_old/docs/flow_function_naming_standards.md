# Flow WCS 函數命名標準

## 建議使用的標準命名（更明確、一致）

### Query 函數
- ✅ `query.locations` - 查詢位置
- ✅ `query.racks` - 查詢料架
- ✅ `query.tasks` - 查詢任務
- ✅ `query.agvs` - 查詢 AGV

### Check 函數
- ✅ `check.empty` - 檢查是否為空
- ✅ `check.rack_status` - 檢查料架狀態
- ✅ `check.task_exists` - 檢查任務是否存在
- ✅ `check.location_available` - 檢查位置是否可用
- ✅ `check.system_ready` - 檢查系統就緒

### Task 函數
- ✅ `task.create_task` - 創建任務（推薦）
- ✅ `task.update_task` - 更新任務（推薦）
- ✅ `task.assign_task` - 分配任務（推薦）
- ✅ `task.cancel_task` - 取消任務（推薦）

### Action 函數
- ✅ `action.log_message` - 記錄訊息（推薦，而非 action.log）
- ✅ `action.send_notification` - 發送通知（推薦，而非 action.notify）
- ✅ `action.rotate_rack` - 旋轉料架
- ✅ `action.optimize_batch` - 優化批次
- ✅ `action.analyze_priorities` - 分析優先級
- ✅ `action.find_optimal_agv` - 尋找最佳 AGV
- ✅ `action.send_alert` - 發送警報
- ✅ `action.cleanup_resources` - 清理資源
- ✅ `action.generate_report` - 生成報告

### Control 函數
- ✅ `control.wait_time` - 等待時間（推薦，而非 control.wait）
- ✅ `control.stop_flow` - 停止流程（推薦，而非 control.stop）
- ✅ `control.count_items` - 計算項目（推薦，而非 control.count）
- ✅ `control.update_variable` - 更新變數
- ✅ `control.switch_case` - Switch case 控制
- ✅ `control.foreach` - 迴圈控制

### 特殊結構（直接使用）
- ✅ `foreach` - ForEach 迴圈結構
- ✅ `parallel` - 平行執行結構（如果支援）

## 命名原則

1. **使用完整動詞**：`log_message` 比 `log` 更清楚
2. **保持一致性**：同類函數使用相同的命名模式
3. **避免縮寫**：除非是廣泛認可的縮寫
4. **描述性命名**：函數名稱應該清楚表達其功能

## 範例 Flow

```yaml
workflow:
  - section: "標準命名範例"
    steps:
      - id: "log_start"
        exec: "action.log_message"  # 推薦
        params:
          message: "開始執行"
          level: "info"
      
      - id: "query_data"
        exec: "query.locations"     # 標準格式
        params:
          type: "entrance"
      
      - id: "send_notice"
        exec: "action.send_notification"  # 推薦
        params:
          message: "查詢完成"
          level: "info"
```

---
*建立時間: 2025-08-13 09:45:00*
*版本: v1.0*