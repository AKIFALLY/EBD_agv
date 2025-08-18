# Linear Flow Designer 靜態備援同步管理

## 🎯 適用場景
- 定期更新 Linear Flow Designer 的靜態備援函數列表
- 保持靜態備援與 flow_wcs API 同步
- 確保離線模式下函數列表的完整性

## 📋 問題背景

Linear Flow Designer 使用三層載入策略：
1. **Live API**: 從 flow_wcs API 即時獲取
2. **Cache YAML**: 從快取檔案載入
3. **Static Fallback**: 硬編碼在程式中的備援

靜態備援需要定期更新以保持與實際 API 的同步。

## 🔧 解決方案

### 自動同步機制

我們提供了兩種同步方式：

#### 1. 手動同步（立即執行）
```bash
# 立即同步靜態備援
scripts/sync-static-fallback.sh sync

# 或使用 r 工具
r sync-fallback
```

#### 2. 定時自動同步
```bash
# 設置每日凌晨 2:00 自動同步
scripts/sync-static-fallback.sh setup

# 移除定時任務
scripts/sync-static-fallback.sh remove

# 查看同步狀態
scripts/sync-static-fallback.sh status
```

### 同步流程

1. **獲取函數列表**
   - 優先從 flow_wcs API 獲取最新函數
   - 如果 API 無法訪問，從快取檔案載入

2. **更新靜態備援**
   - 自動更新 `linear_flow_designer.py` 中的靜態函數列表
   - 保留所有函數的預設值

3. **備份 JSON**
   - 保存一份 JSON 格式的備份至 `/app/config/wcs/static_fallback_functions.json`
   - 便於查看和驗證

## 📂 相關檔案

### 核心檔案
- **同步腳本**: `/app/web_api_ws/src/agvcui/agvcui/routers/sync_static_fallback.py`
- **Shell 腳本**: `/home/ct/RosAGV/scripts/sync-static-fallback.sh`
- **目標檔案**: `/app/web_api_ws/src/agvcui/agvcui/routers/linear_flow_designer.py`

### 資料檔案
- **快取檔案**: `/app/config/wcs/flow_functions_cache.yaml`
- **JSON 備份**: `/app/config/wcs/static_fallback_functions.json`
- **同步日誌**: `/tmp/sync_static_fallback.log`

## 🚀 使用指南

### 初次設置
```bash
# 1. 執行一次手動同步，確認工作正常
scripts/sync-static-fallback.sh sync

# 2. 如果成功，設置定時任務
scripts/sync-static-fallback.sh setup

# 3. 確認狀態
scripts/sync-static-fallback.sh status
```

### 日常維護
```bash
# 查看同步狀態
scripts/sync-static-fallback.sh status

# 查看同步日誌
tail -f /tmp/sync_static_fallback.log

# 手動觸發同步（如有需要）
scripts/sync-static-fallback.sh sync
```

### 故障排除
```bash
# 如果同步失敗，檢查：

# 1. AGVC 容器是否運行
docker compose -f docker-compose.agvc.yml ps

# 2. flow_wcs API 是否可用
curl http://localhost:8000/api/flow/functions

# 3. 快取檔案是否存在
ls -la /home/ct/RosAGV/app/config/wcs/flow_functions_cache.yaml

# 4. 檢查錯誤日誌
grep ERROR /tmp/sync_static_fallback.log
```

## 📊 同步時機建議

### 建議的同步時機
- **每日凌晨**: 2:00 AM（系統負載最低）
- **部署後**: 更新 flow_wcs 後手動同步
- **問題修復後**: 修正函數定義後立即同步

### 不建議的時機
- **高峰時段**: 避免在業務高峰期同步
- **系統維護中**: 等待維護完成後再同步

## 💡 最佳實踐

### 監控建議
1. **定期檢查**: 每週檢查一次同步狀態
2. **日誌審查**: 定期查看同步日誌是否有錯誤
3. **版本追蹤**: 記錄每次同步的函數數量變化

### 備份策略
1. **保留 JSON 備份**: 每次同步都會生成 JSON 備份
2. **版本控制**: 將更新後的 `linear_flow_designer.py` 提交到 Git
3. **快照備份**: 定期備份整個配置目錄

## 🔗 相關文檔
- Linear Flow Designer 測試模式: @docs-ai/knowledge/system/linear-flow-designer-test-cache-system.md
- 系統診斷: @docs-ai/operations/maintenance/system-diagnostics.md
- 統一工具: @docs-ai/operations/tools/unified-tools.md