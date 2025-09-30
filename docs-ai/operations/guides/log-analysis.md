# 日誌分析工具 Prompt

## 🎯 適用場景
- 系統錯誤日誌掃描和分析
- 容器日誌監控和診斷
- 異常模式識別和報告
- 日誌錯誤等級分析
- 故障排除和問題追蹤

## 📋 核心工具概述
RosAGV 日誌分析工具集，提供智能的日誌掃描、錯誤分析、異常檢測和故障診斷功能，支援多容器環境的統一日誌管理。

## 🔧 日誌分析指導

### 🚀 超簡單使用方式 (推薦)
```bash
# 使用統一工具入口 (最簡單)
r log-scan              # 日誌錯誤掃描
r log-errors            # 高級錯誤掃描
```

### 載入專業工具集 (進階使用)
```bash
# 載入日誌分析工具集
source scripts/log-tools/log-tools.sh

# 顯示可用工具
show_log_tools_help
```

## 🔧 主要工具說明

### 1. 智能日誌分析器 (log-analyzer.sh)
核心日誌分析工具，提供智能錯誤識別和解決建議。

```bash
scripts/log-tools/log-analyzer.sh [source] [options]
```

**日誌來源**:
- `agv` - AGV 容器日誌
- `agvc` - AGVC 容器日誌  
- `postgres` - PostgreSQL 資料庫日誌
- `nginx` - Nginx 網頁伺服器日誌
- `all` - 所有容器日誌
- `file <path>` - 自定義日誌檔案

**主要選項**:
- `--stats` - 顯示統計資訊
- `--severity <level>` - 依嚴重程度過濾 (1-5)
- `--suggestions` - 顯示解決建議
- `--timeline` - 顯示錯誤時間軸
- `--export <format>` - 匯出報告 (json/csv/html)

**常用範例**:
```bash
# 分析 AGV 容器日誌
scripts/log-tools/log-analyzer.sh agv --stats

# 分析高嚴重程度錯誤並提供建議
scripts/log-tools/log-analyzer.sh agvc --severity 4 --suggestions

# 產生完整分析報告
scripts/log-tools/log-analyzer.sh all --export html
```

### 2. 統一日誌工具 (log-tools.sh)
整合所有日誌分析功能的統一介面。

```bash
scripts/log-tools/log-tools.sh [command] [options]
```

**便捷命令**:
- `log_analyze <source>` - 快速日誌分析
- `log_quick_scan` - 快速掃描所有容器錯誤
- `log_find_errors <pattern>` - 搜尋特定錯誤模式
- `log_timeline <source>` - 顯示錯誤時間軸
- `log_suggestions <source>` - 獲取解決建議
- `log_monitor <source> <interval>` - 即時監控日誌
- `log_export_report` - 產生完整分析報告
- `log_full_diagnosis` - 執行完整診斷

**常用範例**:
```bash
# 快速掃描所有錯誤
log_quick_scan

# 搜尋超時錯誤
log_find_errors "timeout"

# 即時監控 AGV 日誌 (每10秒更新)
log_monitor agv 10

# 產生完整診斷報告
log_full_diagnosis
```

## 🔍 錯誤模式識別

### 預定義錯誤模式 (12種)
日誌分析器內建 12 種常見錯誤模式識別：

1. **CRITICAL** - 嚴重系統錯誤
2. **ERROR** - 一般錯誤
3. **EXCEPTION** - 程式異常
4. **TIMEOUT** - 超時錯誤
5. **CONNECTION** - 連接問題
6. **FAILED** - 操作失敗
7. **WARNING** - 警告訊息
8. **DENIED** - 存取拒絕
9. **NOT_FOUND** - 資源不存在
10. **INVALID** - 無效操作
11. **MEMORY** - 記憶體問題
12. **DISK** - 磁碟空間問題

### 嚴重程度分級 (5級)
- **Level 5**: 嚴重錯誤 (系統崩潰、服務停止)
- **Level 4**: 重要錯誤 (功能失效、連接中斷)
- **Level 3**: 一般錯誤 (操作失敗、超時)
- **Level 2**: 警告 (性能問題、配置警告)
- **Level 1**: 資訊 (狀態變更、一般資訊)

## 🚀 日誌分析工作流程

### 基本分析流程
```bash
# 1. 快速掃描識別問題
log_quick_scan

# 2. 針對特定容器深入分析
log_analyze agv --stats --suggestions

# 3. 檢視錯誤時間軸
log_timeline agv

# 4. 產生完整報告
log_export_report
```

### 故障排除流程
```bash
# 1. 即時監控問題
log_monitor agv 5

# 2. 搜尋特定錯誤
log_find_errors "connection refused"

# 3. 獲取解決建議
log_suggestions agv

# 4. 執行完整診斷
log_full_diagnosis
```

### 性能分析流程
```bash
# 1. 分析性能相關日誌
scripts/log-tools/log-analyzer.sh agvc --severity 2 --timeline

# 2. 搜尋性能關鍵字
log_find_errors "slow\|timeout\|performance"

# 3. 產生性能分析報告
scripts/log-tools/log-analyzer.sh all --export json --filter performance
```

## 📊 報告和輸出格式

### 支援的輸出格式
- **Console** - 彩色終端輸出，適合即時查看
- **JSON** - 結構化數據，便於程式處理
- **CSV** - 表格格式，便於數據分析
- **HTML** - 詳細網頁報告，包含圖表

### 統計資訊內容
```bash
# 統計報告包含：
- 總日誌行數
- 錯誤模式分佈
- 嚴重程度統計
- 時間分佈分析
- 最頻繁錯誤
- 錯誤趨勢分析
```

### 報告生成範例
```bash
# 產生 HTML 詳細報告
scripts/log-tools/log-analyzer.sh all --stats --suggestions --export html

# 產生 JSON 格式統計
scripts/log-tools/log-analyzer.sh agv --stats --export json > agv_analysis.json

# 產生 CSV 錯誤清單
scripts/log-tools/log-analyzer.sh agvc --severity 3 --export csv > errors.csv
```

## 🔧 進階分析功能

### 時間範圍分析
```bash
# 分析最近 1 小時的日誌
scripts/log-tools/log-analyzer.sh agv --since "1 hour ago"

# 分析特定時間段
scripts/log-tools/log-analyzer.sh agvc --since "2024-01-01 10:00" --until "2024-01-01 12:00"
```

### 自定義模式搜尋
```bash
# 使用正規表達式搜尋
log_find_errors "ERROR.*database.*connection"

# 多模式搜尋
log_find_errors "timeout|failed|error" --case-insensitive
```

### 日誌聚合分析
```bash
# 跨容器關聯分析
scripts/log-tools/log-analyzer.sh all --correlate --timeline

# 錯誤分組統計
scripts/log-tools/log-analyzer.sh agv --group-by error_type --stats
```

## 🔍 智能解決建議

### 建議系統功能
日誌分析器提供針對常見問題的解決建議：

- **連接問題**: 網路配置檢查步驟
- **權限錯誤**: 權限修復命令
- **資源不足**: 系統資源最佳化建議
- **配置錯誤**: 配置檔案檢查和修復
- **服務故障**: 服務重啟和恢復步驟

### 建議輸出範例
```bash
# 獲取 PostgreSQL 連接問題建議
log_analyze postgres --suggestions

# 輸出範例：
# [建議] 連接被拒絕錯誤:
# 1. 檢查 PostgreSQL 服務狀態: systemctl status postgresql
# 2. 驗證連接配置: check_agvc_status
# 3. 測試網路連通性: nc -zv postgres 5432
```

## 🚨 故障排除

### 日誌分析工具問題
```bash
# 檢查工具狀態
scripts/log-tools/log-tools.sh --version

# 重新載入工具集
source scripts/log-tools/log-tools.sh

# 清理快取
rm -rf /tmp/log-analysis-cache/
```

### 日誌存取問題
```bash
# [宿主機] 檢查日誌檔案權限
ls -la /var/log/

# [宿主機] 檢查容器日誌
docker compose -f docker-compose.agvc.yml logs --tail 10

# [宿主機] 檢查磁碟空間
df -h /var/log/
```

### 分析性能問題
```bash
# 限制分析日誌數量
scripts/log-tools/log-analyzer.sh agv --tail 1000

# 使用快速模式
scripts/log-tools/log-analyzer.sh agv --fast-mode

# 並行分析
scripts/log-tools/log-analyzer.sh all --parallel
```

## 💡 最佳實踐

### 日常日誌監控
1. **定期掃描**: 每日執行 `log_quick_scan` 識別新問題
2. **趨勢分析**: 週期性產生統計報告，觀察錯誤趨勢
3. **即時監控**: 對關鍵服務設置即時日誌監控

### 故障排除策略
1. **分層診斷**: 從整體概覽到具體錯誤的分層分析
2. **時間關聯**: 結合錯誤時間軸分析問題根因
3. **跨服務分析**: 分析跨容器的關聯錯誤

### 日誌管理
1. **日誌輪轉**: 設置適當的日誌輪轉策略
2. **保留政策**: 根據重要性設定日誌保留時間
3. **備份策略**: 重要日誌的備份和歸檔

### 自動化分析
```bash
# 創建日誌分析腳本
#!/bin/bash
# daily-log-analysis.sh
echo "開始每日日誌分析..."
log_quick_scan > /tmp/daily-scan-$(date +%Y%m%d).txt
log_export_report
echo "日誌分析完成，報告已產生"
```

## 📋 工具快速參考

| 功能 | 命令 | 選項 | 用途 |
|------|------|------|------|
| 基本分析 | `log_analyze <source>` | `--stats` | 快速日誌分析 |
| 錯誤掃描 | `log_quick_scan` | - | 掃描所有容器錯誤 |
| 模式搜尋 | `log_find_errors <pattern>` | `--case-insensitive` | 搜尋特定錯誤 |
| 時間軸 | `log_timeline <source>` | `--since`, `--until` | 顯示錯誤時間軸 |
| 解決建議 | `log_suggestions <source>` | - | 獲取解決方案 |
| 即時監控 | `log_monitor <source> <sec>` | - | 即時日誌監控 |
| 報告匯出 | `log_export_report` | `--format` | 產生分析報告 |
| 完整診斷 | `log_full_diagnosis` | - | 執行完整診斷 |