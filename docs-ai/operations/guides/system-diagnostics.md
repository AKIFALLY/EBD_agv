# 系統診斷操作指導

## 🎯 適用場景
- 日常系統健康檢查
- 故障排除和問題診斷
- 效能監控和分析
- 預防性維護檢查

## 📋 RosAGV 診斷工具體系

### ⚠️ 診斷工具使用前提條件
**使用 `r` 診斷工具之前，必須將 RosAGV 目錄加入 PATH 環境變數**

在 `~/.bashrc` 中添加以下設定：
```bash
# RosAGV 工具路徑配置
export PATH="/home/ct/EBD_agv:$PATH"
```

設定完成後，重新載入環境：
```bash
source ~/.bashrc
```

驗證配置是否正確：
```bash
# [宿主機] 驗證配置
which r                    # 應該顯示 /home/ct/EBD_agv/r
r agvc-check              # 測試 AGVC 健康檢查功能
```

### 統一診斷入口 (r 命令)
RosAGV 提供統一的診斷工具入口，簡化日常維護操作：

```bash
# [宿主機] 🔍 核心診斷命令
r agvc-check         # AGVC 管理系統健康檢查
r agv-check          # AGV 車載系統健康檢查
r system-health      # 完整系統健康檢查
r quick-diag         # 快速綜合診斷

# [宿主機] 🐳 容器狀態檢查
r containers-status  # 檢查所有容器狀態

# [宿主機] 🌐 網路診斷
r network-check      # 系統端口檢查
r zenoh-check        # Zenoh 連接檢查

# [宿主機] 📋 日誌分析
r log-scan           # 日誌錯誤掃描
r log-errors         # 高級錯誤分析
```

### 專業診斷工具集
```bash
# [宿主機] 載入系統診斷工具集
source scripts/system-tools/system-tools.sh

# [宿主機] 專業診斷函數
system_health_check     # 完整健康檢查
quick_diagnose          # 快速診斷
all_health              # 健康檢查
all_status              # 容器狀態

# [宿主機] Web 服務診斷工具
scripts/system-tools/service-monitor.sh status    # 所有服務監控
scripts/docker-tools/container-status.sh agvc     # AGVC 容器詳細狀態

# [宿主機] 日誌分析工具
scripts/log-tools/log-analyzer.sh agvc --stats     # AGVC 日誌統計
scripts/log-tools/log-analyzer.sh agvc --timeline  # 錯誤時間軸
scripts/log-tools/log-analyzer.sh agvc --suggestions # 解決建議

# [宿主機] 網路和端口診斷
scripts/network-tools/port-check.sh system         # 系統端口檢查
scripts/network-tools/connectivity-test.sh performance --target localhost
scripts/network-tools/port-check.sh --port 8000-8002 # Web 服務端口檢查
```

## 🔍 日常健康檢查流程

### 每日例行檢查
```bash
# [宿主機] 標準每日檢查流程
r agvc-check              # 1. AGVC 系統健康檢查
r containers-status       # 2. 容器運行狀態
r network-check          # 3. 網路連接檢查

# [宿主機] 組合檢查 (一行執行)
r agvc-check && r containers-status && r network-check
```

### 每週深度檢查
```bash
# [宿主機] 深度系統檢查
r system-health          # 完整系統健康檢查
r log-scan              # 日誌錯誤掃描

# [宿主機] 組合深度檢查
r system-health && r log-scan
```

### 故障排除檢查
```bash
# [宿主機] 遇到問題時的診斷流程
r quick-diag             # 1. 快速綜合診斷
r log-errors             # 2. 深度日誌分析
r zenoh-check           # 3. Zenoh 連接專項檢查

# [宿主機] 組合故障診斷
r quick-diag && r log-errors && r zenoh-check
```

## 🐳 容器診斷

### 容器狀態檢查
```bash
# [宿主機] 檢查容器運行狀態
r containers-status

# [宿主機] 詳細容器資訊
# 前提：在 ~/EBD_agv 目錄執行
cd ~/EBD_agv
docker compose -f docker-compose.agvc.yml ps
docker compose -f docker-compose.yml ps

# [宿主機] 容器資源使用
docker stats

# [宿主機] 執行容器內健康檢查
# 前提：在 ~/EBD_agv 目錄執行
cd ~/EBD_agv
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "check_system_status"
```

### 容器日誌檢查
```bash
# [宿主機] 前提：在 ~/EBD_agv 目錄執行
cd ~/EBD_agv

# [宿主機] 查看容器日誌
docker compose -f docker-compose.agvc.yml logs -f agvc_server
docker compose -f docker-compose.agvc.yml logs -f postgres
docker compose -f docker-compose.agvc.yml logs -f nginx

# [宿主機] 查看最近日誌
docker compose -f docker-compose.agvc.yml logs --tail=100 agvc_server
```

### 容器網路診斷
```bash
# [宿主機] 檢查容器網路
docker network ls
docker network inspect rosagv_agvc_network

# [宿主機] 容器間連接測試
# 前提：在 ~/EBD_agv 目錄執行
cd ~/EBD_agv
docker compose -f docker-compose.agvc.yml exec agvc_server ping postgres
docker compose -f docker-compose.agvc.yml exec agvc_server ping nginx
```

## 🌐 網路診斷

### 基礎網路檢查
```bash
# [宿主機] 端口連接檢查
r network-check

# [宿主機] 專業網路診斷工具
scripts/network-tools/port-check.sh system           # 系統端口檢查
scripts/network-tools/port-check.sh --port 8000-8002 # Web 服務端口
scripts/network-tools/connectivity-test.sh performance --target localhost

# [宿主機] 網路端口檢查 (推薦使用 ss)
ss -tulpn | rg "(8000|8001|8002|5432|5050|80|7447)"

# [宿主機] 備選：netstat (舊工具，但仍可用)
netstat -tulpn | rg "(8000|8001|8002|5432|5050|80|7447)"

# [宿主機] 端口可達性測試
telnet localhost 8000
telnet localhost 5432
telnet localhost 7447
```

### Zenoh 通訊診斷
```bash
# [宿主機] Zenoh 連接檢查
r zenoh-check

# [宿主機] 手動 Zenoh 檢查
ps aux | rg zenoh
cat /tmp/zenoh_router.pid
pgrep -f rmw_zenohd

# [容器內] Zenoh 配置檢查
cat /app/routerconfig.json5
```

### 跨環境通訊測試
```bash
# [容器內] AGV 和 AGVC 環境通訊測試
# 在 AGV 容器中
ros2 topic list
ros2 topic echo /agv_status

# 在 AGVC 容器中
ros2 topic list
ros2 topic echo /agv_status
```

## 📊 效能監控

### 系統資源監控
```bash
# [宿主機/容器內] CPU 和記憶體使用
top
htop
free -h

# [宿主機/容器內] 磁碟使用
df -h
du -sh /app/*

# [宿主機/容器內] 網路使用
iftop
nethogs
```

### 容器資源監控
```bash
# [宿主機] 容器資源使用統計
docker stats

# [宿主機] 特定容器資源監控
docker stats agvc_server postgres nginx

# [宿主機] 執行容器內資源檢查
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "top -bn1 | head -20"
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "free -h"
```

### 服務效能監控
```bash
# [宿主機] Web 服務效能
curl -w "@curl-format.txt" -o /dev/null -s "http://localhost:8000/health"

# [宿主機] 資料庫效能監控 (包含詳細欄位說明和健康分析)
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT 
    datname,                                          -- 資料庫名稱
    numbackends as active_connections,                -- 當前活動連接數
    xact_commit,                                      -- 成功提交的交易總數
    xact_rollback,                                    -- 回滾的交易總數  
    blks_read,                                        -- 從磁碟讀取的區塊數
    blks_hit,                                         -- 從緩存命中的區塊數
    temp_files,                                       -- 建立的臨時檔案數量
    temp_bytes,                                       -- 臨時檔案使用的總位元組數
    -- 計算健康指標
    round(blks_hit::numeric / NULLIF(blks_hit + blks_read, 0) * 100, 2) as cache_hit_ratio,
    round(xact_rollback::numeric / NULLIF(xact_commit + xact_rollback, 0) * 100, 2) as rollback_ratio
FROM pg_stat_database 
WHERE datname = 'agvc';"

# [宿主機] 一鍵健康檢查 (自動評估系統狀態)
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
WITH health_metrics AS (
    SELECT 
        datname,
        numbackends as connections,
        round(blks_hit::numeric / NULLIF(blks_hit + blks_read, 0) * 100, 2) as cache_hit_ratio,
        round(xact_rollback::numeric / NULLIF(xact_commit + xact_rollback, 0) * 100, 2) as rollback_ratio,
        temp_files
    FROM pg_stat_database 
    WHERE datname = 'agvc'
)
SELECT 
    datname,
    connections,
    CASE 
        WHEN connections < 50 THEN '✅ 健康'
        WHEN connections < 100 THEN '⚠️ 注意'
        ELSE '❌ 異常'
    END as connection_status,
    cache_hit_ratio || '%' as cache_hit,
    CASE 
        WHEN cache_hit_ratio > 95 THEN '✅ 優秀'
        WHEN cache_hit_ratio > 90 THEN '⚠️ 可接受'
        ELSE '❌ 需優化'
    END as cache_status,
    rollback_ratio || '%' as rollback_rate,
    CASE 
        WHEN rollback_ratio < 10 THEN '✅ 穩定'
        WHEN rollback_ratio < 20 THEN '⚠️ 注意'
        ELSE '❌ 異常'
    END as rollback_status,
    temp_files,
    CASE 
        WHEN temp_files = 0 THEN '✅ 理想'
        ELSE '⚠️ 有溢出'
    END as memory_status
FROM health_metrics;"

# [宿主機] 資料庫活動連接詳細檢查
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT 
    COUNT(*) as total_connections,
    COUNT(CASE WHEN state = 'active' THEN 1 END) as active_queries,
    COUNT(CASE WHEN state = 'idle' THEN 1 END) as idle_connections,
    COUNT(CASE WHEN state = 'idle in transaction' THEN 1 END) as idle_in_transaction
FROM pg_stat_activity 
WHERE datname = 'agvc';"

# [容器內] ROS 2 主題效能
ros2 topic hz /topic_name
ros2 topic bw /topic_name
```

## 🔧 服務診斷

### Web 服務診斷
```bash
# [宿主機] API 服務檢查
curl http://localhost:8000/health    # 或 http://agvc.webapi/health
curl http://localhost:8000/docs      # 或 http://agvc.webapi/docs

# [宿主機] AGVCUI 檢查
curl http://localhost:8001/          # 或 http://agvc.ui/

# [宿主機] OPUI 檢查
curl http://localhost:8002/          # 或 http://op.ui/
```

### 資料庫診斷

#### 基礎連接測試
```bash
# [宿主機] PostgreSQL 連接測試
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "SELECT version();"

# [宿主機] 資料庫狀態檢查
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "SELECT * FROM pg_stat_database;"

# [宿主機] 資料庫大小檢查
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "SELECT pg_size_pretty(pg_database_size('agvc'));"
```

#### PostgreSQL 監控欄位詳解

**⚠️ 重要：理解 PostgreSQL 統計數據是系統健康診斷的關鍵**

| 欄位名稱 | 含義 | 健康標準 | 故障排除指引 |
|---------|------|---------|-------------|
| **active_connections** | 當前活動連接數 | < 50 (正常)<br/>< 100 (可接受) | 超過100需檢查連接池配置或連接洩漏 |
| **xact_commit** | 成功提交交易數 | 穩定增長 | 停止增長可能表示應用程式無法正常運作 |
| **xact_rollback** | 回滾交易數 | < 10% 總交易 | 高回滾率檢查：死鎖、約束衝突、應用邏輯錯誤 |
| **blks_read** | 磁碟讀取區塊數 | 穩定或下降 | 持續增長表示緩存不足，需增加shared_buffers |
| **blks_hit** | 緩存命中區塊數 | 高且穩定增長 | 低增長率表示查詢模式問題或記憶體不足 |
| **temp_files** | 臨時檔案數量 | = 0 (理想) | > 0表示work_mem不足，複雜查詢溢出到磁碟 |
| **temp_bytes** | 臨時檔案大小 | = 0 (理想) | 大值需檢查查詢效率和work_mem配置 |
| **cache_hit_ratio** | 緩存命中率 | > 95% (優秀)<br/>> 90% (可接受) | < 90%需增加shared_buffers或最佳化查詢 |
| **rollback_ratio** | 回滾比例 | < 10% (穩定)<br/>< 20% (可接受) | > 20%嚴重問題，需檢查應用程式邏輯 |

#### 實際資料分析案例

**基於真實系統資料的健康評估範例**：

```bash
# 假設系統返回以下資料:
# datname | active_connections | xact_commit | xact_rollback | blks_read | blks_hit | temp_files | cache_hit_ratio | rollback_ratio
# agvc    |                  4 |        1801 |         16774 |       741 |   167548 |          0 |           99.6% |           9.7%

# 分析結果:
# ✅ active_connections: 4 
#    狀態: 健康 (遠低於50的警戒值)
#    建議: 無需採取行動

# ⚠️ rollback_ratio: 9.7% 
#    狀態: 接近10%警戒線，需要關注
#    建議: 檢查應用程式日誌，查找回滾原因：
#          - 死鎖衝突
#          - 約束違反
#          - 事務邏輯錯誤

# ✅ cache_hit_ratio: 99.6% 
#    狀態: 優秀 (記憶體使用效率極佳)
#    建議: 無需調整，維持當前配置

# ✅ temp_files: 0 
#    狀態: 理想 (沒有記憶體溢出)
#    建議: work_mem配置合適，無需調整
```

#### 問題診斷工作流程

**當發現異常指標時的標準診斷流程**：

```bash
# [宿主機] 1. 高回滾率診斷 (rollback_ratio > 10%)
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT 
    datname, 
    xact_rollback, 
    xact_commit,
    round(xact_rollback::numeric / (xact_commit + xact_rollback) * 100, 2) as rollback_rate
FROM pg_stat_database 
WHERE datname = 'agvc';"

# [宿主機] 檢查當前阻塞的查詢
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT 
    pid, 
    usename, 
    application_name, 
    state, 
    query_start, 
    now() - query_start as duration,
    left(query, 100) as query_preview
FROM pg_stat_activity 
WHERE state = 'active' AND datname = 'agvc'
ORDER BY query_start;"

# [宿主機] 2. 低緩存命中率診斷 (cache_hit_ratio < 90%)
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT 
    shared_buffers, 
    current_setting('effective_cache_size') as effective_cache_size
FROM pg_settings 
WHERE name = 'shared_buffers';"

# [宿主機] 3. 臨時檔案問題診斷 (temp_files > 0)
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT 
    temp_files,
    pg_size_pretty(temp_bytes) as temp_size,
    current_setting('work_mem') as work_mem_setting
FROM pg_stat_database 
WHERE datname = 'agvc';"

# [宿主機] 檢查最耗費臨時空間的查詢
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT 
    query,
    temp_blks_read + temp_blks_written as temp_blocks_used
FROM pg_stat_statements 
ORDER BY temp_blks_read + temp_blks_written DESC 
LIMIT 5;"
```

#### 效能調優建議

**根據監控結果的具體調優措施**：

```bash
# 調優建議1: 高回滾率處理
# 如果 rollback_ratio > 10%:
# 1. 檢查應用程式日誌中的錯誤模式
# 2. 分析死鎖頻率
# 3. 檢查事務隔離等級設定
# 4. 優化事務邊界和持續時間

# 調優建議2: 緩存命中率最佳化  
# 如果 cache_hit_ratio < 90%:
# 1. 增加 shared_buffers (建議為總記憶體的25%)
# 2. 調整 effective_cache_size
# 3. 檢查查詢是否有全表掃描
# 4. 添加適當的索引

# 調優建議3: 臨時檔案問題解決
# 如果 temp_files > 0:
# 1. 增加 work_mem 設定
# 2. 最佳化複雜查詢的 JOIN 順序
# 3. 檢查是否需要分區表
# 4. 考慮使用物化視圖預計算複雜聚合
```

### ROS 2 服務診斷
```bash
# [容器內] ROS 2 環境檢查
echo $ROS_DISTRO
echo $RMW_IMPLEMENTATION

# [容器內] 節點和服務檢查
ros2 node list
ros2 service list
ros2 topic list

# [容器內] 服務可用性測試
ros2 service call /service_name service_type
```

## 🚨 故障排除指導

### 常見問題診斷

#### 容器啟動失敗
```bash
# [宿主機] 診斷步驟
1. r containers-status    # 檢查容器狀態
2. r log-errors          # 檢查錯誤日誌
3. r agvc-stop           # 停止異常服務
4. r agvc-start          # 重新啟動
```

#### 網路連接問題
```bash
# [宿主機] 診斷步驟
1. r network-check       # 檢查端口狀態
2. r zenoh-check        # 檢查 Zenoh 連接
3. r quick-diag         # 綜合診斷
```

#### 系統效能問題
```bash
# [宿主機] 診斷步驟
1. r agvc-check         # 系統健康狀態
2. r quick-diag         # 快速診斷
3. r log-scan           # 檢查系統日誌
4. 分析系統資源使用情況
```

### 診斷決策樹
```
用戶報告問題
├── 容器相關 → r containers-status → 如果異常 → r agvc-start
├── 網路相關 → r network-check → 如果異常 → r zenoh-check
├── 效能相關 → r agvc-check → 如果異常 → r quick-diag
└── 未知問題 → r quick-diag → r log-errors → 深度分析
```

## 📋 診斷報告

### 自動報告生成
```bash
# [宿主機] 生成系統健康報告
r system-health > system-report-$(date +%Y%m%d).txt

# [宿主機] 生成錯誤分析報告
r log-errors > error-analysis-$(date +%Y%m%d).log

# [宿主機] 生成網路診斷報告
r network-check > network-report-$(date +%Y%m%d).txt
```

### 報告內容結構
```
系統診斷報告
├── 容器狀態
├── 服務狀態
├── 網路連接
├── 資源使用
├── 錯誤日誌
└── 建議措施
```

## 💡 最佳實踐

### 預防性維護
1. **定期檢查**: 每日執行基本健康檢查
2. **趨勢監控**: 週期性分析系統效能趨勢
3. **日誌輪轉**: 設置適當的日誌輪轉策略
4. **備份驗證**: 定期驗證備份的完整性

### 故障響應
1. **快速診斷**: 使用統一工具快速定位問題
2. **分層分析**: 從整體到具體的分層診斷
3. **文檔記錄**: 記錄故障現象和解決方案
4. **預防措施**: 分析根因並實施預防措施

### 監控自動化
```bash
# [宿主機] 創建自動監控腳本
#!/bin/bash
# daily-health-check.sh
echo "開始每日健康檢查..."
r agvc-check > /tmp/daily-health-$(date +%Y%m%d).txt
r containers-status >> /tmp/daily-health-$(date +%Y%m%d).txt
r network-check >> /tmp/daily-health-$(date +%Y%m%d).txt
echo "健康檢查完成，報告已產生"
```

## 🔗 交叉引用
- 故障排除流程: docs-ai/operations/guides/troubleshooting.md
- 日誌分析方法: docs-ai/operations/guides/log-analysis.md
- 效能監控詳解: docs-ai/operations/guides/performance-monitoring.md
- 資料庫操作: docs-ai/operations/development/database-operations.md
- 容器管理: docs-ai/operations/deployment/container-management.md
- 網路診斷: docs-ai/knowledge/protocols/zenoh-rmw.md
