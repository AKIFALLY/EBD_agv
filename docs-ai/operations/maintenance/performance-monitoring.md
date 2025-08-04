# 效能監控操作指導

## 🎯 適用場景
- 系統效能監控和分析
- 資料庫效能調優和問題診斷
- 預防性效能維護和容量規劃
- 效能瓶頸識別和解決

## 📋 RosAGV 效能監控體系

### 監控層次架構
```
RosAGV 效能監控
├── 📊 系統層監控
│   ├── CPU、記憶體、磁碟、網路
│   ├── 容器資源使用統計
│   └── 作業系統效能指標
├── 🗄️ 資料庫層監控
│   ├── PostgreSQL 統計指標
│   ├── 查詢效能分析
│   └── 連接池和事務監控
├── 🌐 應用層監控
│   ├── Web API 回應時間
│   ├── ROS 2 主題和服務效能
│   └── 業務邏輯效能指標
└── 🔗 網路層監控
    ├── Zenoh 通訊效能
    ├── 跨容器通訊延遲
    └── 外部系統整合效能
```

## 🗄️ PostgreSQL 效能監控

### 核心效能指標

**⚠️ 重要：理解每個 PostgreSQL 效能指標對於系統調優至關重要**

#### 1. 資料庫整體效能監控
```sql
-- 綜合效能監控查詢 (包含所有關鍵指標)
SELECT 
    datname,                                          -- 資料庫名稱
    numbackends as active_connections,                -- 當前活動連接數
    xact_commit,                                      -- 成功提交的交易總數
    xact_rollback,                                    -- 回滾的交易總數  
    blks_read,                                        -- 從磁碟讀取的區塊數
    blks_hit,                                         -- 從緩存命中的區塊數
    temp_files,                                       -- 建立的臨時檔案數量
    temp_bytes,                                       -- 臨時檔案使用的總位元組數
    tup_returned,                                     -- 查詢返回的記錄數
    tup_fetched,                                      -- 查詢獲取的記錄數
    tup_inserted,                                     -- 插入的記錄數
    tup_updated,                                      -- 更新的記錄數
    tup_deleted,                                      -- 刪除的記錄數
    -- 計算效能指標
    round(blks_hit::numeric / NULLIF(blks_hit + blks_read, 0) * 100, 2) as cache_hit_ratio,
    round(xact_rollback::numeric / NULLIF(xact_commit + xact_rollback, 0) * 100, 2) as rollback_ratio,
    round(tup_fetched::numeric / NULLIF(tup_returned, 0) * 100, 2) as selectivity_ratio
FROM pg_stat_database 
WHERE datname = 'agvc';
```

#### 2. 詳細效能指標說明

| 指標名稱 | 含義 | 健康標準 | 效能影響 | 調優建議 |
|---------|------|---------|----------|----------|
| **active_connections** | 活動連接數 | < 50 (正常)<br/>< 100 (可接受) | 過高影響記憶體和CPU | 調整連接池大小 |
| **cache_hit_ratio** | 緩存命中率 | > 95% (優秀)<br/>> 90% (可接受) | 低命中率增加磁碟I/O | 增加shared_buffers |
| **rollback_ratio** | 回滾比例 | < 10% (穩定)<br/>< 20% (可接受) | 高回滾浪費CPU資源 | 檢查應用邏輯 |
| **temp_files** | 臨時檔案數 | = 0 (理想)<br/>< 10 (可接受) | 產生臨時檔案影響I/O | 增加work_mem |
| **selectivity_ratio** | 查詢選擇性 | > 1% (高效)<br/>> 0.1% (可接受) | 低選擇性表示全表掃描 | 添加索引 |

#### 3. 實際效能分析範例

**基於真實系統資料的效能評估**：

```bash
# 系統實際資料範例:
# datname | active_connections | cache_hit_ratio | rollback_ratio | temp_files | selectivity_ratio
# agvc    |                  4 |           99.6% |           9.7% |          0 |            15.2%

# 效能分析結果:
# ✅ active_connections: 4
#    分析: 優秀 - 連接數低，資源使用效率高
#    影響: 無負面影響
#    建議: 維持當前連接池配置

# ✅ cache_hit_ratio: 99.6%
#    分析: 優秀 - 記憶體緩存效率極佳
#    影響: 磁碟I/O最小化，查詢回應快速
#    建議: 無需調整，當前shared_buffers配置理想

# ⚠️ rollback_ratio: 9.7%
#    分析: 接近警戒線，需要關注
#    影響: 約10%的CPU時間浪費在無效事務上
#    建議: 分析應用程式日誌，查找回滾原因

# ✅ temp_files: 0
#    分析: 理想 - 沒有記憶體溢出
#    影響: 所有查詢都在記憶體內完成
#    建議: work_mem配置合適

# ✅ selectivity_ratio: 15.2%
#    分析: 優秀 - 查詢選擇性高，索引效果好
#    影響: 查詢效率高，避免全表掃描
#    建議: 維持當前索引策略
```

### 高級效能監控

#### 1. 查詢效能統計
```sql
-- 最耗時的查詢分析 (需要 pg_stat_statements 擴展)
SELECT 
    query,
    calls,                                            -- 執行次數
    total_exec_time,                                  -- 總執行時間 (ms)
    mean_exec_time,                                   -- 平均執行時間 (ms)
    max_exec_time,                                    -- 最大執行時間 (ms)
    rows,                                            -- 平均返回記錄數
    100.0 * shared_blks_hit / 
        nullif(shared_blks_hit + shared_blks_read, 0) AS hit_percent,  -- 緩存命中率
    temp_blks_read + temp_blks_written as temp_blocks_used             -- 臨時區塊使用
FROM pg_stat_statements 
WHERE query NOT LIKE '%pg_stat_statements%'
ORDER BY mean_exec_time DESC 
LIMIT 10;
```

#### 2. 索引使用效率監控
```sql
-- 索引使用效率分析
SELECT 
    schemaname,
    tablename,
    indexname,
    idx_tup_read,                                     -- 索引掃描次數
    idx_tup_fetch,                                    -- 透過索引獲取的記錄數
    pg_size_pretty(pg_relation_size(indexrelid)) as index_size,
    -- 計算索引效率指標
    round(idx_tup_read::numeric / NULLIF(idx_tup_read + idx_tup_fetch, 0) * 100, 2) as index_usage_ratio,
    CASE 
        WHEN idx_tup_read = 0 AND idx_tup_fetch = 0 THEN '❌ 未使用'
        WHEN idx_tup_read::numeric / NULLIF(idx_tup_read + idx_tup_fetch, 0) > 0.95 THEN '✅ 高效'
        WHEN idx_tup_read::numeric / NULLIF(idx_tup_read + idx_tup_fetch, 0) > 0.80 THEN '⚠️ 可接受'
        ELSE '❌ 低效'
    END as efficiency_status
FROM pg_stat_user_indexes 
ORDER BY idx_tup_read DESC;
```

#### 3. 資料表效能統計
```sql
-- 資料表活動度和效能分析
SELECT 
    schemaname,
    tablename,
    n_tup_ins as inserts,                             -- 插入記錄數
    n_tup_upd as updates,                             -- 更新記錄數
    n_tup_del as deletes,                             -- 刪除記錄數
    n_live_tup as live_tuples,                        -- 活動記錄數
    n_dead_tup as dead_tuples,                        -- 無效記錄數
    pg_size_pretty(pg_total_relation_size(schemaname||'.'||tablename)) as table_size,
    -- 計算效能指標
    n_tup_ins + n_tup_upd + n_tup_del as total_activity,
    round(n_dead_tup::numeric / NULLIF(n_live_tup + n_dead_tup, 0) * 100, 2) as dead_tuple_ratio,
    CASE 
        WHEN n_dead_tup::numeric / NULLIF(n_live_tup + n_dead_tup, 0) < 0.05 THEN '✅ 健康'
        WHEN n_dead_tup::numeric / NULLIF(n_live_tup + n_dead_tup, 0) < 0.20 THEN '⚠️ 需要VACUUM'
        ELSE '❌ 急需清理'
    END as maintenance_status
FROM pg_stat_user_tables 
ORDER BY total_activity DESC;
```

#### 4. 連接池效能監控
```sql
-- 連接活動分析
SELECT 
    state,
    COUNT(*) as connection_count,
    COUNT(*) * 100.0 / SUM(COUNT(*)) OVER() as percentage,
    AVG(EXTRACT(EPOCH FROM (now() - state_change))) as avg_duration_seconds
FROM pg_stat_activity 
WHERE datname = 'agvc'
GROUP BY state
ORDER BY connection_count DESC;

-- 長時間運行的查詢識別
SELECT 
    pid, 
    usename, 
    application_name, 
    state, 
    query_start, 
    now() - query_start as duration,
    CASE 
        WHEN EXTRACT(EPOCH FROM (now() - query_start)) < 60 THEN '✅ 正常'
        WHEN EXTRACT(EPOCH FROM (now() - query_start)) < 300 THEN '⚠️ 長時間'
        ELSE '❌ 異常長'
    END as duration_status,
    left(query, 100) as query_preview
FROM pg_stat_activity 
WHERE state = 'active' 
    AND datname = 'agvc' 
    AND query != '<IDLE>'
    AND now() - query_start > interval '10 seconds'
ORDER BY query_start;
```

### 效能調優建議

#### 1. 記憶體配置最佳化
```sql
-- 檢查當前記憶體配置
SELECT 
    name,
    setting,
    unit,
    context,
    short_desc,
    CASE 
        WHEN name = 'shared_buffers' AND setting::int < 32768 THEN '⚠️ 可能過小'
        WHEN name = 'work_mem' AND setting::int < 4096 THEN '⚠️ 可能過小'
        WHEN name = 'maintenance_work_mem' AND setting::int < 65536 THEN '⚠️ 可能過小'
        ELSE '✅ 合理'
    END as assessment
FROM pg_settings 
WHERE name IN (
    'shared_buffers', 
    'effective_cache_size', 
    'work_mem', 
    'maintenance_work_mem',
    'max_connections'
)
ORDER BY name;
```

#### 2. 效能調優決策矩陣

| 問題症狀 | 可能原因 | 診斷查詢 | 調優措施 |
|---------|---------|---------|---------|
| 緩存命中率 < 90% | shared_buffers太小 | 檢查blks_read/blks_hit比例 | 增加shared_buffers到記憶體25% |
| 高回滾率 > 10% | 死鎖或約束違反 | 查看pg_stat_activity阻塞 | 最佳化事務邏輯，減少鎖競爭 |
| 臨時檔案 > 0 | work_mem不足 | 檢查temp_files統計 | 增加work_mem，最佳化複雜查詢 |
| 連接數 > 100 | 連接池配置問題 | 分析pg_stat_activity狀態 | 調整連接池大小，檢查洩漏 |
| 查詢緩慢 | 缺少索引 | 分析pg_stat_statements | 添加適當索引，最佳化查詢 |

#### 3. 自動化效能報告
```sql
-- 一鍵效能健康檢查
WITH performance_metrics AS (
    SELECT 
        datname,
        numbackends as connections,
        round(blks_hit::numeric / NULLIF(blks_hit + blks_read, 0) * 100, 2) as cache_hit_ratio,
        round(xact_rollback::numeric / NULLIF(xact_commit + xact_rollback, 0) * 100, 2) as rollback_ratio,
        temp_files,
        round(tup_fetched::numeric / NULLIF(tup_returned, 0) * 100, 2) as selectivity_ratio
    FROM pg_stat_database 
    WHERE datname = 'agvc'
),
table_health AS (
    SELECT 
        COUNT(*) as total_tables,
        SUM(CASE WHEN n_dead_tup::numeric / NULLIF(n_live_tup + n_dead_tup, 0) > 0.20 THEN 1 ELSE 0 END) as tables_need_vacuum
    FROM pg_stat_user_tables
)
SELECT 
    pm.datname as database_name,
    pm.connections,
    CASE 
        WHEN pm.connections < 50 THEN '✅ 優秀'
        WHEN pm.connections < 100 THEN '⚠️ 可接受'
        ELSE '❌ 需要關注'
    END as connection_health,
    pm.cache_hit_ratio || '%' as cache_performance,
    CASE 
        WHEN pm.cache_hit_ratio > 95 THEN '✅ 優秀'
        WHEN pm.cache_hit_ratio > 90 THEN '⚠️ 可接受'
        ELSE '❌ 需要調優'
    END as cache_health,
    pm.rollback_ratio || '%' as rollback_rate,
    CASE 
        WHEN pm.rollback_ratio < 10 THEN '✅ 穩定'
        WHEN pm.rollback_ratio < 20 THEN '⚠️ 需要關注'
        ELSE '❌ 嚴重問題'
    END as transaction_health,
    pm.temp_files,
    CASE 
        WHEN pm.temp_files = 0 THEN '✅ 理想'
        WHEN pm.temp_files < 10 THEN '⚠️ 可接受'
        ELSE '❌ 記憶體不足'
    END as memory_health,
    th.tables_need_vacuum || '/' || th.total_tables as vacuum_status,
    CASE 
        WHEN th.tables_need_vacuum = 0 THEN '✅ 良好'
        WHEN th.tables_need_vacuum < 3 THEN '⚠️ 需要維護'
        ELSE '❌ 急需清理'
    END as maintenance_health
FROM performance_metrics pm
CROSS JOIN table_health th;
```

## 🔧 監控自動化

### 效能監控腳本
```bash
#!/bin/bash
# performance-monitor.sh - PostgreSQL 效能監控腳本

echo "🗄️ PostgreSQL 效能監控報告 - $(date)"
echo "========================================"

# 執行健康檢查
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
WITH performance_metrics AS (
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
    'Connections: ' || connections || 
    CASE 
        WHEN connections < 50 THEN ' ✅'
        WHEN connections < 100 THEN ' ⚠️'
        ELSE ' ❌'
    END as connection_status,
    'Cache Hit: ' || cache_hit_ratio || '%' || 
    CASE 
        WHEN cache_hit_ratio > 95 THEN ' ✅'
        WHEN cache_hit_ratio > 90 THEN ' ⚠️'
        ELSE ' ❌'
    END as cache_status,
    'Rollback Rate: ' || rollback_ratio || '%' || 
    CASE 
        WHEN rollback_ratio < 10 THEN ' ✅'
        WHEN rollback_ratio < 20 THEN ' ⚠️'
        ELSE ' ❌'
    END as rollback_status,
    'Temp Files: ' || temp_files || 
    CASE 
        WHEN temp_files = 0 THEN ' ✅'
        ELSE ' ⚠️'
    END as temp_status
FROM performance_metrics;
"

echo ""
echo "📊 長時間運行查詢檢查:"
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT 
    'PID: ' || pid || 
    ', Duration: ' || EXTRACT(EPOCH FROM (now() - query_start))::int || 's' ||
    ', Query: ' || left(query, 50) || '...'
FROM pg_stat_activity 
WHERE state = 'active' 
    AND datname = 'agvc' 
    AND query != '<IDLE>'
    AND now() - query_start > interval '30 seconds'
ORDER BY query_start;
"
```

### 效能告警設定
```bash
#!/bin/bash
# performance-alert.sh - 效能異常告警

# 設定告警閾值
MAX_CONNECTIONS=80
MIN_CACHE_HIT_RATIO=90
MAX_ROLLBACK_RATIO=15
MAX_TEMP_FILES=5

# 獲取當前效能指標
METRICS=$(docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -t -c "
SELECT 
    numbackends,
    round(blks_hit::numeric / NULLIF(blks_hit + blks_read, 0) * 100, 2),
    round(xact_rollback::numeric / NULLIF(xact_commit + xact_rollback, 0) * 100, 2),
    temp_files
FROM pg_stat_database 
WHERE datname = 'agvc';
")

# 解析指標並檢查告警條件
IFS='|' read -r connections cache_hit rollback_ratio temp_files <<< "$METRICS"

# 檢查並發送告警
if (( $(echo "$connections > $MAX_CONNECTIONS" | bc -l) )); then
    echo "⚠️ 告警: 連接數過高 ($connections > $MAX_CONNECTIONS)"
fi

if (( $(echo "$cache_hit < $MIN_CACHE_HIT_RATIO" | bc -l) )); then
    echo "⚠️ 告警: 緩存命中率過低 ($cache_hit% < $MIN_CACHE_HIT_RATIO%)"
fi

if (( $(echo "$rollback_ratio > $MAX_ROLLBACK_RATIO" | bc -l) )); then
    echo "⚠️ 告警: 回滾率過高 ($rollback_ratio% > $MAX_ROLLBACK_RATIO%)"
fi

if (( temp_files > MAX_TEMP_FILES )); then
    echo "⚠️ 告警: 臨時檔案過多 ($temp_files > $MAX_TEMP_FILES)"
fi
```

## 📋 最佳實踐

### 效能監控原則
1. **定期監控**: 每日檢查效能指標，每週深度分析
2. **基準建立**: 建立系統正常運作時的效能基準
3. **趨勢分析**: 關注效能指標的變化趨勢，而非絕對值
4. **預防性調優**: 在問題影響用戶前主動調優

### 調優優先級
1. **高優先級**: 連接洩漏、記憶體溢出、嚴重死鎖
2. **中優先級**: 緩存命中率低、回滾率高、索引缺失
3. **低優先級**: 小幅效能最佳化、統計資訊更新

### 效能測試建議
```bash
# 壓力測試前的基準測量
# 1. 記錄當前效能指標
# 2. 執行標準化測試負載
# 3. 監控效能變化
# 4. 分析結果並調優
# 5. 重複測試驗證改善
```

## 🔗 交叉引用
- 資料庫操作: @docs-ai/operations/development/database-operations.md
- 系統診斷: @docs-ai/operations/maintenance/system-diagnostics.md
- 故障排除: @docs-ai/operations/maintenance/troubleshooting.md
- 容器管理: @docs-ai/operations/deployment/container-management.md
- 技術棧: @docs-ai/context/system/technology-stack.md