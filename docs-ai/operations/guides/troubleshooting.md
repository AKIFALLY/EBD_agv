# 故障排除操作指導

## 🎯 適用場景
- 系統故障的快速診斷和解決
- 問題根因分析和預防措施
- 緊急情況的應急處理
- 故障排除流程標準化

## 📋 RosAGV 故障排除體系

### 故障分類
```
系統故障分類
├── 🐳 容器相關故障
│   ├── 容器無法啟動
│   ├── 容器異常退出
│   └── 容器資源不足
├── 🌐 網路通訊故障
│   ├── Zenoh 連接失敗
│   ├── 跨容器通訊中斷
│   └── 外部網路問題
├── 🔧 服務運行故障
│   ├── ROS 2 節點異常
│   ├── Web 服務無回應
│   └── 資料庫連接失敗
└── 🚗 AGV 功能故障
    ├── 狀態機異常
    ├── 硬體通訊失敗
    └── 導航定位問題
```

### 故障排除工具
```bash
# [宿主機] 統一診斷工具
r quick-diag           # 快速綜合診斷
r agvc-check          # AGVC 系統健康檢查
r agv-check           # AGV 系統健康檢查
r containers-status   # 容器狀態檢查
r network-check       # 網路連接檢查
r log-errors          # 日誌錯誤分析
```

## 🚨 緊急故障處理流程

### 第一階段：快速評估 (1-2分鐘)
```bash
# [宿主機] 1. 系統整體狀態檢查
r quick-diag

# [宿主機] 2. 容器運行狀態
r containers-status

# [宿主機] 3. 關鍵服務檢查
r agvc-check          # 或 r agv-check
```

### 第二階段：問題定位 (3-5分鐘)
```bash
# 根據第一階段結果選擇對應診斷
# [宿主機] 如果容器異常
r log-errors

# [宿主機] 如果網路異常
r network-check
r zenoh-check

# 如果服務異常
# [宿主機] 進入容器檢查具體服務
docker compose -f docker-compose.agvc.yml exec agvc_server bash
# [容器內] 檢查系統狀態
check_system_status
```

### 第三階段：問題解決 (5-15分鐘)
```bash
# 根據問題類型執行對應解決方案
# 詳見下方具體故障處理流程
```

## 🐳 容器相關故障

### 容器無法啟動
**症狀**: `docker compose up` 失敗或容器立即退出

**診斷步驟**:
```bash
# [宿主機] 1. 檢查容器狀態
r containers-status

# [宿主機] 2. 查看啟動日誌
docker compose -f docker-compose.agvc.yml logs agvc_server

# [宿主機] 3. 檢查端口衝突 (推薦使用 ss)
ss -tulpn | rg "(8000|8001|8002|5432|7447)"

# [宿主機] 備選：netstat
netstat -tulpn | rg "(8000|8001|8002|5432|7447)"

# [宿主機] 4. 檢查磁碟空間
df -h
```

**常見原因和解決方案**:
```bash
# 原因1: 端口被佔用
# [宿主機] 解決: 停止衝突服務或修改端口配置
sudo lsof -i :8000
sudo kill -9 <PID>

# 原因2: 磁碟空間不足
# [宿主機] 解決: 清理磁碟空間
docker system prune -f
docker volume prune -f

# 原因3: 映像損壞
# [宿主機] 解決: 重新建置映像
docker compose -f docker-compose.agvc.yml build --no-cache

# 原因4: 配置檔案錯誤
# [宿主機] 解決: 檢查並修正配置
docker compose -f docker-compose.agvc.yml config
```

### 容器異常退出
**症狀**: 容器運行一段時間後自動停止

**診斷步驟**:
```bash
# [宿主機] 1. 檢查退出碼
docker compose -f docker-compose.agvc.yml ps

# [宿主機] 2. 查看詳細日誌
docker compose -f docker-compose.agvc.yml logs --tail=100 agvc_server

# [宿主機] 3. 檢查系統資源
docker stats
free -h
```

**解決方案**:
```bash
# 記憶體不足導致的 OOM Kill
# [宿主機] 解決: 增加記憶體限制或優化程式
docker compose -f docker-compose.agvc.yml up -d --scale agvc_server=1

# 程式異常退出
# [宿主機] 解決: 檢查程式日誌，修復程式錯誤
docker compose -f docker-compose.agvc.yml exec agvc_server bash
# [容器內] 查看日誌
tail -f /tmp/agv.log
```

## 🌐 網路通訊故障

### Zenoh 連接失敗
**症狀**: 跨容器 ROS 2 通訊無法正常工作

**診斷步驟**:
```bash
# [宿主機] 1. 檢查 Zenoh Router 狀態
r zenoh-check

# [容器內] 2. 檢查 Zenoh 進程
ps aux | rg zenoh
cat /tmp/zenoh_router.pid

# [宿主機] 3. 檢查端口監聽 (推薦使用 ss)
ss -tulpn | rg 7447

# [宿主機] 備選：netstat
netstat -tulpn | rg 7447

# 4. 測試跨容器連接
# [容器內] 在 AGV 容器中
telnet 192.168.100.100 7447

# [容器內] 在 AGVC 容器中
telnet <AGV_IP> 7447
```

**解決方案**:
```bash
# Zenoh Router 未啟動
# [宿主機] 解決: 重啟 Zenoh Router
docker compose -f docker-compose.agvc.yml restart agvc_server

# 網路配置問題
# [宿主機] 解決: 檢查 Docker 網路配置
docker network ls
docker network inspect rosagv_agvc_network

# 防火牆阻擋
# [宿主機] 解決: 開放必要端口
sudo ufw allow 7447
sudo iptables -A INPUT -p tcp --dport 7447 -j ACCEPT
```

### 跨容器通訊中斷
**症狀**: ROS 2 主題和服務無法跨容器發現

**診斷步驟**:
```bash
# [容器內] 1. 檢查 RMW 設定
echo $RMW_IMPLEMENTATION
# 應該是: rmw_zenohd

# [容器內] 2. 測試本地通訊
ros2 topic list
ros2 node list

# 3. 測試跨容器通訊
# [容器內] 在一個容器中發布
ros2 topic pub /test_topic std_msgs/String "data: 'test'"

# [容器內] 在另一個容器中訂閱
ros2 topic echo /test_topic
```

**解決方案**:
```bash
# RMW 設定錯誤
# [容器內] 解決: 重新設定 RMW
export RMW_IMPLEMENTATION=rmw_zenohd
all_source

# ROS 2 daemon 問題
# [容器內] 解決: 重啟 daemon
ros2 daemon stop
ros2 daemon start

# Zenoh 配置問題
# [容器內] 解決: 檢查配置檔案
cat /app/routerconfig.json5
```

## 🔧 服務運行故障

### Web 服務無回應
**症狀**: Web API 或界面無法存取

**診斷步驟**:
```bash
# [宿主機] 1. 檢查服務端口
curl http://localhost:8000/health    # 或 http://agvc.webapi/health
curl http://localhost:8001/          # 或 http://agvc.ui/
curl http://localhost:8002/          # 或 http://op.ui/

# [宿主機] 2. 檢查 Nginx 狀態
docker compose -f docker-compose.agvc.yml exec nginx nginx -t

# [宿主機] 3. 檢查應用程式日誌
docker compose -f docker-compose.agvc.yml logs web_api
```

**解決方案**:
```bash
# FastAPI 服務異常
# [宿主機] 解決: 重啟服務
docker compose -f docker-compose.agvc.yml restart agvc_server

# Nginx 配置錯誤
# [宿主機] 解決: 檢查並重載配置
docker compose -f docker-compose.agvc.yml exec nginx nginx -s reload

# 端口衝突
# [宿主機] 解決: 檢查端口佔用
sudo lsof -i :8000
```

### 資料庫連接失敗
**症狀**: 應用程式無法連接到 PostgreSQL

**診斷步驟**:
```bash
# [宿主機] 1. 檢查資料庫容器
docker compose -f docker-compose.agvc.yml ps postgres

# [宿主機] 2. 測試資料庫連接
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "SELECT version();"

# [宿主機] 3. 檢查連接配置和資料庫健康狀態
docker compose -f docker-compose.agvc.yml exec agvc_server env | rg POSTGRES

# [宿主機] 4. 執行完整資料庫健康檢查
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
    END as rollback_status
FROM health_metrics;"
```

**解決方案**:
```bash
# PostgreSQL 服務未啟動
# [宿主機] 解決: 啟動資料庫服務
docker compose -f docker-compose.agvc.yml up -d postgres

# 連接參數錯誤
# [宿主機] 解決: 檢查環境變數和配置
docker compose -f docker-compose.agvc.yml exec agvc_server env | rg POSTGRES

# 資料庫效能問題 (根據健康檢查結果)
# 問題1: 連接數過高 (> 100)
# [宿主機] 解決: 檢查連接池配置，查找連接洩漏
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT pid, usename, application_name, state, query_start 
FROM pg_stat_activity 
WHERE datname = 'agvc' AND state != 'idle';"

# 問題2: 高回滾率 (> 10%)
# [宿主機] 解決: 檢查應用程式日誌，查找死鎖和約束違反
docker compose -f docker-compose.agvc.yml logs agvc_server | rg -i "rollback|deadlock|constraint"

# 問題3: 低緩存命中率 (< 90%)
# 解決: 檢查記憶體配置和查詢效率
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT name, setting, unit, short_desc 
FROM pg_settings 
WHERE name IN ('shared_buffers', 'effective_cache_size', 'work_mem');"

# 資料庫損壞或數據不一致
# [宿主機] 解決: 檢查資料庫完整性並備份
docker compose -f docker-compose.agvc.yml exec postgres pg_dump -U agvc -d agvc > backup-$(date +%Y%m%d).sql

# [宿主機] 檢查資料庫完整性
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT schemaname, tablename, n_dead_tup, n_live_tup,
       round(n_dead_tup::numeric / NULLIF(n_live_tup + n_dead_tup, 0) * 100, 2) as dead_ratio
FROM pg_stat_user_tables 
WHERE n_dead_tup > 0
ORDER BY dead_ratio DESC;"
```

### 資料庫效能問題
**症狀**: 資料庫回應緩慢、查詢超時、高資源使用

**診斷步驟**:
```bash
# [宿主機] 1. 檢查資料庫統計指標並分析健康狀態
docker compose -f docker-compose.agvc.yml exec postgres psql -U agv -d agvc -c "
SELECT 
    datname,                                          -- 資料庫名稱
    numbackends as active_connections,                -- 當前活動連接數
    xact_commit,                                      -- 成功提交的交易總數
    xact_rollback,                                    -- 回滾的交易總數  
    blks_read,                                        -- 從磁碟讀取的區塊數
    blks_hit,                                         -- 從緩存命中的區塊數
    temp_files,                                       -- 建立的臨時檔案數量
    -- 計算健康指標
    round(blks_hit::numeric / NULLIF(blks_hit + blks_read, 0) * 100, 2) as cache_hit_ratio,
    round(xact_rollback::numeric / NULLIF(xact_commit + xact_rollback, 0) * 100, 2) as rollback_ratio
FROM pg_stat_database 
WHERE datname = 'agvc';"

# [宿主機] 2. 檢查當前運行的查詢
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
WHERE state = 'active' AND datname = 'agvc' AND query != '<IDLE>'
ORDER BY query_start;"

# [宿主機] 3. 檢查阻塞和鎖定情況
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT 
    blocked_locks.pid AS blocked_pid,
    blocked_activity.usename AS blocked_user,
    blocking_locks.pid AS blocking_pid,
    blocking_activity.usename AS blocking_user,
    blocked_activity.query AS blocked_statement,
    blocking_activity.query AS current_statement_in_blocking_process
FROM pg_catalog.pg_locks blocked_locks
JOIN pg_catalog.pg_stat_activity blocked_activity ON blocked_activity.pid = blocked_locks.pid
JOIN pg_catalog.pg_locks blocking_locks ON blocking_locks.locktype = blocked_locks.locktype
JOIN pg_catalog.pg_stat_activity blocking_activity ON blocking_activity.pid = blocking_locks.pid
WHERE NOT blocked_locks.granted AND blocking_locks.granted;"
```

**解決方案** (基於診斷結果):
```bash
# 根據 PostgreSQL 監控指標的不同問題採取相應措施:

# 問題1: active_connections > 50 (連接數過高)
# 原因: 連接池配置不當或連接洩漏
# [宿主機] 解決:
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "
# 檢查應用程式連接池配置
python3 -c 'from db_proxy.connection_pool import ConnectionPoolManager; print(ConnectionPoolManager.get_pool_status())'
"

# 問題2: rollback_ratio > 10% (回滾率過高)  
# 原因: 死鎖、約束衝突、事務邏輯錯誤
# 解決:
# 查看具體的回滾原因
docker compose -f docker-compose.agvc.yml logs agvc_server | rg -i "IntegrityError|DeadlockDetected|rollback" | tail -20

# [宿主機] 分析死鎖模式
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT query, calls, mean_exec_time, rows, 100.0 * shared_blks_hit / nullif(shared_blks_hit + shared_blks_read, 0) AS hit_percent
FROM pg_stat_statements 
ORDER BY mean_exec_time DESC 
LIMIT 10;"

# 問題3: cache_hit_ratio < 90% (緩存命中率低)
# 原因: shared_buffers太小或查詢模式問題
# 解決:
# [宿主機] 檢查當前記憶體配置
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT 
    name,
    setting,
    unit,
    boot_val,
    short_desc
FROM pg_settings 
WHERE name IN ('shared_buffers', 'effective_cache_size', 'work_mem', 'maintenance_work_mem');"

# 問題4: temp_files > 0 (有臨時檔案)
# 原因: work_mem不足，複雜查詢溢出到磁碟
# 解決:
# [宿主機] 找出產生臨時檔案的查詢
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
SELECT 
    temp_files,
    pg_size_pretty(temp_bytes) as temp_size,
    current_setting('work_mem') as current_work_mem
FROM pg_stat_database 
WHERE datname = 'agvc';"

# 長時間運行查詢終止 (謹慎使用)
# docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "SELECT pg_terminate_backend(PID);"
```

## 🚗 AGV 功能故障

### 狀態機異常
**症狀**: AGV 狀態機卡在某個狀態或狀態轉換異常

**診斷步驟**:
```bash
# [容器內] 1. 檢查 AGV 狀態
ros2 topic echo /agv_status

# [容器內] 2. 檢查狀態機日誌
ros2 log view

# [容器內] 3. 檢查事件處理
ros2 topic echo /agv_events
```

**解決方案**:
```bash
# 狀態機死鎖
# [容器內] 解決: 發送重置事件
ros2 topic pub /agv_events agv_interfaces/Event "event_type: 'reset'"

# 狀態轉換邏輯錯誤
# 解決: 檢查狀態機程式碼
# 查看 agv_base/agv_states/ 目錄下的狀態定義

# 事件處理異常
# [容器內] 解決: 重啟 AGV 節點
ros2 lifecycle set /agv_node shutdown
ros2 run agv_base agv_node
```

### 硬體通訊失敗
**症狀**: PLC 或感測器通訊異常

**診斷步驟**:
```bash
# [容器內] 1. 檢查 PLC 連接
ros2 service call /plc_read plc_interfaces/PLCRead "address: 'D100'"

# [容器內] 2. 檢查網路連接
ping <PLC_IP>
telnet <PLC_IP> <PLC_PORT>

# [容器內] 3. 檢查硬體狀態
ros2 topic echo /hardware_status
```

**解決方案**:
```bash
# PLC 連接失敗
# 解決: 檢查網路配置和 PLC 設定
# [容器內] 重啟 PLC 通訊節點
ros2 lifecycle set /plc_node shutdown
ros2 run keyence_plc plc_node

# 感測器異常
# [容器內] 解決: 檢查感測器連接和配置
ros2 param get /sensor_node sensor_config
```

## 📊 故障排除決策樹

### 系統無回應
```
系統無回應
├── 檢查容器狀態 → r containers-status
├── 如果容器未運行 → r agvc-start
├── 如果容器運行異常 → r log-errors
└── 如果容器正常 → r network-check
```

### 網路連接問題
```
網路連接問題
├── 檢查基礎網路 → r network-check
├── 檢查 Zenoh 連接 → r zenoh-check
├── 如果 Zenoh 異常 → 重啟 Zenoh Router
└── 如果網路正常 → 檢查應用程式配置
```

### 效能問題
```
效能問題
├── [宿主機] 檢查系統資源 → docker stats, free -h
├── [容器內] 檢查應用程式效能 → ros2 topic hz
├── 如果資源不足 → 優化配置或擴容
└── 如果應用異常 → 檢查程式邏輯
```

## 📋 故障預防措施

### 監控和預警
```bash
# [宿主機] 設置定期健康檢查
#!/bin/bash
# health-monitor.sh
while true; do
    r agvc-check > /tmp/health-$(date +%Y%m%d-%H%M).log
    if [ $? -ne 0 ]; then
        echo "Health check failed at $(date)" >> /tmp/health-alerts.log
        # 可配置為發送通知到系統管理員或 Slack/Teams 頻道
    fi
    sleep 300  # 每5分鐘檢查一次
done
```

### 備份和恢復
```bash
# [宿主機] 定期備份重要資料
docker compose -f docker-compose.agvc.yml exec postgres pg_dump -U agvc -d agvc > backup-$(date +%Y%m%d).sql

# [宿主機] 配置檔案備份
tar -czf config-backup-$(date +%Y%m%d).tar.gz app/config/
```

### 日誌輪轉
```bash
# 設置日誌輪轉，防止磁碟空間耗盡
# 在 /etc/logrotate.d/rosagv 中配置
/tmp/*.log {
    daily
    rotate 7
    compress
    missingok
    notifempty
}
```

## 🔗 交叉引用
- 系統診斷: docs-ai/operations/guides/system-diagnostics.md
- 效能監控: docs-ai/operations/guides/performance-monitoring.md
- 資料庫操作: docs-ai/operations/development/database-operations.md
- 日誌分析: docs-ai/operations/guides/log-analysis.md
- 容器管理: docs-ai/operations/deployment/container-management.md
- 雙環境架構: docs-ai/context/system/dual-environment.md
- Zenoh 通訊: docs-ai/knowledge/protocols/zenoh-rmw.md
