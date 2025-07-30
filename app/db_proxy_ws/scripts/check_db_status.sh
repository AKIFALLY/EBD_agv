#!/bin/bash

# =============================================================================
# 資料庫狀態檢查腳本
# =============================================================================
# 用途：檢查 PostgreSQL 資料庫和相關服務的運行狀態
# 檢查項目：
#   - Docker 容器狀態 (postgres, pgadmin)
#   - 資料庫連線數和活動
#   - 資料庫大小和效能統計
#   - ROS 2 服務狀態
# 
# 使用方法：
#   ./check_db_status.sh
# 
# 基於：docker-compose.agvc.yml 配置
# =============================================================================

# 設定顏色輸出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 資料庫連線參數 (基於 docker-compose.agvc.yml)
DB_HOST="192.168.100.254"
DB_PORT="5432"
APP_USER="agvc"
APP_PASSWORD="password"
PRODUCTION_DB="agvc"

echo -e "${BLUE}=== PostgreSQL 資料庫狀態檢查 ===${NC}"
echo "檢查時間: $(date)"
echo ""

# 1. 檢查 Docker 容器狀態
echo -e "${BLUE}1. Docker 容器狀態${NC}"
echo "PostgreSQL 容器:"
if docker compose -f /home/ct/RosAGV/docker-compose.agvc.yml ps postgres | rg -q "Up"; then
    echo -e "${GREEN}✅ postgres_container 運行中${NC}"
    docker compose -f /home/ct/RosAGV/docker-compose.agvc.yml ps postgres | tail -n +2
else
    echo -e "${RED}❌ postgres_container 未運行${NC}"
fi

echo ""
echo "pgAdmin4 容器:"
if docker compose -f /home/ct/RosAGV/docker-compose.agvc.yml ps pgadmin | rg -q "Up"; then
    echo -e "${GREEN}✅ pgadmin_container 運行中${NC}"
    docker compose -f /home/ct/RosAGV/docker-compose.agvc.yml ps pgadmin | tail -n +2
else
    echo -e "${RED}❌ pgadmin_container 未運行${NC}"
fi

# 2. 檢查容器資源使用
echo ""
echo -e "${BLUE}2. 容器資源使用${NC}"
if docker stats --no-stream postgres_container pgadmin_container 2>/dev/null; then
    echo -e "${GREEN}✅ 容器資源統計獲取成功${NC}"
else
    echo -e "${YELLOW}⚠️ 無法獲取容器資源統計${NC}"
fi

# 3. 檢查網路連線
echo ""
echo -e "${BLUE}3. 網路連線檢查${NC}"
if pg_isready -h $DB_HOST -p $DB_PORT >/dev/null 2>&1; then
    echo -e "${GREEN}✅ PostgreSQL 服務可用 ($DB_HOST:$DB_PORT)${NC}"
else
    echo -e "${RED}❌ PostgreSQL 服務不可用${NC}"
    exit 1
fi

# 檢查端口
echo "端口使用情況:"
ss -tulpn 2>/dev/null | rg -E "(5432|5050)" || netstat -tulpn 2>/dev/null | rg -E "(5432|5050)"

# 4. 檢查資料庫連線數
echo ""
echo -e "${BLUE}4. 資料庫連線統計${NC}"
PGPASSWORD=$APP_PASSWORD psql -h $DB_HOST -p $DB_PORT -U $APP_USER -d $PRODUCTION_DB -c "
SELECT 
    'Total Connections' as metric,
    count(*) as value
FROM pg_stat_activity 
WHERE datname = '$PRODUCTION_DB'
UNION ALL
SELECT 
    'Active Connections',
    count(*)
FROM pg_stat_activity 
WHERE datname = '$PRODUCTION_DB' AND state = 'active'
UNION ALL
SELECT 
    'Idle Connections',
    count(*)
FROM pg_stat_activity 
WHERE datname = '$PRODUCTION_DB' AND state = 'idle';" 2>/dev/null

# 5. 檢查資料庫大小
echo ""
echo -e "${BLUE}5. 資料庫大小${NC}"
PGPASSWORD=$APP_PASSWORD psql -h $DB_HOST -p $DB_PORT -U $APP_USER -d $PRODUCTION_DB -c "
SELECT 
    datname as database,
    pg_size_pretty(pg_database_size(datname)) AS size
FROM pg_database
WHERE datname IN ('agvc', 'test_db')
ORDER BY pg_database_size(datname) DESC;" 2>/dev/null

# 6. 檢查資料表統計
echo ""
echo -e "${BLUE}6. 資料表統計 (前10個最大的資料表)${NC}"
PGPASSWORD=$APP_PASSWORD psql -h $DB_HOST -p $DB_PORT -U $APP_USER -d $PRODUCTION_DB -c "
SELECT 
    schemaname,
    tablename,
    n_live_tup as live_rows,
    n_dead_tup as dead_rows,
    pg_size_pretty(pg_total_relation_size(schemaname||'.'||tablename)) as size
FROM pg_stat_user_tables
ORDER BY pg_total_relation_size(schemaname||'.'||tablename) DESC
LIMIT 10;" 2>/dev/null

# 7. 檢查資料庫活動
echo ""
echo -e "${BLUE}7. 資料庫活動統計${NC}"
PGPASSWORD=$APP_PASSWORD psql -h $DB_HOST -p $DB_PORT -U $APP_USER -d $PRODUCTION_DB -c "
SELECT 
    schemaname,
    tablename,
    n_tup_ins as inserts,
    n_tup_upd as updates,
    n_tup_del as deletes,
    last_vacuum,
    last_analyze
FROM pg_stat_user_tables
WHERE n_tup_ins + n_tup_upd + n_tup_del > 0
ORDER BY (n_tup_ins + n_tup_upd + n_tup_del) DESC
LIMIT 10;" 2>/dev/null

# 8. 檢查 ROS 2 服務狀態
echo ""
echo -e "${BLUE}8. ROS 2 服務狀態${NC}"
if command -v ros2 >/dev/null 2>&1; then
    echo "AGVC 相關服務:"
    ros2 service list 2>/dev/null | rg agvc || echo "沒有找到 AGVC 相關服務"
    
    echo ""
    echo "AGVC 相關節點:"
    ros2 node list 2>/dev/null | rg agvc || echo "沒有找到 AGVC 相關節點"
    
    echo ""
    echo "AGVC 相關主題:"
    ros2 topic list 2>/dev/null | rg agvc || echo "沒有找到 AGVC 相關主題"
else
    echo -e "${YELLOW}⚠️ ROS 2 不可用，跳過 ROS 2 服務檢查${NC}"
fi

# 9. 檢查系統資源
echo ""
echo -e "${BLUE}9. 系統資源${NC}"
echo "記憶體使用:"
free -h

echo ""
echo "磁碟使用:"
df -h | rg -E "(Filesystem|/app|/var/lib/docker)"

# 10. 檢查 PostgreSQL 配置
echo ""
echo -e "${BLUE}10. PostgreSQL 重要配置${NC}"
PGPASSWORD=$APP_PASSWORD psql -h $DB_HOST -p $DB_PORT -U $APP_USER -d $PRODUCTION_DB -c "
SELECT 
    name,
    setting,
    unit,
    short_desc
FROM pg_settings 
WHERE name IN ('max_connections', 'shared_buffers', 'work_mem', 'maintenance_work_mem', 'checkpoint_timeout')
ORDER BY name;" 2>/dev/null

echo ""
echo -e "${BLUE}=== 狀態檢查完成 ===${NC}"
echo "檢查完成時間: $(date)"

# 提供建議
echo ""
echo -e "${BLUE}建議操作:${NC}"
echo "- 如果容器未運行: docker compose -f /home/ct/RosAGV/docker-compose.agvc.yml up -d postgres pgadmin"
echo "- 如果需要重啟服務: docker compose -f /home/ct/RosAGV/docker-compose.agvc.yml restart postgres"
echo "- 如果需要檢查日誌: docker compose -f /home/ct/RosAGV/docker-compose.agvc.yml logs postgres"
echo "- pgAdmin4 Web 介面: http://localhost:5050/"
