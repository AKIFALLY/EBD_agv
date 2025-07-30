#!/bin/bash

# =============================================================================
# 一鍵資料庫初始化腳本
# =============================================================================
# 用途：自動建立 agvc 使用者、agvc 和 test_db 資料庫，並授予權限
# 功能：
#   - 建立 agvc 使用者 (如果不存在)
#   - 建立 agvc 生產資料庫 (如果不存在)
#   - 建立 test_db 測試資料庫 (如果不存在)
#   - 授予 agvc 使用者完整權限
#   - 驗證初始化結果
# 
# 使用方法：
#   ./init_database.sh
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
ADMIN_USER="postgres"
ADMIN_PASSWORD="password"
APP_USER="agvc"
APP_PASSWORD="password"
PRODUCTION_DB="agvc"
TEST_DB="test_db"

echo -e "${BLUE}=== PostgreSQL 資料庫一鍵初始化 ===${NC}"
echo "初始化時間: $(date)"
echo ""

# 檢查先決條件
echo -e "${BLUE}檢查先決條件...${NC}"

# 1. 檢查 PostgreSQL 容器狀態
if ! docker compose -f /home/ct/RosAGV/docker-compose.agvc.yml ps postgres | rg -q "Up"; then
    echo -e "${RED}❌ PostgreSQL 容器未運行${NC}"
    echo "正在啟動 PostgreSQL 容器..."
    docker compose -f /home/ct/RosAGV/docker-compose.agvc.yml up -d postgres
    sleep 10
fi

# 2. 檢查 PostgreSQL 服務可用性
echo "等待 PostgreSQL 服務啟動..."
for i in {1..30}; do
    if pg_isready -h $DB_HOST -p $DB_PORT >/dev/null 2>&1; then
        echo -e "${GREEN}✅ PostgreSQL 服務已就緒${NC}"
        break
    fi
    echo "等待中... ($i/30)"
    sleep 2
done

if ! pg_isready -h $DB_HOST -p $DB_PORT >/dev/null 2>&1; then
    echo -e "${RED}❌ PostgreSQL 服務無法連線${NC}"
    exit 1
fi

# 3. 檢查管理員連線
if ! PGPASSWORD=$ADMIN_PASSWORD psql -h $DB_HOST -p $DB_PORT -U $ADMIN_USER -d postgres -c "SELECT 1;" >/dev/null 2>&1; then
    echo -e "${RED}❌ 無法使用管理員帳號連線${NC}"
    exit 1
fi

# 建立 SQL 初始化腳本
INIT_SQL="/tmp/init_agvc_db.sql"
cat > $INIT_SQL << 'EOF'
-- PostgreSQL 資料庫初始化腳本
-- 建立 agvc 使用者和資料庫

-- 建立 agvc 使用者
DO $$
BEGIN
    IF NOT EXISTS (SELECT FROM pg_user WHERE usename = 'agvc') THEN
        CREATE USER agvc WITH PASSWORD 'password';
        RAISE NOTICE '✅ User agvc created successfully';
    ELSE
        RAISE NOTICE '⚠️ User agvc already exists';
    END IF;
END
$$;

-- 建立 agvc 資料庫
SELECT 'CREATE DATABASE agvc OWNER agvc'
WHERE NOT EXISTS (SELECT FROM pg_database WHERE datname = 'agvc')\gexec

-- 建立 test_db 資料庫
SELECT 'CREATE DATABASE test_db OWNER agvc'
WHERE NOT EXISTS (SELECT FROM pg_database WHERE datname = 'test_db')\gexec

-- 授予權限
GRANT ALL PRIVILEGES ON DATABASE agvc TO agvc;
GRANT ALL PRIVILEGES ON DATABASE test_db TO agvc;

-- 顯示結果
SELECT '✅ 初始化完成' as status;
SELECT usename as created_users FROM pg_user WHERE usename = 'agvc';
SELECT datname as created_databases FROM pg_database WHERE datname IN ('agvc', 'test_db');
EOF

# 執行初始化
echo -e "${BLUE}執行資料庫初始化...${NC}"
if PGPASSWORD=$ADMIN_PASSWORD psql -h $DB_HOST -p $DB_PORT -U $ADMIN_USER -d postgres -f $INIT_SQL; then
    echo -e "${GREEN}✅ 初始化腳本執行成功${NC}"
else
    echo -e "${RED}❌ 初始化腳本執行失敗${NC}"
    rm -f $INIT_SQL
    exit 1
fi

# 清理腳本
rm -f $INIT_SQL

# 驗證初始化結果
echo ""
echo -e "${BLUE}驗證初始化結果...${NC}"

# 測試 agvc 使用者連線到生產資料庫
echo "測試生產資料庫連線..."
if PGPASSWORD=$APP_PASSWORD psql -h $DB_HOST -p $DB_PORT -U $APP_USER -d $PRODUCTION_DB -c "SELECT current_user, current_database();" >/dev/null 2>&1; then
    echo -e "${GREEN}✅ agvc 使用者可正常連線生產資料庫${NC}"
else
    echo -e "${RED}❌ agvc 使用者無法連線生產資料庫${NC}"
fi

# 測試 agvc 使用者連線到測試資料庫
echo "測試測試資料庫連線..."
if PGPASSWORD=$APP_PASSWORD psql -h $DB_HOST -p $DB_PORT -U $APP_USER -d $TEST_DB -c "SELECT current_user, current_database();" >/dev/null 2>&1; then
    echo -e "${GREEN}✅ agvc 使用者可正常連線測試資料庫${NC}"
else
    echo -e "${RED}❌ agvc 使用者無法連線測試資料庫${NC}"
fi

# 檢查權限
echo "檢查資料庫權限..."
PGPASSWORD=$APP_PASSWORD psql -h $DB_HOST -p $DB_PORT -U $APP_USER -d $PRODUCTION_DB -c "
SELECT 
    datname as database,
    has_database_privilege('agvc', datname, 'CREATE') as can_create,
    has_database_privilege('agvc', datname, 'CONNECT') as can_connect
FROM pg_database 
WHERE datname IN ('agvc', 'test_db');" 2>/dev/null

echo ""
echo -e "${BLUE}=== 初始化完成 ===${NC}"
echo -e "${GREEN}✅ 資料庫初始化成功！${NC}"
echo ""
echo "下一步操作："
echo "1. 執行資料表初始化: cd /app/db_proxy_ws/src/db_proxy && python3 -m db_proxy.sql.db_install"
echo "2. 啟動資料庫服務: ros2 run db_proxy agvc_database_node"
echo ""
echo "完成時間: $(date)"
