#!/bin/bash

# =============================================================================
# 資料庫初始化檢查腳本
# =============================================================================
# 用途：檢查 PostgreSQL 資料庫是否已完成初始化
# 檢查項目：
#   - PostgreSQL 容器狀態
#   - agvc 使用者是否存在
#   - agvc 和 test_db 資料庫是否存在
#   - 連線測試
# 
# 使用方法：
#   ./check_db_init.sh
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

echo -e "${BLUE}=== PostgreSQL 資料庫初始化檢查 ===${NC}"
echo "檢查時間: $(date)"
echo ""

# 1. 檢查 PostgreSQL 容器狀態
echo -e "${BLUE}1. 檢查 PostgreSQL 容器狀態${NC}"
if docker compose -f /home/ct/RosAGV/docker-compose.agvc.yml ps postgres | rg -q "Up"; then
    echo -e "${GREEN}✅ PostgreSQL 容器運行中${NC}"
else
    echo -e "${RED}❌ PostgreSQL 容器未運行${NC}"
    echo "請執行: docker compose -f /home/ct/RosAGV/docker-compose.agvc.yml up -d postgres"
    exit 1
fi

# 2. 檢查 PostgreSQL 服務可用性
echo -e "${BLUE}2. 檢查 PostgreSQL 服務可用性${NC}"
if pg_isready -h $DB_HOST -p $DB_PORT >/dev/null 2>&1; then
    echo -e "${GREEN}✅ PostgreSQL 服務可用${NC}"
else
    echo -e "${RED}❌ PostgreSQL 服務不可用${NC}"
    echo "請檢查容器狀態和網路連線"
    exit 1
fi

# 3. 檢查管理員連線
echo -e "${BLUE}3. 檢查管理員連線${NC}"
if PGPASSWORD=$ADMIN_PASSWORD psql -h $DB_HOST -p $DB_PORT -U $ADMIN_USER -d postgres -c "SELECT 1;" >/dev/null 2>&1; then
    echo -e "${GREEN}✅ 管理員連線成功${NC}"
else
    echo -e "${RED}❌ 管理員連線失敗${NC}"
    echo "請檢查管理員帳號密碼: $ADMIN_USER/$ADMIN_PASSWORD"
    exit 1
fi

# 4. 檢查 agvc 使用者是否存在
echo -e "${BLUE}4. 檢查 agvc 使用者${NC}"
USER_EXISTS=$(PGPASSWORD=$ADMIN_PASSWORD psql -h $DB_HOST -p $DB_PORT -U $ADMIN_USER -d postgres -t -c "SELECT COUNT(*) FROM pg_user WHERE usename = '$APP_USER';" 2>/dev/null | tr -d ' ')

if [ "$USER_EXISTS" = "1" ]; then
    echo -e "${GREEN}✅ agvc 使用者已存在${NC}"
else
    echo -e "${YELLOW}⚠️ agvc 使用者不存在${NC}"
    echo "需要執行初始化腳本: ./init_database.sh"
fi

# 5. 檢查資料庫是否存在
echo -e "${BLUE}5. 檢查資料庫${NC}"
PROD_DB_EXISTS=$(PGPASSWORD=$ADMIN_PASSWORD psql -h $DB_HOST -p $DB_PORT -U $ADMIN_USER -d postgres -t -c "SELECT COUNT(*) FROM pg_database WHERE datname = '$PRODUCTION_DB';" 2>/dev/null | tr -d ' ')
TEST_DB_EXISTS=$(PGPASSWORD=$ADMIN_PASSWORD psql -h $DB_HOST -p $DB_PORT -U $ADMIN_USER -d postgres -t -c "SELECT COUNT(*) FROM pg_database WHERE datname = '$TEST_DB';" 2>/dev/null | tr -d ' ')

if [ "$PROD_DB_EXISTS" = "1" ]; then
    echo -e "${GREEN}✅ 生產資料庫 '$PRODUCTION_DB' 已存在${NC}"
else
    echo -e "${YELLOW}⚠️ 生產資料庫 '$PRODUCTION_DB' 不存在${NC}"
fi

if [ "$TEST_DB_EXISTS" = "1" ]; then
    echo -e "${GREEN}✅ 測試資料庫 '$TEST_DB' 已存在${NC}"
else
    echo -e "${YELLOW}⚠️ 測試資料庫 '$TEST_DB' 不存在${NC}"
fi

# 6. 檢查 agvc 使用者連線 (如果使用者和資料庫都存在)
if [ "$USER_EXISTS" = "1" ] && [ "$PROD_DB_EXISTS" = "1" ]; then
    echo -e "${BLUE}6. 檢查 agvc 使用者連線${NC}"
    if PGPASSWORD=$APP_PASSWORD psql -h $DB_HOST -p $DB_PORT -U $APP_USER -d $PRODUCTION_DB -c "SELECT current_user, current_database();" >/dev/null 2>&1; then
        echo -e "${GREEN}✅ agvc 使用者可正常連線生產資料庫${NC}"
    else
        echo -e "${RED}❌ agvc 使用者無法連線生產資料庫${NC}"
        echo "可能是權限問題，請檢查使用者權限"
    fi
fi

# 7. 總結
echo ""
echo -e "${BLUE}=== 檢查總結 ===${NC}"
if [ "$USER_EXISTS" = "1" ] && [ "$PROD_DB_EXISTS" = "1" ] && [ "$TEST_DB_EXISTS" = "1" ]; then
    echo -e "${GREEN}✅ 資料庫已完成初始化，可以執行 db_install${NC}"
    echo "下一步: cd /app/db_proxy_ws/src/db_proxy && python3 -m db_proxy.sql.db_install"
else
    echo -e "${YELLOW}⚠️ 資料庫尚未完成初始化${NC}"
    echo "請執行: ./init_database.sh"
fi

echo ""
echo "檢查完成時間: $(date)"
