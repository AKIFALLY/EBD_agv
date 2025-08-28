#!/bin/bash
# TAFL WCS 測試執行腳本

# 顏色定義
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}=== TAFL WCS 測試套件 ===${NC}"

# 檢查是否在容器內
if [ ! -f /app/setup.bash ]; then
    echo -e "${RED}錯誤: 請在 AGVC 容器內執行此腳本${NC}"
    exit 1
fi

# 載入 ROS 2 環境
source /app/setup.bash
source /app/tafl_wcs_ws/install/setup.bash 2>/dev/null

# 測試類型選擇
TEST_TYPE=${1:-all}

case $TEST_TYPE in
    unit)
        echo -e "${GREEN}執行單元測試...${NC}"
        python3 -m pytest src/tafl_wcs/test/test_copyright.py -v
        python3 -m pytest src/tafl_wcs/test/test_flake8.py -v
        python3 -m pytest src/tafl_wcs/test/test_pep257.py -v
        ;;
    db)
        echo -e "${GREEN}執行資料庫測試...${NC}"
        python3 src/tafl_wcs/test/test_simple_db.py
        python3 src/tafl_wcs/test/test_check_all_data.py
        ;;
    integration)
        echo -e "${GREEN}執行整合測試...${NC}"
        python3 src/tafl_wcs/test/test_tafl_system.py
        ;;
    all)
        echo -e "${GREEN}執行所有測試...${NC}"
        colcon test --packages-select tafl_wcs
        colcon test-result --verbose
        ;;
    *)
        echo "使用方法: $0 [unit|db|integration|all]"
        echo "  unit        - 執行單元測試"
        echo "  db          - 執行資料庫測試"
        echo "  integration - 執行整合測試"
        echo "  all         - 執行所有測試 (預設)"
        exit 1
        ;;
esac

echo -e "${YELLOW}=== 測試完成 ===${NC}"