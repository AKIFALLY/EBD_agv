#!/bin/bash
# 測試所有 manage_ 函數的啟動、停止功能，並檢查殭屍進程
# 作者: AI Assistant
# 日期: 2025-08-15

set -e

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 載入 setup.bash
source /app/setup.bash

# 測試結果記錄
TEST_RESULTS=()
ZOMBIE_CHECK_RESULTS=()

# 檢查殭屍進程的函數
check_zombies() {
    local service_name=$1
    local zombie_count=$(ps aux | grep -E "defunct|<zombie>" | grep -v grep | wc -l)
    if [ $zombie_count -gt 0 ]; then
        echo -e "${RED}⚠️  發現 $zombie_count 個殭屍進程！${NC}"
        ps aux | grep -E "defunct|<zombie>" | grep -v grep
        ZOMBIE_CHECK_RESULTS+=("$service_name: 發現 $zombie_count 個殭屍進程")
        return 1
    else
        echo -e "${GREEN}✅ 沒有殭屍進程${NC}"
        ZOMBIE_CHECK_RESULTS+=("$service_name: 無殭屍進程")
        return 0
    fi
}

# 檢查 PID 檔案中的進程是否都已清理
check_pid_cleanup() {
    local pid_file=$1
    local service_name=$2
    
    if [ -f "$pid_file" ]; then
        echo -e "${RED}❌ PID 檔案未清理: $pid_file${NC}"
        return 1
    fi
    
    echo -e "${GREEN}✅ PID 檔案已清理${NC}"
    return 0
}

# 測試單個 manage 函數
test_manage_function() {
    local func_name=$1
    local pid_file=$2
    
    echo ""
    echo "========================================="
    echo -e "${BLUE}測試: $func_name${NC}"
    echo "========================================="
    
    # 1. 先停止（確保乾淨狀態）
    echo -e "${YELLOW}步驟 1: 初始停止（清理環境）${NC}"
    $func_name stop 2>/dev/null || true
    sleep 1
    
    # 2. 檢查初始狀態
    echo -e "${YELLOW}步驟 2: 檢查初始狀態${NC}"
    $func_name status
    
    # 3. 啟動服務
    echo -e "${YELLOW}步驟 3: 啟動服務${NC}"
    if $func_name start; then
        echo -e "${GREEN}✅ 啟動成功${NC}"
        sleep 2
        
        # 4. 檢查運行狀態
        echo -e "${YELLOW}步驟 4: 檢查運行狀態${NC}"
        $func_name status
        
        # 5. 檢查 PID 檔案內容
        if [ -f "$pid_file" ]; then
            echo -e "${YELLOW}步驟 5: PID 檔案內容:${NC}"
            cat "$pid_file"
            echo ""
            
            # 驗證所有 PID 都在運行
            local all_running=true
            while IFS= read -r pid; do
                if kill -0 $pid 2>/dev/null; then
                    echo -e "  PID $pid: ${GREEN}運行中${NC}"
                else
                    echo -e "  PID $pid: ${RED}未運行${NC}"
                    all_running=false
                fi
            done < "$pid_file"
            
            if [ "$all_running" = false ]; then
                echo -e "${RED}❌ 部分進程未運行${NC}"
            fi
        fi
        
        # 6. 停止服務
        echo -e "${YELLOW}步驟 6: 停止服務${NC}"
        if $func_name stop; then
            echo -e "${GREEN}✅ 停止成功${NC}"
            sleep 2
            
            # 7. 檢查停止後狀態
            echo -e "${YELLOW}步驟 7: 檢查停止後狀態${NC}"
            $func_name status
            
            # 8. 檢查殭屍進程
            echo -e "${YELLOW}步驟 8: 檢查殭屍進程${NC}"
            if check_zombies "$func_name"; then
                # 9. 檢查 PID 檔案清理
                echo -e "${YELLOW}步驟 9: 檢查 PID 檔案清理${NC}"
                if check_pid_cleanup "$pid_file" "$func_name"; then
                    TEST_RESULTS+=("$func_name: ✅ 通過")
                else
                    TEST_RESULTS+=("$func_name: ⚠️  PID 檔案未清理")
                fi
            else
                TEST_RESULTS+=("$func_name: ❌ 有殭屍進程")
            fi
        else
            echo -e "${RED}❌ 停止失敗${NC}"
            TEST_RESULTS+=("$func_name: ❌ 停止失敗")
        fi
    else
        echo -e "${RED}❌ 啟動失敗${NC}"
        TEST_RESULTS+=("$func_name: ❌ 啟動失敗")
    fi
    
    echo ""
}

# 主測試流程
main() {
    echo "========================================="
    echo -e "${BLUE}開始測試所有 manage_ 函數${NC}"
    echo "========================================="
    echo "環境: $(hostname)"
    echo "時間: $(date)"
    echo ""
    
    # 檢查環境
    if is_agvc_environment; then
        echo -e "${GREEN}✅ 在 AGVC 環境中${NC}"
        
        # 測試 AGVC 環境的管理函數
        test_manage_function "manage_ssh" "/tmp/ssh.pid"
        test_manage_function "manage_zenoh" "/tmp/zenoh_router.pid"
        test_manage_function "manage_web_api_launch" "/tmp/web_api_launch.pid"
        test_manage_function "manage_tafl_wcs" "/tmp/tafl_wcs.pid"
        test_manage_function "manage_ecs_core" "/tmp/ecs.pid"
        test_manage_function "manage_db_proxy" "/tmp/db_proxy.pid"
        test_manage_function "manage_rcs_core" "/tmp/rcs_launch.pid"
        test_manage_function "manage_kuka_fleet" "/tmp/kuka_fleet.pid"
        
    elif is_agv_environment; then
        echo -e "${GREEN}✅ 在 AGV 環境中${NC}"
        
        # 測試 AGV 環境的管理函數
        test_manage_function "manage_ssh" "/tmp/ssh.pid"
        test_manage_function "manage_zenoh" "/tmp/zenoh_router.pid"
        test_manage_function "manage_agvui" "/tmp/agvui.pid"
        
    else
        echo -e "${RED}❌ 未知環境${NC}"
        exit 1
    fi
    
    # 顯示測試總結
    echo ""
    echo "========================================="
    echo -e "${BLUE}測試總結${NC}"
    echo "========================================="
    
    for result in "${TEST_RESULTS[@]}"; do
        echo "$result"
    done
    
    echo ""
    echo "========================================="
    echo -e "${BLUE}殭屍進程檢查總結${NC}"
    echo "========================================="
    
    for result in "${ZOMBIE_CHECK_RESULTS[@]}"; do
        echo "$result"
    done
    
    # 最終殭屍進程檢查
    echo ""
    echo "========================================="
    echo -e "${BLUE}最終系統殭屍進程檢查${NC}"
    echo "========================================="
    
    local final_zombie_count=$(ps aux | grep -E "defunct|<zombie>" | grep -v grep | wc -l)
    if [ $final_zombie_count -gt 0 ]; then
        echo -e "${RED}⚠️  系統中仍有 $final_zombie_count 個殭屍進程：${NC}"
        ps aux | grep -E "defunct|<zombie>" | grep -v grep
    else
        echo -e "${GREEN}✅ 系統中沒有殭屍進程${NC}"
    fi
    
    echo ""
    echo "測試完成時間: $(date)"
}

# 執行主測試
main