#!/bin/bash
# AGVUI 測試輔助腳本
# 用於在 AGVC 環境中測試 AGV 狀態顯示

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== AGVUI 測試工具 ===${NC}"
echo ""

# 功能選單
show_menu() {
    echo "請選擇測試方式："
    echo "1) 建立模擬 AGV 狀態檔案"
    echo "2) 使用 URL 參數測試特定 AGV"
    echo "3) 使用環境變數測試"
    echo "4) 清理測試檔案"
    echo "5) 退出"
}

# 建立模擬狀態檔案
create_mock_status() {
    local agv_id="${1:-loader01}"
    local status_file="/tmp/agv_status_${agv_id}.json"
    local main_file="/tmp/agv_status.json"
    
    echo -e "${YELLOW}建立模擬 AGV 狀態: $agv_id${NC}"
    
    # 生成包含 330+ 屬性的完整狀態
    cat > "$status_file" << EOF
{
  "AGV_ID": "$agv_id",
  "MAGIC": 12345,
  "AGV_SLAM_X": $(echo "scale=2; $RANDOM/1000" | bc),
  "AGV_SLAM_Y": $(echo "scale=2; $RANDOM/1000" | bc),
  "AGV_SLAM_THETA": $(echo "scale=2; $RANDOM/100" | bc),
  "POWER": $(echo "scale=1; 50 + $RANDOM/1000" | bc),
  "AGV_X_SPEED": 0.5,
  "AGV_Y_SPEED": 0.0,
  "AGV_THETA_SPEED": 0.0,
  "AGV_Auto": true,
  "AGV_Manual": false,
  "AGV_MOVING": false,
  "AGV_TURN_LEFT": false,
  "AGV_TURN_RIGHT": false,
  "AGV_FPGV": 1001,
  "AGV_BPGV": 1002,
  "AGV_START_POINT": 100,
  "AGV_END_POINT": 200,
  "AGV_ACTION": 1,
  "AGV_ZONE": 5,
  "AGV_STATUS1": 1,
  "AGV_STATUS2": 0,
  "AGV_STATUS3": 0,
  "AGV_ALARM1": 0,
  "AGV_ALARM2": 0,
  "AGV_ALARM3": 0,
  "AGV_ALARM4": 0,
  "AGV_ALARM5": 0,
  "AGV_ALARM6": 0,
  "AGV_LAYER": 1,
EOF

    # 添加門控狀態 (8個門)
    for i in {1..8}; do
        echo "  \"DOOR_OPEN_$i\": false," >> "$status_file"
        echo "  \"DOOR_CLOSE_$i\": true," >> "$status_file"
    done
    
    # 添加輸入狀態 (80個)
    for i in {1..80}; do
        echo "  \"Input$i\": $((RANDOM % 2))," >> "$status_file"
    done
    
    # 添加輸出狀態 (80個)
    for i in {1..80}; do
        echo "  \"Output$i\": $((RANDOM % 2))," >> "$status_file"
    done
    
    # 添加警報狀態 (99個)
    for i in {1..99}; do
        if [ $i -eq 99 ]; then
            echo "  \"Alarm$i\": 0," >> "$status_file"
        else
            echo "  \"Alarm$i\": 0," >> "$status_file"
        fi
    done
    
    # 添加時間戳和其他資訊
    echo "  \"timestamp\": $(date +%s)," >> "$status_file"
    echo "  \"namespace\": \"/$agv_id\"," >> "$status_file"
    echo "  \"node_name\": \"agv_node_base\"" >> "$status_file"
    echo "}" >> "$status_file"
    
    echo -e "${GREEN}✅ 模擬狀態檔案已建立: $status_file${NC}"
    echo "   總屬性數: $(grep -c '":' $status_file)"
}

# URL 參數測試
test_with_url() {
    local agv_id="${1:-loader01}"
    echo -e "${YELLOW}使用 URL 參數測試: $agv_id${NC}"
    echo ""
    echo "請在瀏覽器開啟以下網址："
    echo -e "${GREEN}http://localhost:8003/?agv_id=$agv_id${NC}"
    echo ""
    echo "這將會："
    echo "1. 顯示測試模式標示"
    echo "2. 只顯示 AGV ID = '$agv_id' 的狀態"
}

# 環境變數測試
test_with_env() {
    local agv_id="${1:-cargo01}"
    echo -e "${YELLOW}使用環境變數啟動 AGVUI:${NC}"
    echo ""
    echo "執行以下指令："
    echo -e "${GREEN}AGV_ID=$agv_id python3 /app/web_api_ws/src/agvui/agvui/agv_ui_server.py${NC}"
}

# 清理測試檔案
cleanup() {
    echo -e "${YELLOW}清理測試檔案...${NC}"
    rm -f /tmp/agv_status.json
    rm -f /tmp/test_device_identity
    rm -f /tmp/test_agv_identity
    echo -e "${GREEN}✅ 清理完成${NC}"
}

# 主程式
while true; do
    show_menu
    read -p "選擇 (1-5): " choice
    
    case $choice in
        1)
            read -p "輸入 AGV ID (預設: loader01): " agv_id
            agv_id="${agv_id:-loader01}"
            create_mock_status "$agv_id"
            ;;
        2)
            read -p "輸入要測試的 AGV ID (預設: loader01): " agv_id
            agv_id="${agv_id:-loader01}"
            test_with_url "$agv_id"
            ;;
        3)
            read -p "輸入要測試的 AGV ID (預設: cargo01): " agv_id
            agv_id="${agv_id:-cargo01}"
            test_with_env "$agv_id"
            ;;
        4)
            cleanup
            ;;
        5)
            echo "退出測試工具"
            exit 0
            ;;
        *)
            echo -e "${RED}無效選項${NC}"
            ;;
    esac
    echo ""
done