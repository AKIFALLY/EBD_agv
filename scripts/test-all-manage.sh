#!/bin/bash
# 測試所有 manage_xxx 函數的腳本

echo "========================================"
echo "測試所有 manage_xxx 函數的運作狀態"
echo "========================================"
echo ""

# 測試函數
test_service() {
    local service_name=$1
    local manage_cmd="manage_$service_name"
    
    echo "🔍 測試 $manage_cmd..."
    
    # 停止服務
    echo "  ⏹️  停止服務..."
    docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && agvc_source && $manage_cmd stop" >/dev/null 2>&1
    sleep 1
    
    # 檢查狀態（應該是停止）
    status=$(docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && agvc_source && $manage_cmd status" 2>&1 | tail -1)
    if echo "$status" | grep -q "未運行\|未啟動\|stopped"; then
        echo "  ✅ 停止狀態正確"
    else
        echo "  ⚠️  停止狀態異常: $status"
    fi
    
    # 啟動服務
    echo "  ▶️  啟動服務..."
    docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && agvc_source && $manage_cmd start" >/dev/null 2>&1
    sleep 2
    
    # 檢查狀態（應該是運行中）
    status=$(docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && agvc_source && $manage_cmd status" 2>&1 | tail -1)
    if echo "$status" | grep -q "運行中\|已啟動\|running"; then
        echo "  ✅ 啟動狀態正確"
    else
        echo "  ⚠️  啟動狀態異常: $status"
    fi
    
    # 重啟服務
    echo "  🔄 重啟服務..."
    docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && agvc_source && $manage_cmd restart" >/dev/null 2>&1
    sleep 2
    
    # 最終狀態檢查
    status=$(docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && agvc_source && $manage_cmd status" 2>&1 | tail -1)
    if echo "$status" | grep -q "運行中\|已啟動\|running"; then
        echo "  ✅ 重啟後狀態正確"
    else
        echo "  ⚠️  重啟後狀態異常: $status"
    fi
    
    # 檢查殭屍進程
    zombie_count=$(docker compose -f docker-compose.agvc.yml exec agvc_server ps aux | grep -c defunct)
    if [ "$zombie_count" -eq 0 ]; then
        echo "  ✅ 無殭屍進程"
    else
        echo "  ⚠️  發現 $zombie_count 個殭屍進程"
    fi
    
    # 停止服務（清理）
    docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && agvc_source && $manage_cmd stop" >/dev/null 2>&1
    
    echo "  ✔️  $manage_cmd 測試完成"
    echo ""
}

# 進入正確目錄
cd ~/EBD_agv

# 測試各個服務
echo "1️⃣ RCS 服務"
test_service "rcs"

echo "2️⃣ Web API Launch 服務"
test_service "web_api_launch"

echo "3️⃣ TAFL WCS 服務"
test_service "tafl_wcs"

echo "4️⃣ ECS Core 服務"
test_service "ecs_core"

# Zenoh 和 SSH 是系統服務，測試方式不同
echo "5️⃣ Zenoh Router 服務"
echo "  🔍 測試 manage_zenoh..."
status=$(docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && agvc_source && manage_zenoh status" 2>&1 | tail -1)
if echo "$status" | grep -q "運行中\|running"; then
    echo "  ✅ Zenoh Router 運行正常"
else
    echo "  ⚠️  Zenoh Router 狀態異常: $status"
fi
echo ""

echo "8️⃣ SSH 服務"
echo "  🔍 測試 manage_ssh..."
status=$(docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && agvc_source && manage_ssh status" 2>&1 | tail -1)
if echo "$status" | grep -q "運行中\|running"; then
    echo "  ✅ SSH 服務運行正常"
else
    echo "  ⚠️  SSH 服務狀態異常: $status"
fi
echo ""

# 最終殭屍進程檢查
echo "========================================"
echo "最終檢查"
echo "========================================"
zombie_count=$(docker compose -f docker-compose.agvc.yml exec agvc_server ps aux | grep -c defunct)
if [ "$zombie_count" -eq 0 ]; then
    echo "✅ 系統無殭屍進程"
else
    echo "⚠️  系統有 $zombie_count 個殭屍進程"
fi

echo ""
echo "測試完成！"