#!/bin/bash

echo "重啟 API 服務 (含 CORS 支援)"
echo "============================"

# 停止現有的 api_server
echo "1. 停止現有 API 服務..."
pkill -f "api_server.py" 2>/dev/null
sleep 2

# 在 AGVC 容器內重啟服務
echo "2. 在 AGVC 容器內重啟 API 服務..."
docker compose -f /home/ct/RosAGV/docker-compose.agvc.yml exec -d agvc_server bash -c "
source /app/setup.bash && 
agvc_source && 
cd /app/web_api_ws/src/web_api && 
python3 web_api/api_server.py > /tmp/api_server.log 2>&1 &
"

echo "3. 等待服務啟動..."
sleep 3

# 檢查服務狀態
echo "4. 檢查服務狀態:"
curl -s http://localhost:8000/docs > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ API 服務已成功啟動"
    
    # 測試 CORS
    echo -e "\n5. 測試 CORS 配置:"
    response=$(curl -s -I -X OPTIONS http://localhost:8000/api/flow/execute \
      -H "Origin: http://localhost:8001" \
      -H "Access-Control-Request-Method: POST" 2>/dev/null | grep -i "access-control")
    
    if [ -n "$response" ]; then
        echo "✅ CORS 已正確配置:"
        echo "$response"
    else
        echo "⚠️ CORS 可能未正確配置"
    fi
else
    echo "❌ API 服務啟動失敗"
    echo "請檢查日誌: docker compose -f docker-compose.agvc.yml exec agvc_server cat /tmp/api_server.log"
fi

echo -e "\n完成！"