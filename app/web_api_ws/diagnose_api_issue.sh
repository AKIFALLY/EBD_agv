#!/bin/bash

echo "診斷 Linear Flow Designer API 問題"
echo "==================================="

# 1. 檢查 API 服務
echo -e "\n1. 檢查 API 服務運行狀態:"
if curl -s http://localhost:8000/docs > /dev/null 2>&1; then
    echo "✅ API 服務正在運行 (port 8000)"
else
    echo "❌ API 服務未運行"
    echo "   請執行: ./restart_api_with_cors.sh"
    exit 1
fi

# 2. 檢查 CORS 配置
echo -e "\n2. 檢查 CORS 配置:"
cors_headers=$(curl -s -I -X OPTIONS http://localhost:8000/api/flow/execute \
  -H "Origin: http://localhost:8001" \
  -H "Access-Control-Request-Method: POST" \
  -H "Access-Control-Request-Headers: Content-Type" 2>/dev/null | grep -i "access-control")

if [ -n "$cors_headers" ]; then
    echo "✅ CORS 已配置:"
    echo "$cors_headers"
else
    echo "❌ CORS 未配置或配置錯誤"
fi

# 3. 檢查函數庫端點
echo -e "\n3. 測試函數庫端點:"
response=$(curl -s http://localhost:8000/api/flow/functions)
if echo "$response" | jq -e '.success' > /dev/null 2>&1; then
    echo "✅ 函數庫端點正常"
    echo "   函數類別: $(echo "$response" | jq -r '.functions | keys | join(", ")')"
else
    echo "❌ 函數庫端點異常"
    echo "   響應: $response"
fi

# 4. 測試執行端點
echo -e "\n4. 測試函數執行端點:"
exec_response=$(curl -s -X POST http://localhost:8000/api/flow/execute \
  -H "Content-Type: application/json" \
  -H "Origin: http://localhost:8001" \
  -d '{
    "function_name": "check.empty",
    "params": {"data": []},
    "variables": {}
  }')

if echo "$exec_response" | jq -e '.success' > /dev/null 2>&1; then
    echo "✅ 執行端點正常"
    echo "   結果: $(echo "$exec_response" | jq -c '.result')"
else
    echo "❌ 執行端點異常"
    echo "   錯誤: $exec_response"
fi

# 5. 檢查資料庫連接
echo -e "\n5. 檢查資料庫連接:"
db_test=$(docker compose -f /home/ct/RosAGV/docker-compose.agvc.yml exec agvc_server bash -c "
python3 -c '
from sqlalchemy import create_engine
try:
    engine = create_engine(\"postgresql://agvc:password@192.168.100.254:5432/agvc\")
    with engine.connect() as conn:
        result = conn.execute(\"SELECT 1\")
        print(\"OK\")
except Exception as e:
    print(f\"ERROR: {e}\")
' 2>&1")

if echo "$db_test" | grep -q "OK"; then
    echo "✅ 資料庫連接正常"
else
    echo "❌ 資料庫連接失敗"
    echo "   錯誤: $db_test"
fi

# 6. 檢查 nginx proxy
echo -e "\n6. 檢查 nginx proxy (agvc.webapi):"
if curl -s http://agvc.webapi/api/flow/functions > /dev/null 2>&1; then
    echo "✅ nginx proxy 正常工作"
else
    echo "⚠️ nginx proxy 可能未配置"
    echo "   這不影響使用 localhost:8000"
fi

# 7. 瀏覽器測試建議
echo -e "\n7. 瀏覽器測試建議:"
echo "   1. 打開瀏覽器開發者工具 (F12)"
echo "   2. 切換到 Network 分頁"
echo "   3. 執行測試時觀察請求"
echo "   4. 檢查是否有 CORS 錯誤在 Console"

echo -e "\n診斷完成！"
echo "如果仍有問題，請查看 API 日誌:"
echo "docker compose -f docker-compose.agvc.yml logs agvc_server | tail -50"