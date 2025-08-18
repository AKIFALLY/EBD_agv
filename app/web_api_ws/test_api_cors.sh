#!/bin/bash

echo "測試 API CORS 和執行端點"
echo "========================="

# 測試 1: 檢查 API 服務是否運行
echo -e "\n1. 檢查 API 服務狀態:"
curl -s http://localhost:8000/docs > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ API 服務正在運行"
else
    echo "❌ API 服務未運行，請先啟動 api_server.py"
    exit 1
fi

# 測試 2: 測試 CORS preflight (OPTIONS)
echo -e "\n2. 測試 CORS preflight 請求:"
curl -i -X OPTIONS http://localhost:8000/api/flow/execute \
  -H "Origin: http://localhost:8001" \
  -H "Access-Control-Request-Method: POST" \
  -H "Access-Control-Request-Headers: Content-Type" 2>/dev/null | head -20

# 測試 3: 測試獲取函數庫
echo -e "\n3. 測試獲取函數庫:"
curl -s http://localhost:8000/api/flow/functions | jq '.success, .source, .version' 2>/dev/null || echo "無法獲取函數庫"

# 測試 4: 測試執行函數
echo -e "\n4. 測試執行函數 (check.empty):"
curl -X POST http://localhost:8000/api/flow/execute \
  -H "Content-Type: application/json" \
  -H "Origin: http://localhost:8001" \
  -d '{
    "function_name": "check.empty",
    "params": {"data": []},
    "variables": {}
  }' 2>/dev/null | jq '.' 2>/dev/null || echo "執行失敗"

# 測試 5: 測試透過 nginx proxy
echo -e "\n5. 測試透過 nginx proxy (如果配置了):"
curl -s http://agvc.webapi/api/flow/functions 2>/dev/null | jq '.success' 2>/dev/null || echo "nginx proxy 未配置或無法連接"

echo -e "\n測試完成！"
echo "如果看到 CORS 相關 header (Access-Control-Allow-*), 表示 CORS 已正確配置"