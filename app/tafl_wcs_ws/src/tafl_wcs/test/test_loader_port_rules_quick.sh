#!/bin/bash
# Loader AGV 端口使用規則快速測試腳本

echo "================================================================"
echo "Loader AGV 端口使用規則測試"
echo "================================================================"
echo ""
echo "📋 測試範圍:"
echo "  Port 1、3（前段流程端口）:"
echo "    - 來源: 入口傳送箱(101)、清洗機(303)"
echo "    - 優先順序: Port 1 → Port 3"
echo "  Port 2、4（後段流程端口）:"
echo "    - 來源: 泡藥機(403)"
echo "    - 優先順序: Port 2 → Port 4"
echo "  🔴 關鍵業務規則:"
echo "    - 只有 status_id=303（清洗完成）才能放入泡藥機"
echo ""

cd /app/tafl_wcs_ws/src/tafl_wcs/test

echo "開始執行測試..."
python3 test_loader_port_rules.py

exit_code=$?

echo ""
if [ $exit_code -eq 0 ]; then
    echo "✅ 測試通過"
else
    echo "❌ 測試失敗 (exit code: $exit_code)"
fi

exit $exit_code
