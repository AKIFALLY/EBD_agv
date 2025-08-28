#!/bin/bash
# Flow Functions 快取狀態檢查工具

echo "========================================="
echo "📊 Flow Functions 快取狀態檢查"
echo "========================================="

# 檢查快取文件
CACHE_FILE="/home/ct/RosAGV/app/config/wcs/flow_functions_cache.yaml"

if [ -f "$CACHE_FILE" ]; then
    echo ""
    echo "📁 快取文件: $CACHE_FILE"
    
    # 獲取更新時間
    UPDATE_TIME=$(grep "updated_at:" "$CACHE_FILE" | head -1 | cut -d"'" -f2)
    echo "🕐 更新時間: $UPDATE_TIME"
    
    # 統計函數數量
    echo ""
    echo "📊 函數統計:"
    for category in query check task action control special; do
        COUNT=$(grep -A1 "^  - name: $category\." "$CACHE_FILE" | grep "name:" | wc -l)
        printf "   %-10s: %2d 個函數\n" "$category" "$COUNT"
    done
    
    # 檢查特定函數
    echo ""
    echo "🔍 檢查關鍵函數:"
    for func in "control.switch" "control.foreach" "action.log_message"; do
        if grep -q "name: $func" "$CACHE_FILE"; then
            echo "   ✅ $func"
        else
            echo "   ❌ $func (缺失)"
        fi
    done
    
    # 比較文件時間戳
    echo ""
    echo "📅 文件時間戳比較:"
    CACHE_TIMESTAMP=$(stat -c %Y "$CACHE_FILE" 2>/dev/null)
    EXECUTOR_FILE="/home/ct/RosAGV/app/flow_wcs_ws/src/flow_wcs/flow_wcs/flow_executor.py"
    
    if [ -f "$EXECUTOR_FILE" ]; then
        EXECUTOR_TIMESTAMP=$(stat -c %Y "$EXECUTOR_FILE" 2>/dev/null)
        
        if [ "$EXECUTOR_TIMESTAMP" -gt "$CACHE_TIMESTAMP" ]; then
            echo "   ⚠️ flow_executor.py 比快取新！可能需要更新快取"
            echo "   建議執行: manage_web_api_launch restart"
        else
            echo "   ✅ 快取是最新的"
        fi
    fi
else
    echo "❌ 快取文件不存在: $CACHE_FILE"
fi

echo ""
echo "========================================="
echo "💡 提示: 如果快取過時，請執行:"
echo "   1. 重啟 Web 服務: manage_web_api_launch restart"
echo "   2. 或調用 API: curl http://localhost:8001/linear-flow/api/functions"
echo "========================================="