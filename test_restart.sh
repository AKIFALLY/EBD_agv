#!/bin/bash
# 测试 manage_web_api_launch restart 修复效果

echo "========================================="
echo "测试开始: $(date)"
echo "========================================="

# 进入容器并执行 restart
docker compose -f docker-compose.agvc.yml exec -T agvc_server bash -i -c "
source /app/setup.bash &&
agvc_source > /dev/null 2>&1 &&
echo '准备执行 manage_web_api_launch restart...' &&
manage_web_api_launch restart
EXIT_CODE=\$?
echo ''
echo '========================================='
echo '测试结束，退出码:' \$EXIT_CODE
echo '========================================='
exit \$EXIT_CODE
"

TEST_RESULT=$?
echo ""
echo "========================================="
echo "最终结果: 退出码 $TEST_RESULT"
if [ $TEST_RESULT -eq 0 ]; then
    echo "✅ restart 测试成功完成！"
else
    echo "❌ restart 测试失败，退出码: $TEST_RESULT"
fi
echo "========================================="
