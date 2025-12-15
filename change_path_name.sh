#!/bin/bash
# 路徑名稱變更腳本（參數化版本）
# 用法: ./change_path_name.sh <舊專案名> <新專案名>
# 範例: ./change_path_name.sh RosAGV EBD_agv
#       ./change_path_name.sh EBD_agv NewProject

set -e

# 參數檢查
if [ $# -ne 2 ]; then
    echo "用法: $0 <舊專案名> <新專案名>"
    echo "範例: $0 RosAGV EBD_agv"
    exit 1
fi

OLD_NAME="$1"
NEW_NAME="$2"

OLD_PATH_FULL="/home/ct/$OLD_NAME"
NEW_PATH_FULL="/home/ct/$NEW_NAME"
OLD_PATH_SHORT="~/$OLD_NAME"
NEW_PATH_SHORT="~/$NEW_NAME"

# 確認當前目錄
CURRENT_DIR=$(pwd)
if [[ "$CURRENT_DIR" != *"$NEW_NAME"* ]]; then
    echo "⚠️  請先 cd 到新專案目錄再執行此腳本"
    echo "   cd ~/$NEW_NAME && ./change_path_name.sh $OLD_NAME $NEW_NAME"
    exit 1
fi

echo "=== 路徑名稱變更：$OLD_NAME → $NEW_NAME ==="
echo ""
echo "🔍 替換規則："
echo "   $OLD_PATH_FULL → $NEW_PATH_FULL"
echo "   $OLD_PATH_SHORT → $NEW_PATH_SHORT"
echo ""

# 1. Docker Compose 配置
echo "🔴 [1/6] 修改 Docker Compose 配置..."
sed -i "s|$OLD_PATH_SHORT|$NEW_PATH_SHORT|g" docker-compose.yml 2>/dev/null || true
sed -i "s|$OLD_PATH_SHORT|$NEW_PATH_SHORT|g" docker-compose.agvc.yml 2>/dev/null || true
echo "   ✅ Docker Compose 配置已更新"

# 2. Claude 配置（JSON 檔案）
echo "🔴 [2/6] 修改 Claude 配置..."
find .claude/ -name "*.json" -exec sed -i "s|$OLD_PATH_SHORT|$NEW_PATH_SHORT|g" {} \; 2>/dev/null || true
find .claude/ -name "*.json" -exec sed -i "s|$OLD_PATH_FULL|$NEW_PATH_FULL|g" {} \; 2>/dev/null || true
echo "   ✅ Claude 配置已更新"

# 3. Shell 腳本
echo "🟠 [3/6] 修改 Shell 腳本..."
find . -name "*.sh" -type f -exec sed -i "s|$OLD_PATH_FULL|$NEW_PATH_FULL|g" {} \; 2>/dev/null
find . -name "*.sh" -type f -exec sed -i "s|$OLD_PATH_SHORT|$NEW_PATH_SHORT|g" {} \; 2>/dev/null
echo "   ✅ Shell 腳本已更新"

# 4. Python 腳本
echo "🟡 [4/6] 修改 Python 腳本..."
find . -name "*.py" -type f -exec sed -i "s|$OLD_PATH_FULL|$NEW_PATH_FULL|g" {} \; 2>/dev/null
echo "   ✅ Python 腳本已更新"

# 5. AI Agent 規則
echo "🟡 [5/6] 修改 AI Agent 規則..."
sed -i "s|$OLD_PATH_FULL|$NEW_PATH_FULL|g" CLAUDE.md 2>/dev/null || true
sed -i "s|$OLD_PATH_SHORT|$NEW_PATH_SHORT|g" CLAUDE.md 2>/dev/null || true
find ai-agents/ -name "*.md" -exec sed -i "s|$OLD_PATH_FULL|$NEW_PATH_FULL|g" {} \; 2>/dev/null
find ai-agents/ -name "*.md" -exec sed -i "s|$OLD_PATH_SHORT|$NEW_PATH_SHORT|g" {} \; 2>/dev/null
echo "   ✅ AI Agent 規則已更新"

# 6. 文檔檔案
echo "🟢 [6/6] 修改文檔檔案..."
find docs-ai/ -name "*.md" -exec sed -i "s|$OLD_PATH_FULL|$NEW_PATH_FULL|g" {} \; 2>/dev/null
find docs-ai/ -name "*.md" -exec sed -i "s|$OLD_PATH_SHORT|$NEW_PATH_SHORT|g" {} \; 2>/dev/null
find design/ -name "*.md" -exec sed -i "s|$OLD_PATH_FULL|$NEW_PATH_FULL|g" {} \; 2>/dev/null
find design/ -name "*.md" -exec sed -i "s|$OLD_PATH_SHORT|$NEW_PATH_SHORT|g" {} \; 2>/dev/null
find docs/ -name "*.md" -exec sed -i "s|$OLD_PATH_FULL|$NEW_PATH_FULL|g" {} \; 2>/dev/null
find docs/ -name "*.md" -exec sed -i "s|$OLD_PATH_SHORT|$NEW_PATH_SHORT|g" {} \; 2>/dev/null
echo "   ✅ 文檔檔案已更新"

echo ""
echo "=== 變更完成 ==="
echo ""
echo "📋 後續步驟："
echo "   1. 重新啟動 Docker 容器："
echo "      docker compose -f docker-compose.yml down"
echo "      docker compose -f docker-compose.agvc.yml down"
echo "      docker compose -f docker-compose.agvc.yml up -d"
echo "      docker compose -f docker-compose.yml up -d"
echo ""
echo "   2. 更新 ~/.bashrc 中的 PATH（如有設定）："
echo "      將 $OLD_PATH_FULL 改為 $NEW_PATH_FULL"
echo ""
echo "   3. 驗證系統運行："
echo "      docker compose -f docker-compose.yml ps"
echo "      docker compose -f docker-compose.agvc.yml ps"
echo ""
echo "⚠️  注意：Docker 服務名 (rosagv, agvc_server) 不會被修改，這是正確的！"
