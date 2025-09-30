#!/bin/bash
# 統一更新所有索引檔案
# 用於自動化更新 docs-ai 索引和 CLAUDE 架構統計

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 工作目錄
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

echo -e "${BLUE}=================================================${NC}"
echo -e "${BLUE}   📊 RosAGV 知識庫索引更新器${NC}"
echo -e "${BLUE}=================================================${NC}"
echo ""

# 檢查 Python 環境
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}❌ 錯誤: 需要 Python 3${NC}"
    exit 1
fi

echo -e "${YELLOW}🔍 步驟 1/2: 更新 docs-ai 知識文檔索引${NC}"
echo "----------------------------------------"
if python3 generate-docs-ai-index.py; then
    echo -e "${GREEN}✅ docs-ai 索引更新成功${NC}"
else
    echo -e "${RED}❌ docs-ai 索引更新失敗${NC}"
    exit 1
fi

echo ""
echo -e "${YELLOW}🏗️  步驟 2/2: 更新 CLAUDE 架構統計${NC}"
echo "----------------------------------------"
if python3 generate-claude-architecture-stats.py; then
    echo -e "${GREEN}✅ CLAUDE 架構統計更新成功${NC}"
else
    echo -e "${RED}❌ CLAUDE 架構統計更新失敗${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}=================================================${NC}"
echo -e "${GREEN}🎉 所有索引更新完成！${NC}"
echo -e "${BLUE}=================================================${NC}"
echo ""
echo "📁 生成的檔案:"
echo "   • js/docs-ai-index.json"
echo "   • js/claude-architecture.json"
echo ""
echo "🌐 查看結果:"
echo "   訪問 http://agvc.ui/docs/index.html"
echo "   切換到「🤖 AI 知識庫」頁籤"
echo ""