#!/bin/bash
# Flow Designer 發布標籤創建腳本
# 用途: 自動化創建版本標籤和更新版本資訊

VERSION="$1"
DESCRIPTION="$2"

if [ -z "$VERSION" ] || [ -z "$DESCRIPTION" ]; then
    echo "使用方式: $0 <version> <description>"
    echo "範例: $0 v2.0.1 '效能優化和錯誤修復'"
    echo ""
    echo "版本格式: vMAJOR.MINOR.PATCH"
    echo "- MAJOR: 不向後兼容的 API 變更"
    echo "- MINOR: 向後兼容的功能新增"  
    echo "- PATCH: 向後兼容的錯誤修復"
    exit 1
fi

# 驗證版本格式
if ! [[ $VERSION =~ ^v[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "❌ 錯誤: 版本格式必須是 vX.Y.Z"
    echo "範例: v2.0.1, v2.1.0, v3.0.0"
    exit 1
fi

echo "🏷️ 創建 Flow Designer 發布標籤: $VERSION"
echo "📝 描述: $DESCRIPTION"

# 檢查工作目錄是否乾淨
if ! git diff-index --quiet HEAD --; then
    echo "⚠️ 警告: 工作目錄有未提交的變更"
    echo "請先提交所有變更後再創建標籤"
    exit 1
fi

# 檢查是否已存在該標籤
if git tag | grep -q "^${VERSION}$"; then
    echo "❌ 錯誤: 標籤 $VERSION 已存在"
    echo "現有標籤:"
    git tag | sort -V | tail -5
    exit 1
fi

echo "1️⃣ 更新 Flow Designer 版本資訊..."

# 更新 JavaScript 檔案中的版本號
JS_FILE="app/web_api_ws/src/agvcui/agvcui/static/js/flowDesignerPage.js"
if [ -f "$JS_FILE" ]; then
    # 使用 sed 更新版本號（兼容 macOS 和 Linux）
    if [[ "$OSTYPE" == "darwin"* ]]; then
        sed -i '' "s/Flow Designer v[0-9]\+\.[0-9]\+\.[0-9]\+/Flow Designer $VERSION/g" "$JS_FILE"
    else
        sed -i "s/Flow Designer v[0-9]\+\.[0-9]\+\.[0-9]\+/Flow Designer $VERSION/g" "$JS_FILE"
    fi
    echo "✅ 已更新 JavaScript 檔案版本號"
else
    echo "⚠️ 警告: JavaScript 檔案不存在: $JS_FILE"
fi

# 更新文檔中的版本號
DOC_FILES=(
    "docs-ai/operations/development/flow-designer-phase4-4-production-deployment.md"
    "docs-ai/operations/development/flow-designer-complete-user-manual.md"
    "docs-ai/operations/development/flow-designer-best-practices-guide.md"
)

for doc_file in "${DOC_FILES[@]}"; do
    if [ -f "$doc_file" ]; then
        if [[ "$OSTYPE" == "darwin"* ]]; then
            sed -i '' "s/📝 \*\*文檔版本\*\*: v[0-9]\+\.[0-9]\+/📝 **文檔版本**: ${VERSION#v}/g" "$doc_file"
        else
            sed -i "s/📝 \*\*文檔版本\*\*: v[0-9]\+\.[0-9]\+/📝 **文檔版本**: ${VERSION#v}/g" "$doc_file"
        fi
        echo "✅ 已更新文檔版本號: $(basename $doc_file)"
    fi
done

echo "2️⃣ 創建 Git 提交..."

# 添加變更到 Git
git add .

# 創建提交
COMMIT_MESSAGE="chore: bump version to $VERSION

$DESCRIPTION

更新項目:
- Flow Designer JavaScript 版本
- 相關文檔版本標記
- 發布準備完成"

git commit -m "$COMMIT_MESSAGE"

echo "3️⃣ 創建 Git 標籤..."

# 創建註釋標籤
TAG_MESSAGE="Flow Designer $VERSION

$DESCRIPTION

發布內容:
- 視覺化流程設計器
- YAML DSL 雙向轉換
- 38個 WCS 函數整合
- 效能最佳化系統
- 完整的用戶培訓文檔

部署說明:
請參考 docs-ai/operations/development/flow-designer-phase4-4-production-deployment.md"

git tag -a "$VERSION" -m "$TAG_MESSAGE"

echo ""
echo "🎉 發布標籤 $VERSION 創建成功！"
echo ""
echo "📋 後續操作:"
echo "1. 推送提交: git push origin main"
echo "2. 推送標籤: git push origin $VERSION"
echo "3. 執行部署: ./scripts/deployment/deploy-flow-designer.sh production $VERSION"
echo ""
echo "📊 版本資訊:"
echo "   版本: $VERSION"
echo "   描述: $DESCRIPTION"
echo "   提交: $(git rev-parse HEAD)"
echo "   日期: $(date)"
echo ""
echo "🔗 相關資源:"
echo "   - 部署文檔: docs-ai/operations/development/flow-designer-phase4-4-production-deployment.md"
echo "   - 使用手冊: docs-ai/operations/development/flow-designer-complete-user-manual.md"
echo "   - 故障排除: docs-ai/operations/development/flow-designer-troubleshooting-manual.md"