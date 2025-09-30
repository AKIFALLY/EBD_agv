#!/bin/bash

# 檢查 README.md 中樹狀結構的文件是否存在
# 專門檢查樹狀圖中的文件

echo "🌳 檢查樹狀結構中的文件..."
echo "=================================================="

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 從 docs-ai/README.md 提取樹狀結構中的文件
extract_tree_files() {
    local readme_file="$1"

    # 提取樹狀結構中的 .md 文件名（包含 ├── 和 └── 的行）
    rg "^[│├└].*\.md" "$readme_file" 2>/dev/null | sed 's/^[│├└─ ]*//g' | sed 's/ *#.*$//'
}

# 檢查文件是否存在
check_files() {
    local base_dir="$1"
    local readme_file="$2"

    echo -e "\n📂 檢查目錄: ${BLUE}$base_dir${NC}"
    echo "----------------------------------------"

    local total=0
    local found=0
    local missing=0
    local missing_files=()

    # 提取該 README 中的所有樹狀文件
    local tree_files=$(extract_tree_files "$readme_file")

    while IFS= read -r filename; do
        if [ -z "$filename" ] || [[ "$filename" == *"/"* ]]; then
            continue
        fi

        total=$((total + 1))

        # 在 base_dir 下遞歸查找文件
        local found_path=$(find "$base_dir" -name "$filename" -type f 2>/dev/null | head -1)

        if [ -n "$found_path" ]; then
            echo -e "  ✅ ${GREEN}$filename${NC} → $found_path"
            found=$((found + 1))
        else
            echo -e "  ❌ ${RED}$filename${NC} (未找到)"
            missing=$((missing + 1))
            missing_files+=("$filename")
        fi
    done <<< "$tree_files"

    # 顯示統計
    echo ""
    echo -e "  總計: ${BLUE}$total${NC} 個文件"
    echo -e "  找到: ${GREEN}$found${NC} 個"
    echo -e "  缺失: ${RED}$missing${NC} 個"

    if [ $missing -gt 0 ]; then
        echo -e "\n  ${YELLOW}缺失的文件列表:${NC}"
        for file in "${missing_files[@]}"; do
            echo "    - $file"
        done
    fi

    return $missing
}

# 主程序
main() {
    local total_missing=0

    # 檢查 docs-ai/README.md
    if [ -f "docs-ai/README.md" ]; then
        check_files "docs-ai" "docs-ai/README.md"
        total_missing=$((total_missing + $?))
    else
        echo -e "${YELLOW}⚠️  docs-ai/README.md 不存在${NC}"
    fi

    # 檢查其他包含樹狀結構的 README
    for readme in $(find . -name "README.md" -type f | grep -E "(docs-ai|design)" | head -10); do
        if [ "$readme" != "./docs-ai/README.md" ]; then
            local dir=$(dirname "$readme")
            if rg "^[│├└].*\.md" "$readme" &>/dev/null; then
                check_files "$dir" "$readme"
                total_missing=$((total_missing + $?))
            fi
        fi
    done

    # 總結
    echo ""
    echo "=================================================="
    if [ $total_missing -eq 0 ]; then
        echo -e "🎉 ${GREEN}所有樹狀結構中的文件都存在！${NC}"
        exit 0
    else
        echo -e "⚠️  ${YELLOW}總共有 $total_missing 個文件在樹狀結構中提到但不存在${NC}"
        exit 1
    fi
}

# 執行主程序
main "$@"