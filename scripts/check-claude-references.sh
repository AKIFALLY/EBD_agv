#!/bin/bash

# 檢查 CLAUDE.md 文件中引用的 prompts 文件是否存在
# 使用方式: ./scripts/check-claude-references.sh

echo "🔍 檢查 CLAUDE.md 文件中的 @ 引用..."
echo "=================================================="

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 計數器
total_references=0
missing_references=0
found_references=0

# 檢查函數
check_reference() {
    local claude_file="$1"
    local reference="$2"
    
    # 移除 @ 符號
    local file_path="${reference#@}"
    
    total_references=$((total_references + 1))
    
    if [ -f "$file_path" ]; then
        echo -e "  ✅ ${GREEN}$reference${NC}"
        found_references=$((found_references + 1))
    else
        echo -e "  ❌ ${RED}$reference${NC} (檔案不存在)"
        missing_references=$((missing_references + 1))
    fi
}

# 檢查單個 CLAUDE.md 文件
check_claude_file() {
    local claude_file="$1"
    
    if [ ! -f "$claude_file" ]; then
        echo -e "⚠️  ${YELLOW}$claude_file 不存在${NC}"
        return
    fi
    
    echo -e "\n📄 檢查 ${YELLOW}$claude_file${NC}"
    echo "----------------------------------------"
    
    # 提取所有 @ 引用
    local references=$(rg -o '@[^[:space:]]*\.md' "$claude_file" | sort | uniq)
    
    if [ -z "$references" ]; then
        echo "  ℹ️  沒有找到 @ 引用"
        return
    fi
    
    # 檢查每個引用
    while IFS= read -r reference; do
        check_reference "$claude_file" "$reference"
    done <<< "$references"
}

# 主要檢查邏輯
main() {
    echo "開始檢查 CLAUDE.md 文件..."

    # 動態查找所有 CLAUDE.md 文件（根目錄 + 所有工作空間）
    claude_files=()

    # 添加根目錄的 CLAUDE.md（如果存在）
    if [ -f "CLAUDE.md" ]; then
        claude_files+=("CLAUDE.md")
    fi

    # 添加所有工作空間的 CLAUDE.md
    while IFS= read -r -d '' file; do
        claude_files+=("$file")
    done < <(find app -name "CLAUDE.md" -type f -print0 | sort -z)

    # 如果沒有找到任何 CLAUDE.md 文件
    if [ ${#claude_files[@]} -eq 0 ]; then
        echo -e "${YELLOW}⚠️  沒有找到任何 CLAUDE.md 文件${NC}"
        exit 1
    fi

    echo -e "找到 ${GREEN}${#claude_files[@]}${NC} 個 CLAUDE.md 文件需要檢查\n"

    # 檢查每個文件
    for claude_file in "${claude_files[@]}"; do
        check_claude_file "$claude_file"
    done
    
    # 顯示總結
    echo ""
    echo "=================================================="
    echo "📊 檢查結果總結:"
    echo "  總引用數: $total_references"
    echo -e "  找到文件: ${GREEN}$found_references${NC}"
    echo -e "  缺失文件: ${RED}$missing_references${NC}"
    
    if [ $missing_references -eq 0 ]; then
        echo -e "\n🎉 ${GREEN}所有引用的文件都存在！${NC}"
        exit 0
    else
        echo -e "\n⚠️  ${YELLOW}有 $missing_references 個文件缺失，需要創建或修正引用${NC}"
        exit 1
    fi
}

# 執行主函數
main "$@"
