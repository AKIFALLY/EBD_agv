#!/bin/bash

# 檢查 CLAUDE.md 和其他 Markdown 文件中引用的文件是否存在
# 支援檢查：
# 1. @docs-ai/ 格式的引用
# 2. docs-ai/ 直接引用（沒有 @ 符號）
# 3. @design/business-process-docs/ 格式的引用
# 使用方式: ./scripts/check-claude-references.sh

echo "🔍 檢查 Markdown 文件中的文檔引用..."
echo "=================================================="

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 計數器
total_references=0
missing_references=0
found_references=0
checked_files=0

# 用於收集所有檢查過的文件
all_checked_files=()

# 檢查函數
check_reference() {
    local source_file="$1"
    local reference="$2"

    # 判斷引用類型並處理
    local file_path=""
    if [[ "$reference" =~ ^@ ]]; then
        # 移除 @ 符號
        file_path="${reference#@}"
    else
        # 直接使用（沒有 @ 的引用）
        file_path="$reference"
    fi

    total_references=$((total_references + 1))

    if [ -f "$file_path" ]; then
        echo -e "  ✅ ${GREEN}$reference${NC}"
        found_references=$((found_references + 1))
    else
        echo -e "  ❌ ${RED}$reference${NC} (檔案不存在)"
        missing_references=$((missing_references + 1))
    fi
}

# 檢查單個文件中的引用
check_file_references() {
    local file="$1"
    local display_name="$2"

    if [ ! -f "$file" ]; then
        echo -e "⚠️  ${YELLOW}$file 不存在${NC}"
        return
    fi

    checked_files=$((checked_files + 1))
    all_checked_files+=("$file")
    echo -e "\n📄 檢查 ${YELLOW}$display_name${NC}"
    echo "----------------------------------------"

    # 提取所有引用（同時匹配多種格式）
    # 1. @docs-ai/*.md
    # 2. docs-ai/*.md (沒有 @ 符號，但需要是獨立的路徑)
    # 3. @design/business-process-docs/*.md
    local references=""

    # 先提取 @ 開頭的引用
    local at_refs=$(rg -o '@(docs-ai|design/business-process-docs)/[^[:space:]]*\.md' "$file" 2>/dev/null | sort | uniq)

    # 再提取沒有 @ 的 docs-ai 引用（需要確保是路徑開頭）
    local direct_refs=$(rg -o '(^|[[:space:]]|:|#)docs-ai/[^[:space:]]*\.md' "$file" 2>/dev/null | sed 's/^[[:space:]:#]*//g' | sort | uniq)

    # 提取樹狀結構中的 .md 文件（處理 ├── 和 └── 符號）
    local tree_refs=""
    if [[ "$file" == *"README.md" ]]; then
        # 提取基礎路徑（如 docs-ai/context/system/）
        local base_paths=$(rg -o "^docs-ai/[^/]+/[^/]*/$" "$file" 2>/dev/null | sort | uniq)

        # 對每個基礎路徑，提取其下的文件
        while IFS= read -r base_path; do
            if [ -n "$base_path" ]; then
                # 提取該路徑下的樹狀結構文件名
                local tree_files=$(rg "^[│├└].*\.md" "$file" 2>/dev/null | sed 's/^[│├└─ ]*//g' | rg "^[^/]+\.md" -o)
                while IFS= read -r tree_file; do
                    if [ -n "$tree_file" ]; then
                        # 組合完整路徑
                        local full_path="${base_path}${tree_file}"
                        tree_refs="${tree_refs}${full_path}\n"
                    fi
                done <<< "$tree_files"
            fi
        done <<< "$base_paths"
    fi

    # 合併引用
    local all_refs=""
    [ -n "$at_refs" ] && all_refs="${all_refs}${at_refs}\n"
    [ -n "$direct_refs" ] && all_refs="${all_refs}${direct_refs}\n"
    [ -n "$tree_refs" ] && all_refs="${all_refs}${tree_refs}"

    if [ -n "$all_refs" ]; then
        references=$(echo -e "$all_refs" | sort | uniq)
    else
        references=""
    fi

    if [ -z "$references" ]; then
        echo "  ℹ️  沒有找到文檔引用"
        return
    fi

    # 檢查每個引用
    while IFS= read -r reference; do
        if [ -n "$reference" ]; then
            check_reference "$file" "$reference"
        fi
    done <<< "$references"
}

# 主要檢查邏輯
main() {
    echo -e "${BLUE}=== 第一部分：檢查 CLAUDE.md 文件 ===${NC}"
    echo ""

    # 動態查找所有 CLAUDE.md 文件（根目錄 + 所有工作空間）
    claude_files=()

    # 添加根目錄的 CLAUDE.md（如果存在）
    if [ -f "CLAUDE.md" ]; then
        claude_files+=("CLAUDE.md")
    fi

    # 添加所有工作空間的 CLAUDE.md
    while IFS= read -r -d '' file; do
        claude_files+=("$file")
    done < <(find app -name "CLAUDE.md" -type f -print0 2>/dev/null | sort -z)

    if [ ${#claude_files[@]} -gt 0 ]; then
        echo -e "找到 ${GREEN}${#claude_files[@]}${NC} 個 CLAUDE.md 文件\n"

        # 檢查每個 CLAUDE.md 文件
        for claude_file in "${claude_files[@]}"; do
            check_file_references "$claude_file" "$claude_file"
        done
    else
        echo -e "${YELLOW}⚠️  沒有找到任何 CLAUDE.md 文件${NC}\n"
    fi

    # 檢查 docs-ai 目錄下的所有 .md 文件
    echo ""
    echo -e "${BLUE}=== 第二部分：檢查 docs-ai/ 目錄下的 Markdown 文件 ===${NC}"
    echo ""

    if [ -d "docs-ai" ]; then
        local docs_ai_files=()
        while IFS= read -r -d '' file; do
            docs_ai_files+=("$file")
        done < <(find docs-ai -name "*.md" -type f -print0 2>/dev/null | sort -z)

        if [ ${#docs_ai_files[@]} -gt 0 ]; then
            echo -e "找到 ${GREEN}${#docs_ai_files[@]}${NC} 個 docs-ai/*.md 文件\n"

            for md_file in "${docs_ai_files[@]}"; do
                check_file_references "$md_file" "$md_file"
            done
        else
            echo -e "${YELLOW}⚠️  docs-ai/ 目錄下沒有 .md 文件${NC}\n"
        fi
    else
        echo -e "${YELLOW}⚠️  docs-ai/ 目錄不存在${NC}\n"
    fi

    # 檢查 design/business-process-docs 目錄下的所有 .md 文件
    echo ""
    echo -e "${BLUE}=== 第三部分：檢查 design/business-process-docs/ 目錄下的 Markdown 文件 ===${NC}"
    echo ""

    if [ -d "design/business-process-docs" ]; then
        local design_files=()
        while IFS= read -r -d '' file; do
            design_files+=("$file")
        done < <(find design/business-process-docs -name "*.md" -type f -print0 2>/dev/null | sort -z)

        if [ ${#design_files[@]} -gt 0 ]; then
            echo -e "找到 ${GREEN}${#design_files[@]}${NC} 個 design/business-process-docs/*.md 文件\n"

            for md_file in "${design_files[@]}"; do
                check_file_references "$md_file" "$md_file"
            done
        else
            echo -e "${YELLOW}⚠️  design/business-process-docs/ 目錄下沒有 .md 文件${NC}\n"
        fi
    else
        echo -e "${YELLOW}⚠️  design/business-process-docs/ 目錄不存在${NC}\n"
    fi

    # 顯示總結
    echo ""
    echo "=================================================="
    echo "📊 檢查結果總結:"
    echo -e "  檢查的文件數量: ${BLUE}$checked_files${NC}"
    echo -e "  總引用數: ${BLUE}$total_references${NC}"
    echo -e "  找到文件: ${GREEN}$found_references${NC}"
    echo -e "  缺失文件: ${RED}$missing_references${NC}"

    # 分析缺失文件類型
    if [ $missing_references -gt 0 ]; then
        echo ""
        echo "📝 缺失文件分析:"

        # 收集所有缺失的文件
        local template_refs=0
        local actual_missing=0

        echo "" > /tmp/missing_files.txt

        # 重新掃描所有文件找出缺失的引用
        for file in "${all_checked_files[@]}"; do
            if [ -f "$file" ]; then
                # 提取 @ 開頭的引用
                local at_refs=$(rg -o '@(docs-ai|design/business-process-docs)/[^[:space:]]*\.md' "$file" 2>/dev/null | sort | uniq)
                # 提取沒有 @ 的 docs-ai 引用
                local direct_refs=$(rg -o '(^|[[:space:]]|:|#)docs-ai/[^[:space:]]*\.md' "$file" 2>/dev/null | sed 's/^[[:space:]:#]*//g' | sort | uniq)

                # 合併引用
                local references=""
                if [ -n "$at_refs" ] && [ -n "$direct_refs" ]; then
                    references=$(echo -e "$at_refs\n$direct_refs" | sort | uniq)
                elif [ -n "$at_refs" ]; then
                    references="$at_refs"
                elif [ -n "$direct_refs" ]; then
                    references="$direct_refs"
                fi

                # 檢查每個引用
                while IFS= read -r reference; do
                    if [ -n "$reference" ]; then
                        local file_path=""
                        if [[ "$reference" =~ ^@ ]]; then
                            file_path="${reference#@}"
                        else
                            file_path="$reference"
                        fi

                        if [ ! -f "$file_path" ]; then
                            echo "$reference" >> /tmp/missing_files.txt
                        fi
                    fi
                done <<< "$references"
            fi
        done

        # 分析缺失文件
        echo -e "\n  ${YELLOW}模板佔位符（文檔示例）:${NC}"
        local has_template=false
        while IFS= read -r missing_file; do
            if [[ "$missing_file" =~ \[.*\] ]]; then
                echo -e "    📝 $missing_file"
                template_refs=$((template_refs + 1))
                has_template=true
            fi
        done < <(sort -u /tmp/missing_files.txt)

        if [ "$has_template" = false ]; then
            echo "    （無）"
        fi

        echo -e "\n  ${RED}實際缺失的文件:${NC}"
        local has_actual=false
        while IFS= read -r missing_file; do
            if [[ ! "$missing_file" =~ \[.*\] ]] && [ -n "$missing_file" ]; then
                echo -e "    ❌ $missing_file"
                actual_missing=$((actual_missing + 1))
                has_actual=true
            fi
        done < <(sort -u /tmp/missing_files.txt)

        if [ "$has_actual" = false ]; then
            echo "    （無）"
        fi

        # 清理臨時文件
        rm -f /tmp/missing_files.txt

        echo ""
        echo "------------------------------------------------"
        echo -e "  模板佔位符: ${YELLOW}$template_refs${NC} 個（文檔結構示例，無需修復）"
        echo -e "  實際缺失: ${RED}$actual_missing${NC} 個（需要創建或修正引用）"
    fi

    if [ $missing_references -eq 0 ]; then
        if [ $total_references -eq 0 ]; then
            echo -e "\n📝 ${YELLOW}沒有找到任何文檔引用${NC}"
        else
            echo -e "\n🎉 ${GREEN}所有 $total_references 個引用的文件都存在！${NC}"
        fi
        exit 0
    else
        if [ $actual_missing -eq 0 ]; then
            echo -e "\n✅ ${GREEN}所有實際文件都存在！只有 $template_refs 個模板佔位符。${NC}"
            exit 0
        else
            echo -e "\n⚠️  ${YELLOW}有 $actual_missing 個實際文件缺失，需要創建或修正引用${NC}"
            exit 1
        fi
    fi
}

# 執行主函數
main "$@"
