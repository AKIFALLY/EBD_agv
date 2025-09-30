#!/bin/bash

# Git Tools 統一工具集
# RosAGV 專案 Git 管理工具包

# 腳本目錄
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 載入顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 載入自動提交輔助功能
source "$SCRIPT_DIR/auto_commit_helper.sh"

# Git 工具集主選單
show_git_tools_menu() {
    echo -e "${CYAN}🔧 RosAGV Git Tools 工具集${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo -e "${YELLOW}📦 提交管理工具:${NC}"
    echo -e "  ${GREEN}commit-stage${NC}     階段性自動提交"
    echo -e "  ${GREEN}commit-quick${NC}     快速提交 (fix/feat/style/docs/etc.)"
    echo -e "  ${GREEN}commit-backup${NC}    建立備份分支"
    echo ""
    echo -e "${YELLOW}📊 版本查看工具:${NC}"
    echo -e "  ${GREEN}show-commits${NC}     顯示最近提交"
    echo -e "  ${GREEN}compare-changes${NC}  比較版本差異"
    echo -e "  ${GREEN}show-status${NC}      顯示 Git 狀態"
    echo ""
    echo -e "${YELLOW}🔄 恢復管理工具:${NC}"
    echo -e "  ${GREEN}rollback${NC}         回滾到指定提交"
    echo -e "  ${GREEN}reset-hard${NC}       硬重置 (危險操作)"
    echo -e "  ${GREEN}show-branches${NC}    顯示所有分支"
    echo ""
    echo -e "${YELLOW}🧹 清理工具:${NC}"
    echo -e "  ${GREEN}fix-chinese${NC}      修復簡體中文字符"
    echo -e "  ${GREEN}clean-cache${NC}      清理 Git 快取"
    echo ""
    echo -e "${YELLOW}使用範例:${NC}"
    echo -e "  ${BLUE}source scripts/git-tools/git-tools.sh${NC}"
    echo -e "  ${BLUE}commit-stage 'TAFL Editor 修復' '修復節點連接問題'${NC}"
    echo -e "  ${BLUE}commit-quick fix '修復小錯誤'${NC}"
    echo -e "  ${BLUE}show-commits 10${NC}"
    echo ""
}

# 階段性提交
commit-stage() {
    local stage_name="$1"
    local description="$2"
    local commit_type="${3:-feat}"
    
    if [[ -z "$stage_name" || -z "$description" ]]; then
        echo -e "${RED}❌ 用法: commit-stage '<階段名稱>' '<描述>' [commit_type]${NC}"
        return 1
    fi
    
    auto_commit_stage "$stage_name" "$description" "$commit_type"
}

# 快速提交
commit-quick() {
    local type="$1"
    local message="$2"
    
    if [[ -z "$type" || -z "$message" ]]; then
        echo -e "${RED}❌ 用法: commit-quick <type> '<訊息>'${NC}"
        echo -e "${YELLOW}   類型: fix, feat, style, docs, test, perf, refactor${NC}"
        return 1
    fi
    
    case "$type" in
        fix|feat|style|docs|test|perf|refactor)
            quick_commit "$message" "$type"
            ;;
        *)
            echo -e "${RED}❌ 不支援的提交類型: $type${NC}"
            echo -e "${YELLOW}   支援的類型: fix, feat, style, docs, test, perf, refactor${NC}"
            return 1
            ;;
    esac
}

# 建立備份分支
commit-backup() {
    create_backup_branch
}

# 顯示最近提交
show-commits() {
    local count="${1:-10}"
    show_recent_commits "$count"
}

# 比較變更
compare-changes() {
    local from="${1:-HEAD~1}"
    local to="${2:-HEAD}"
    compare_changes "$from" "$to"
}

# 顯示 Git 狀態
show-status() {
    echo -e "${CYAN}📋 Git 狀態:${NC}"
    git status --short
    echo ""
    echo -e "${YELLOW}🌿 當前分支:${NC}"
    git branch --show-current
    echo ""
    echo -e "${PURPLE}📊 未推送的提交:${NC}"
    git log --oneline @{u}.. 2>/dev/null || echo "無未推送的提交"
}

# 回滾到指定提交
rollback() {
    local commit_hash="$1"
    if [[ -z "$commit_hash" ]]; then
        echo -e "${YELLOW}📚 最近 10 次提交:${NC}"
        show-commits 10
        echo ""
        echo -e "${RED}❌ 用法: rollback <commit_hash>${NC}"
        return 1
    fi
    
    rollback_to_commit "$commit_hash"
}

# 硬重置 (危險操作)
reset-hard() {
    local target="${1:-HEAD}"
    
    echo -e "${RED}⚠️  警告: 這是危險操作，將會永久丟失未提交的變更！${NC}"
    echo -e "${YELLOW}目標: $target${NC}"
    echo -e "${CYAN}是否繼續? (輸入 'YES' 確認):${NC}"
    read -r confirm
    
    if [[ "$confirm" == "YES" ]]; then
        create_backup_branch
        git reset --hard "$target"
        echo -e "${GREEN}✅ 硬重置完成${NC}"
    else
        echo -e "${BLUE}❌ 操作已取消${NC}"
    fi
}

# 顯示所有分支
show-branches() {
    echo -e "${CYAN}🌿 本地分支:${NC}"
    git branch -v
    echo ""
    echo -e "${PURPLE}🌐 遠端分支:${NC}"
    git branch -r -v
}

# 修復簡體中文字符
fix-chinese() {
    echo -e "${BLUE}🔧 執行簡體中文字符修復...${NC}"
    "$SCRIPT_DIR/fix_chinese_characters.sh"
}

# 清理 Git 快取
clean-cache() {
    echo -e "${BLUE}🧹 清理 Git 快取...${NC}"
    git gc --prune=all
    git repack -ad
    echo -e "${GREEN}✅ Git 快取清理完成${NC}"
}

# 主函數 - 根據參數調用對應功能
git_tools_main() {
    case "${1:-menu}" in
        commit-stage)
            commit-stage "$2" "$3" "$4"
            ;;
        commit-quick)
            commit-quick "$2" "$3"
            ;;
        commit-backup)
            commit-backup
            ;;
        show-commits)
            show-commits "$2"
            ;;
        compare-changes)
            compare-changes "$2" "$3"
            ;;
        show-status)
            show-status
            ;;
        rollback)
            rollback "$2"
            ;;
        reset-hard)
            reset-hard "$2"
            ;;
        show-branches)
            show-branches
            ;;
        fix-chinese)
            fix-chinese
            ;;
        clean-cache)
            clean-cache
            ;;
        menu|help|*)
            show_git_tools_menu
            ;;
    esac
}

# 如果腳本被直接執行，調用主函數
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    git_tools_main "$@"
fi

# 腳本載入完成提示
echo -e "${GREEN}✅ Git Tools 工具集載入完成${NC}"
echo -e "${CYAN}💡 輸入任意函數名稱使用，或查看 git_tools_main 獲取完整選單${NC}"