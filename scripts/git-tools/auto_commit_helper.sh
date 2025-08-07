#!/bin/bash

# 自動提交工作流程輔助腳本
# Auto Commit Workflow Helper Script
# 用於在每個開發階段自動建立 Git 檢查點

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 函數：自動提交階段性變更
auto_commit_stage() {
    local stage_name="$1"
    local description="$2"
    local commit_type="${3:-feat}" # 預設為 feat
    
    echo -e "${BLUE}🔄 開始階段性提交: ${stage_name}${NC}"
    
    # 檢查是否有變更
    if git diff --quiet && git diff --cached --quiet; then
        echo -e "${YELLOW}⚠️  沒有檢測到變更，跳過提交${NC}"
        return 0
    fi
    
    # 顯示變更摘要
    echo -e "${CYAN}📋 變更摘要:${NC}"
    git status --porcelain | head -10
    
    # 詢問是否繼續（可選，用於交互模式）
    if [[ "${AUTO_COMMIT_INTERACTIVE:-false}" == "true" ]]; then
        echo -e "${YELLOW}是否繼續提交? (y/N)${NC}"
        read -r confirm
        if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
            echo -e "${RED}❌ 提交已取消${NC}"
            return 1
        fi
    fi
    
    # 添加所有變更
    echo -e "${BLUE}📁 添加變更到暫存區...${NC}"
    git add .
    
    # 生成提交訊息
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    local commit_message="${commit_type}: ${stage_name}

${description}

⏰ 提交時間: ${timestamp}
🎯 階段性檢查點: 可用於恢復和比較
🔧 Generated with [Claude Code](https://claude.ai/code)

Co-Authored-By: Claude <noreply@anthropic.com>"

    # 執行提交
    echo -e "${GREEN}💾 執行提交...${NC}"
    if git commit -m "$commit_message"; then
        echo -e "${GREEN}✅ 提交成功: ${stage_name}${NC}"
        
        # 顯示提交雜湊
        local commit_hash=$(git rev-parse --short HEAD)
        echo -e "${PURPLE}🏷️  提交 ID: ${commit_hash}${NC}"
        
        # 可選：顯示變更統計
        git show --stat --oneline HEAD | head -10
        
        return 0
    else
        echo -e "${RED}❌ 提交失敗${NC}"
        return 1
    fi
}

# 函數：快速提交（簡化版）
quick_commit() {
    local message="$1"
    local commit_type="${2:-fix}"
    
    echo -e "${BLUE}⚡ 快速提交: ${message}${NC}"
    
    git add .
    git commit -m "${commit_type}: ${message}

🔧 Generated with [Claude Code](https://claude.ai/code)

Co-Authored-By: Claude <noreply@anthropic.com>"
    
    if [[ $? -eq 0 ]]; then
        echo -e "${GREEN}✅ 快速提交完成${NC}"
        git log --oneline -1
    else
        echo -e "${RED}❌ 快速提交失敗${NC}"
    fi
}

# 函數：創建備份分支
create_backup_branch() {
    local branch_name="backup-$(date '+%Y%m%d-%H%M%S')"
    echo -e "${PURPLE}🌿 創建備份分支: ${branch_name}${NC}"
    
    git branch "$branch_name"
    if [[ $? -eq 0 ]]; then
        echo -e "${GREEN}✅ 備份分支創建完成: ${branch_name}${NC}"
        echo -e "${CYAN}📝 恢復指令: git checkout ${branch_name}${NC}"
    else
        echo -e "${RED}❌ 備份分支創建失敗${NC}"
    fi
}

# 函數：顯示提交歷史
show_recent_commits() {
    local count="${1:-10}"
    echo -e "${CYAN}📚 最近 ${count} 次提交:${NC}"
    git log --oneline -n "$count" --graph --decorate
}

# 函數：比較變更
compare_changes() {
    local from_commit="${1:-HEAD~1}"
    local to_commit="${2:-HEAD}"
    
    echo -e "${CYAN}🔍 比較變更 ${from_commit}..${to_commit}:${NC}"
    git diff --stat "$from_commit" "$to_commit"
    
    echo -e "\n${YELLOW}詳細差異預覽 (前20行):${NC}"
    git diff "$from_commit" "$to_commit" | head -20
}

# 函數：恢復到指定提交
rollback_to_commit() {
    local commit_hash="$1"
    
    if [[ -z "$commit_hash" ]]; then
        echo -e "${RED}❌ 請提供提交雜湊值${NC}"
        echo -e "${CYAN}💡 使用方式: rollback_to_commit <commit_hash>${NC}"
        show_recent_commits 5
        return 1
    fi
    
    echo -e "${YELLOW}⚠️  警告: 即將恢復到提交 ${commit_hash}${NC}"
    echo -e "${CYAN}這將會丟失當前未提交的變更！${NC}"
    echo -e "${YELLOW}是否繼續? (y/N)${NC}"
    read -r confirm
    
    if [[ "$confirm" == "y" || "$confirm" == "Y" ]]; then
        create_backup_branch
        git reset --hard "$commit_hash"
        echo -e "${GREEN}✅ 已恢復到提交 ${commit_hash}${NC}"
    else
        echo -e "${BLUE}取消恢復操作${NC}"
    fi
}

# 函數：顯示幫助
show_help() {
    echo -e "${CYAN}🚀 Auto Commit Helper - 使用指南${NC}"
    echo ""
    echo -e "${YELLOW}主要函數:${NC}"
    echo -e "  ${GREEN}auto_commit_stage${NC} '<階段名稱>' '<描述>' [commit_type]"
    echo -e "  ${GREEN}quick_commit${NC} '<訊息>' [commit_type]"
    echo -e "  ${GREEN}create_backup_branch${NC}"
    echo -e "  ${GREEN}show_recent_commits${NC} [數量]"
    echo -e "  ${GREEN}compare_changes${NC} [from_commit] [to_commit]"
    echo -e "  ${GREEN}rollback_to_commit${NC} <commit_hash>"
    echo ""
    echo -e "${YELLOW}範例:${NC}"
    echo -e "  ${BLUE}auto_commit_stage${NC} 'Flow Designer 節點修復' '修復節點連接問題和視覺渲染'"
    echo -e "  ${BLUE}quick_commit${NC} '修復小錯誤'"
    echo -e "  ${BLUE}show_recent_commits${NC} 5"
    echo -e "  ${BLUE}compare_changes${NC} HEAD~2 HEAD"
    echo ""
    echo -e "${YELLOW}環境變數:${NC}"
    echo -e "  ${PURPLE}AUTO_COMMIT_INTERACTIVE${NC}=true  # 啟用交互模式"
}

# 主程序
case "${1:-help}" in
    stage)
        auto_commit_stage "$2" "$3" "$4"
        ;;
    quick)
        quick_commit "$2" "$3"
        ;;
    backup)
        create_backup_branch
        ;;
    commits|log)
        show_recent_commits "$2"
        ;;
    compare|diff)
        compare_changes "$2" "$3"
        ;;
    rollback)
        rollback_to_commit "$2"
        ;;
    help|*)
        show_help
        ;;
esac