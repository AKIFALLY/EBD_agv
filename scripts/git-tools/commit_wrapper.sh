#!/bin/bash

# Claude Code 自動提交包裝器
# 簡化 Claude Code 中的提交操作

# 載入自動提交輔助腳本
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/auto_commit_helper.sh"

# 階段性自動提交函數
commit_stage() {
    local stage="$1"
    local desc="$2" 
    local type="${3:-feat}"
    
    echo "🤖 Claude Code 自動提交階段: $stage"
    auto_commit_stage "$stage" "$desc" "$type"
}

# 快速修復提交
commit_fix() {
    local message="$1"
    echo "🔧 Claude Code 快速修復提交: $message"
    quick_commit "$message" "fix"
}

# 功能完成提交
commit_feat() {
    local message="$1"
    echo "✨ Claude Code 功能提交: $message"
    quick_commit "$message" "feat"
}

# 重構提交
commit_refactor() {
    local message="$1"
    echo "♻️ Claude Code 重構提交: $message"
    quick_commit "$message" "refactor"
}

# 樣式修復提交
commit_style() {
    local message="$1" 
    echo "💄 Claude Code 樣式提交: $message"
    quick_commit "$message" "style"
}

# 文檔更新提交
commit_docs() {
    local message="$1"
    echo "📚 Claude Code 文檔提交: $message"
    quick_commit "$message" "docs"
}

# 測試相關提交
commit_test() {
    local message="$1"
    echo "🧪 Claude Code 測試提交: $message"
    quick_commit "$message" "test"
}

# 效能優化提交
commit_perf() {
    local message="$1"
    echo "⚡ Claude Code 效能提交: $message"
    quick_commit "$message" "perf"
}

# 顯示可用的提交類型
show_commit_types() {
    echo "🚀 Claude Code 自動提交類型:"
    echo "  commit_stage '<階段>' '<描述>' [類型]  - 階段性提交"
    echo "  commit_fix '<訊息>'                  - 錯誤修復"
    echo "  commit_feat '<訊息>'                 - 新功能"
    echo "  commit_refactor '<訊息>'             - 程式碼重構"
    echo "  commit_style '<訊息>'                - 樣式調整"
    echo "  commit_docs '<訊息>'                 - 文檔更新"
    echo "  commit_test '<訊息>'                 - 測試相關"
    echo "  commit_perf '<訊息>'                 - 效能優化"
}

# 根據參數執行對應的提交
case "${1:-help}" in
    stage)
        commit_stage "$2" "$3" "$4"
        ;;
    fix)
        commit_fix "$2"
        ;;
    feat)
        commit_feat "$2"
        ;;
    refactor)
        commit_refactor "$2"
        ;;
    style)
        commit_style "$2"
        ;;
    docs)
        commit_docs "$2"
        ;;
    test)
        commit_test "$2"
        ;;
    perf)
        commit_perf "$2"
        ;;
    help|*)
        show_commit_types
        ;;
esac