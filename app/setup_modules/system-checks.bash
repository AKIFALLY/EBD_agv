#!/bin/bash
# RosAGV System Checks Module
# 包含所有系統狀態檢查和診斷函數

# ============================================================================
# ROS 環境檢查函數
# ============================================================================

check_ros_env() {
    echo "🔍 檢查 ROS 2 環境..."

    # 檢查 ROS_DISTRO
    if [ -n "$ROS_DISTRO" ]; then
        echo "✅ ROS_DISTRO: $ROS_DISTRO"
    else
        echo "❌ ROS_DISTRO 未設定"
        return 1
    fi

    # 檢查 RMW_IMPLEMENTATION
    if [ "$RMW_IMPLEMENTATION" = "rmw_zenoh_cpp" ]; then
        echo "✅ RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
    else
        echo "⚠️ RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-未設定} (預期: rmw_zenoh_cpp)"
    fi

    # 檢查 ros2 指令
    if command -v ros2 > /dev/null; then
        echo "✅ ros2 指令可用"
    else
        echo "❌ ros2 指令不可用"
        return 1
    fi

    # 檢查 Python 虛擬環境
    if [ -n "$VIRTUAL_ENV" ]; then
        echo "✅ Python 虛擬環境: $VIRTUAL_ENV"
    else
        echo "⚠️ Python 虛擬環境未啟用"
    fi

    echo "✅ ROS 2 環境檢查完成"
}

# 檢查 Zenoh 狀態
check_zenoh_status() {
    echo "🔍 檢查 Zenoh Router 狀態..."

    # 檢查程序是否運行
    if pgrep -f rmw_zenohd > /dev/null; then
        local pid=$(pgrep -f rmw_zenohd)
        echo "✅ Zenoh Router 運行中 (PID: $pid)"
    else
        echo "❌ Zenoh Router 未運行"
        return 1
    fi

    # 檢查 PID 檔案
    if [ -f "/tmp/zenoh_router.pid" ]; then
        local stored_pid=$(cat /tmp/zenoh_router.pid)
        echo "✅ PID 檔案存在: $stored_pid"
    else
        echo "⚠️ PID 檔案不存在"
    fi

    # 檢查日誌檔案
    if [ -f "/tmp/zenoh_router.log" ]; then
        echo "✅ 日誌檔案存在"
        echo "📝 最近日誌:"
        tail -3 /tmp/zenoh_router.log | sed 's/^/   /'
    else
        echo "⚠️ 日誌檔案不存在"
    fi

    # 檢查配置檔案
    if [ -f "/app/routerconfig.json5" ]; then
        echo "✅ Zenoh 配置檔案存在"
    else
        echo "❌ Zenoh 配置檔案不存在"
        return 1
    fi

    # 檢查端口
    if ss -tuln 2>/dev/null | rg ":7447 " > /dev/null; then
        echo "✅ Zenoh Router 端口 7447 已開啟"
    else
        echo "⚠️ Zenoh Router 端口 7447 未開啟"
    fi

    echo "✅ Zenoh 狀態檢查完成"
}

# 系統整體狀態檢查
check_system_status() {
    echo "🔍 系統整體狀態檢查..."
    echo "========================================"

    # 基礎服務檢查
    echo ""
    echo "=== 基礎服務 ==="

    # SSH 服務
    if pgrep -f sshd > /dev/null; then
        echo "✅ SSH 服務運行中"
    else
        echo "❌ SSH 服務未運行"
    fi

    # Zenoh Router
    check_zenoh_status

    # ROS 2 環境
    echo ""
    echo "=== ROS 2 環境 ==="
    check_ros_env

    # 工作空間狀態
    echo ""
    echo "=== 工作空間狀態 ==="
    local built_count=0
    local total_count=0

    for dir in /app/*_ws; do
        if [ -d "$dir" ]; then
            ((total_count++))
            local ws_name=$(basename "$dir")
            if [ -d "$dir/install" ]; then
                echo "✅ $ws_name (已建置)"
                ((built_count++))
            else
                echo "⚠️ $ws_name (未建置)"
            fi
        fi
    done

    echo ""
    echo "========================================"
    echo "📊 工作空間統計: $built_count/$total_count 已建置"
    echo "✅ 系統狀態檢查完成"
}

# 自動啟動 Zenoh Router
manage_zenoh start

# ===== 便捷別名 =====

# 為常用指令創建別名
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'
alias ..='cd ..'
alias ...='cd ../..'

# ROS 2 相關別名
alias rn='ros2 node list'
alias rt='ros2 topic list'
alias rs='ros2 service list'
alias ri='ros2 interface list'

# 工作空間管理別名
alias ba='build_all'
alias bas='build_all_smart'    # 依賴解析建置 (使用 colcon 依賴解析)
alias ta='test_all'
alias ca='clean_all'
alias sa='all_source'
alias load_all='all_source'  # 簡化的載入指令

# 專用工作空間載入別名
alias agv='agv_source'       # AGV 專用工作空間載入
alias agvc='agvc_source'     # AGVC 專用工作空間載入

# 單一工作空間操作別名
alias build1='build_single'
alias test1='test_single'
alias clean1='clean_single'

# 幫助和狀態別名
alias help='show_help'
alias status='check_system_status'

# 狀態檢查別名
alias status='check_system_status'
alias zenoh='check_zenoh_status'
alias rosenv='check_ros_env'

# 別名載入提示 (僅在互動式 shell 中顯示)
if [[ $- == *i* ]]; then
    log_debug "別名已載入: status, zenoh, rosenv, help, build1, test1, clean1, agv, agvc"
fi

# ===== AGVC 專用函數 =====

# 檢測當前是否為 AGVC 環境

# ===== 統一節點管理系統 =====

# 管理 PLC 服務節點 (AGVC)
