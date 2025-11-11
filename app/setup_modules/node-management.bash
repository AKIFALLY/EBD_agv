#!/bin/bash
# RosAGV Node Management Module
# 包含所有節點管理和服務控制函數

# =============================================================================
# 📚 節點啟動驗證方法決策樹
# =============================================================================
# 本模組使用 3 種不同的節點驗證方法，每種方法都有其適用場景和技術原因。
# 這些方法定義在 setup_modules/common.bash 中，提供統一的重試邏輯。
#
# ┌─────────────────────────────────────────────────────────────────────┐
# │ 驗證方法選擇指南                                                    │
# └─────────────────────────────────────────────────────────────────────┘
#
# 方法 1️⃣: verify_ros2_node_startup() - ROS 2 節點列表驗證
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 原理: 使用 `ros2 node list | grep "node_name"` 檢查節點是否註冊到 ROS 2 網路
# 檢測時間: 1-3 秒（需要等待節點完成 ROS 2 網路註冊）
#
# 適用場景:
#   ✅ ros2 run 啟動的單一節點（有明確命名空間）
#   ✅ ros2 launch 產生的命名節點（例如 tafl_wcs_node）
#   ✅ 核心基礎設施服務（需要確認 ROS 2 通訊正常）
#
# 為何使用:
#   • 不只檢查進程存在，還確認節點已註冊到 ROS 2 網路
#   • 可以偵測「進程活著但 ROS 2 網路有問題」的情況
#   • 對於核心服務（PLC、ECS、Database），這種深度驗證是必要的
#
# 使用範例:
#   manage_plc_service_agvc    - 核心 PLC 服務，需要 ROS 2 網路驗證
#   manage_ecs_core            - 核心 ECS 服務，需要 ROS 2 網路驗證
#   manage_tafl_wcs            - TAFL WCS 節點，launch 產生的命名節點
#   manage_agvc_database_node  - 資料庫代理節點，核心服務
#   manage_room_task_build     - 任務建置節點，需要網路驗證
#
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#
# 方法 2️⃣: verify_process_startup() - 進程模式匹配驗證
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 原理: 使用 `pgrep -f "process_pattern"` 檢查進程是否存在
# 檢測時間: < 100ms（即時檢測，查詢 /proc 文件系統）
#
# 適用場景:
#   ✅ ros2 launch 啟動的複雜多進程服務（檢查 launch 進程）
#   ✅ 原生進程服務（非 ROS 2 節點，如 Zenoh Router、SSH）
#   ✅ 當 ROS 2 網路可能不穩定時的快速檢查
#
# 為何使用:
#   • 即時響應，無需等待 ROS 2 網路註冊
#   • 適合 launch 檔案（會產生父子多進程）
#   • 無 ROS 2 網路依賴，即使 Zenoh 有問題也能檢測
#
# 使用範例:
#   manage_rcs_core           - RCS launch 檔案，檢查 rcs_launch.py 進程
#   manage_web_agv_launch     - Web AGV launch，檢查 agv_ui_server 進程
#
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#
# 方法 3️⃣: PID 檔案 + kill -0 驗證（manage_web_api_launch 特殊處理）
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 原理: 記錄所有子進程 PID 到檔案，使用 `kill -0 $pid` 驗證每個進程
# 檢測時間: < 50ms（最快，直接系統調用）
#
# 適用場景:
#   ✅ 多進程服務群組（需要追蹤多個 PID）
#   ✅ 需要精確控制每個子進程的場景
#   ✅ 服務重啟時需要確保所有舊進程都已清理
#
# 為何使用:
#   • 精確追蹤多個相關進程（例如 Web API Launch 啟動 3 個伺服器）
#   • 狀態持久化（PID 檔案可在重啟後檢查）
#   • 可以檢測部分進程掛掉的情況（如 3 個進程只活 2 個）
#
# 使用範例:
#   manage_web_api_launch     - 啟動 3 個 Web 伺服器（8000, 8001, 8002）
#   manage_zenoh              - Zenoh Router 進程管理
#
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#
# 🎯 超時建議值
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 節點類型                          建議超時    原因
# ────────────────────────────────────────────────────────────────────
# ros2 run (單一節點)                 10 秒     節點註冊通常需要 2-5 秒
# ros2 run (with namespace)           10 秒     命名空間增加的開銷很小
# ros2 launch (簡單，單節點)          10 秒     Launch 開銷 + 節點註冊
# ros2 launch (複雜，多節點)          15 秒     多節點 + 依賴關係初始化
# 原生進程（無 ROS 2）                 5 秒     無 ROS 2 開銷，啟動快
# 舊系統或重量級服務               15-20 秒     可能有複雜初始化邏輯
# ────────────────────────────────────────────────────────────────────
#
# 📖 決策樹使用範例
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Q: 我要啟動一個 ros2 run 的節點，有 namespace，應該用什麼方法？
# A: 使用 verify_ros2_node_startup "/namespace/node_name" 10
#
# Q: 我要啟動一個 ros2 launch 檔案，會產生多個節點，應該用什麼方法？
# A: 使用 verify_process_startup "launch_file.py" 10
#    （檢查 launch 進程，而非個別節點）
#
# Q: 我要啟動的服務不是 ROS 2 節點（如 Zenoh），應該用什麼方法？
# A: 使用 verify_process_startup "process_name" 5
#    （原生進程通常啟動較快）
#
# Q: 我需要同時檢查進程和 ROS 2 註冊，應該用什麼方法？
# A: 使用 verify_node_startup_combined "$PID" "/node/name" "process_pattern" 15
#    （最全面但也最慢的驗證）
#
# =============================================================================

# =============================================================================
# 🔌 端口監聽驗證函式
# =============================================================================
# 動態檢查端口是否開啟，帶重試機制
# 用於等待 Web 服務等網路服務完全啟動
wait_for_port_with_retry() {
    local port=$1
    local max_wait_seconds=${2:-30}  # 預設最多等待 30 秒
    local retry_interval=1           # 每秒檢查一次
    local elapsed=0

    while [ $elapsed -lt $max_wait_seconds ]; do
        # 使用 ss 檢查端口（比 lsof 更通用）
        if ss -tuln 2>/dev/null | grep -q ":$port "; then
            return 0  # 端口已開啟
        fi

        # 顯示等待進度（每 2 秒顯示一次，避免刷屏）
        if [ $((elapsed % 2)) -eq 0 ] || [ $elapsed -eq 0 ]; then
            echo "⏳ 等待端口 $port 開啟... ($elapsed/$max_wait_seconds 秒)"
        fi

        sleep $retry_interval
        elapsed=$((elapsed + retry_interval))
    done

    return 1  # 超時失敗
}

# =============================================================================
# 🧹 清理臨時文件函式
# =============================================================================
cleanup_temp_files() {
    local dry_run=false
    local force=false
    local all=false
    local days=7  # 預設清理 7 天前的文件

    # 解析參數
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --dry-run)
                dry_run=true
                shift
                ;;
            --force)
                force=true
                days=1  # force 模式清理 1 天前的
                shift
                ;;
            --all)
                all=true
                days=0  # all 模式清理所有（但保護正在運行的）
                shift
                ;;
            --days)
                days="$2"
                shift 2
                ;;
            --help|-h)
                echo "用法: cleanup_temp_files [選項]"
                echo ""
                echo "選項:"
                echo "  --dry-run       模擬執行，不實際刪除文件"
                echo "  --force         強制清理（清理 1 天前的文件）"
                echo "  --all           清理所有臨時文件（保護正在運行的進程）"
                echo "  --days N        清理 N 天前的文件（預設: 7）"
                echo "  --help, -h      顯示此幫助訊息"
                echo ""
                echo "範例:"
                echo "  cleanup_temp_files                # 清理 7 天前的臨時文件"
                echo "  cleanup_temp_files --dry-run      # 模擬清理"
                echo "  cleanup_temp_files --force        # 清理 1 天前的文件"
                echo "  cleanup_temp_files --all          # 清理所有文件"
                echo "  cleanup_temp_files --days 3       # 清理 3 天前的文件"
                return 0
                ;;
            *)
                log_error "未知參數: $1"
                echo "使用 --help 查看用法"
                return 1
                ;;
        esac
    done

    # 顯示模式
    if [ "$dry_run" = true ]; then
        log_info "🔍 模擬模式（dry-run）- 不會實際刪除文件"
    fi

    if [ "$all" = true ]; then
        log_warning "⚠️  ALL 模式 - 將清理所有臨時文件（保護運行中的進程）"
    elif [ "$force" = true ]; then
        log_warning "⚠️  FORCE 模式 - 將清理 1 天前的文件"
    else
        log_info "🧹 標準清理模式 - 清理 $days 天前的文件"
    fi

    local total_cleaned=0
    local total_space=0

    # -------------------------------------------------------------------------
    # 1. 清理 launch_params_* 目錄
    # -------------------------------------------------------------------------
    echo ""
    log_info "📂 檢查 launch_params_* 目錄..."

    if [ "$all" = true ]; then
        # ALL 模式：清理所有
        local launch_dirs=$(find /tmp -maxdepth 1 -name 'launch_params_*' -type d 2>/dev/null)
    else
        # 標準模式：按天數清理
        local launch_dirs=$(find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime +"$days" 2>/dev/null)
    fi

    local launch_count=$(echo "$launch_dirs" | grep -c '^' 2>/dev/null || echo "0")

    if [ "$launch_count" -gt 0 ]; then
        if [ "$dry_run" = true ]; then
            echo "  [DRY-RUN] 將清理 $launch_count 個 launch_params 目錄"
            echo "$launch_dirs" | head -5
            if [ "$launch_count" -gt 5 ]; then
                echo "  ... 還有 $((launch_count - 5)) 個目錄"
            fi
        else
            echo "$launch_dirs" | xargs rm -rf 2>/dev/null
            log_success "✅ 清理 $launch_count 個 launch_params 目錄"
            total_cleaned=$((total_cleaned + launch_count))
        fi
    else
        echo "  ℹ️  無需清理的 launch_params 目錄"
    fi

    # -------------------------------------------------------------------------
    # 2. 清理孤立的 PID 文件
    # -------------------------------------------------------------------------
    echo ""
    log_info "🔍 檢查孤立的 PID 文件..."

    local orphan_count=0
    for pid_file in /tmp/*.pid; do
        if [ -f "$pid_file" ]; then
            local all_dead=true
            local dead_pids=""
            local running_pids=""

            # 逐行讀取 PID 文件，正確處理多行 PID
            while read -r pid; do
                # 跳過空行
                [ -z "$pid" ] && continue

                # 檢查每個 PID 是否在運行
                if kill -0 "$pid" 2>/dev/null; then
                    all_dead=false
                    running_pids="$running_pids $pid"
                else
                    dead_pids="$dead_pids $pid"
                fi
            done < "$pid_file"

            # 只有當所有 PID 都不運行時才刪除文件
            if [ "$all_dead" = true ] && [ -n "$dead_pids" ]; then
                if [ "$dry_run" = true ]; then
                    echo "  [DRY-RUN] 將清理: $(basename $pid_file) (PID:$dead_pids 不存在)"
                else
                    rm -f "$pid_file"
                    echo "  ✅ 清理: $(basename $pid_file) (PID:$dead_pids 不存在)"
                fi
                orphan_count=$((orphan_count + 1))
            elif [ -n "$running_pids" ]; then
                # 有部分 PID 在運行，保留文件（可選：顯示調試信息）
                if [ "$dry_run" = true ]; then
                    echo "  ℹ️  保留: $(basename $pid_file) (部分 PID$running_pids 仍在運行)"
                fi
            fi
        fi
    done

    if [ "$orphan_count" -eq 0 ]; then
        echo "  ℹ️  無孤立的 PID 文件"
    else
        if [ "$dry_run" = false ]; then
            log_success "✅ 清理 $orphan_count 個孤立的 PID 文件"
            total_cleaned=$((total_cleaned + orphan_count))
        fi
    fi

    # -------------------------------------------------------------------------
    # 3. 清理過期的日誌文件
    # -------------------------------------------------------------------------
    echo ""
    log_info "📄 檢查過期的日誌文件..."

    if [ "$all" = true ]; then
        # ALL 模式：清理所有（但保護特定的系統日誌）
        local old_logs=$(find /tmp -maxdepth 1 -name '*.log' -type f ! -name 'web_api_launch.log' ! -name 'zenoh_router.log' 2>/dev/null)
    else
        # 標準模式：按天數清理
        local old_logs=$(find /tmp -maxdepth 1 -name '*.log' -type f -mtime +"$days" 2>/dev/null)
    fi

    local log_count=$(echo "$old_logs" | grep -c '^' 2>/dev/null || echo "0")

    if [ "$log_count" -gt 0 ]; then
        if [ "$dry_run" = true ]; then
            echo "  [DRY-RUN] 將清理 $log_count 個日誌文件"
            echo "$old_logs" | head -5 | xargs -I {} basename {}
            if [ "$log_count" -gt 5 ]; then
                echo "  ... 還有 $((log_count - 5)) 個日誌"
            fi
        else
            echo "$old_logs" | xargs rm -f 2>/dev/null
            log_success "✅ 清理 $log_count 個過期日誌文件"
            total_cleaned=$((total_cleaned + log_count))
        fi
    else
        echo "  ℹ️  無需清理的日誌文件"
    fi

    # -------------------------------------------------------------------------
    # 4. 清理其他臨時文件（node-compile-cache 等）
    # -------------------------------------------------------------------------
    if [ "$all" = true ] || [ "$force" = true ]; then
        echo ""
        log_info "🗑️  檢查其他臨時緩存..."

        # 清理 Node.js 編譯緩存
        if [ -d "/tmp/node-compile-cache" ]; then
            local cache_size=$(du -sh /tmp/node-compile-cache 2>/dev/null | cut -f1)
            if [ "$dry_run" = true ]; then
                echo "  [DRY-RUN] 將清理 node-compile-cache ($cache_size)"
            else
                rm -rf /tmp/node-compile-cache
                echo "  ✅ 清理 node-compile-cache ($cache_size)"
                total_cleaned=$((total_cleaned + 1))
            fi
        fi
    fi

    # -------------------------------------------------------------------------
    # 統計報告
    # -------------------------------------------------------------------------
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    if [ "$dry_run" = true ]; then
        log_info "📊 模擬清理統計 (DRY-RUN):"
        echo "  • 預計清理項目數: $total_cleaned"
    else
        log_success "📊 清理完成統計:"
        echo "  • 已清理項目數: $total_cleaned"
    fi
    echo "  • 清理條件: $days 天前的文件"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    # 顯示當前 /tmp 目錄使用情況
    echo ""
    log_info "💾 當前 /tmp 目錄使用情況:"
    du -sh /tmp 2>/dev/null | awk '{print "  • 總使用空間: " $1}'
    echo "  • launch_params 目錄數: $(find /tmp -maxdepth 1 -name 'launch_params_*' -type d 2>/dev/null | wc -l)"
    echo "  • PID 文件數: $(find /tmp -maxdepth 1 -name '*.pid' -type f 2>/dev/null | wc -l)"
    echo "  • 日誌文件數: $(find /tmp -maxdepth 1 -name '*.log' -type f 2>/dev/null | wc -l)"
}

# 別名
alias ct='cleanup_temp_files'
alias cleanup_temp='cleanup_temp_files'


# ============================================================================
# 基礎服務管理函數
# ============================================================================

manage_ssh() {
    case "$1" in
        start)
            if ! pgrep -f "sshd" > /dev/null; then
                echo "🚀 啟動 SSH 服務..."
                service ssh start

                # 動態驗證服務啟動（最多等待 5 秒）
                echo "🔍 驗證 SSH 服務..."
                if verify_process_startup "sshd" 5; then
                    echo "✅ SSH 服務已啟動"
                else
                    echo "❌ SSH 服務啟動失敗（5 秒超時）"
                    echo "💡 診斷建議:"
                    echo "   1. 檢查 SSH 套件: dpkg -l | grep openssh-server"
                    echo "   2. 查看系統日誌: journalctl -u ssh -n 20"
                    return 1
                fi
            else
                echo "✅ SSH 服務已經在運行中"
            fi
            ;;
        stop)
            if pgrep -f "sshd" > /dev/null; then
                echo "⏳ 停止 SSH 服務..."
                service ssh stop
                echo "✅ SSH 服務已停止"
            else
                echo "❌ SSH 服務未運行"
            fi
            ;;
        restart)
            if pgrep -f "sshd" > /dev/null; then
                echo "🔄 重新啟動 SSH 服務..."
                service ssh restart
                echo "✅ SSH 服務已重新啟動"
            else
                echo "❌ SSH 服務未運行，無法重新啟動"
            fi
            ;;
        status)
            if pgrep -f "sshd" > /dev/null; then
                echo "✅ SSH 服務正在運行"
            else
                echo "❌ SSH 服務未運行"
            fi
            ;;
        *)
            echo "用法: manage_ssh {start|stop|restart|status}"
            return 1
            ;;
    esac
}

# ===== Zenoh Router 控制函式 =====
manage_zenoh() {
    ZENOH_LOG_FILE="/tmp/zenoh_router.log"
    ZENOH_PID_FILE="/tmp/zenoh_router.pid"

    case "$1" in
        start)
            # 檢查 PID 檔案是否存在且進程仍在運行
            if [ -f "$ZENOH_PID_FILE" ]; then
                # 檢查檔案中記錄的所有進程是否還在運行
                local all_running=true
                while read pid; do
                    if ! kill -0 $pid 2>/dev/null; then
                        all_running=false
                        break
                    fi
                done < "$ZENOH_PID_FILE"
                
                if [ "$all_running" = true ]; then
                    echo "✅ Zenoh Router 已經在運行中"
                    echo "   PID: $(cat $ZENOH_PID_FILE | tr '\n' ' ')"
                    return 0
                else
                    # 清理過時的 PID 檔案
                    rm -f "$ZENOH_PID_FILE"
                fi
            fi
            
            # 啟動新的 Zenoh Router
            echo "🚀 啟動 Zenoh Router..."
            nohup ros2 run rmw_zenoh_cpp rmw_zenohd > "$ZENOH_LOG_FILE" 2>&1 &
            local PARENT_PID=$!

            # 記錄父進程
            echo $PARENT_PID > "$ZENOH_PID_FILE"

            # 動態驗證進程啟動（最多等待 10 秒）
            echo "🔍 驗證 Zenoh Router 進程..."
            if verify_process_startup "rmw_zenohd" 10; then
                echo "✅ Zenoh Router 進程已啟動"

                # 找出所有子進程並記錄
                local CHILD_PIDS=$(pgrep -P $PARENT_PID)
                if [ -n "$CHILD_PIDS" ]; then
                    for pid in $CHILD_PIDS; do
                        echo $pid >> "$ZENOH_PID_FILE"
                    done
                fi

                # 動態驗證端口開啟（確保完全就緒）
                echo "🔍 驗證 Zenoh Router 端口 7447..."
                if wait_for_port_with_retry 7447 10; then
                    echo "✅ Zenoh Router 端口 7447 已開啟"
                    echo "   記錄的 PID: $(cat $ZENOH_PID_FILE | tr '\n' ' ')"
                else
                    echo "⚠️ Zenoh Router 端口 7447 未開啟（10 秒超時）"
                    echo "💡 進程已啟動但端口未綁定，可能配置有誤"
                    echo "   查看日誌: tail -f $ZENOH_LOG_FILE"
                    # 不返回錯誤，因為進程已啟動（某些配置可能不綁定端口）
                fi
            else
                echo "❌ Zenoh Router 進程啟動失敗（10 秒超時）"
                echo "💡 診斷建議:"
                echo "   1. 查看日誌: tail -f $ZENOH_LOG_FILE"
                echo "   2. 檢查 ROS 2 環境: echo \$ROS_DISTRO"
                echo "   3. 檢查 rmw_zenoh_cpp 套件: ros2 pkg list | grep zenoh"
                rm -f "$ZENOH_PID_FILE"
                return 1
            fi
            ;;

        stop)
            if [ -f "$ZENOH_PID_FILE" ]; then
                echo "⏳ 停止 Zenoh Router..."
                
                # 讀取並殺掉所有記錄的進程（先殺子進程，再殺父進程）
                local PIDS=$(tac "$ZENOH_PID_FILE")  # 反向讀取（先子後父）
                for pid in $PIDS; do
                    if kill -0 $pid 2>/dev/null; then
                        echo "   停止進程 PID: $pid"
                        kill $pid 2>/dev/null
                    fi
                done
                
                # 等待進程結束
                sleep 2
                
                # 強制終止仍在運行的進程
                for pid in $PIDS; do
                    if kill -0 $pid 2>/dev/null; then
                        echo "   強制終止 PID: $pid"
                        kill -9 $pid 2>/dev/null
                    fi
                done
                
                rm -f "$ZENOH_PID_FILE"
                echo "✅ Zenoh Router 已停止"
            else
                # 確保停止所有與 Zenoh Router 相關的進程
                echo "⚠️ PID 檔案不存在，嘗試清理所有 rmw_zenohd 進程..."
                if pgrep -f "rmw_zenohd" > /dev/null; then
                    echo "   找到 rmw_zenohd 進程，正在停止..."
                    pkill -f "rmw_zenohd"
                    sleep 2
                    echo "✅ Zenoh Router 進程已停止"
                else
                    echo "ℹ️ Zenoh Router 未運行"
                fi
            fi

            # 檢查端口 7447 是否仍被佔用，並強制釋放
            if lsof -i :7447 > /dev/null; then
                echo "🚨 端口 7447 仍然被占用，強制釋放..."
                # 查找佔用該端口的進程並強制終止
                lsof -i :7447 | awk 'NR>1 {print $2}' | xargs kill -9
                sleep 2
                echo "✅ 端口 7447 已強制釋放"
            else
                echo "✅ 端口 7447 沒有被占用"
            fi
            ;;

        restart)
            echo "🔄 重新啟動 Zenoh Router..."
            manage_zenoh stop
            sleep 2
            manage_zenoh start
            ;;

        status)
            if [ -f "$ZENOH_PID_FILE" ]; then
                # 檢查所有記錄的進程
                local all_running=true
                local running_pids=""
                while read pid; do
                    if kill -0 $pid 2>/dev/null; then
                        running_pids="$running_pids $pid"
                    else
                        all_running=false
                    fi
                done < "$ZENOH_PID_FILE"
                
                if [ -n "$running_pids" ]; then
                    echo "✅ Zenoh Router 正在運行"
                    echo "   運行中的 PID:$running_pids"
                    if [ "$all_running" = false ]; then
                        echo "   ⚠️ 部分進程已停止"
                    fi
                else
                    echo "❌ Zenoh Router 未運行（進程已停止）"
                    rm -f "$ZENOH_PID_FILE"
                fi
            else
                echo "❌ Zenoh Router 未運行"
            fi
            ;;

        *)
            echo "用法: manage_zenoh {start|stop|restart|status}"
            return 1
            ;;
    esac
}

# ===== Web API Launch 控制函式 =====
manage_web_api_launch() {
    # 環境檢測：僅限 AGVC 容器
    if ! is_agvc_environment; then
        echo "❌ 此功能僅適用於 AGVC 環境"
        echo "💡 AGV 容器請使用: check_agv_status"
        return 1
    fi

    local WEB_API_LOG_FILE="/tmp/web_api_launch.log"
    local WEB_API_PID_FILE="/tmp/web_api_launch.pid"

    case "$1" in
        start)
            # 檢查 PID 檔案是否存在且進程仍在運行
            if [ -f "$WEB_API_PID_FILE" ]; then
                # 檢查檔案中記錄的所有進程是否還在運行
                local all_running=true
                while read pid; do
                    if ! kill -0 $pid 2>/dev/null; then
                        all_running=false
                        break
                    fi
                done < "$WEB_API_PID_FILE"
                
                if [ "$all_running" = true ]; then
                    echo "✅ Web API Launch 已經在運行中"
                    echo "   PID: $(cat $WEB_API_PID_FILE | tr '\n' ' ')"
                    return 0
                else
                    # 清理過時的 PID 檔案
                    echo "🧹 清理過時的 PID 檔案..."
                    rm -f "$WEB_API_PID_FILE"
                fi
            fi
            
            # 啟動前檢查環境
            echo "🔍 啟動前環境檢查..."
            
            # 檢查工作空間是否已建置
            if [ ! -d "/app/web_api_ws/install" ]; then
                echo "⚠️ 警告: web_api_ws 未建置，請先執行: build_ws web_api_ws"
            fi

            # 檢查並清理被佔用的端口
            local port_conflict=false
            for port in 8000 8001 8002; do
                if ss -tulnp 2>/dev/null | grep -q ":$port "; then
                    echo "⚠️ 端口 $port 被佔用，嘗試清理..."

                    # 提取 PID 並終止
                    local pids=$(ss -tulnp 2>/dev/null | grep ":$port " | grep -oP 'pid=\K[0-9]+' | sort -u)

                    if [ -n "$pids" ]; then
                        for pid in $pids; do
                            echo "   終止佔用端口 $port 的進程 PID: $pid"
                            kill -9 $pid 2>/dev/null
                        done
                        sleep 1

                        # 再次檢查端口是否釋放
                        if ss -tulnp 2>/dev/null | grep -q ":$port "; then
                            echo "❌ 端口 $port 清理失敗，請手動檢查: ss -tulnp | grep :$port"
                            port_conflict=true
                        else
                            echo "✅ 端口 $port 已清理"
                        fi
                    fi
                fi
            done

            # 如果有端口衝突無法解決，返回錯誤
            if [ "$port_conflict" = true ]; then
                echo "❌ 端口衝突無法解決，請手動清理後重試"
                return 1
            fi

            echo "🚀 啟動 Web API Launch 服務群組..."
            # 載入完整 AGVC 環境確保所有依賴可用
            # 注意：移除 -i 標志避免 job control 暫停問題
            nohup bash -c "source /app/setup.bash && agvc_source > /dev/null 2>&1 && ros2 launch web_api_launch launch.py" > "$WEB_API_LOG_FILE" 2>&1 &
            local PARENT_PID=$!
            
            # 記錄父進程
            echo $PARENT_PID > "$WEB_API_PID_FILE"
            
            # 等待子進程啟動
            sleep 5
            
            # 找出所有子進程並記錄（launch 會產生多個子進程）
            local CHILD_PIDS=$(pgrep -P $PARENT_PID)
            if [ -n "$CHILD_PIDS" ]; then
                for pid in $CHILD_PIDS; do
                    echo $pid >> "$WEB_API_PID_FILE"
                done
            fi
            
            # 也記錄實際的服務進程（agvc_ui_server, op_ui_server, api_server）
            sleep 2
            for service in "agvc_ui_server" "op_ui_server" "api_server"; do
                local SERVICE_PID=$(pgrep -f "$service" | head -n1)
                if [ -n "$SERVICE_PID" ]; then
                    # 檢查是否已經記錄（避免重複）
                    if ! grep -q "^$SERVICE_PID$" "$WEB_API_PID_FILE" 2>/dev/null; then
                        echo $SERVICE_PID >> "$WEB_API_PID_FILE"
                    fi
                fi
            done

            # 檢查是否正常啟動
            if kill -0 $PARENT_PID 2>/dev/null; then
                echo "✅ Web API Launch 已啟動"
                echo "   記錄的 PID: $(cat $WEB_API_PID_FILE | tr '\n' ' ')"

                # 動態等待端口開啟（最多 30 秒，自動重試）
                echo "🔍 等待 Web 服務端口開啟..."

                local port_check_failed=false

                # 檢查 Web API (8000)
                if wait_for_port_with_retry 8000 30; then
                    echo "✅ Web API 端口 8000 已開啟"
                else
                    echo "❌ Web API 端口 8000 等待超時（30 秒）"
                    port_check_failed=true
                fi

                # 檢查 AGVCUI (8001)
                if wait_for_port_with_retry 8001 30; then
                    echo "✅ AGVCUI 端口 8001 已開啟"
                else
                    echo "❌ AGVCUI 端口 8001 等待超時（30 秒）"
                    port_check_failed=true
                fi

                # 檢查 OPUI (8002)
                if wait_for_port_with_retry 8002 30; then
                    echo "✅ OPUI 端口 8002 已開啟"
                else
                    echo "❌ OPUI 端口 8002 等待超時（30 秒）"
                    port_check_failed=true
                fi

                # 如果有端口檢查失敗，提供詳細診斷資訊
                if [ "$port_check_failed" = true ]; then
                    echo ""
                    echo "💡 診斷建議:"
                    echo "   1. 查看詳細日誌: tail -f $WEB_API_LOG_FILE"
                    echo "   2. 檢查個別服務狀態:"
                    echo "      ps aux | grep -E '(agvc_ui_server|op_ui_server|api_server)'"
                    echo "   3. 檢查資料庫連線:"
                    echo "      docker compose -f docker-compose.agvc.yml ps postgres"
                    echo "   4. 手動啟動單個服務進行測試:"
                    echo "      python3 /app/web_api_ws/src/agvcui/agvcui/agvc_ui_server.py"
                    echo "      python3 /app/web_api_ws/src/opui/opui/core/op_ui_server.py"
                    echo "      python3 /app/web_api_ws/src/web_api/web_api/api_server.py"
                    echo ""
                    echo "⚠️ 注意: 如果進程存在但端口未開啟，可能是服務啟動失敗"
                fi
                
                return 0
            else
                echo "❌ Web API Launch 啟動失敗"
                echo "📝 檢查日誌: tail -f $WEB_API_LOG_FILE"
                echo "💡 可能的原因:"
                echo "   - 套件未正確建置 (執行: build_ws web_api_ws)"
                echo "   - Python 依賴未安裝"
                echo "   - 端口已被佔用 (檢查: ss -tuln | grep -E '800[0-2]')"
                return 1
            fi
            ;;

        stop)
            if [ -f "$WEB_API_PID_FILE" ]; then
                echo "⏳ 停止 Web API Launch 服務群組..."
                
                # 讀取並殺掉所有記錄的進程（反向順序：先子後父）
                local PIDS=$(tac "$WEB_API_PID_FILE")
                for pid in $PIDS; do
                    if kill -0 $pid 2>/dev/null; then
                        echo "   停止進程 PID: $pid"
                        kill $pid 2>/dev/null
                    fi
                done
                
                # 等待進程結束
                sleep 3
                
                # 強制終止仍在運行的進程
                for pid in $PIDS; do
                    if kill -0 $pid 2>/dev/null; then
                        echo "   強制終止 PID: $pid"
                        kill -9 $pid 2>/dev/null
                    fi
                done
                
                rm -f "$WEB_API_PID_FILE"

                # 清理當前會話的 launch_params 臨時目錄 (1 天內的)
                echo "🧹 清理臨時文件..."
                local launch_params_count=$(find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 2>/dev/null | wc -l)
                if [ "$launch_params_count" -gt 0 ]; then
                    find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 -exec rm -rf {} + 2>/dev/null
                    echo "   清理了 $launch_params_count 個 launch_params 臨時目錄"
                fi

                echo "✅ Web API Launch 已停止"
            else
                # 確保停止所有與 Web API Launch 相關的進程
                echo "🚨 Web API Launch PID 檔案未找到，檢查相關進程..."
                if pgrep -f "web_api_launch" > /dev/null || pgrep -f "agvc_ui_server" > /dev/null || pgrep -f "op_ui_server" > /dev/null; then
                    echo "⏳ 停止 Web API Launch 相關進程..."
                    pkill -f "web_api_launch"
                    pkill -f "agvc_ui_server"
                    pkill -f "op_ui_server"
                    pkill -f "api_server"
                    sleep 2
                    echo "✅ Web API Launch 相關進程已停止"
                else
                    echo "ℹ️ 未發現運行中的 Web API Launch 進程"
                fi

                # 清理可能存在的 PID 檔案
                rm -f "$WEB_API_PID_FILE"

                # 清理當前會話的 launch_params 臨時目錄
                echo "🧹 清理臨時文件..."
                local launch_params_count=$(find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 2>/dev/null | wc -l)
                if [ "$launch_params_count" -gt 0 ]; then
                    find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 -exec rm -rf {} + 2>/dev/null
                    echo "   清理了 $launch_params_count 個 launch_params 臨時目錄"
                fi
            fi

            # ===== 強制清理所有相關進程 (避免殘留) =====
            echo "🔍 檢查並清理殘留進程..."
            local found_residual=false

            for service in "agvc_ui_server" "op_ui_server" "api_server" "web_api_launch"; do
                if pgrep -f "$service" > /dev/null; then
                    echo "   發現殘留進程: $service"
                    pkill -9 -f "$service" 2>/dev/null
                    found_residual=true
                fi
            done

            if [ "$found_residual" = true ]; then
                sleep 2
                echo "   殘留進程已清理"
            fi

            # ===== 檢查並強制釋放端口 =====
            echo "🔍 檢查 Web 服務端口..."
            local port_released=false

            for port in 8000 8001 8002; do
                # 使用 ss 檢查端口（更通用）
                if ss -tulnp 2>/dev/null | grep -q ":$port "; then
                    echo "🚨 端口 $port 仍被佔用，強制釋放..."

                    # 提取 PID 並終止（ss 輸出格式: users:(("process",pid=12345,...))）
                    local pids=$(ss -tulnp 2>/dev/null | grep ":$port " | grep -oP 'pid=\K[0-9]+' | sort -u)

                    if [ -n "$pids" ]; then
                        for pid in $pids; do
                            echo "   終止佔用端口 $port 的進程 PID: $pid"
                            kill -9 $pid 2>/dev/null
                        done
                        port_released=true
                    fi
                fi
            done

            if [ "$port_released" = true ]; then
                sleep 2
                echo "✅ 端口已強制釋放"
            else
                echo "✅ 所有端口正常釋放"
            fi
            ;;

        restart)
            echo "🔄 重新啟動 Web API Launch..."
            manage_web_api_launch stop
            sleep 2
            manage_web_api_launch start
            ;;

        status)
            # 檢查實際服務進程狀態（不檢查 launch 父進程）
            local service_count=0
            local running_count=0

            # 檢查 AGVCUI 服務
            local agvcui_running=false
            local agvcui_pid=""
            local agvcui_port_status=""
            if pgrep -f "agvc_ui_server" > /dev/null; then
                agvcui_running=true
                agvcui_pid=$(pgrep -f 'agvc_ui_server' | head -n1)
                running_count=$((running_count + 1))
                if ss -tuln 2>/dev/null | grep -q ":8001 "; then
                    agvcui_port_status="端口: 8001 ✓"
                else
                    agvcui_port_status="端口: 8001 ✗"
                fi
            fi
            service_count=$((service_count + 1))

            # 檢查 OPUI 服務
            local opui_running=false
            local opui_pid=""
            local opui_port_status=""
            if pgrep -f "op_ui_server" > /dev/null; then
                opui_running=true
                opui_pid=$(pgrep -f 'op_ui_server' | head -n1)
                running_count=$((running_count + 1))
                if ss -tuln 2>/dev/null | grep -q ":8002 "; then
                    opui_port_status="端口: 8002 ✓"
                else
                    opui_port_status="端口: 8002 ✗"
                fi
            fi
            service_count=$((service_count + 1))

            # 檢查 Web API 服務
            local api_running=false
            local api_pid=""
            local api_port_status=""
            if pgrep -f "api_server" > /dev/null; then
                api_running=true
                api_pid=$(pgrep -f 'api_server' | head -n1)
                running_count=$((running_count + 1))
                if ss -tuln 2>/dev/null | grep -q ":8000 "; then
                    api_port_status="端口: 8000 ✓"
                else
                    api_port_status="端口: 8000 ✗"
                fi
            fi
            service_count=$((service_count + 1))

            # 顯示整體狀態
            if [ $running_count -eq $service_count ]; then
                echo "✅ Web API Launch 正在運行 ($running_count/$service_count 服務正常)"
            elif [ $running_count -gt 0 ]; then
                echo "⚠️ Web API Launch 部分服務異常 ($running_count/$service_count 服務正常)"
            else
                echo "❌ Web API Launch 未運行 (0/$service_count 服務)"
                if [ -f "$WEB_API_PID_FILE" ]; then
                    echo "💡 提示: PID 檔案存在但服務未運行，建議執行 'manage_web_api_launch restart'"
                fi
                return 1
            fi

            # 顯示各服務詳細狀態
            echo "🔍 服務狀態："
            if [ "$agvcui_running" = true ]; then
                echo "  ✅ AGVCUI 服務運行中 (PID: $agvcui_pid, $agvcui_port_status)"
            else
                echo "  ❌ AGVCUI 服務未運行"
            fi

            if [ "$opui_running" = true ]; then
                echo "  ✅ OPUI 服務運行中 (PID: $opui_pid, $opui_port_status)"
            else
                echo "  ❌ OPUI 服務未運行"
            fi

            if [ "$api_running" = true ]; then
                echo "  ✅ Web API 服務運行中 (PID: $api_pid, $api_port_status)"
            else
                echo "  ❌ Web API 服務未運行"
            fi

            # 如果有服務異常，提供診斷建議
            if [ $running_count -lt $service_count ]; then
                echo "💡 診斷建議:"
                echo "   1. 查看日誌: tail -f $WEB_API_LOG_FILE"
                echo "   2. 重啟服務: manage_web_api_launch restart"
            fi

            return 0
            ;;

        *)
            echo "用法: manage_web_api_launch {start|stop|restart|status}"
            return 1
            ;;
    esac
}

# ===== Web AGV Launch 控制函式 (使用 ROS 2 Launch) =====
manage_web_agv_launch() {
    # 環境檢測：僅限 AGVC 容器
    if ! is_agvc_environment; then
        echo "❌ 此功能僅適用於 AGVC 環境"
        echo "💡 AGV 容器請使用: check_agv_status"
        return 1
    fi

    local WEB_AGV_LOG_FILE="/tmp/web_agv_launch.log"
    local WEB_AGV_PID_FILE="/tmp/web_agv_launch.pid"

    case "$1" in
        start)
            # 檢查 PID 檔案是否存在且進程仍在運行
            if [ -f "$WEB_AGV_PID_FILE" ]; then
                local all_running=true
                while IFS= read -r pid; do
                    if ! kill -0 $pid 2>/dev/null; then
                        all_running=false
                        break
                    fi
                done < "$WEB_AGV_PID_FILE"
                
                if [ "$all_running" = true ]; then
                    echo "✅ Web AGV Launch 已經在運行中"
                    return 0
                else
                    echo "🧹 清理過時的 PID 檔案..."
                    rm -f "$WEB_AGV_PID_FILE"
                fi
            fi
            
            echo "🔍 啟動前環境檢查..."
            
            # 檢查工作空間是否已建置
            if [ ! -d "/app/web_api_ws/install/agvui" ]; then
                echo "⚠️ 警告: agvui 未建置，請先執行: build_ws web_api_ws"
            fi
            
            if [ ! -d "/app/launch_ws/install/web_agv_launch" ]; then
                echo "⚠️ 警告: web_agv_launch 未建置，請先執行: build_ws launch_ws"
            fi
            
            # 檢查並清理被佔用的端口
            if ss -tulnp 2>/dev/null | grep -q ":8003 "; then
                echo "⚠️ 端口 8003 被佔用，嘗試清理..."

                # 提取 PID 並終止
                local pids=$(ss -tulnp 2>/dev/null | grep ":8003 " | grep -oP 'pid=\K[0-9]+' | sort -u)

                if [ -n "$pids" ]; then
                    for pid in $pids; do
                        echo "   終止佔用端口 8003 的進程 PID: $pid"
                        kill -9 $pid 2>/dev/null
                    done
                    sleep 1

                    # 再次檢查端口是否釋放
                    if ss -tulnp 2>/dev/null | grep -q ":8003 "; then
                        echo "❌ 端口 8003 清理失敗，請手動檢查: ss -tulnp | grep :8003"
                        return 1
                    else
                        echo "✅ 端口 8003 已清理"
                    fi
                fi
            fi

            echo "🚀 啟動 Web AGV Launch (AGVUI)..."
            
            # 確保所有必要的工作空間已載入，然後啟動
            # 注意：agvui 需要 agv_interfaces，所以必須載入 agv_ws
            (
                source /app/agv_ws/install/setup.bash 2>/dev/null
                source /app/launch_ws/install/setup.bash 2>/dev/null
                source /app/web_api_ws/install/setup.bash 2>/dev/null
                exec ros2 launch web_agv_launch launch.py
            ) > "$WEB_AGV_LOG_FILE" 2>&1 &
            local PARENT_PID=$!
            
            # 記錄 PID
            echo $PARENT_PID > "$WEB_AGV_PID_FILE"
            
            # 等待一下讓子進程產生
            sleep 2
            
            # 記錄所有子進程 PID
            local CHILD_PIDS=$(pgrep -P $PARENT_PID 2>/dev/null)
            if [ -n "$CHILD_PIDS" ]; then
                for pid in $CHILD_PIDS; do
                    echo $pid >> "$WEB_AGV_PID_FILE"
                done
            fi

            # 使用 pgrep 驗證：Launch 檔案產生多進程，檢查主要服務進程
            # Web AGV Launch 啟動 AGVUI 服務器，使用進程檢查確認啟動成功
            # 參考決策樹：方法 2️⃣ verify_process_startup() 用於 launch 多進程服務
            if verify_process_startup "agv_ui_server" 10; then
                echo "✅ Web AGV Launch 已啟動"
                echo "   記錄的 PID: $(cat $WEB_AGV_PID_FILE | tr '\n' ' ')"

                # 動態等待端口開啟（最多 30 秒，自動重試）
                echo "🔍 等待 AGVUI 服務端口開啟..."

                if wait_for_port_with_retry 8003 30; then
                    echo "✅ AGVUI 端口 8003 已開啟"
                    echo "📍 監控界面: http://$(hostname -I | awk '{print $1}'):8003"
                else
                    echo "❌ AGVUI 端口 8003 等待超時（30 秒）"
                    echo ""
                    echo "💡 診斷建議:"
                    echo "   1. 查看詳細日誌: tail -f $WEB_AGV_LOG_FILE"
                    echo "   2. 檢查 agvui 服務狀態:"
                    echo "      ps aux | grep agv_ui_server"
                    echo "   3. 檢查工作空間建置:"
                    echo "      ls -la /app/web_api_ws/install/agvui"
                    echo "      ls -la /app/launch_ws/install/web_agv_launch"
                    echo "   4. 手動啟動服務進行測試:"
                    echo "      python3 /app/web_api_ws/install/agvui/lib/python3.12/site-packages/agvui/agv_ui_server.py"
                    echo "   5. 檢查環境載入:"
                    echo "      ros2 pkg list | grep agvui"
                    echo "   6. 檢查端口佔用:"
                    echo "      ss -tulnp | grep :8003"
                    echo ""
                    echo "⚠️ 注意: 如果進程存在但端口未開啟，可能是服務啟動失敗"
                fi

                return 0
            else
                echo "❌ Web AGV Launch 啟動失敗"
                echo "📝 檢查日誌: tail -f $WEB_AGV_LOG_FILE"
                echo "💡 可能的原因:"
                echo "   - 套件未正確建置 (執行: build_ws web_api_ws && build_ws launch_ws)"
                echo "   - Python 依賴未安裝"
                echo "   - 端口已被佔用 (檢查: ss -tulnp | grep 8003)"
                echo "   - 工作空間未正確載入 (檢查: ros2 pkg list | grep agvui)"
                return 1
            fi
            ;;
        stop)
            if [ -f "$WEB_AGV_PID_FILE" ]; then
                echo "⏳ 停止 Web AGV Launch 服務..."

                # 讀取並殺掉所有記錄的進程（反向順序：先子後父）
                local PIDS=$(tac "$WEB_AGV_PID_FILE")
                for pid in $PIDS; do
                    if kill -0 $pid 2>/dev/null; then
                        echo "   停止進程 PID: $pid"
                        kill $pid 2>/dev/null
                    fi
                done

                # 等待進程結束
                sleep 3

                # 強制終止仍在運行的進程
                for pid in $PIDS; do
                    if kill -0 $pid 2>/dev/null; then
                        echo "   強制終止 PID: $pid"
                        kill -9 $pid 2>/dev/null
                    fi
                done

                rm -f "$WEB_AGV_PID_FILE"

                # 清理當前會話的 launch_params 臨時目錄 (1 天內的)
                echo "🧹 清理臨時文件..."
                local launch_params_count=$(find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 2>/dev/null | wc -l)
                if [ "$launch_params_count" -gt 0 ]; then
                    find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 -exec rm -rf {} + 2>/dev/null
                    echo "   清理了 $launch_params_count 個 launch_params 臨時目錄"
                fi

                echo "✅ Web AGV Launch 已停止"
            else
                # 確保停止所有與 Web AGV Launch 相關的進程
                echo "🚨 Web AGV Launch PID 檔案未找到，檢查相關進程..."
                if pgrep -f "web_agv_launch" > /dev/null || pgrep -f "agv_ui_server" > /dev/null; then
                    echo "⏳ 停止 Web AGV Launch 相關進程..."
                    pkill -f "web_agv_launch"
                    pkill -f "agv_ui_server"
                    sleep 2
                    echo "✅ Web AGV Launch 相關進程已停止"
                else
                    echo "ℹ️ 未發現運行中的 Web AGV Launch 進程"
                fi

                # 清理可能存在的 PID 檔案
                rm -f "$WEB_AGV_PID_FILE"

                # 清理當前會話的 launch_params 臨時目錄
                echo "🧹 清理臨時文件..."
                local launch_params_count=$(find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 2>/dev/null | wc -l)
                if [ "$launch_params_count" -gt 0 ]; then
                    find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 -exec rm -rf {} + 2>/dev/null
                    echo "   清理了 $launch_params_count 個 launch_params 臨時目錄"
                fi
            fi

            # ===== 強制清理所有相關進程 (避免殘留) =====
            echo "🔍 檢查並清理殘留進程..."
            local found_residual=false

            for service in "agv_ui_server" "web_agv_launch"; do
                if pgrep -f "$service" > /dev/null; then
                    echo "   發現殘留進程: $service"
                    pkill -9 -f "$service" 2>/dev/null
                    found_residual=true
                fi
            done

            if [ "$found_residual" = true ]; then
                sleep 2
                echo "   殘留進程已清理"
            fi

            # ===== 檢查並強制釋放端口 8003 =====
            echo "🔍 檢查 Web AGV 服務端口..."
            local port_released=false

            # 使用 ss 檢查端口（更通用）
            if ss -tulnp 2>/dev/null | grep -q ":8003 "; then
                echo "🚨 端口 8003 仍被佔用，強制釋放..."

                # 提取 PID 並終止（ss 輸出格式: users:(("process",pid=12345,...))）
                local pids=$(ss -tulnp 2>/dev/null | grep ":8003 " | grep -oP 'pid=\K[0-9]+' | sort -u)

                if [ -n "$pids" ]; then
                    for pid in $pids; do
                        echo "   終止佔用端口 8003 的進程 PID: $pid"
                        kill -9 $pid 2>/dev/null
                    done
                    port_released=true
                fi
            fi

            if [ "$port_released" = true ]; then
                sleep 2
                echo "✅ 端口 8003 已強制釋放"
            else
                echo "✅ 端口 8003 正常釋放"
            fi
            ;;
        restart)
            echo "🔄 重新啟動 Web AGV Launch..."
            manage_web_agv_launch stop
            sleep 2
            manage_web_agv_launch start
            ;;
        status)
            if [ -f "$WEB_AGV_PID_FILE" ]; then
                local all_pids=""
                local any_running=false
                
                while IFS= read -r pid; do
                    if kill -0 $pid 2>/dev/null; then
                        all_pids="$all_pids $pid"
                        any_running=true
                    fi
                done < "$WEB_AGV_PID_FILE"
                
                if [ "$any_running" = true ]; then
                    echo "✅ Web AGV Launch 正在運行 (PIDs:$all_pids)"
                    
                    if ss -tuln 2>/dev/null | grep -q ":8003 "; then
                        echo "✅ 端口 8003 已開啟"
                    fi
                    
                    if [ -f "$WEB_AGV_LOG_FILE" ]; then
                        echo ""
                        echo "📋 最近日誌:"
                        tail -5 "$WEB_AGV_LOG_FILE"
                    fi
                else
                    echo "❌ Web AGV Launch 未運行"
                fi
            else
                echo "❌ Web AGV Launch 未運行"
            fi
            ;;
        logs)
            if [ -f "$WEB_AGV_LOG_FILE" ]; then
                echo "📋 Web AGV Launch 日誌:"
                tail -f "$WEB_AGV_LOG_FILE"
            else
                echo "❌ 日誌檔案不存在: $WEB_AGV_LOG_FILE"
            fi
            ;;
        *)
            echo "用法: manage_web_agv_launch {start|stop|restart|status|logs}"
            return 1
            ;;
    esac
}

# ===== AGVUI 控制函式 (向後相容，內部調用 manage_web_agv_launch) =====
manage_agvui() {
    # 環境檢測：僅限 AGVC 容器
    if ! is_agvc_environment; then
        echo "❌ 此功能僅適用於 AGVC 環境"
        echo "💡 AGV 容器請使用: check_agv_status"
        return 1
    fi

    # 為了向後相容，保留 manage_agvui 函數名稱
    # 但內部直接調用新的 manage_web_agv_launch 函數
    echo "📝 注意: manage_agvui 現在使用 ROS 2 Launch 方式 (manage_web_agv_launch)"

    case "$1" in
        start|stop|restart|status)
            manage_web_agv_launch "$1"
            return $?
            ;;
            
        logs)
            # 保留 logs 功能，查看 web_agv_launch 的日誌
            local WEB_AGV_LOG_FILE="/tmp/web_agv_launch.log"
            if [ -f "$WEB_AGV_LOG_FILE" ]; then
                echo "📋 AGVUI 日誌 (Web AGV Launch):"
                tail -f "$WEB_AGV_LOG_FILE"
            else
                echo "❌ 日誌檔案不存在: $WEB_AGV_LOG_FILE"
            fi
            ;;
            
        *)
            echo "用法: manage_agvui {start|stop|restart|status|logs}"
            echo "📝 此函數現在內部調用 manage_web_agv_launch"
            return 1
            ;;
    esac
}


# ===== TAFL WCS 控制函式 (新一代 WCS 系統) =====
# =============================================================================
# 🏭 TAFL WCS 節點管理
# =============================================================================
manage_tafl_wcs() {
    # 環境檢測：僅限 AGVC 容器
    if ! is_agvc_environment; then
        echo "❌ 此功能僅適用於 AGVC 環境"
        echo "💡 AGV 容器請使用: check_agv_status"
        return 1
    fi

    local TAFL_WCS_LOG_FILE="/tmp/tafl_wcs.log"
    local TAFL_WCS_PID_FILE="/tmp/tafl_wcs.pid"

    case "$1" in
        start)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  🚀 啟動 TAFL WCS 節點"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            # ========== 階段 1: 啟動前檢查 ==========
            echo "📋 階段 1/4: 啟動前檢查"

            # 1.1 檢查是否已運行（幂等性）
            if [ -f "$TAFL_WCS_PID_FILE" ]; then
                local all_running=true
                while IFS= read -r pid; do
                    # 跳過空行
                    [ -z "$pid" ] && continue
                    if ! kill -0 "$pid" 2>/dev/null; then
                        all_running=false
                        break
                    fi
                done < "$TAFL_WCS_PID_FILE"

                if [ "$all_running" = true ]; then
                    echo "✅ TAFL WCS 已經在運行中"
                    echo "💡 如需重新啟動，請使用: manage_tafl_wcs restart"
                    return 0
                else
                    echo "🧹 清理過期的 PID 文件"
                    rm -f "$TAFL_WCS_PID_FILE"
                fi
            fi

            # ========== 階段 2: 依賴檢查 ==========
            echo "📋 階段 2/4: 依賴檢查"

            # 2.1 檢查資料庫連接
            if ! pgrep -f "db_proxy_node" > /dev/null 2>&1; then
                echo "⚠️ 警告：資料庫代理未運行，TAFL WCS 可能無法正常工作"
                echo "💡 建議先啟動: manage_agvc_database_node start"
            fi

            # 2.2 檢查工作空間建置
            if [ ! -d "/app/tafl_wcs_ws/install" ]; then
                echo "❌ TAFL WCS 工作空間未建置"
                echo "💡 診斷建議："
                echo "   1. 執行建置: cd /app/tafl_wcs_ws && colcon build"
                echo "   2. 或使用快速建置: ba"
                return 1
            fi

            # ========== 階段 3: 啟動服務 ==========
            echo "📋 階段 3/4: 啟動服務"

            # 3.1 確保日誌目錄存在
            touch "$TAFL_WCS_LOG_FILE"

            # 3.2 啟動 TAFL WCS Launch
            echo "🔧 啟動 TAFL WCS Launch..."
            nohup bash -c "source /app/setup.bash && agvc_source > /dev/null 2>&1 && ros2 launch tafl_wcs tafl_wcs.launch.py" > "$TAFL_WCS_LOG_FILE" 2>&1 &
            local PARENT_PID=$!

            # 3.3 記錄父進程 PID
            echo "$PARENT_PID" > "$TAFL_WCS_PID_FILE"
            echo "  📌 父進程 PID: $PARENT_PID"

            # 3.4 等待 1 秒讓子進程啟動
            sleep 1

            # 3.5 記錄子進程 PID（bash -c 產生的子 shell）
            local CHILD_PIDS=$(pgrep -P $PARENT_PID)
            if [ -n "$CHILD_PIDS" ]; then
                echo "  📌 子進程 PIDs: $CHILD_PIDS"
                echo "$CHILD_PIDS" >> "$TAFL_WCS_PID_FILE"
            fi

            # 3.6 記錄實際服務進程（python3 執行 launch）
            sleep 1
            local SERVICE_PIDS=$(pgrep -f "ros2 launch tafl_wcs")
            if [ -n "$SERVICE_PIDS" ]; then
                echo "  📌 服務進程 PIDs: $SERVICE_PIDS"
                echo "$SERVICE_PIDS" >> "$TAFL_WCS_PID_FILE"
            fi

            # ========== 階段 4: 驗證啟動 ==========
            echo "📋 階段 4/4: 驗證啟動"

            # 4.1 驗證父進程
            if ! kill -0 $PARENT_PID 2>/dev/null; then
                echo "❌ TAFL WCS 啟動失敗（父進程未運行）"
                echo "💡 診斷建議："
                echo "   1. 查看日誌: tail -f $TAFL_WCS_LOG_FILE"
                echo "   2. 檢查工作空間建置: ls /app/tafl_wcs_ws/install"
                echo "   3. 驗證環境: source /app/setup.bash && agvc_source && ros2 pkg list | grep tafl_wcs"
                rm -f "$TAFL_WCS_PID_FILE"
                return 1
            fi

            # 4.2 使用 ros2 node list 驗證
            # TAFL WCS 是核心流程控制服務，必須確保節點正常註冊到 ROS 2 網路
            # 參考決策樹：方法 1️⃣ verify_ros2_node_startup() 用於 launch 產生的命名節點
            echo "  🔍 等待 ROS 2 節點註冊..."
            if verify_ros2_node_startup "tafl_wcs_node" 15; then
                echo "  ✅ ROS 2 節點已註冊"
            else
                echo "  ❌ ROS 2 節點註冊失敗或超時"
                echo "💡 診斷建議："
                echo "   1. 查看日誌: tail -f $TAFL_WCS_LOG_FILE"
                echo "   2. 檢查 ROS 2 環境: ros2 node list"
                echo "   3. 檢查 Zenoh Router: check_zenoh_status"
                echo "   4. 驗證網路: ros2 doctor --report"

                # 清理失敗的啟動
                echo "🧹 清理失敗的啟動..."
                manage_tafl_wcs stop > /dev/null 2>&1
                return 1
            fi

            # 4.3 最終進程驗證
            if pgrep -f "tafl_wcs" > /dev/null 2>&1; then
                echo "  ✅ 服務進程運行正常"
            else
                echo "  ⚠️ 警告：服務進程可能異常"
            fi

            # 4.4 顯示啟動摘要
            echo ""
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "✅ TAFL WCS 啟動完成"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "📝 日誌檔案: $TAFL_WCS_LOG_FILE"
            echo "📊 PID 文件: $TAFL_WCS_PID_FILE"
            echo "💡 查看日誌: manage_tafl_wcs logs"
            echo "💡 檢查狀態: manage_tafl_wcs status"
            echo ""

            # 顯示節點資訊
            echo "📊 節點資訊："
            ros2 node info /agvc/tafl_wcs_node 2>/dev/null | head -n 10 || echo "  ⚠️ 無法獲取節點資訊"
            ;;

        stop)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  🛑 停止 TAFL WCS 節點"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            # ========== 階段 1: 優雅停止主進程 (SIGTERM) ==========
            echo "📋 階段 1/6: 優雅停止 (SIGTERM)"

            if [ -f "$TAFL_WCS_PID_FILE" ]; then
                local stopped_count=0
                while IFS= read -r pid; do
                    [ -z "$pid" ] && continue
                    if kill -0 "$pid" 2>/dev/null; then
                        echo "  🔄 發送 SIGTERM 到進程 $pid"
                        kill -TERM "$pid" 2>/dev/null || true
                        ((stopped_count++))
                    fi
                done < <(tac "$TAFL_WCS_PID_FILE")

                if [ $stopped_count -gt 0 ]; then
                    echo "  ⏳ 等待 3 秒讓進程優雅退出..."
                    sleep 3
                fi
            else
                echo "  ℹ️ 未找到 PID 文件"
            fi

            # ========== 階段 2: 強制終止殘留進程 (SIGKILL) ==========
            echo "📋 階段 2/6: 強制終止 (SIGKILL)"

            if [ -f "$TAFL_WCS_PID_FILE" ]; then
                local killed_count=0
                while IFS= read -r pid; do
                    [ -z "$pid" ] && continue
                    if kill -0 "$pid" 2>/dev/null; then
                        echo "  ⚠️ 進程 $pid 仍在運行，強制終止"
                        kill -KILL "$pid" 2>/dev/null || true
                        ((killed_count++))
                    fi
                done < <(tac "$TAFL_WCS_PID_FILE")

                if [ $killed_count -gt 0 ]; then
                    echo "  ✅ 已強制終止 $killed_count 個殘留進程"
                    sleep 1
                fi
            fi

            # ========== 階段 3: 備份清理（檢查僵屍進程） ==========
            echo "📋 階段 3/6: 備份清理"

            local zombie_pids=$(pgrep -f "tafl_wcs" | while read p; do
                if [ -d "/proc/$p" ] && grep -q "Z (zombie)" "/proc/$p/status" 2>/dev/null; then
                    echo $p
                fi
            done)

            if [ -n "$zombie_pids" ]; then
                echo "  🧟 發現僵屍進程: $zombie_pids"
                echo "  🔧 嘗試清理僵屍進程的父進程"
                for zpid in $zombie_pids; do
                    local parent=$(ps -o ppid= -p $zpid 2>/dev/null | tr -d ' ')
                    if [ -n "$parent" ] && [ "$parent" != "1" ]; then
                        echo "    終止父進程 $parent"
                        kill -KILL "$parent" 2>/dev/null || true
                    fi
                done
                sleep 1
            fi

            # ========== 階段 4: 殘留進程清理 ==========
            echo "📋 階段 4/6: 殘留進程清理"

            local remaining_pids=$(pgrep -f "tafl_wcs")
            if [ -n "$remaining_pids" ]; then
                echo "  🔍 發現殘留進程: $remaining_pids"
                for pid in $remaining_pids; do
                    echo "    終止進程 $pid"
                    kill -KILL "$pid" 2>/dev/null || true
                done
                sleep 1
            else
                echo "  ✅ 無殘留進程"
            fi

            # ========== 階段 5: 端口資源釋放 ==========
            echo "📋 階段 5/6: 端口資源釋放"

            # TAFL WCS 主要使用 ROS 2 DDS/Zenoh，無特定端口需要釋放
            echo "  ✅ TAFL WCS 無需額外端口清理"

            # ========== 階段 6: 清理臨時文件 ==========
            echo "📋 階段 6/6: 清理臨時文件"

            rm -f "$TAFL_WCS_PID_FILE"
            echo "  ✅ 已清理 PID 文件"

            # 清理 launch 臨時參數文件
            local launch_params=$(find /tmp -maxdepth 1 -name "launch_params_*" -path "*/tafl_wcs*" 2>/dev/null)
            if [ -n "$launch_params" ]; then
                echo "  🧹 清理 Launch 參數目錄: $launch_params"
                rm -rf $launch_params
            fi

            echo ""
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "✅ TAFL WCS 已完全停止"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            ;;

        restart)
            echo "🔄 重新啟動 TAFL WCS..."
            manage_tafl_wcs stop
            sleep 2
            manage_tafl_wcs start
            ;;

        status)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  📊 TAFL WCS 狀態檢查"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            # 檢查進程狀態
            if pgrep -f "tafl_wcs_node" > /dev/null 2>&1; then
                local pids=$(pgrep -f "tafl_wcs_node")
                echo "✅ TAFL WCS 正在運行"
                echo "   PIDs: $pids"

                # 檢查 PID 文件
                if [ -f "$TAFL_WCS_PID_FILE" ]; then
                    echo "   PID 文件: $TAFL_WCS_PID_FILE"
                    echo "   記錄的 PIDs: $(cat $TAFL_WCS_PID_FILE | tr '\n' ' ')"
                fi

                # 檢查 ROS 2 節點狀態
                echo ""
                echo "🔍 ROS 2 節點狀態："
                if ros2 node list 2>/dev/null | grep -q "tafl_wcs_node"; then
                    echo "  ✅ tafl_wcs_node 節點已註冊到 ROS 2 網路"

                    # 顯示節點資訊
                    echo ""
                    echo "📊 節點詳細資訊："
                    ros2 node info /agvc/tafl_wcs_node 2>/dev/null | head -n 15 || echo "  ⚠️ 無法獲取節點資訊"
                else
                    echo "  ⚠️ tafl_wcs_node 節點未在 ROS 2 網路中註冊"
                    echo "  💡 可能原因："
                    echo "     1. 節點仍在啟動中"
                    echo "     2. Zenoh Router 未運行"
                    echo "     3. 網路連接問題"
                fi

                # 顯示最新日誌
                if [ -f "$TAFL_WCS_LOG_FILE" ]; then
                    echo ""
                    echo "📜 最新日誌 (最後 5 行):"
                    tail -n 5 "$TAFL_WCS_LOG_FILE"
                fi
            else
                echo "🚫 TAFL WCS 未在運行"

                if [ -f "$TAFL_WCS_PID_FILE" ]; then
                    echo "⚠️ 發現殘留的 PID 文件"
                    echo "💡 建議執行: manage_tafl_wcs stop"
                fi
            fi

            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            ;;

        logs)
            if [ -f "$TAFL_WCS_LOG_FILE" ]; then
                echo "📜 TAFL WCS 日誌 (Ctrl+C 退出):"
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                tail -f "$TAFL_WCS_LOG_FILE"
            else
                echo "❌ 找不到日誌檔案: $TAFL_WCS_LOG_FILE"
                echo "💡 服務可能未啟動過，請先執行: manage_tafl_wcs start"
                return 1
            fi
            ;;

        *)
            echo "用法: manage_tafl_wcs {start|stop|restart|status|logs}"
            return 1
            ;;
    esac
}

# ============================================================================
# 核心節點管理函數
# ============================================================================

# =============================================================================
# PLC 服務管理函數（AGVC）
# =============================================================================
#
# 功能說明：
#   管理 PLC 服務的啟動、停止、重啟和狀態檢查
#
# 依賴檢查：
#   - 工作空間：plc_proxy_ws
#   - 配置檔案：/app/config/ecs_config.yaml
#   - ROS2 節點：/agvc/plc_service
#
# 使用方式：
#   manage_plc_service_agvc start   - 啟動服務
#   manage_plc_service_agvc stop    - 停止服務
#   manage_plc_service_agvc restart - 重啟服務
#   manage_plc_service_agvc status  - 查看服務狀態
#   manage_plc_service_agvc logs    - 查看實時日誌
#
# =============================================================================

manage_plc_service_agvc() {
    local PLC_SERVICE_LOG_FILE="/tmp/plc_service_agvc.log"
    local PLC_SERVICE_PID_FILE="/tmp/plc_service_agvc.pid"

    # 環境檢查
    if ! is_agvc_environment; then
        echo "❌ 此功能僅適用於 AGVC 環境"
        return 1
    fi

    case "$1" in
        start)
            # ========== 階段 1: 啟動前檢查 ==========

            # 1.1 檢查是否已運行（幂等性）
            if [ -f "$PLC_SERVICE_PID_FILE" ]; then
                local all_running=true
                while read pid; do
                    if ! kill -0 $pid 2>/dev/null; then
                        all_running=false
                        break
                    fi
                done < "$PLC_SERVICE_PID_FILE"

                if [ "$all_running" = true ]; then
                    echo "✅ PLC 服務已經在運行中"
                    return 0
                else
                    echo "⏳ 檢測到過時的 PID 文件，正在清理..."
                    rm -f "$PLC_SERVICE_PID_FILE"
                fi
            fi

            # 1.2 檢查工作空間是否已建置
            if [ ! -d "/app/plc_proxy_ws/install" ]; then
                echo "⚠️ 警告: plc_proxy_ws 未建置，請先執行: build_ws plc_proxy_ws"
            fi

            # 1.3 檢查配置檔案
            if [ ! -f "/app/config/ecs_config.yaml" ]; then
                echo "❌ 配置檔案不存在: /app/config/ecs_config.yaml"
                return 1
            fi

            # ========== 階段 2: 啟動服務 ==========

            echo "🚀 啟動 PLC 服務節點 (AGVC)..."

            # 確保日誌檔案存在
            touch "$PLC_SERVICE_LOG_FILE"

            # 使用 ros2 run 啟動單一節點
            nohup bash -c "source /app/setup.bash && agvc_source > /dev/null 2>&1 && ros2 run plc_proxy plc_service --ros-args -r __ns:=/agvc --params-file /app/config/ecs_config.yaml" > "$PLC_SERVICE_LOG_FILE" 2>&1 &
            local PARENT_PID=$!

            # 記錄父進程
            echo $PARENT_PID > "$PLC_SERVICE_PID_FILE"

            # 等待子進程啟動
            sleep 3

            # 記錄子進程
            local CHILD_PIDS=$(pgrep -P $PARENT_PID)
            if [ -n "$CHILD_PIDS" ]; then
                for pid in $CHILD_PIDS; do
                    echo $pid >> "$PLC_SERVICE_PID_FILE"
                done
            fi

            # 記錄實際的服務進程
            sleep 2
            local SERVICE_PID=$(pgrep -f "plc_proxy.*plc_service" | head -n1)
            if [ -n "$SERVICE_PID" ]; then
                if ! grep -q "^$SERVICE_PID$" "$PLC_SERVICE_PID_FILE" 2>/dev/null; then
                    echo $SERVICE_PID >> "$PLC_SERVICE_PID_FILE"
                fi
            fi

            # ========== 階段 3: 驗證啟動 ==========

            # 驗證父進程
            if ! kill -0 $PARENT_PID 2>/dev/null; then
                echo "❌ PLC 服務啟動失敗（父進程未運行）"
                echo ""
                echo "💡 診斷建議："
                echo "   1. 查看日誌: tail -f $PLC_SERVICE_LOG_FILE"
                echo "   2. 檢查工作空間建置: ls /app/plc_proxy_ws/install"
                echo "   3. 重新建置: cd /app/plc_proxy_ws && colcon build --packages-select plc_proxy"
                echo "   4. 檢查 Zenoh Router 狀態: manage_zenoh status"
                return 1
            fi

            echo "✅ PLC 服務已啟動"
            echo "   記錄的 PID: $(cat $PLC_SERVICE_PID_FILE | tr '\n' ' ')"

            # 驗證 ROS2 節點注冊
            if verify_ros2_node_startup "/agvc/plc_service" 15; then
                echo "✅ ROS2 節點已註冊"
                echo "✅ PLC 服務啟動完成"
            else
                echo "⚠️ ROS2 節點註冊超時"
                echo ""
                echo "💡 建議："
                echo "   1. 查看日誌: tail -f $PLC_SERVICE_LOG_FILE"
                echo "   2. 檢查 Zenoh Router: manage_zenoh status"
                echo "   3. 檢查 ROS2 環境: echo \$ROS_DISTRO"
                return 1
            fi
            ;;

        stop)
            echo "⏳ 停止 PLC 服務節點..."

            # ========== 階段 1: 優雅停止 ==========
            if [ -f "$PLC_SERVICE_PID_FILE" ]; then
                local PIDS=$(tac "$PLC_SERVICE_PID_FILE")
                for pid in $PIDS; do
                    if kill -0 $pid 2>/dev/null; then
                        echo "   停止進程 PID: $pid"
                        kill $pid 2>/dev/null  # SIGTERM
                    fi
                done

                sleep 3  # 等待優雅退出

                # ========== 階段 2: 強制終止 ==========
                for pid in $PIDS; do
                    if kill -0 $pid 2>/dev/null; then
                        echo "   強制終止 PID: $pid"
                        kill -9 $pid 2>/dev/null  # SIGKILL
                    fi
                done

                rm -f "$PLC_SERVICE_PID_FILE"
            fi

            # ========== 階段 3: 備用清理（無 PID 文件時） ==========
            if [ ! -f "$PLC_SERVICE_PID_FILE" ]; then
                echo "🚨 PID 文件未找到，檢查相關進程..."
                if pgrep -f "plc_proxy.*plc_service" > /dev/null; then
                    echo "   發現進程: plc_service"
                    pkill -f "plc_proxy.*plc_service"
                    sleep 2
                    echo "   相關進程已停止"
                fi
            fi

            # ========== 階段 4: 殘留進程清理 ==========
            echo "🔍 檢查並清理殘留進程..."
            local found_residual=false

            if pgrep -f "plc_proxy.*plc_service" > /dev/null; then
                echo "   發現殘留進程: plc_service"
                pkill -9 -f "plc_proxy.*plc_service" 2>/dev/null
                found_residual=true
            fi

            if [ "$found_residual" = true ]; then
                sleep 2
                echo "   殘留進程已清理"
            fi

            # ========== 階段 5: 臨時文件清理 ==========
            echo "🧹 清理臨時文件..."
            local launch_params_count=$(find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 2>/dev/null | wc -l)
            if [ "$launch_params_count" -gt 0 ]; then
                find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 -exec rm -rf {} + 2>/dev/null
                echo "   清理了 $launch_params_count 個 launch_params 臨時目錄"
            fi

            echo "✅ PLC 服務已停止"
            ;;

        restart)
            manage_plc_service_agvc stop
            sleep 2
            manage_plc_service_agvc start
            ;;

        status)
            if [ -f "$PLC_SERVICE_PID_FILE" ]; then
                local all_running=true
                local pids=""

                while read pid; do
                    if kill -0 $pid 2>/dev/null; then
                        pids="$pids $pid"
                    else
                        all_running=false
                    fi
                done < "$PLC_SERVICE_PID_FILE"

                if [ "$all_running" = true ]; then
                    echo "✅ PLC 服務運行中 (PIDs:$pids)"

                    # 顯示 ROS2 節點信息
                    if ros2 node list 2>/dev/null | grep -q "/agvc/plc_service"; then
                        echo "   ROS2 節點: /agvc/plc_service 已註冊"
                    fi

                    return 0
                else
                    echo "⚠️ PLC 服務部分進程未運行"
                    rm -f "$PLC_SERVICE_PID_FILE"
                    return 1
                fi
            else
                echo "🚫 PLC 服務未運行"
                return 1
            fi
            ;;

        logs)
            if [ -f "$PLC_SERVICE_LOG_FILE" ]; then
                echo "📄 PLC 服務實時日誌 (Ctrl+C 退出):"
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                tail -f "$PLC_SERVICE_LOG_FILE"
            else
                echo "❌ 日誌文件不存在: $PLC_SERVICE_LOG_FILE"
                return 1
            fi
            ;;

        *)
            echo "用法: manage_plc_service_agvc {start|stop|restart|status|logs}"
            echo ""
            echo "指令說明："
            echo "  start   - 啟動 PLC 服務"
            echo "  stop    - 停止 PLC 服務"
            echo "  restart - 重啟 PLC 服務"
            echo "  status  - 查看服務狀態"
            echo "  logs    - 查看實時日誌"
            return 1
            ;;
    esac
}

# =============================================================================
# ECS 核心管理函數（AGVC）
# =============================================================================
#
# 功能說明：
#   管理 ECS 核心服務的啟動、停止、重啟和狀態檢查
#
# 依賴檢查：
#   - 工作空間：ecs_ws
#   - 配置檔案：/app/config/ecs_config.yaml
#   - ROS2 節點：/agvc/ecs_core
#
# 使用方式：
#   manage_ecs_core start   - 啟動服務
#   manage_ecs_core stop    - 停止服務
#   manage_ecs_core restart - 重啟服務
#   manage_ecs_core status  - 查看服務狀態
#   manage_ecs_core logs    - 查看實時日誌
#
# =============================================================================

manage_ecs_core() {
    local ECS_CORE_LOG_FILE="/tmp/ecs_core.log"
    local ECS_CORE_PID_FILE="/tmp/ecs_core.pid"

    # 環境檢查
    if ! is_agvc_environment; then
        echo "❌ 此功能僅適用於 AGVC 環境"
        return 1
    fi

    case "$1" in
        start)
            # ========== 階段 1: 啟動前檢查 ==========

            # 1.1 檢查是否已運行（幂等性）
            if [ -f "$ECS_CORE_PID_FILE" ]; then
                local all_running=true
                while read pid; do
                    if ! kill -0 $pid 2>/dev/null; then
                        all_running=false
                        break
                    fi
                done < "$ECS_CORE_PID_FILE"

                if [ "$all_running" = true ]; then
                    echo "✅ ECS 核心已經在運行中"
                    return 0
                else
                    echo "⏳ 檢測到過時的 PID 文件，正在清理..."
                    rm -f "$ECS_CORE_PID_FILE"
                fi
            fi

            # 1.2 檢查工作空間是否已建置
            if [ ! -d "/app/ecs_ws/install" ]; then
                echo "⚠️ 警告: ecs_ws 未建置，請先執行: build_ws ecs_ws"
            fi

            # 1.3 檢查配置檔案
            if [ ! -f "/app/config/ecs_config.yaml" ]; then
                echo "❌ 配置檔案不存在: /app/config/ecs_config.yaml"
                return 1
            fi

            # ========== 階段 2: 啟動服務 ==========

            echo "🚀 啟動 ECS 核心節點..."

            # 確保日誌檔案存在
            touch "$ECS_CORE_LOG_FILE"

            # 使用 ros2 run 啟動節點
            nohup bash -c "source /app/setup.bash && agvc_source > /dev/null 2>&1 && ros2 run ecs ecs_core --ros-args -r __ns:=/agvc --params-file /app/config/ecs_config.yaml" > "$ECS_CORE_LOG_FILE" 2>&1 &
            local PARENT_PID=$!

            # 記錄父進程
            echo $PARENT_PID > "$ECS_CORE_PID_FILE"

            # 等待子進程啟動
            sleep 3

            # 記錄子進程
            local CHILD_PIDS=$(pgrep -P $PARENT_PID)
            if [ -n "$CHILD_PIDS" ]; then
                for pid in $CHILD_PIDS; do
                    echo $pid >> "$ECS_CORE_PID_FILE"
                done
            fi

            # 記錄實際的服務進程
            sleep 2
            local SERVICE_PID=$(pgrep -f "ecs.*ecs_core" | head -n1)
            if [ -n "$SERVICE_PID" ]; then
                if ! grep -q "^$SERVICE_PID$" "$ECS_CORE_PID_FILE" 2>/dev/null; then
                    echo $SERVICE_PID >> "$ECS_CORE_PID_FILE"
                fi
            fi

            # ========== 階段 3: 驗證啟動 ==========

            # 驗證父進程
            if ! kill -0 $PARENT_PID 2>/dev/null; then
                echo "❌ ECS 核心啟動失敗（父進程未運行）"
                echo ""
                echo "💡 診斷建議："
                echo "   1. 查看日誌: tail -f $ECS_CORE_LOG_FILE"
                echo "   2. 檢查工作空間建置: ls /app/ecs_ws/install"
                echo "   3. 重新建置: cd /app/ecs_ws && colcon build --packages-select ecs"
                echo "   4. 檢查 PLC 服務狀態: manage_plc_service_agvc status"
                return 1
            fi

            echo "✅ ECS 核心已啟動"
            echo "   記錄的 PID: $(cat $ECS_CORE_PID_FILE | tr '\n' ' ')"

            # 驗證 ROS2 節點注冊
            if verify_ros2_node_startup "/agvc/ecs_core" 15; then
                echo "✅ ROS2 節點已註冊"
                echo "✅ ECS 核心啟動完成"
            else
                echo "⚠️ ROS2 節點註冊超時"
                echo ""
                echo "💡 建議："
                echo "   1. 查看日誌: tail -f $ECS_CORE_LOG_FILE"
                echo "   2. 檢查 Zenoh Router: manage_zenoh status"
                echo "   3. 檢查 ROS2 環境: echo \$ROS_DISTRO"
                return 1
            fi
            ;;

        stop)
            echo "⏳ 停止 ECS 核心節點..."

            # ========== 階段 1: 優雅停止 ==========
            if [ -f "$ECS_CORE_PID_FILE" ]; then
                local PIDS=$(tac "$ECS_CORE_PID_FILE")
                for pid in $PIDS; do
                    if kill -0 $pid 2>/dev/null; then
                        echo "   停止進程 PID: $pid"
                        kill $pid 2>/dev/null  # SIGTERM
                    fi
                done

                sleep 3  # 等待優雅退出

                # ========== 階段 2: 強制終止 ==========
                for pid in $PIDS; do
                    if kill -0 $pid 2>/dev/null; then
                        echo "   強制終止 PID: $pid"
                        kill -9 $pid 2>/dev/null  # SIGKILL
                    fi
                done

                rm -f "$ECS_CORE_PID_FILE"
            fi

            # ========== 階段 3: 備用清理（無 PID 文件時） ==========
            if [ ! -f "$ECS_CORE_PID_FILE" ]; then
                echo "🚨 PID 文件未找到，檢查相關進程..."
                if pgrep -f "ecs.*ecs_core" > /dev/null; then
                    echo "   發現進程: ecs_core"
                    pkill -f "ecs.*ecs_core"
                    sleep 2
                    echo "   相關進程已停止"
                fi
            fi

            # ========== 階段 4: 殘留進程清理 ==========
            echo "🔍 檢查並清理殘留進程..."
            local found_residual=false

            if pgrep -f "ecs.*ecs_core" > /dev/null; then
                echo "   發現殘留進程: ecs_core"
                pkill -9 -f "ecs.*ecs_core" 2>/dev/null
                found_residual=true
            fi

            if [ "$found_residual" = true ]; then
                sleep 2
                echo "   殘留進程已清理"
            fi

            # ========== 階段 5: 臨時文件清理 ==========
            echo "🧹 清理臨時文件..."
            local launch_params_count=$(find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 2>/dev/null | wc -l)
            if [ "$launch_params_count" -gt 0 ]; then
                find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 -exec rm -rf {} + 2>/dev/null
                echo "   清理了 $launch_params_count 個 launch_params 臨時目錄"
            fi

            echo "✅ ECS 核心已停止"
            ;;

        restart)
            manage_ecs_core stop
            sleep 2
            manage_ecs_core start
            ;;

        status)
            if [ -f "$ECS_CORE_PID_FILE" ]; then
                local all_running=true
                local pids=""

                while read pid; do
                    if kill -0 $pid 2>/dev/null; then
                        pids="$pids $pid"
                    else
                        all_running=false
                    fi
                done < "$ECS_CORE_PID_FILE"

                if [ "$all_running" = true ]; then
                    echo "✅ ECS 核心運行中 (PIDs:$pids)"

                    # 顯示 ROS2 節點信息
                    if ros2 node list 2>/dev/null | grep -q "/agvc/ecs_core"; then
                        echo "   ROS2 節點: /agvc/ecs_core 已註冊"
                    fi

                    return 0
                else
                    echo "⚠️ ECS 核心部分進程未運行"
                    rm -f "$ECS_CORE_PID_FILE"
                    return 1
                fi
            else
                echo "🚫 ECS 核心未運行"
                return 1
            fi
            ;;

        logs)
            if [ -f "$ECS_CORE_LOG_FILE" ]; then
                echo "📄 ECS 核心實時日誌 (Ctrl+C 退出):"
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                tail -f "$ECS_CORE_LOG_FILE"
            else
                echo "❌ 日誌文件不存在: $ECS_CORE_LOG_FILE"
                return 1
            fi
            ;;

        *)
            echo "用法: manage_ecs_core {start|stop|restart|status|logs}"
            echo ""
            echo "指令說明："
            echo "  start   - 啟動 ECS 核心"
            echo "  stop    - 停止 ECS 核心"
            echo "  restart - 重啟 ECS 核心"
            echo "  status  - 查看服務狀態"
            echo "  logs    - 查看實時日誌"
            return 1
            ;;
    esac
}

# =============================================================================
# RCS 核心管理函數（AGVC）
# =============================================================================
#
# 功能說明：
#   管理 RCS 核心服務的啟動、停止、重啟和狀態檢查
#   使用 ros2 launch 啟動多節點服務
#
# 依賴檢查：
#   - 工作空間：rcs_ws
#   - Launch 文件：rcs_launch.py
#
# 使用方式：
#   manage_rcs_core start   - 啟動服務
#   manage_rcs_core stop    - 停止服務
#   manage_rcs_core restart - 重啟服務
#   manage_rcs_core status  - 查看服務狀態
#   manage_rcs_core logs    - 查看實時日誌
#
# =============================================================================

manage_rcs_core() {
    local RCS_CORE_LOG_FILE="/tmp/rcs_launch.log"
    local RCS_CORE_PID_FILE="/tmp/rcs_core.pid"

    # 環境檢查
    if ! is_agvc_environment; then
        echo "❌ 此功能僅適用於 AGVC 環境"
        return 1
    fi

    case "$1" in
        start)
            # ========== 階段 1: 啟動前檢查 ==========

            # 1.1 檢查是否已運行（幂等性）
            if [ -f "$RCS_CORE_PID_FILE" ]; then
                local all_running=true
                while read pid; do
                    if ! kill -0 $pid 2>/dev/null; then
                        all_running=false
                        break
                    fi
                done < "$RCS_CORE_PID_FILE"

                if [ "$all_running" = true ]; then
                    echo "✅ RCS 核心已經在運行中"
                    return 0
                else
                    echo "⏳ 檢測到過時的 PID 文件，正在清理..."
                    rm -f "$RCS_CORE_PID_FILE"
                fi
            fi

            # 1.2 檢查工作空間是否已建置
            if [ ! -f "/app/rcs_ws/install/setup.bash" ]; then
                echo "❌ RCS 工作空間未建置，請先執行: cd /app/rcs_ws && colcon build"
                return 1
            fi

            # ========== 階段 2: 啟動服務 ==========

            echo "🚀 啟動 RCS 核心節點 (使用 ROS 2 Launch)..."

            # 確保日誌檔案存在
            touch "$RCS_CORE_LOG_FILE"

            # 使用 ros2 launch 啟動
            nohup bash -c "source /app/setup.bash && agvc_source > /dev/null 2>&1 && ros2 launch rcs rcs_launch.py" > "$RCS_CORE_LOG_FILE" 2>&1 &
            local PARENT_PID=$!

            # 記錄父進程
            echo $PARENT_PID > "$RCS_CORE_PID_FILE"

            # 等待子進程啟動
            sleep 3

            # 記錄子進程
            local CHILD_PIDS=$(pgrep -P $PARENT_PID)
            if [ -n "$CHILD_PIDS" ]; then
                for pid in $CHILD_PIDS; do
                    echo $pid >> "$RCS_CORE_PID_FILE"
                done
            fi

            # 記錄實際的服務進程
            sleep 2
            local service_patterns=("rcs_launch.py" "rcs_core")
            for pattern in "${service_patterns[@]}"; do
                local SERVICE_PID=$(pgrep -f "$pattern" | head -n1)
                if [ -n "$SERVICE_PID" ]; then
                    if ! grep -q "^$SERVICE_PID$" "$RCS_CORE_PID_FILE" 2>/dev/null; then
                        echo $SERVICE_PID >> "$RCS_CORE_PID_FILE"
                    fi
                fi
            done

            # ========== 階段 3: 驗證啟動 ==========

            # 驗證父進程
            if ! kill -0 $PARENT_PID 2>/dev/null; then
                echo "❌ RCS 核心啟動失敗（父進程未運行）"
                echo ""
                echo "💡 診斷建議："
                echo "   1. 查看日誌: tail -f $RCS_CORE_LOG_FILE"
                echo "   2. 檢查工作空間建置: ls /app/rcs_ws/install"
                echo "   3. 重新建置: cd /app/rcs_ws && colcon build"
                echo "   4. 檢查 Zenoh Router: manage_zenoh status"
                return 1
            fi

            echo "✅ RCS 核心已啟動"
            echo "   記錄的 PID: $(cat $RCS_CORE_PID_FILE | tr '\n' ' ')"

            # 驗證 Launch 進程
            if verify_process_startup "rcs_launch.py" 10; then
                echo "✅ RCS Launch 進程已驗證"
                echo "✅ RCS 核心啟動完成"
                echo ""
                echo "💡 提示："
                echo "   查看日誌: manage_rcs_core logs"
            else
                echo "⚠️ RCS Launch 進程驗證超時"
                echo ""
                echo "💡 建議："
                echo "   1. 查看日誌: tail -f $RCS_CORE_LOG_FILE"
                echo "   2. 檢查 ROS2 環境: echo \$ROS_DISTRO"
                return 1
            fi
            ;;

        stop)
            echo "⏳ 停止 RCS 核心節點..."

            # ========== 階段 1: 優雅停止 ==========
            if [ -f "$RCS_CORE_PID_FILE" ]; then
                local PIDS=$(tac "$RCS_CORE_PID_FILE")
                for pid in $PIDS; do
                    if kill -0 $pid 2>/dev/null; then
                        echo "   停止進程 PID: $pid"
                        kill $pid 2>/dev/null  # SIGTERM
                    fi
                done

                sleep 3  # 等待優雅退出

                # ========== 階段 2: 強制終止 ==========
                for pid in $PIDS; do
                    if kill -0 $pid 2>/dev/null; then
                        echo "   強制終止 PID: $pid"
                        kill -9 $pid 2>/dev/null  # SIGKILL
                    fi
                done

                rm -f "$RCS_CORE_PID_FILE"
            fi

            # 舊的 PID 檔案相容性清理
            if [ -f "/tmp/rcs.pid" ]; then
                rm -f "/tmp/rcs.pid"
            fi

            # ========== 階段 3: 備用清理（無 PID 文件時） ==========
            if [ ! -f "$RCS_CORE_PID_FILE" ]; then
                echo "🚨 PID 文件未找到，檢查相關進程..."
                local service_patterns=("ros2 launch rcs" "rcs_launch.py" "rcs_core")
                local found_process=false

                for pattern in "${service_patterns[@]}"; do
                    if pgrep -f "$pattern" > /dev/null; then
                        echo "   發現進程: $pattern"
                        pkill -f "$pattern"
                        found_process=true
                    fi
                done

                if [ "$found_process" = true ]; then
                    sleep 2
                    echo "   相關進程已停止"
                fi
            fi

            # ========== 階段 4: 殘留進程清理 ==========
            echo "🔍 檢查並清理殘留進程..."
            local found_residual=false
            local service_patterns=("ros2 launch rcs" "rcs_launch.py" "rcs_core")

            for pattern in "${service_patterns[@]}"; do
                if pgrep -f "$pattern" > /dev/null; then
                    echo "   發現殘留進程: $pattern"
                    pkill -9 -f "$pattern" 2>/dev/null
                    found_residual=true
                fi
            done

            if [ "$found_residual" = true ]; then
                sleep 2
                echo "   殘留進程已清理"
            fi

            # ========== 階段 5: 僵屍進程清理 ==========
            local zombie=$(pgrep -af "rcs_core" | grep "defunct" | awk '{print $1}')
            if [ -n "$zombie" ]; then
                echo "🧹 清理殭屍進程..."
                # 發送 SIGCHLD 給 init 進程，讓它回收殭屍進程
                kill -SIGCHLD 1 2>/dev/null || true
            fi

            # ========== 階段 6: 臨時文件清理 ==========
            echo "🧹 清理臨時文件..."
            local launch_params_count=$(find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 2>/dev/null | wc -l)
            if [ "$launch_params_count" -gt 0 ]; then
                find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 -exec rm -rf {} + 2>/dev/null
                echo "   清理了 $launch_params_count 個 launch_params 臨時目錄"
            fi

            echo "✅ RCS 核心已停止"
            ;;

        restart)
            manage_rcs_core stop
            sleep 2
            manage_rcs_core start
            ;;

        status)
            if [ -f "$RCS_CORE_PID_FILE" ]; then
                local all_running=true
                local pids=""

                while read pid; do
                    if kill -0 $pid 2>/dev/null; then
                        pids="$pids $pid"
                    else
                        all_running=false
                    fi
                done < "$RCS_CORE_PID_FILE"

                if [ "$all_running" = true ]; then
                    echo "✅ RCS 核心運行中 (PIDs:$pids)"

                    # 顯示 Launch 進程信息
                    local launch_pid=$(pgrep -f "rcs_launch.py" | head -n1)
                    if [ -n "$launch_pid" ]; then
                        echo "   Launch 進程: PID $launch_pid"
                    fi

                    # 顯示 Core 進程信息
                    local core_pid=$(pgrep -f "lib/rcs/rcs_core" | head -n1)
                    if [ -n "$core_pid" ]; then
                        echo "   RCS Core: PID $core_pid"
                    fi

                    return 0
                else
                    echo "⚠️ RCS 核心部分進程未運行"
                    rm -f "$RCS_CORE_PID_FILE"
                    return 1
                fi
            else
                echo "🚫 RCS 核心未運行"

                # 檢查是否有殭屍進程
                local zombie=$(pgrep -af "rcs_core" | grep "defunct" | wc -l)
                if [ "$zombie" -gt 0 ]; then
                    echo "   ⚠️ 發現 $zombie 個殭屍進程"
                fi

                return 1
            fi
            ;;

        logs)
            if [ -f "$RCS_CORE_LOG_FILE" ]; then
                echo "📄 RCS 核心實時日誌 (Ctrl+C 退出):"
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                tail -f "$RCS_CORE_LOG_FILE"
            else
                echo "❌ 日誌文件不存在: $RCS_CORE_LOG_FILE"
                return 1
            fi
            ;;

        *)
            echo "用法: manage_rcs_core {start|stop|restart|status|logs}"
            echo ""
            echo "指令說明："
            echo "  start   - 啟動 RCS 核心"
            echo "  stop    - 停止 RCS 核心"
            echo "  restart - 重啟 RCS 核心"
            echo "  status  - 查看服務狀態"
            echo "  logs    - 查看實時日誌"
            return 1
            ;;
    esac
}

# =============================================================================
# 資料庫節點管理函數（AGVC）
# =============================================================================
#
# 功能說明：
#   管理 AGVC 資料庫節點的啟動、停止、重啟和狀態檢查
#
# 依賴檢查：
#   - 工作空間：db_proxy_ws
#   - ROS2 節點：agvc_database_node
#   - PostgreSQL：需要資料庫服務運行
#
# 使用方式：
#   manage_agvc_database_node start   - 啟動服務
#   manage_agvc_database_node stop    - 停止服務
#   manage_agvc_database_node restart - 重啟服務
#   manage_agvc_database_node status  - 查看服務狀態
#   manage_agvc_database_node logs    - 查看實時日誌
#
# =============================================================================

manage_agvc_database_node() {
    local DB_NODE_LOG_FILE="/tmp/agvc_database_node.log"
    local DB_NODE_PID_FILE="/tmp/agvc_database_node.pid"

    # 環境檢查
    if ! is_agvc_environment; then
        echo "❌ 此功能僅適用於 AGVC 環境"
        return 1
    fi

    case "$1" in
        start)
            # ========== 階段 1: 啟動前檢查 ==========

            # 1.1 檢查是否已運行（幂等性）
            if [ -f "$DB_NODE_PID_FILE" ]; then
                local all_running=true
                while read pid; do
                    if ! kill -0 $pid 2>/dev/null; then
                        all_running=false
                        break
                    fi
                done < "$DB_NODE_PID_FILE"

                if [ "$all_running" = true ]; then
                    echo "✅ 資料庫節點已經在運行中"
                    return 0
                else
                    echo "⏳ 檢測到過時的 PID 文件，正在清理..."
                    rm -f "$DB_NODE_PID_FILE"
                fi
            fi

            # 1.2 檢查工作空間是否已建置
            if [ ! -d "/app/db_proxy_ws/install" ]; then
                echo "⚠️ 警告: db_proxy_ws 未建置，請先執行: build_ws db_proxy_ws"
            fi

            # ========== 階段 2: 啟動服務 ==========

            echo "🚀 啟動資料庫節點..."

            # 確保日誌檔案存在
            touch "$DB_NODE_LOG_FILE"

            # 啟動節點
            nohup bash -c "source /app/setup.bash && agvc_source > /dev/null 2>&1 && ros2 run db_proxy agvc_database_node --ros-args -r __ns:=/agvc" > "$DB_NODE_LOG_FILE" 2>&1 &
            local PARENT_PID=$!

            # 記錄父進程
            echo $PARENT_PID > "$DB_NODE_PID_FILE"

            # 等待子進程啟動
            sleep 3

            # 記錄子進程
            local CHILD_PIDS=$(pgrep -P $PARENT_PID)
            if [ -n "$CHILD_PIDS" ]; then
                for pid in $CHILD_PIDS; do
                    echo $pid >> "$DB_NODE_PID_FILE"
                done
            fi

            # 記錄實際的服務進程
            sleep 2
            local SERVICE_PID=$(pgrep -f "agvc_database_node" | head -n1)
            if [ -n "$SERVICE_PID" ]; then
                if ! grep -q "^$SERVICE_PID$" "$DB_NODE_PID_FILE" 2>/dev/null; then
                    echo $SERVICE_PID >> "$DB_NODE_PID_FILE"
                fi
            fi

            # ========== 階段 3: 驗證啟動 ==========

            # 驗證父進程
            if ! kill -0 $PARENT_PID 2>/dev/null; then
                echo "❌ 資料庫節點啟動失敗（父進程未運行）"
                echo ""
                echo "💡 診斷建議："
                echo "   1. 查看日誌: tail -f $DB_NODE_LOG_FILE"
                echo "   2. 檢查工作空間建置: ls /app/db_proxy_ws/install"
                echo "   3. 檢查資料庫連接: psql -h 192.168.100.254 -U agvc -d agvc"
                echo "   4. 重新建置: cd /app/db_proxy_ws && colcon build --packages-select db_proxy"
                return 1
            fi

            echo "✅ 資料庫節點已啟動"
            echo "   記錄的 PID: $(cat $DB_NODE_PID_FILE | tr '\n' ' ')"

            # 驗證 ROS2 節點注冊
            if verify_ros2_node_startup "agvc_database_node" 15; then
                echo "✅ ROS2 節點已註冊"
                echo "✅ 資料庫節點啟動完成"
            else
                echo "⚠️ ROS2 節點註冊超時"
                echo ""
                echo "💡 建議："
                echo "   1. 查看日誌: tail -f $DB_NODE_LOG_FILE"
                echo "   2. 檢查 Zenoh Router: manage_zenoh status"
                return 1
            fi
            ;;

        stop)
            echo "⏳ 停止資料庫節點..."

            # ========== 階段 1: 優雅停止 ==========
            if [ -f "$DB_NODE_PID_FILE" ]; then
                local PIDS=$(tac "$DB_NODE_PID_FILE")
                for pid in $PIDS; do
                    if kill -0 $pid 2>/dev/null; then
                        echo "   停止進程 PID: $pid"
                        kill $pid 2>/dev/null
                    fi
                done

                sleep 3

                # ========== 階段 2: 強制終止 ==========
                for pid in $PIDS; do
                    if kill -0 $pid 2>/dev/null; then
                        echo "   強制終止 PID: $pid"
                        kill -9 $pid 2>/dev/null
                    fi
                done

                rm -f "$DB_NODE_PID_FILE"
            fi

            # ========== 階段 3: 備用清理 ==========
            if [ ! -f "$DB_NODE_PID_FILE" ]; then
                echo "🚨 PID 文件未找到，檢查相關進程..."
                if pgrep -f "agvc_database_node" > /dev/null; then
                    echo "   發現進程: agvc_database_node"
                    pkill -f "agvc_database_node"
                    sleep 2
                    echo "   相關進程已停止"
                fi
            fi

            # ========== 階段 4: 殘留進程清理 ==========
            echo "🔍 檢查並清理殘留進程..."
            if pgrep -f "agvc_database_node" > /dev/null; then
                echo "   發現殘留進程: agvc_database_node"
                pkill -9 -f "agvc_database_node" 2>/dev/null
                sleep 2
                echo "   殘留進程已清理"
            fi

            # ========== 階段 5: 臨時文件清理 ==========
            echo "🧹 清理臨時文件..."
            local launch_params_count=$(find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 2>/dev/null | wc -l)
            if [ "$launch_params_count" -gt 0 ]; then
                find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 -exec rm -rf {} + 2>/dev/null
                echo "   清理了 $launch_params_count 個 launch_params 臨時目錄"
            fi

            echo "✅ 資料庫節點已停止"
            ;;

        restart)
            manage_agvc_database_node stop
            sleep 2
            manage_agvc_database_node start
            ;;

        status)
            if [ -f "$DB_NODE_PID_FILE" ]; then
                local all_running=true
                local pids=""

                while read pid; do
                    if kill -0 $pid 2>/dev/null; then
                        pids="$pids $pid"
                    else
                        all_running=false
                    fi
                done < "$DB_NODE_PID_FILE"

                if [ "$all_running" = true ]; then
                    echo "✅ 資料庫節點運行中 (PIDs:$pids)"

                    # 顯示 ROS2 節點信息
                    if ros2 node list 2>/dev/null | grep -q "agvc_database_node"; then
                        echo "   ROS2 節點: agvc_database_node 已註冊"
                        # 顯示部分服務列表
                        local services=$(ros2 service list 2>/dev/null | grep -E "(sql_query|carrier_query|rack_query)" | head -3)
                        if [ -n "$services" ]; then
                            echo "   資料庫服務: 已就緒"
                        fi
                    fi

                    return 0
                else
                    echo "⚠️ 資料庫節點部分進程未運行"
                    rm -f "$DB_NODE_PID_FILE"
                    return 1
                fi
            else
                echo "🚫 資料庫節點未運行"
                return 1
            fi
            ;;

        logs)
            if [ -f "$DB_NODE_LOG_FILE" ]; then
                echo "📄 資料庫節點實時日誌 (Ctrl+C 退出):"
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                tail -f "$DB_NODE_LOG_FILE"
            else
                echo "❌ 日誌文件不存在: $DB_NODE_LOG_FILE"
                return 1
            fi
            ;;

        *)
            echo "用法: manage_agvc_database_node {start|stop|restart|status|logs}"
            echo ""
            echo "指令說明："
            echo "  start   - 啟動資料庫節點"
            echo "  stop    - 停止資料庫節點"
            echo "  restart - 重啟資料庫節點"
            echo "  status  - 查看服務狀態"
            echo "  logs    - 查看實時日誌"
            return 1
            ;;
    esac
}

# =============================================================================
# 🏭 Room Task Build Node 管理
# =============================================================================
manage_room_task_build() {
    # 環境檢測：僅限 AGVC 容器
    if ! is_agvc_environment; then
        echo "❌ 此功能僅適用於 AGVC 環境"
        echo "💡 AGV 容器請使用: check_agv_status"
        return 1
    fi

    local ROOM_TASK_LOG_FILE="/tmp/room_task_build_node.log"
    local ROOM_TASK_PID_FILE="/tmp/room_task_build_node.pid"

    case "$1" in
        start)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  🚀 啟動 Room Task Build Node"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            # ========== 階段 1: 啟動前檢查 ==========
            echo "📋 階段 1/4: 啟動前檢查"

            # 1.1 檢查是否已運行（幂等性）
            if [ -f "$ROOM_TASK_PID_FILE" ]; then
                local all_running=true
                while IFS= read -r pid; do
                    # 跳過空行
                    [ -z "$pid" ] && continue
                    if ! kill -0 "$pid" 2>/dev/null; then
                        all_running=false
                        break
                    fi
                done < "$ROOM_TASK_PID_FILE"

                if [ "$all_running" = true ]; then
                    echo "✅ Room Task Build Node 已經在運行中"
                    echo "💡 如需重新啟動，請使用: manage_room_task_build restart"
                    return 0
                else
                    echo "🧹 清理過期的 PID 文件"
                    rm -f "$ROOM_TASK_PID_FILE"
                fi
            fi

            # ========== 階段 2: 依賴檢查 ==========
            echo "📋 階段 2/4: 依賴檢查"

            # 2.1 檢查資料庫連接
            if ! pgrep -f "db_proxy_node" > /dev/null 2>&1; then
                echo "⚠️ 警告：資料庫代理未運行，Room Task Build 可能無法正常工作"
                echo "💡 建議先啟動: manage_agvc_database_node start"
            fi

            # 2.2 檢查 TAFL WCS（上游服務）
            if ! pgrep -f "tafl_wcs_node" > /dev/null 2>&1; then
                echo "⚠️ 警告：TAFL WCS 未運行，Room Task Build 可能無法接收任務"
                echo "💡 建議先啟動: manage_tafl_wcs start"
            fi

            # 2.3 檢查工作空間建置
            if [ ! -d "/app/tafl_wcs_ws/install" ]; then
                echo "❌ Room Task Build 工作空間未建置"
                echo "💡 診斷建議："
                echo "   1. 執行建置: cd /app/tafl_wcs_ws && colcon build"
                echo "   2. 或使用快速建置: ba"
                return 1
            fi

            # ========== 階段 3: 啟動服務 ==========
            echo "📋 階段 3/4: 啟動服務"

            # 3.1 確保日誌目錄存在
            touch "$ROOM_TASK_LOG_FILE"

            # 3.2 啟動 Room Task Build Node
            echo "🔧 啟動 Room Task Build Node..."
            nohup bash -c "source /app/setup.bash && agvc_source > /dev/null 2>&1 && ros2 run alan_room_task_build room_task_build_node --ros-args -r __ns:=/agvc" > "$ROOM_TASK_LOG_FILE" 2>&1 &
            local PARENT_PID=$!

            # 3.3 記錄父進程 PID
            echo "$PARENT_PID" > "$ROOM_TASK_PID_FILE"
            echo "  📌 父進程 PID: $PARENT_PID"

            # 3.4 等待 1 秒讓子進程啟動
            sleep 1

            # 3.5 記錄子進程 PID（bash -c 產生的子 shell）
            local CHILD_PIDS=$(pgrep -P $PARENT_PID)
            if [ -n "$CHILD_PIDS" ]; then
                echo "  📌 子進程 PIDs: $CHILD_PIDS"
                echo "$CHILD_PIDS" >> "$ROOM_TASK_PID_FILE"
            fi

            # 3.6 記錄實際服務進程（python3 執行節點）
            sleep 1
            local SERVICE_PIDS=$(pgrep -f "room_task_build_node")
            if [ -n "$SERVICE_PIDS" ]; then
                echo "  📌 服務進程 PIDs: $SERVICE_PIDS"
                echo "$SERVICE_PIDS" >> "$ROOM_TASK_PID_FILE"
            fi

            # ========== 階段 4: 驗證啟動 ==========
            echo "📋 階段 4/4: 驗證啟動"

            # 4.1 驗證父進程
            if ! kill -0 $PARENT_PID 2>/dev/null; then
                echo "❌ Room Task Build Node 啟動失敗（父進程未運行）"
                echo "💡 診斷建議："
                echo "   1. 查看日誌: tail -f $ROOM_TASK_LOG_FILE"
                echo "   2. 檢查工作空間建置: ls /app/tafl_wcs_ws/install"
                echo "   3. 驗證環境: source /app/setup.bash && agvc_source && ros2 pkg list | grep alan_room_task_build"
                rm -f "$ROOM_TASK_PID_FILE"
                return 1
            fi

            # 4.2 使用 ros2 node list 驗證
            # Room Task Build 是任務建置核心節點，必須確保節點正常註冊到 ROS 2 網路
            # 參考決策樹：方法 1️⃣ verify_ros2_node_startup() 用於核心服務的深度驗證
            echo "  🔍 等待 ROS 2 節點註冊..."
            if verify_ros2_node_startup "/agvc/room_task_build_node" 15; then
                echo "  ✅ ROS 2 節點已註冊"
            else
                echo "  ❌ ROS 2 節點註冊失敗或超時"
                echo "💡 診斷建議："
                echo "   1. 查看日誌: tail -f $ROOM_TASK_LOG_FILE"
                echo "   2. 檢查 ROS 2 環境: ros2 node list"
                echo "   3. 檢查 Zenoh Router: check_zenoh_status"
                echo "   4. 驗證網路: ros2 doctor --report"

                # 清理失敗的啟動
                echo "🧹 清理失敗的啟動..."
                manage_room_task_build stop > /dev/null 2>&1
                return 1
            fi

            # 4.3 最終進程驗證
            if pgrep -f "room_task_build_node" > /dev/null 2>&1; then
                echo "  ✅ 服務進程運行正常"
            else
                echo "  ⚠️ 警告：服務進程可能異常"
            fi

            # 4.4 顯示啟動摘要
            echo ""
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "✅ Room Task Build Node 啟動完成"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "📝 日誌檔案: $ROOM_TASK_LOG_FILE"
            echo "📊 PID 文件: $ROOM_TASK_PID_FILE"
            echo "💡 查看日誌: manage_room_task_build logs"
            echo "💡 檢查狀態: manage_room_task_build status"
            echo ""

            # 顯示節點資訊
            echo "📊 節點資訊："
            ros2 node info /agvc/room_task_build_node 2>/dev/null | head -n 10 || echo "  ⚠️ 無法獲取節點資訊"
            ;;

        stop)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  🛑 停止 Room Task Build Node"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            # ========== 階段 1: 優雅停止主進程 (SIGTERM) ==========
            echo "📋 階段 1/6: 優雅停止 (SIGTERM)"

            if [ -f "$ROOM_TASK_PID_FILE" ]; then
                local stopped_count=0
                while IFS= read -r pid; do
                    [ -z "$pid" ] && continue
                    if kill -0 "$pid" 2>/dev/null; then
                        echo "  🔄 發送 SIGTERM 到進程 $pid"
                        kill -TERM "$pid" 2>/dev/null || true
                        ((stopped_count++))
                    fi
                done < <(tac "$ROOM_TASK_PID_FILE")

                if [ $stopped_count -gt 0 ]; then
                    echo "  ⏳ 等待 3 秒讓進程優雅退出..."
                    sleep 3
                fi
            else
                echo "  ℹ️ 未找到 PID 文件"
            fi

            # ========== 階段 2: 強制終止殘留進程 (SIGKILL) ==========
            echo "📋 階段 2/6: 強制終止 (SIGKILL)"

            if [ -f "$ROOM_TASK_PID_FILE" ]; then
                local killed_count=0
                while IFS= read -r pid; do
                    [ -z "$pid" ] && continue
                    if kill -0 "$pid" 2>/dev/null; then
                        echo "  ⚠️ 進程 $pid 仍在運行，強制終止"
                        kill -KILL "$pid" 2>/dev/null || true
                        ((killed_count++))
                    fi
                done < <(tac "$ROOM_TASK_PID_FILE")

                if [ $killed_count -gt 0 ]; then
                    echo "  ✅ 已強制終止 $killed_count 個殘留進程"
                    sleep 1
                fi
            fi

            # ========== 階段 3: 備份清理（檢查僵屍進程） ==========
            echo "📋 階段 3/6: 備份清理"

            local zombie_pids=$(pgrep -f "room_task_build_node" | while read p; do
                if [ -d "/proc/$p" ] && grep -q "Z (zombie)" "/proc/$p/status" 2>/dev/null; then
                    echo $p
                fi
            done)

            if [ -n "$zombie_pids" ]; then
                echo "  🧟 發現僵屍進程: $zombie_pids"
                echo "  🔧 嘗試清理僵屍進程的父進程"
                for zpid in $zombie_pids; do
                    local parent=$(ps -o ppid= -p $zpid 2>/dev/null | tr -d ' ')
                    if [ -n "$parent" ] && [ "$parent" != "1" ]; then
                        echo "    終止父進程 $parent"
                        kill -KILL "$parent" 2>/dev/null || true
                    fi
                done
                sleep 1
            fi

            # ========== 階段 4: 殘留進程清理 ==========
            echo "📋 階段 4/6: 殘留進程清理"

            local remaining_pids=$(pgrep -f "room_task_build_node")
            if [ -n "$remaining_pids" ]; then
                echo "  🔍 發現殘留進程: $remaining_pids"
                for pid in $remaining_pids; do
                    echo "    終止進程 $pid"
                    kill -KILL "$pid" 2>/dev/null || true
                done
                sleep 1
            else
                echo "  ✅ 無殘留進程"
            fi

            # ========== 階段 5: 端口資源釋放 ==========
            echo "📋 階段 5/6: 端口資源釋放"

            # Room Task Build 主要使用 ROS 2 DDS/Zenoh，無特定端口需要釋放
            echo "  ✅ Room Task Build 無需額外端口清理"

            # ========== 階段 6: 清理臨時文件 ==========
            echo "📋 階段 6/6: 清理臨時文件"

            rm -f "$ROOM_TASK_PID_FILE"
            echo "  ✅ 已清理 PID 文件"

            echo ""
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "✅ Room Task Build Node 已完全停止"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            ;;

        restart)
            echo "🔄 重新啟動 Room Task Build Node..."
            manage_room_task_build stop
            sleep 2
            manage_room_task_build start
            ;;

        status)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  📊 Room Task Build Node 狀態檢查"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            # 檢查進程狀態
            if pgrep -f "room_task_build_node" > /dev/null 2>&1; then
                local pids=$(pgrep -f "room_task_build_node")
                echo "✅ Room Task Build Node 正在運行"
                echo "   PIDs: $pids"

                # 檢查 PID 文件
                if [ -f "$ROOM_TASK_PID_FILE" ]; then
                    echo "   PID 文件: $ROOM_TASK_PID_FILE"
                    echo "   記錄的 PIDs: $(cat $ROOM_TASK_PID_FILE | tr '\n' ' ')"
                fi

                # 檢查 ROS 2 節點狀態
                echo ""
                echo "🔍 ROS 2 節點狀態："
                if ros2 node list 2>/dev/null | grep -q "/agvc/room_task_build_node"; then
                    echo "  ✅ room_task_build_node 節點已註冊到 ROS 2 網路"

                    # 顯示節點資訊
                    echo ""
                    echo "📊 節點詳細資訊："
                    ros2 node info /agvc/room_task_build_node 2>/dev/null | head -n 15 || echo "  ⚠️ 無法獲取節點資訊"
                else
                    echo "  ⚠️ room_task_build_node 節點未在 ROS 2 網路中註冊"
                    echo "  💡 可能原因："
                    echo "     1. 節點仍在啟動中"
                    echo "     2. Zenoh Router 未運行"
                    echo "     3. 網路連接問題"
                fi

                # 顯示最新日誌
                if [ -f "$ROOM_TASK_LOG_FILE" ]; then
                    echo ""
                    echo "📜 最新日誌 (最後 5 行):"
                    tail -n 5 "$ROOM_TASK_LOG_FILE"
                fi
            else
                echo "🚫 Room Task Build Node 未在運行"

                if [ -f "$ROOM_TASK_PID_FILE" ]; then
                    echo "⚠️ 發現殘留的 PID 文件"
                    echo "💡 建議執行: manage_room_task_build stop"
                fi
            fi

            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            ;;

        logs)
            if [ -f "$ROOM_TASK_LOG_FILE" ]; then
                echo "📜 Room Task Build Node 日誌 (Ctrl+C 退出):"
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                tail -f "$ROOM_TASK_LOG_FILE"
            else
                echo "❌ 找不到日誌檔案: $ROOM_TASK_LOG_FILE"
                echo "💡 服務可能未啟動過，請先執行: manage_room_task_build start"
                return 1
            fi
            ;;

        *)
            echo "用法: manage_room_task_build {start|stop|restart|status|logs}"
            return 1
            ;;
    esac
}

# =============================================================================
# 📦 Transfer Box Task Build Node 管理
# =============================================================================
manage_transfer_box_task_build() {
    # 環境檢測：僅限 AGVC 容器
    if ! is_agvc_environment; then
        echo "❌ 此功能僅適用於 AGVC 環境"
        echo "💡 AGV 容器請使用: check_agv_status"
        return 1
    fi

    local TRANSFER_BOX_LOG_FILE="/tmp/transfer_box_task_build_node.log"
    local TRANSFER_BOX_PID_FILE="/tmp/transfer_box_task_build_node.pid"

    case "$1" in
        start)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  🚀 啟動 Transfer Box Task Build Node"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            # ========== 階段 1: 啟動前檢查 ==========
            echo "📋 階段 1/4: 啟動前檢查"

            # 1.1 檢查是否已運行（幂等性）
            if [ -f "$TRANSFER_BOX_PID_FILE" ]; then
                local all_running=true
                while IFS= read -r pid; do
                    # 跳過空行
                    [ -z "$pid" ] && continue
                    if ! kill -0 "$pid" 2>/dev/null; then
                        all_running=false
                        break
                    fi
                done < "$TRANSFER_BOX_PID_FILE"

                if [ "$all_running" = true ]; then
                    echo "✅ Transfer Box Task Build Node 已經在運行中"
                    echo "💡 如需重新啟動，請使用: manage_transfer_box_task_build restart"
                    return 0
                else
                    echo "🧹 清理過期的 PID 文件"
                    rm -f "$TRANSFER_BOX_PID_FILE"
                fi
            fi

            # ========== 階段 2: 依賴檢查 ==========
            echo "📋 階段 2/4: 依賴檢查"

            # 2.1 檢查資料庫連接
            if ! pgrep -f "db_proxy_node" > /dev/null 2>&1; then
                echo "⚠️ 警告：資料庫代理未運行，Transfer Box Task Build 可能無法正常工作"
                echo "💡 建議先啟動: manage_agvc_database_node start"
            fi

            # 2.2 檢查 PLC Proxy（必要依賴）
            if ! pgrep -f "plc_service" > /dev/null 2>&1; then
                echo "❌ PLC 代理未運行，Transfer Box Task Build 無法與 PLC 通訊"
                echo "💡 必須先啟動: manage_plc_service_agvc start"
                return 1
            fi

            # 2.3 檢查工作空間建置
            if [ ! -d "/app/wcs_ws/install" ]; then
                echo "❌ Transfer Box Task Build 工作空間未建置"
                echo "💡 診斷建議："
                echo "   1. 執行建置: cd /app/wcs_ws && colcon build"
                echo "   2. 或使用快速建置: ba"
                return 1
            fi

            # 2.4 檢查套件是否已安裝
            if [ ! -d "/app/wcs_ws/install/transfer_box_task_build" ]; then
                echo "❌ transfer_box_task_build 套件未安裝"
                echo "💡 診斷建議："
                echo "   1. 執行建置: cd /app/wcs_ws && colcon build --packages-select transfer_box_task_build"
                echo "   2. 或使用快速建置: ba"
                return 1
            fi

            # ========== 階段 3: 啟動服務 ==========
            echo "📋 階段 3/4: 啟動服務"

            # 3.1 確保日誌目錄存在
            touch "$TRANSFER_BOX_LOG_FILE"

            # 3.2 啟動 Transfer Box Task Build Node (使用 launch 檔案)
            echo "🔧 啟動 Transfer Box Task Build Node..."
            nohup bash -c "source /app/setup.bash && agvc_source > /dev/null 2>&1 && ros2 launch transfer_box_task_build transfer_box_task_build.launch.py" > "$TRANSFER_BOX_LOG_FILE" 2>&1 &
            local PARENT_PID=$!

            # 3.3 記錄父進程 PID
            echo "$PARENT_PID" > "$TRANSFER_BOX_PID_FILE"
            echo "  📌 父進程 PID: $PARENT_PID"

            # 3.4 等待 1 秒讓子進程啟動
            sleep 1

            # 3.5 記錄子進程 PID（bash -c 產生的子 shell）
            local CHILD_PIDS=$(pgrep -P $PARENT_PID)
            if [ -n "$CHILD_PIDS" ]; then
                echo "  📌 子進程 PIDs: $CHILD_PIDS"
                echo "$CHILD_PIDS" >> "$TRANSFER_BOX_PID_FILE"
            fi

            # 3.6 記錄實際服務進程（python3 執行節點）
            sleep 1
            local SERVICE_PIDS=$(pgrep -f "transfer_box_task_build_node")
            if [ -n "$SERVICE_PIDS" ]; then
                echo "  📌 服務進程 PIDs: $SERVICE_PIDS"
                echo "$SERVICE_PIDS" >> "$TRANSFER_BOX_PID_FILE"
            fi

            # ========== 階段 4: 驗證啟動 ==========
            echo "📋 階段 4/4: 驗證啟動"

            # 4.1 驗證父進程
            if ! kill -0 $PARENT_PID 2>/dev/null; then
                echo "❌ Transfer Box Task Build Node 啟動失敗（父進程未運行）"
                echo "💡 診斷建議："
                echo "   1. 查看日誌: tail -f $TRANSFER_BOX_LOG_FILE"
                echo "   2. 檢查工作空間建置: ls /app/wcs_ws/install/transfer_box_task_build"
                echo "   3. 驗證環境: source /app/setup.bash && agvc_source && ros2 pkg list | grep transfer_box_task_build"
                rm -f "$TRANSFER_BOX_PID_FILE"
                return 1
            fi

            # 4.2 使用 ros2 node list 驗證
            # Transfer Box Task Build 是傳送箱任務建置節點，負責 PLC 雙向通訊與自動任務建立
            # 參考決策樹：方法 1️⃣ verify_ros2_node_startup() 用於核心服務的深度驗證
            echo "  🔍 等待 ROS 2 節點註冊..."
            if verify_ros2_node_startup "/agvc/transfer_box_task_build_node" 15; then
                echo "  ✅ ROS 2 節點已註冊"
            else
                echo "  ❌ ROS 2 節點註冊失敗或超時"
                echo "💡 診斷建議："
                echo "   1. 查看日誌: tail -f $TRANSFER_BOX_LOG_FILE"
                echo "   2. 檢查 ROS 2 環境: ros2 node list"
                echo "   3. 檢查 Zenoh Router: check_zenoh_status"
                echo "   4. 驗證網路: ros2 doctor --report"

                # 清理失敗的啟動
                echo "🧹 清理失敗的啟動..."
                manage_transfer_box_task_build stop > /dev/null 2>&1
                return 1
            fi

            # 4.3 最終進程驗證
            if pgrep -f "transfer_box_task_build_node" > /dev/null 2>&1; then
                echo "  ✅ 服務進程運行正常"
            else
                echo "  ⚠️ 警告：服務進程可能異常"
            fi

            # 4.4 顯示啟動摘要
            echo ""
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "✅ Transfer Box Task Build Node 啟動完成"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "📝 日誌檔案: $TRANSFER_BOX_LOG_FILE"
            echo "📊 PID 文件: $TRANSFER_BOX_PID_FILE"
            echo "💡 查看日誌: manage_transfer_box_task_build logs"
            echo "💡 檢查狀態: manage_transfer_box_task_build status"
            echo ""

            # 顯示節點資訊
            echo "📊 節點資訊："
            ros2 node info /agvc/transfer_box_task_build_node 2>/dev/null | head -n 10 || echo "  ⚠️ 無法獲取節點資訊"
            ;;

        stop)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  🛑 停止 Transfer Box Task Build Node"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            # ========== 階段 1: 優雅停止主進程 (SIGTERM) ==========
            echo "📋 階段 1/6: 優雅停止 (SIGTERM)"

            if [ -f "$TRANSFER_BOX_PID_FILE" ]; then
                local stopped_count=0
                while IFS= read -r pid; do
                    [ -z "$pid" ] && continue
                    if kill -0 "$pid" 2>/dev/null; then
                        echo "  🔄 發送 SIGTERM 到進程 $pid"
                        kill -TERM "$pid" 2>/dev/null || true
                        ((stopped_count++))
                    fi
                done < <(tac "$TRANSFER_BOX_PID_FILE")

                if [ $stopped_count -gt 0 ]; then
                    echo "  ⏳ 等待 3 秒讓進程優雅退出..."
                    sleep 3
                fi
            else
                echo "  ℹ️ 未找到 PID 文件"
            fi

            # ========== 階段 2: 強制終止殘留進程 (SIGKILL) ==========
            echo "📋 階段 2/6: 強制終止 (SIGKILL)"

            if [ -f "$TRANSFER_BOX_PID_FILE" ]; then
                local killed_count=0
                while IFS= read -r pid; do
                    [ -z "$pid" ] && continue
                    if kill -0 "$pid" 2>/dev/null; then
                        echo "  ⚠️ 進程 $pid 仍在運行，強制終止"
                        kill -KILL "$pid" 2>/dev/null || true
                        ((killed_count++))
                    fi
                done < <(tac "$TRANSFER_BOX_PID_FILE")

                if [ $killed_count -gt 0 ]; then
                    echo "  ✅ 已強制終止 $killed_count 個殘留進程"
                    sleep 1
                fi
            fi

            # ========== 階段 3: 備份清理（檢查僵屍進程） ==========
            echo "📋 階段 3/6: 備份清理"

            local zombie_pids=$(pgrep -f "transfer_box_task_build_node" | while read p; do
                if [ -d "/proc/$p" ] && grep -q "Z (zombie)" "/proc/$p/status" 2>/dev/null; then
                    echo $p
                fi
            done)

            if [ -n "$zombie_pids" ]; then
                echo "  🧟 發現僵屍進程: $zombie_pids"
                echo "  🔧 嘗試清理僵屍進程的父進程"
                for zpid in $zombie_pids; do
                    local parent=$(ps -o ppid= -p $zpid 2>/dev/null | tr -d ' ')
                    if [ -n "$parent" ] && [ "$parent" != "1" ]; then
                        echo "    終止父進程 $parent"
                        kill -KILL "$parent" 2>/dev/null || true
                    fi
                done
                sleep 1
            fi

            # ========== 階段 4: 殘留進程清理 ==========
            echo "📋 階段 4/6: 殘留進程清理"

            local remaining_pids=$(pgrep -f "transfer_box_task_build_node")
            if [ -n "$remaining_pids" ]; then
                echo "  🔍 發現殘留進程: $remaining_pids"
                for pid in $remaining_pids; do
                    echo "    終止進程 $pid"
                    kill -KILL "$pid" 2>/dev/null || true
                done
                sleep 1
            else
                echo "  ✅ 無殘留進程"
            fi

            # ========== 階段 5: 端口資源釋放 ==========
            echo "📋 階段 5/6: 端口資源釋放"

            # Transfer Box Task Build 主要使用 ROS 2 DDS/Zenoh，無特定端口需要釋放
            echo "  ✅ Transfer Box Task Build 無需額外端口清理"

            # ========== 階段 6: 清理臨時文件 ==========
            echo "📋 階段 6/6: 清理臨時文件"

            rm -f "$TRANSFER_BOX_PID_FILE"
            echo "  ✅ 已清理 PID 文件"

            echo ""
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "✅ Transfer Box Task Build Node 已完全停止"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            ;;

        restart)
            echo "🔄 重新啟動 Transfer Box Task Build Node..."
            manage_transfer_box_task_build stop
            sleep 2
            manage_transfer_box_task_build start
            ;;

        status)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  📊 Transfer Box Task Build Node 狀態檢查"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            # 檢查進程狀態
            if pgrep -f "transfer_box_task_build_node" > /dev/null 2>&1; then
                local pids=$(pgrep -f "transfer_box_task_build_node")
                echo "✅ Transfer Box Task Build Node 正在運行"
                echo "   PIDs: $pids"

                # 檢查 PID 文件
                if [ -f "$TRANSFER_BOX_PID_FILE" ]; then
                    echo "   PID 文件: $TRANSFER_BOX_PID_FILE"
                    echo "   記錄的 PIDs: $(cat $TRANSFER_BOX_PID_FILE | tr '\n' ' ')"
                fi

                # 檢查 ROS 2 節點狀態
                echo ""
                echo "🔍 ROS 2 節點狀態："
                if ros2 node list 2>/dev/null | grep -q "/agvc/transfer_box_task_build_node"; then
                    echo "  ✅ transfer_box_task_build_node 節點已註冊到 ROS 2 網路"

                    # 顯示節點資訊
                    echo ""
                    echo "📊 節點詳細資訊："
                    ros2 node info /agvc/transfer_box_task_build_node 2>/dev/null | head -n 15 || echo "  ⚠️ 無法獲取節點資訊"
                else
                    echo "  ⚠️ transfer_box_task_build_node 節點未在 ROS 2 網路中註冊"
                    echo "  💡 可能原因："
                    echo "     1. 節點仍在啟動中"
                    echo "     2. Zenoh Router 未運行"
                    echo "     3. 網路連接問題"
                fi

                # 顯示最新日誌
                if [ -f "$TRANSFER_BOX_LOG_FILE" ]; then
                    echo ""
                    echo "📜 最新日誌 (最後 5 行):"
                    tail -n 5 "$TRANSFER_BOX_LOG_FILE"
                fi
            else
                echo "🚫 Transfer Box Task Build Node 未在運行"

                if [ -f "$TRANSFER_BOX_PID_FILE" ]; then
                    echo "⚠️ 發現殘留的 PID 文件"
                    echo "💡 建議執行: manage_transfer_box_task_build stop"
                fi
            fi

            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            ;;

        logs)
            if [ -f "$TRANSFER_BOX_LOG_FILE" ]; then
                echo "📜 Transfer Box Task Build Node 日誌 (Ctrl+C 退出):"
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                tail -f "$TRANSFER_BOX_LOG_FILE"
            else
                echo "❌ 找不到日誌檔案: $TRANSFER_BOX_LOG_FILE"
                echo "💡 服務可能未啟動過，請先執行: manage_transfer_box_task_build start"
                return 1
            fi
            ;;

        *)
            echo "用法: manage_transfer_box_task_build {start|stop|restart|status|logs}"
            return 1
            ;;
    esac
}

# =============================================================================
# 📦 Cargo Move Task Build Node 管理
# =============================================================================
manage_cargo_move_task_build() {
    # 環境檢測：僅限 AGVC 容器
    if ! is_agvc_environment; then
        echo "❌ 此功能僅適用於 AGVC 環境"
        echo "💡 AGV 容器請使用: check_agv_status"
        return 1
    fi

    local CARGO_TASK_LOG_FILE="/tmp/cargo_move_task_build_node.log"
    local CARGO_TASK_PID_FILE="/tmp/cargo_move_task_build_node.pid"

    case "$1" in
        start)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  🚀 啟動 Cargo Move Task Build Node"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            # ========== 階段 1: 啟動前檢查 ==========
            echo "📋 階段 1/4: 啟動前檢查"

            # 1.1 檢查是否已運行（幂等性）
            if [ -f "$CARGO_TASK_PID_FILE" ]; then
                local all_running=true
                while IFS= read -r pid; do
                    # 跳過空行
                    [ -z "$pid" ] && continue
                    if ! kill -0 "$pid" 2>/dev/null; then
                        all_running=false
                        break
                    fi
                done < "$CARGO_TASK_PID_FILE"

                if [ "$all_running" = true ]; then
                    echo "✅ Cargo Move Task Build Node 已經在運行中"
                    echo "💡 如需重新啟動，請使用: manage_cargo_move_task_build restart"
                    return 0
                else
                    echo "🧹 清理過期的 PID 文件"
                    rm -f "$CARGO_TASK_PID_FILE"
                fi
            fi

            # ========== 階段 2: 依賴檢查 ==========
            echo "📋 階段 2/4: 依賴檢查"

            # 2.1 檢查資料庫連接
            if ! pgrep -f "db_proxy_node" > /dev/null 2>&1; then
                echo "⚠️ 警告：資料庫代理未運行，Cargo Move Task Build 可能無法正常工作"
                echo "💡 建議先啟動: manage_agvc_database_node start"
            fi

            # 2.2 檢查 PLC 服務
            if ! pgrep -f "plc_service" > /dev/null 2>&1; then
                echo "⚠️ 警告：PLC 服務未運行，Cargo Move Task Build 無法與 PLC 通訊"
                echo "💡 建議先啟動: manage_plc_service_agvc start"
            fi

            # 2.3 檢查工作空間建置
            if [ ! -d "/app/wcs_ws/install/cargo_move_task_build" ]; then
                echo "❌ Cargo Move Task Build 工作空間未建置"
                echo "💡 診斷建議："
                echo "   1. 執行建置: cd /app/wcs_ws && colcon build --packages-select cargo_move_task_build"
                echo "   2. 或使用快速建置: ba"
                return 1
            fi

            # ========== 階段 3: 啟動服務 ==========
            echo "📋 階段 3/4: 啟動服務"

            # 3.1 確保日誌目錄存在
            touch "$CARGO_TASK_LOG_FILE"

            # 3.2 啟動 Cargo Move Task Build Node
            echo "🔧 啟動 Cargo Move Task Build Node..."
            nohup bash -c "source /app/setup.bash && agvc_source > /dev/null 2>&1 && ros2 run cargo_move_task_build cargo_move_task_build_node --ros-args -r __ns:=/agvc" > "$CARGO_TASK_LOG_FILE" 2>&1 &
            local PARENT_PID=$!

            # 3.3 記錄父進程 PID
            echo "$PARENT_PID" > "$CARGO_TASK_PID_FILE"
            echo "  📌 父進程 PID: $PARENT_PID"

            # 3.4 等待 1 秒讓子進程啟動
            sleep 1

            # 3.5 記錄子進程 PID（bash -c 產生的子 shell）
            local CHILD_PIDS=$(pgrep -P $PARENT_PID)
            if [ -n "$CHILD_PIDS" ]; then
                echo "  📌 子進程 PIDs: $CHILD_PIDS"
                echo "$CHILD_PIDS" >> "$CARGO_TASK_PID_FILE"
            fi

            # 3.6 記錄實際服務進程（python3 執行節點）
            sleep 1
            local SERVICE_PIDS=$(pgrep -f "cargo_move_task_build_node")
            if [ -n "$SERVICE_PIDS" ]; then
                echo "  📌 服務進程 PIDs: $SERVICE_PIDS"
                echo "$SERVICE_PIDS" >> "$CARGO_TASK_PID_FILE"
            fi

            # ========== 階段 4: 驗證啟動 ==========
            echo "📋 階段 4/4: 驗證啟動"

            # 4.1 驗證父進程
            if ! kill -0 $PARENT_PID 2>/dev/null; then
                echo "❌ Cargo Move Task Build Node 啟動失敗（父進程未運行）"
                echo "💡 診斷建議："
                echo "   1. 查看日誌: tail -f $CARGO_TASK_LOG_FILE"
                echo "   2. 檢查工作空間建置: ls /app/wcs_ws/install/cargo_move_task_build"
                echo "   3. 驗證環境: source /app/setup.bash && agvc_source && ros2 pkg list | grep cargo_move_task_build"
                rm -f "$CARGO_TASK_PID_FILE"
                return 1
            fi

            # 4.2 使用 ros2 node list 驗證
            echo "  🔍 等待 ROS 2 節點註冊..."
            if verify_ros2_node_startup "/agvc/cargo_move_task_build_node" 15; then
                echo "  ✅ ROS 2 節點已註冊"
            else
                echo "  ❌ ROS 2 節點註冊失敗或超時"
                echo "💡 診斷建議:"
                echo "   1. 查看日誌: tail -f $CARGO_TASK_LOG_FILE"
                echo "   2. 檢查 ROS 2 環境: ros2 node list"
                echo "   3. 檢查 Zenoh Router: check_zenoh_status"
                echo "   4. 驗證網路: ros2 doctor --report"

                # 清理失敗的啟動
                echo "🧹 清理失敗的啟動..."
                manage_cargo_move_task_build stop > /dev/null 2>&1
                return 1
            fi

            # 4.3 最終進程驗證
            if pgrep -f "cargo_move_task_build_node" > /dev/null 2>&1; then
                echo "  ✅ 服務進程運行正常"
            else
                echo "  ⚠️ 警告：服務進程可能異常"
            fi

            # 4.4 顯示啟動摘要
            echo ""
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "✅ Cargo Move Task Build Node 啟動完成"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "📝 日誌檔案: $CARGO_TASK_LOG_FILE"
            echo "📊 PID 文件: $CARGO_TASK_PID_FILE"
            echo "💡 查看日誌: manage_cargo_move_task_build logs"
            echo "💡 檢查狀態: manage_cargo_move_task_build status"
            echo ""

            # 顯示節點資訊
            echo "📊 節點資訊："
            ros2 node info /agvc/cargo_move_task_build_node 2>/dev/null | head -n 10 || echo "  ⚠️ 無法獲取節點資訊"
            ;;

        stop)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  🛑 停止 Cargo Move Task Build Node"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            # ========== 階段 1: 優雅停止主進程 (SIGTERM) ==========
            echo "📋 階段 1/6: 優雅停止 (SIGTERM)"

            if [ -f "$CARGO_TASK_PID_FILE" ]; then
                local stopped_count=0
                while IFS= read -r pid; do
                    [ -z "$pid" ] && continue
                    if kill -0 "$pid" 2>/dev/null; then
                        echo "  🔄 發送 SIGTERM 到進程 $pid"
                        kill -TERM "$pid" 2>/dev/null || true
                        ((stopped_count++))
                    fi
                done < <(tac "$CARGO_TASK_PID_FILE")

                if [ $stopped_count -gt 0 ]; then
                    echo "  ⏳ 等待 3 秒讓進程優雅退出..."
                    sleep 3
                fi
            else
                echo "  ℹ️ 未找到 PID 文件"
            fi

            # ========== 階段 2: 強制終止殘留進程 (SIGKILL) ==========
            echo "📋 階段 2/6: 強制終止 (SIGKILL)"

            if [ -f "$CARGO_TASK_PID_FILE" ]; then
                local killed_count=0
                while IFS= read -r pid; do
                    [ -z "$pid" ] && continue
                    if kill -0 "$pid" 2>/dev/null; then
                        echo "  ⚠️ 進程 $pid 仍在運行，強制終止"
                        kill -KILL "$pid" 2>/dev/null || true
                        ((killed_count++))
                    fi
                done < <(tac "$CARGO_TASK_PID_FILE")

                if [ $killed_count -gt 0 ]; then
                    echo "  ✅ 已強制終止 $killed_count 個殘留進程"
                    sleep 1
                fi
            fi

            # ========== 階段 3: 備份清理（檢查僵屍進程） ==========
            echo "📋 階段 3/6: 備份清理"

            local zombie_pids=$(pgrep -f "cargo_move_task_build_node" | while read p; do
                if [ -d "/proc/$p" ] && grep -q "Z (zombie)" "/proc/$p/status" 2>/dev/null; then
                    echo $p
                fi
            done)

            if [ -n "$zombie_pids" ]; then
                echo "  🧟 發現僵屍進程: $zombie_pids"
                echo "  🔧 嘗試清理僵屍進程的父進程"
                for zpid in $zombie_pids; do
                    local parent=$(ps -o ppid= -p $zpid 2>/dev/null | tr -d ' ')
                    if [ -n "$parent" ] && [ "$parent" != "1" ]; then
                        echo "    終止父進程 $parent"
                        kill -KILL "$parent" 2>/dev/null || true
                    fi
                done
                sleep 1
            fi

            # ========== 階段 4: 殘留進程清理 ==========
            echo "📋 階段 4/6: 殘留進程清理"

            local remaining_pids=$(pgrep -f "cargo_move_task_build_node")
            if [ -n "$remaining_pids" ]; then
                echo "  🔍 發現殘留進程: $remaining_pids"
                for pid in $remaining_pids; do
                    echo "    終止進程 $pid"
                    kill -KILL "$pid" 2>/dev/null || true
                done
                sleep 1
            else
                echo "  ✅ 無殘留進程"
            fi

            # ========== 階段 5: 端口資源釋放 ==========
            echo "📋 階段 5/6: 端口資源釋放"
            echo "  ✅ Cargo Move Task Build 無需額外端口清理"

            # ========== 階段 6: 清理臨時文件 ==========
            echo "📋 階段 6/6: 清理臨時文件"

            rm -f "$CARGO_TASK_PID_FILE"
            echo "  ✅ 已清理 PID 文件"

            echo ""
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "✅ Cargo Move Task Build Node 已完全停止"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            ;;

        restart)
            echo "🔄 重新啟動 Cargo Move Task Build Node..."
            manage_cargo_move_task_build stop
            sleep 2
            manage_cargo_move_task_build start
            ;;

        status)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  📊 Cargo Move Task Build Node 狀態檢查"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            # 檢查進程狀態
            if pgrep -f "cargo_move_task_build_node" > /dev/null 2>&1; then
                local pids=$(pgrep -f "cargo_move_task_build_node")
                echo "✅ Cargo Move Task Build Node 正在運行"
                echo "   PIDs: $pids"

                # 檢查 PID 文件
                if [ -f "$CARGO_TASK_PID_FILE" ]; then
                    echo "   PID 文件: $CARGO_TASK_PID_FILE"
                    echo "   記錄的 PIDs: $(cat $CARGO_TASK_PID_FILE | tr '\n' ' ')"
                fi

                # 檢查 ROS 2 節點狀態
                echo ""
                echo "🔍 ROS 2 節點狀態："
                if ros2 node list 2>/dev/null | grep -q "/agvc/cargo_move_task_build_node"; then
                    echo "  ✅ cargo_move_task_build_node 節點已註冊到 ROS 2 網路"

                    # 顯示節點資訊
                    echo ""
                    echo "📊 節點詳細資訊："
                    ros2 node info /agvc/cargo_move_task_build_node 2>/dev/null | head -n 15 || echo "  ⚠️ 無法獲取節點資訊"
                else
                    echo "  ⚠️ cargo_move_task_build_node 節點未在 ROS 2 網路中註冊"
                    echo "  💡 可能原因："
                    echo "     1. 節點仍在啟動中"
                    echo "     2. Zenoh Router 未運行"
                    echo "     3. 網路連接問題"
                fi

                # 顯示最新日誌
                if [ -f "$CARGO_TASK_LOG_FILE" ]; then
                    echo ""
                    echo "📜 最新日誌 (最後 5 行):"
                    tail -n 5 "$CARGO_TASK_LOG_FILE"
                fi
            else
                echo "🚫 Cargo Move Task Build Node 未在運行"

                if [ -f "$CARGO_TASK_PID_FILE" ]; then
                    echo "⚠️ 發現殘留的 PID 文件"
                    echo "💡 建議執行: manage_cargo_move_task_build stop"
                fi
            fi

            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            ;;

        logs)
            if [ -f "$CARGO_TASK_LOG_FILE" ]; then
                echo "📜 Cargo Move Task Build Node 日誌 (Ctrl+C 退出):"
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                tail -f "$CARGO_TASK_LOG_FILE"
            else
                echo "❌ 找不到日誌檔案: $CARGO_TASK_LOG_FILE"
                echo "💡 服務可能未啟動過，請先執行: manage_cargo_move_task_build start"
                return 1
            fi
            ;;

        *)
            echo "用法: manage_cargo_move_task_build {start|stop|restart|status|logs}"
            return 1
            ;;
    esac
}

# =============================================================================
# 📊 統一狀態查看函數
# =============================================================================
# 顯示所有 AGVC 服務的運行狀態（僅狀態查看，不提供統一啟動/停止）
manage() {
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  📊 AGVC 系統服務狀態總覽"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""

    # 檢查環境
    if ! is_agvc_environment; then
        echo "❌ 此功能僅適用於 AGVC 環境"
        echo "💡 AGV 容器請使用: check_agv_status"
        return 1
    fi

    # 定義服務列表（按照邏輯分組）
    echo "🔧 核心服務:"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    check_service_status "PLC 服務              " "plc_service" "manage_plc_service_agvc"
    check_service_status "ECS 核心              " "ecs_core" "manage_ecs_core"
    check_service_status "RCS 核心              " "rcs_core" "manage_rcs_core"
    echo ""

    echo "📦 資料服務:"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    check_service_status "資料庫代理            " "agvc_database_node" "manage_agvc_database_node"
    check_service_status "Room Task Build       " "room_task_build_node" "manage_room_task_build"
    check_service_status "Cargo Move Task Build " "cargo_move_task_build_node" "manage_cargo_move_task_build"
    echo ""

    echo "🌐 Web 服務:"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    check_service_status "API Gateway (8000)    " "api_server" "manage_web_api_launch"
    check_service_status "AGVCUI (8001)         " "agvc_ui_server" "manage_web_api_launch"
    check_service_status "OPUI (8002)           " "op_ui_server" "manage_web_api_launch"
    check_service_status "AGVUI (8003)          " "agv_ui_server" "manage_web_api_launch"
    echo ""

    echo "🔌 基礎服務:"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    check_service_status "Zenoh Router          " "zenohd" "manage_zenoh"
    check_service_status "SSH 服務              " "sshd" "manage_ssh"
    echo ""

    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "💡 管理服務:"
    echo "   • 單獨啟動: manage_<service_name> start"
    echo "   • 單獨停止: manage_<service_name> stop"
    echo "   • 查看狀態: manage_<service_name> status"
    echo "   • 查看日誌: manage_<service_name> logs"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
}

# 檢查單個服務狀態的輔助函數
check_service_status() {
    local display_name="$1"
    local process_name="$2"
    local manage_func="$3"

    # 使用 printf 確保對齊
    printf "  %-25s" "$display_name"

    if pgrep -f "$process_name" > /dev/null 2>&1; then
        local pids=$(pgrep -f "$process_name" | head -n 3 | tr '\n' ',' | sed 's/,$//')
        local pid_count=$(pgrep -f "$process_name" | wc -l)

        if [ $pid_count -gt 3 ]; then
            echo "✅ 運行中 (PIDs: $pids... +$((pid_count-3)) 更多)"
        else
            echo "✅ 運行中 (PIDs: $pids)"
        fi
    else
        echo "🚫 未運行"
    fi
}

# ============================================================================
# AGV 本地 Launch 管理（不通過 SSH）
# ============================================================================

# AGV 本地 Launch 管理函數
# 用於在 AGV 容器內直接管理本地的 launch 服務，不需要通過 SSH
# 注意：AGV_ID 環境變數必須事先設置，否則函數將報錯退出
manage_agv_launch() {
    local action="${1}"
    local AGV_LAUNCH_LOG_FILE="/tmp/agv_launch.log"
    local AGV_LAUNCH_PID_FILE="/tmp/agv_launch.pid"

    # 如果沒有提供參數，顯示幫助信息
    if [ -z "$action" ]; then
        action="help"
    fi

    case "$action" in
        start)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  🚀 啟動 AGV 本地 Launch 服務"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            # 檢查是否已經運行
            if [ -f "$AGV_LAUNCH_PID_FILE" ]; then
                local existing_pid=$(cat "$AGV_LAUNCH_PID_FILE")
                if kill -0 "$existing_pid" 2>/dev/null; then
                    echo "✅ AGV Launch 已經在運行中 (PID: $existing_pid)"
                    echo "💡 使用 'manage_agv_launch restart' 重啟服務"
                    return 0
                else
                    # PID 文件存在但進程不存在，清理
                    rm -f "$AGV_LAUNCH_PID_FILE"
                fi
            fi

            # 檢查 AGV_ID 是否已設置
            if [[ -z "${AGV_ID}" ]]; then
                echo "❌ 錯誤：AGV_ID 環境變數未設置"
                echo ""
                echo "📋 AGV_ID 必須明確設置才能啟動 AGV Launch 服務，以策安全。"
                echo ""
                echo "💡 解決方案："
                echo "   1. 手動設置環境變數："
                echo "      export AGV_ID=cargo01    # 或 loader02, unloader02"
                echo ""
                echo "   2. 檢查設備硬體配置："
                echo "      cat /home/ct/RosAGV/app/config/device_mappings.yaml"
                echo ""
                echo "   3. 支援的 AGV_ID 值："
                echo "      - cargo*     → cargo_mover_agv"
                echo "      - loader*    → loader_agv"
                echo "      - unloader*  → unloader_agv"
                echo ""
                echo "📚 詳細文檔: /home/ct/RosAGV/docs/node-management-system.md"
                return 1
            fi

            # 讀取 AGV_ID 環境變數（必須）
            local agv_id="${AGV_ID}"
            local package_name=""

            # 根據 agv_id 確定 package
            case "$agv_id" in
                cargo*)
                    package_name="cargo_mover_agv"
                    ;;
                loader*)
                    package_name="loader_agv"
                    ;;
                unloader*)
                    package_name="unloader_agv"
                    ;;
                *)
                    echo "❌ 未知的 AGV_ID: $agv_id"
                    echo "💡 請設置環境變數 AGV_ID 為: cargo01, loader02, 或 unloader02"
                    return 1
                    ;;
            esac

            echo "📍 AGV ID: $agv_id"
            echo "📦 Package: $package_name"
            echo "🎯 Action: ros2 launch $package_name launch.py"
            echo ""
            echo "⏳ 啟動 AGV Launch..."

            # 啟動 Launch（後台運行）
            # 注意：移除 -i 標志避免 job control 暫停問題
            nohup bash -c "source /app/setup.bash > /dev/null 2>&1 && agv_source > /dev/null 2>&1 && ros2 launch $package_name launch.py" > "$AGV_LAUNCH_LOG_FILE" 2>&1 &
            local launch_pid=$!
            echo $launch_pid > "$AGV_LAUNCH_PID_FILE"

            echo "📝 Launch 進程已啟動 (PID: $launch_pid)"
            echo "⏳ 等待 AGV 節點啟動..."
            sleep 3

            # 驗證啟動（使用智能重試機制）
            if verify_process_startup "ros2 launch" 10; then
                echo ""
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                echo "  ✅ AGV Launch 啟動成功"
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                echo "📊 檢查 ROS 2 節點狀態..."

                # 檢查關鍵節點
                local nodes=$(ros2 node list 2>/dev/null | grep -E '(plc_service|joy_linux|agv_core)')
                if [ -n "$nodes" ]; then
                    echo "✅ 檢測到 AGV 節點："
                    echo "$nodes" | while read -r node; do
                        echo "   • $node"
                    done
                else
                    echo "⚠️  尚未檢測到 AGV 節點，可能仍在啟動中"
                    echo "💡 使用 'manage_agv_launch status' 檢查狀態"
                fi

                echo ""
                echo "📜 日誌文件: $AGV_LAUNCH_LOG_FILE"
                echo "💡 查看日誌: manage_agv_launch logs"
            else
                echo ""
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                echo "  ❌ AGV Launch 啟動失敗"
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                echo "💡 檢查日誌: tail -50 $AGV_LAUNCH_LOG_FILE"
                rm -f "$AGV_LAUNCH_PID_FILE"
                return 1
            fi
            ;;

        stop)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  🛑 停止 AGV 本地 Launch 服務"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            if [ -f "$AGV_LAUNCH_PID_FILE" ]; then
                local pid=$(cat "$AGV_LAUNCH_PID_FILE")

                if kill -0 "$pid" 2>/dev/null; then
                    echo "⏳ 停止進程 PID: $pid"

                    # 優雅停止：先發送 TERM 信號
                    kill -TERM "$pid" 2>/dev/null

                    # 等待 2 秒
                    sleep 2

                    # 如果還在運行，強制終止
                    if kill -0 "$pid" 2>/dev/null; then
                        echo "⚠️  進程未響應 TERM 信號，強制終止..."
                        kill -9 "$pid" 2>/dev/null || true
                    fi

                    rm -f "$AGV_LAUNCH_PID_FILE"
                    echo "✅ AGV Launch 已停止"
                else
                    echo "⚠️  PID 文件存在但進程不存在"
                    rm -f "$AGV_LAUNCH_PID_FILE"
                fi
            else
                echo "⚠️  未找到 PID 文件"
            fi

            # 額外清理：確保沒有遺留的 launch 進程
            local remaining_procs=$(pgrep -f "ros2 launch.*agv" || true)
            if [ -n "$remaining_procs" ]; then
                echo "🧹 清理遺留進程..."
                pkill -f "ros2 launch.*agv" || true
                echo "✅ 清理完成"
            fi

            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            ;;

        restart)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  🔄 重啟 AGV 本地 Launch 服務"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo ""
            echo "步驟 1/2: 停止現有服務"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            manage_agv_launch stop

            echo ""
            echo "⏳ 等待 2 秒..."
            sleep 2
            echo ""

            echo "步驟 2/2: 啟動新服務"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            manage_agv_launch start
            ;;

        status)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  📊 AGV 本地 Launch 狀態"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            # 檢查 PID 文件和進程
            if [ -f "$AGV_LAUNCH_PID_FILE" ]; then
                local pid=$(cat "$AGV_LAUNCH_PID_FILE")

                if kill -0 "$pid" 2>/dev/null; then
                    echo "✅ AGV Launch 正在運行 (PID: $pid)"

                    # 檢查進程詳情
                    local process_info=$(ps -p "$pid" -o pid,etime,cmd --no-headers 2>/dev/null)
                    if [ -n "$process_info" ]; then
                        echo "📝 進程信息:"
                        echo "   $process_info"
                    fi
                else
                    echo "❌ PID 文件存在但進程不存在 (可能異常終止)"
                    echo "💡 使用 'manage_agv_launch start' 啟動服務"
                    return 1
                fi
            else
                echo "❌ AGV Launch 未運行（未找到 PID 文件）"
                echo "💡 使用 'manage_agv_launch start' 啟動服務"
                return 1
            fi

            echo ""
            echo "🔍 檢查 ROS 2 節點..."

            # 檢查 ROS 2 節點
            local nodes=$(ros2 node list 2>/dev/null | grep -E '(plc_service|joy_linux|agv_core)')
            if [ -n "$nodes" ]; then
                echo "✅ 檢測到 AGV 節點："
                echo "$nodes" | while read -r node; do
                    echo "   • $node"
                done
            else
                echo "⚠️  未檢測到 AGV 節點"
                echo "💡 可能 Launch 仍在啟動中，或啟動失敗"
                echo "💡 檢查日誌: manage_agv_launch logs"
            fi

            echo ""
            echo "📜 日誌文件: $AGV_LAUNCH_LOG_FILE"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            ;;

        logs)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  📜 AGV Launch 日誌"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            if [ -f "$AGV_LAUNCH_LOG_FILE" ]; then
                echo "📍 日誌文件: $AGV_LAUNCH_LOG_FILE"
                echo "💡 按 Ctrl+C 退出日誌查看"
                echo ""
                tail -f "$AGV_LAUNCH_LOG_FILE"
            else
                echo "⚠️  日誌文件不存在"
                echo "💡 可能 AGV Launch 從未啟動過"
                echo "💡 使用 'manage_agv_launch start' 啟動服務"
                return 1
            fi
            ;;

        help|*)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  🚗 AGV 本地 Launch 管理工具"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "用法: manage_agv_launch {start|stop|restart|status|logs}"
            echo ""
            echo "指令說明:"
            echo "  start   - 啟動 AGV Launch 服務"
            echo "  stop    - 停止 AGV Launch 服務"
            echo "  restart - 重啟 AGV Launch 服務"
            echo "  status  - 檢查服務狀態和節點列表"
            echo "  logs    - 查看實時日誌（tail -f）"
            echo ""
            echo "環境需求:"
            echo "  - AGV_ID 環境變數必須設置（cargo01/loader02/unloader02）"
            echo "  - 當前 AGV_ID: ${AGV_ID:-未設置}"
            echo ""
            echo "示例:"
            echo "  manage_agv_launch start    # 啟動服務"
            echo "  manage_agv_launch status   # 檢查狀態"
            echo "  manage_agv_launch logs     # 查看日誌"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

            if [ "$action" != "help" ]; then
                # 不是 help 命令時返回錯誤
                return 1
            fi
            ;;
    esac
}

# ==================== AGV 容器狀態概覽 ====================
# 功能：提供 AGV 容器的統一狀態查看命令
# 作者：Claude Code
# 日期：2025-11-04
# ========================================================

check_agv_status() {
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  🚗 AGV 容器狀態概覽"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    # 1. 基礎服務
    echo ""
    echo "=== 基礎服務 ==="
    manage_ssh status
    echo ""
    manage_zenoh status

    # 2. AGV Launch 狀態
    echo ""
    echo "=== AGV Launch 服務 ==="
    manage_agv_launch status

    # 3. ROS2 節點狀態
    echo ""
    echo "=== ROS 2 節點狀態 ==="
    local nodes=$(ros2 node list 2>/dev/null | grep -E "$(echo $HOSTNAME | tr '[:upper:]' '[:lower:]')" | grep -E '(plc_service|joy_linux|agv_core)')
    if [ -n "$nodes" ]; then
        echo "✅ 檢測到 AGV 節點："
        echo "$nodes" | while read -r node; do
            echo "   • $node"
        done
    else
        echo "⚠️  未檢測到 AGV 節點"
        echo "💡 請確認 AGV Launch 服務是否已啟動"
    fi

    # 4. AGV UI 服務
    echo ""
    echo "=== AGV UI 服務 ==="
    if pgrep -f "agv_ui_server" > /dev/null 2>&1; then
        local ui_pid=$(pgrep -f "agv_ui_server")
        echo "✅ AGV UI Server 運行中 (PID: $ui_pid, Port: 8003)"
    else
        echo "🚫 AGV UI Server 未運行"
    fi

    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "💡 詳細日誌: manage_agv_launch logs"
    echo "💡 重啟服務: manage_agv_launch restart"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
}

# 別名定義
alias local_agv='manage_agv_launch'
alias lagv='manage_agv_launch'

# ===== 模組初始化完成 =====
log_debug "✅ Node Management 模組已載入（含本地 AGV Launch 管理）"

# =============================================================================
# KUKA WCS 節點管理（KUKA Warehouse Control System）
# =============================================================================

manage_kuka_wcs() {
    # 環境檢測：僅限 AGVC 容器
    if ! is_agvc_environment; then
        echo "❌ 此功能僅適用於 AGVC 環境"
        return 1
    fi

    local KUKA_WCS_LOG_FILE="/tmp/kuka_wcs.log"
    local KUKA_WCS_PID_FILE="/tmp/kuka_wcs.pid"
    local NODE_NAME="/agvc/kuka_wcs_node"

    case "$1" in
        start)
            # ===== 階段 1: 啟動前檢查（幂等性驗證）=====
            if [ -f "$KUKA_WCS_PID_FILE" ]; then
                local all_running=true
                while read pid; do
                    if ! kill -0 $pid 2>/dev/null; then
                        all_running=false
                        break
                    fi
                done < "$KUKA_WCS_PID_FILE"
                
                if [ "$all_running" = true ]; then
                    echo "✅ KUKA WCS 節點已經在運行中"
                    echo "   PID: $(cat $KUKA_WCS_PID_FILE | tr '\n' ' ')"
                    return 0
                else
                    echo "🧹 清理過時的 PID 檔案..."
                    rm -f "$KUKA_WCS_PID_FILE"
                fi
            fi
            
            # ===== 階段 2: 依賴檢查 =====
            echo "🔍 啟動前環境檢查..."
            
            # 檢查工作空間是否已建置
            if [ ! -d "/app/kuka_wcs_ws/install" ]; then
                echo "⚠️  警告: kuka_wcs_ws 未建置"
                echo "💡 請先執行: cd /app && build_ws kuka_wcs_ws"
                return 1
            fi
            
            # 檢查資料庫連接
            if ! docker compose -f /home/ct/RosAGV/docker-compose.agvc.yml ps postgres | grep -q "Up"; then
                echo "❌ PostgreSQL 資料庫未運行"
                echo "💡 請先啟動資料庫: docker compose -f docker-compose.agvc.yml up -d postgres"
                return 1
            fi

            # ===== 階段 3: 啟動服務 =====
            echo "🚀 啟動 KUKA WCS 節點..."
            nohup bash -c "source /app/setup.bash && agvc_source > /dev/null 2>&1 && ros2 run kuka_wcs kuka_wcs_node" > "$KUKA_WCS_LOG_FILE" 2>&1 &
            local PARENT_PID=$!
            
            # 記錄父進程
            echo $PARENT_PID > "$KUKA_WCS_PID_FILE"
            
            # 等待子進程啟動
            sleep 3
            
            # 找出並記錄子進程
            local CHILD_PIDS=$(pgrep -P $PARENT_PID)
            if [ -n "$CHILD_PIDS" ]; then
                for pid in $CHILD_PIDS; do
                    echo $pid >> "$KUKA_WCS_PID_FILE"
                done
            fi
            
            # 記錄實際的服務進程
            sleep 2
            local SERVICE_PID=$(pgrep -f "kuka_wcs_node" | head -n1)
            if [ -n "$SERVICE_PID" ] && ! grep -q "^$SERVICE_PID$" "$KUKA_WCS_PID_FILE" 2>/dev/null; then
                echo $SERVICE_PID >> "$KUKA_WCS_PID_FILE"
            fi

            # ===== 階段 4: 驗證啟動（使用 ROS2 節點驗證）=====
            if verify_ros2_node_startup "$NODE_NAME" 15; then
                echo "✅ KUKA WCS 節點已成功啟動"
                echo "   記錄的 PID: $(cat $KUKA_WCS_PID_FILE | tr '\n' ' ')"
                echo "   節點名稱: $NODE_NAME"
                echo "💡 查看日誌: tail -f $KUKA_WCS_LOG_FILE"
                return 0
            else
                echo "❌ KUKA WCS 節點啟動失敗或驗證超時"
                echo "📝 檢查日誌: tail -f $KUKA_WCS_LOG_FILE"
                echo "💡 可能的原因:"
                echo "   - 套件未正確建置 (執行: build_ws kuka_wcs_ws)"
                echo "   - Python 依賴未安裝"
                echo "   - 資料庫連接失敗"
                echo "   - ROS2 環境未正確載入"
                return 1
            fi
            ;;

        stop)
            if [ ! -f "$KUKA_WCS_PID_FILE" ]; then
                echo "⚠️  KUKA WCS 節點未運行（無 PID 檔案）"
                return 0
            fi
            
            # ===== 6 階段停止流程 =====
            echo "⏳ 停止 KUKA WCS 節點..."
            
            # 階段 1: 優雅停止 (SIGTERM)
            local PIDS=$(tac "$KUKA_WCS_PID_FILE")
            for pid in $PIDS; do
                if kill -0 $pid 2>/dev/null; then
                    echo "   停止進程 PID: $pid (SIGTERM)"
                    kill $pid 2>/dev/null
                fi
            done
            
            # 等待進程結束
            sleep 3
            
            # 階段 2: 強制終止 (SIGKILL)
            for pid in $PIDS; do
                if kill -0 $pid 2>/dev/null; then
                    echo "   強制終止 PID: $pid (SIGKILL)"
                    kill -9 $pid 2>/dev/null
                fi
            done
            
            # 階段 3-4: 清理殘留進程
            local remaining=$(pgrep -f "kuka_wcs_node")
            if [ -n "$remaining" ]; then
                echo "   清理殘留進程: $remaining"
                kill -9 $remaining 2>/dev/null
            fi
            
            # 階段 6: 清理臨時文件
            rm -f "$KUKA_WCS_PID_FILE"
            
            echo "✅ KUKA WCS 節點已停止"
            ;;

        restart)
            echo "🔄 重啟 KUKA WCS 節點..."
            manage_kuka_wcs stop
            sleep 2
            manage_kuka_wcs start
            ;;

        status)
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "📊 KUKA WCS 節點狀態"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            
            # 檢查 PID 檔案
            if [ -f "$KUKA_WCS_PID_FILE" ]; then
                local pids=$(cat "$KUKA_WCS_PID_FILE" | tr '\n' ' ')
                echo "✅ PID 檔案存在: $pids"
                
                # 檢查進程是否運行
                local running_count=0
                for pid in $pids; do
                    if kill -0 $pid 2>/dev/null; then
                        ((running_count++))
                    fi
                done
                echo "   運行中的進程數: $running_count / $(wc -l < $KUKA_WCS_PID_FILE)"
            else
                echo "🚫 PID 檔案不存在"
            fi
            
            # 檢查 ROS2 節點
            if ros2 node list 2>/dev/null | grep -q "$NODE_NAME"; then
                echo "✅ ROS2 節點已註冊: $NODE_NAME"
            else
                echo "🚫 ROS2 節點未註冊: $NODE_NAME"
            fi
            
            # 檢查進程
            if pgrep -f "kuka_wcs_node" > /dev/null 2>&1; then
                local proc_pid=$(pgrep -f "kuka_wcs_node")
                echo "✅ 進程運行中 (PID: $proc_pid)"
            else
                echo "🚫 進程未運行"
            fi
            
            # 檢查日誌檔案
            if [ -f "$KUKA_WCS_LOG_FILE" ]; then
                local log_size=$(du -h "$KUKA_WCS_LOG_FILE" | cut -f1)
                echo "📝 日誌檔案: $KUKA_WCS_LOG_FILE ($log_size)"
            else
                echo "   無日誌檔案"
            fi
            
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "💡 詳細日誌: tail -f $KUKA_WCS_LOG_FILE"
            echo "💡 重啟服務: manage_kuka_wcs restart"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            ;;

        logs)
            if [ -f "$KUKA_WCS_LOG_FILE" ]; then
                echo "📝 KUKA WCS 節點日誌（Ctrl+C 退出）:"
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                tail -f "$KUKA_WCS_LOG_FILE"
            else
                echo "❌ 日誌檔案不存在: $KUKA_WCS_LOG_FILE"
                return 1
            fi
            ;;

        *)
            echo "用法: manage_kuka_wcs {start|stop|restart|status|logs}"
            echo ""
            echo "指令說明:"
            echo "  start   - 啟動 KUKA WCS 節點"
            echo "  stop    - 停止 KUKA WCS 節點"
            echo "  restart - 重啟 KUKA WCS 節點"
            echo "  status  - 顯示 KUKA WCS 節點狀態"
            echo "  logs    - 查看 KUKA WCS 節點日誌"
            return 1
            ;;
    esac
}

# 別名定義
alias kuka_wcs='manage_kuka_wcs'

