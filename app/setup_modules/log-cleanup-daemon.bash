#!/bin/bash
# ============================================================================
# Log Cleanup Daemon - 日誌清理守護進程
# ============================================================================
# 功能：定期自動輪轉 /tmp 下的日誌檔案，防止磁碟空間耗盡
# 執行頻率：每 6 小時（21600 秒）
# 輪轉策略：單檔案最大 10 MB，保留 5 個舊版本
#
# 由 startup.agvc.bash 在容器啟動時自動在背景執行
# ============================================================================

# 載入通用函數（包含 rotate_log_file）
if [ -f "/app/setup_modules/common.bash" ]; then
    source /app/setup_modules/common.bash
else
    echo "[ERROR] 無法載入 common.bash，日誌清理守護進程無法啟動"
    exit 1
fi

# 守護進程自己的日誌檔案
DAEMON_LOG="/tmp/log-cleanup-daemon.log"
DAEMON_PID_FILE="/tmp/log-cleanup-daemon.pid"

# 記錄守護進程 PID
echo $$ > "$DAEMON_PID_FILE"

# 記錄啟動資訊
{
    echo "========================================="
    echo "日誌清理守護進程啟動"
    echo "時間: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "PID: $$"
    echo "執行頻率: 每 6 小時"
    echo "輪轉策略: 單檔案最大 10 MB，保留 5 個版本"
    echo "========================================="
} > "$DAEMON_LOG"

# ============================================================================
# 主循環 - 無限執行
# ============================================================================
while true; do
    # 等待 6 小時 (21600 秒)
    # 第一次啟動後立即執行一次，之後每 6 小時執行
    sleep 21600

    # 記錄執行開始時間
    EXEC_START_TIME=$(date '+%Y-%m-%d %H:%M:%S')
    echo "" >> "$DAEMON_LOG"
    echo "[$EXEC_START_TIME] =========================================" >> "$DAEMON_LOG"
    echo "[$EXEC_START_TIME] 開始執行日誌輪轉檢查..." >> "$DAEMON_LOG"

    # 計數器
    checked_count=0
    rotated_count=0
    error_count=0

    # 輪轉所有 /tmp 下的 .log 檔案
    for log_file in /tmp/*.log; do
        # 檢查檔案是否存在（處理萬用字元無匹配的情況）
        [ -f "$log_file" ] || continue

        # 跳過守護進程自己的日誌
        [ "$log_file" = "$DAEMON_LOG" ] && continue

        # 計數
        ((checked_count++))

        # 執行輪轉（輸出重定向到守護進程日誌）
        if rotate_log_file "$log_file" 10 5 >> "$DAEMON_LOG" 2>&1; then
            # 檢查是否真的執行了輪轉（透過檢查 .log.1 的修改時間）
            if [ -f "${log_file}.1" ]; then
                # 如果 .log.1 是剛剛建立的（1 分鐘內），代表發生了輪轉
                if [ $(find "${log_file}.1" -mmin -1 2>/dev/null | wc -l) -gt 0 ]; then
                    ((rotated_count++))
                    echo "[$EXEC_START_TIME] ✅ 已輪轉: $(basename $log_file)" >> "$DAEMON_LOG"
                fi
            fi
        else
            ((error_count++))
            echo "[$EXEC_START_TIME] ❌ 輪轉失敗: $(basename $log_file)" >> "$DAEMON_LOG"
        fi
    done

    # 記錄執行結果
    EXEC_END_TIME=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$EXEC_END_TIME] 執行完成" >> "$DAEMON_LOG"
    echo "[$EXEC_END_TIME] 統計: 檢查 $checked_count 個，輪轉 $rotated_count 個，錯誤 $error_count 個" >> "$DAEMON_LOG"
    echo "[$EXEC_END_TIME] =========================================" >> "$DAEMON_LOG"

    # ========================================================================
    # 限制守護進程自己的日誌大小（防止無限增長）
    # ========================================================================
    # 保留最近 200 行，大約 10-20 KB
    if [ -f "$DAEMON_LOG" ]; then
        # 檢查日誌行數
        line_count=$(wc -l < "$DAEMON_LOG")
        if [ $line_count -gt 200 ]; then
            tail -n 200 "$DAEMON_LOG" > "${DAEMON_LOG}.tmp"
            mv "${DAEMON_LOG}.tmp" "$DAEMON_LOG"
            echo "[$EXEC_END_TIME] 🧹 守護進程日誌已截斷至 200 行" >> "$DAEMON_LOG"
        fi
    fi
done
