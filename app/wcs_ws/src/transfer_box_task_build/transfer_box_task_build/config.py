"""
transfer_box_task_build 配置常數
通用傳送箱任務建立系統配置
"""

# ==================== 資料庫配置 ====================
DATABASE_URL = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'

# ==================== 傳送箱配置列表 ====================
# 每個傳送箱的配置包含：
# - name: 傳送箱名稱
# - type: 傳送箱類型 ("entrance" 入口 / "exit" 出口)
# - location_id: Rack 位置 ID
# - work_ids: [A面 work_id, B面 work_id]
# - dm_write_start: 寫入 PLC DM 起始位址（系統 → PLC）
#   * DM[0~1]: carrier_bitmap (32-bit, 小端序)
#   * DM[2~3]: carrier_enable_bitmap (32-bit, 小端序)
#   * DM[4]: direction 確認值 (16-bit)
# - dm_feedback_start: 回饋 PLC DM 起始位址（PLC → 系統）
#   * DM[0~1]: 更新後的 carrier_bitmap (32-bit, 小端序)
#
# 資料來源：
# - carrier_bitmap: 任務建立前由 Web UI 給予，建立後從 PLC DM3012/3014 feedback 讀取
# - 系統從資料庫 Rack 讀取資訊，寫入到 PLC DM2010~2014 (入口) / DM2020~2024 (出口)
#
# 寫入條件：
# - entrance (入口)：有載具時寫入 (bitmap > 0)
# - exit (出口)：無載具時寫入 (bitmap = 0)

TRANSFER_BOXES = [
    {
        "name": "入口傳送箱",
        "type": "entrance",
        "location_id": 27,
        "work_ids": [2000102, 2002102],  # A面, B面
        "dm_write_start": "2010",        # DM2010-2014 (系統寫入，PLC 讀取)
        "dm_feedback_start": "3012",     # DM3012-3013 (PLC 寫入，系統讀取)
    },
    {
        "name": "出口傳送箱",
        "type": "exit",
        "location_id": 26,
        "work_ids": [2000201, 2001201],  # A面, B面
        "dm_write_start": "2020",        # DM2020-2024 (系統寫入，PLC 讀取)
        "dm_feedback_start": "3014",     # DM3014-3015 (PLC 寫入，系統讀取)
    },
]

# ==================== PLC DM 位址配置（共用）====================
# 所有傳送箱共用的 work_id 讀取位址
DM_READ_WORK_ID_START = "3010"  # DM3010-3011 (32-bit work_id)
DM_READ_WORK_ID_COUNT = 2       # 2個 words

# DM 寫入參數（系統 → PLC）
DM_WRITE_COUNT = 5               # 5個 words (carrier_bitmap[2] + enable_bitmap[2] + direction[1])
DM_WRITE_CONFIRM_VALUE = 1       # DM 確認值預設

# DM 回饋參數（PLC → 系統）
DM_FEEDBACK_COUNT = 2            # 2個 words (A面 + B面 carrier_bitmap)

# ==================== Timer 頻率配置 ====================
# Rack 檢查頻率（秒）- Timer 1
RACK_CHECK_INTERVAL = 3.0

# PLC work_id 監控頻率（秒）- Timer 2
PLC_MONITOR_INTERVAL = 1.0

# 自動清理頻率（秒）- Timer 3
AUTO_CLEANUP_INTERVAL = 2.0

# PLC 回饋更新頻率（秒）- Timer 4
FEEDBACK_UPDATE_INTERVAL = 10.0

# ==================== Task 配置 ====================
# 預設狀態
DEFAULT_STATUS_ID = 1      # PENDING
DEFAULT_PRIORITY = 5
DEFAULT_NODE_ID = None

# AGV 類型
AGV_TYPE_CARGO = "CARGO"

# 清理狀態（已完成或已取消的任務）
CLEANUP_STATUS_IDS = [4, 54]  # 4=COMPLETED, 54=CANCELLED
