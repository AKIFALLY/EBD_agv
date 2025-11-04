"""
alan_room_task_build 套件配置常數
"""

# ==================== PLC DM 位址配置 ====================

# DM 讀取範圍
DM_START_ADDRESS = "3000"  # 起始位址 DM3000
DM_READ_COUNT = 10          # 讀取 10 個 word (DM3000-DM3009)

# Loader AGV DM 映射（DM3000-3001，2 words）
LOADER_DM_LOW = 0    # DM3000 在陣列中的索引（低位 word）
LOADER_DM_HIGH = 1   # DM3001 在陣列中的索引（高位 word）
LOADER_DM_START_ADDRESS = "3000"  # 清除時使用

# Unloader AGV DM 映射（DM3002-3003，2 words）
UNLOADER_DM_LOW = 2    # DM3002 在陣列中的索引（低位 word）
UNLOADER_DM_HIGH = 3   # DM3003 在陣列中的索引（高位 word）
UNLOADER_DM_START_ADDRESS = "3002"  # 清除時使用

# ==================== 資料庫配置 ====================

# PostgreSQL 連接字串
DATABASE_URL = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"

# ==================== 監控配置 ====================

# 監控頻率（秒）
MONITOR_INTERVAL = 1.0  # 每 1 秒檢查一次 PLC DM

# 觸發模式：邊緣觸發（只在 work_id 值改變時觸發任務建立）
# 不再使用定期檢查，避免重複發送任務

# ==================== 自動清理配置 ====================

# 自動清理頻率（秒）
AUTO_CLEANUP_INTERVAL = 2.0  # 每 2 秒檢查一次並刪除已完成/已取消的 Task

# 需要清理的 Task 狀態
CLEANUP_STATUS_IDS = [4, 54]  # 4=已完成, 54=已取消

# ==================== Task 預設值 ====================

# Task 狀態 ID
DEFAULT_STATUS_ID = 1  # 1 = PENDING (待處理)

# Task 優先級
DEFAULT_PRIORITY = 5  # 優先級 1-10，5 為中等

# AGV ID
DEFAULT_AGV_ID = 0  # 0 = 尚未指派 AGV

# Node ID
DEFAULT_NODE_ID = None  # 預設節點 ID（NULL，後續由系統分配）

# ==================== AGV 類型 ====================

AGV_TYPE_LOADER = "LOADER"
AGV_TYPE_UNLOADER = "UNLOADER"

# ==================== 日誌配置 ====================

# 日誌等級（可選：DEBUG, INFO, WARNING, ERROR）
LOG_LEVEL = "INFO"
