# OPUI 配置設定
import os
from typing import Optional

class Settings:
    """OPUI 應用程式配置"""
    
    # 伺服器設定
    HOST: str = os.getenv("OPUI_HOST", "0.0.0.0")
    PORT: int = int(os.getenv("OPUI_PORT", "8002"))
    
    # 資料庫設定
    DATABASE_URL: str = os.getenv(
        "DATABASE_URL", 
        "postgresql+psycopg2://agvc:password@192.168.100.254/agvc?client_encoding=utf8"
    )
    
    # CORS 設定
    CORS_ALLOWED_ORIGINS: str = os.getenv("CORS_ALLOWED_ORIGINS", "*")
    
    # Socket.IO 設定
    SOCKET_CORS_ALLOWED_ORIGINS: str = os.getenv("SOCKET_CORS_ALLOWED_ORIGINS", "*")
    
    # 任務監控設定
    TASK_MONITOR_INTERVAL: float = float(os.getenv("TASK_MONITOR_INTERVAL", "1.0"))
    
    # 日誌設定
    LOG_LEVEL: str = os.getenv("LOG_LEVEL", "INFO")
    
    # 開發模式
    DEBUG: bool = os.getenv("DEBUG", "False").lower() == "true"

# 全域設定實例
settings = Settings()
