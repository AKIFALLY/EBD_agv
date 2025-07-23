#!/usr/bin/env python3
"""
task_condition_config.py 使用示範
展示配置管理系統的實際用途
"""

from wcs_base.task_condition_config import (
    get_config_manager, 
    set_real_time_mode, 
    set_query_timeout,
    print_current_config
)

def demo_basic_usage():
    """基本使用示範"""
    print("🎯 基本使用示範")
    print("=" * 50)
    
    # 查看當前配置
    print("\n📋 當前配置:")
    print_current_config()
    
    # 切換到預存結果模式（適合資料庫負荷重的時候）
    print("\n🔄 切換到預存結果模式...")
    set_real_time_mode(False)
    
    # 增加查詢超時時間（適合資料庫回應慢的時候）
    print("\n⏰ 設定查詢超時時間為 60 秒...")
    set_query_timeout(60)
    
    # 查看更新後的配置
    print("\n📋 更新後的配置:")
    print_current_config()

def demo_scenario_management():
    """情境管理示範"""
    print("\n🎭 情境管理示範")
    print("=" * 50)
    
    config_mgr = get_config_manager()
    
    # 情境 1: 高負荷模式（資料庫壓力大）
    print("\n🔥 情境 1: 高負荷模式")
    config_mgr.update_config(
        real_time_mode=False,      # 使用預存結果
        query_timeout=120,         # 增加超時時間
        max_iterations=50,         # 減少迭代次數
        log_sql_queries=False      # 關閉 SQL 日誌
    )
    config_mgr.print_config()
    
    # 情境 2: 除錯模式（需要詳細資訊）
    print("\n🐛 情境 2: 除錯模式")
    config_mgr.update_config(
        real_time_mode=True,       # 使用即時查詢
        query_timeout=30,          # 標準超時時間
        max_iterations=100,        # 標準迭代次數
        log_sql_queries=True,      # 啟用 SQL 日誌
        enable_sql_validation=True # 啟用 SQL 驗證
    )
    config_mgr.print_config()
    
    # 情境 3: 效能模式（追求最佳效能）
    print("\n🚀 情境 3: 效能模式")
    config_mgr.update_config(
        real_time_mode=True,       # 使用即時查詢
        query_timeout=15,          # 短超時時間
        max_iterations=200,        # 增加迭代次數
        log_sql_queries=False,     # 關閉 SQL 日誌
        enable_sql_validation=False # 關閉 SQL 驗證（提升效能）
    )
    config_mgr.print_config()

def demo_real_world_scenarios():
    """真實世界使用場景"""
    print("\n🌍 真實世界使用場景")
    print("=" * 50)
    
    config_mgr = get_config_manager()
    
    # 場景 1: 系統啟動時的初始化
    print("\n🚀 場景 1: 系統啟動初始化")
    print("   - 載入預設配置")
    print("   - 根據環境調整設定")
    
    # 模擬生產環境設定
    if True:  # 假設這是生產環境
        config_mgr.update_config(
            real_time_mode=True,
            query_timeout=30,
            max_iterations=100,
            enable_sql_validation=True,
            log_sql_queries=False
        )
        print("   ✅ 生產環境配置已套用")
    
    # 場景 2: 運行時動態調整
    print("\n⚡ 場景 2: 運行時動態調整")
    print("   - 監控到資料庫負荷過高")
    print("   - 動態切換到預存結果模式")
    
    # 模擬資料庫負荷過高的情況
    database_load_high = True
    if database_load_high:
        set_real_time_mode(False)
        set_query_timeout(60)
        print("   ✅ 已切換到低負荷模式")
    
    # 場景 3: 問題排查
    print("\n🔍 場景 3: 問題排查")
    print("   - 啟用詳細日誌")
    print("   - 增加超時時間")
    
    config_mgr.enable_sql_logging(True)
    config_mgr.set_query_timeout(120)
    print("   ✅ 除錯模式已啟用")

def demo_config_file_management():
    """配置檔案管理示範"""
    print("\n📁 配置檔案管理示範")
    print("=" * 50)
    
    config_mgr = get_config_manager()
    
    # 顯示配置檔案位置
    print(f"📍 配置檔案位置: {config_mgr.config_file}")
    
    # 顯示配置摘要
    summary = config_mgr.get_config_summary()
    print("\n📊 配置摘要:")
    for key, value in summary.items():
        print(f"   {key}: {value}")
    
    # 重置為預設配置
    print("\n🔄 重置為預設配置...")
    config_mgr.reset_to_defaults()
    
    print("\n📋 重置後的配置:")
    config_mgr.print_config()

def main():
    """主示範函數"""
    print("🎮 task_condition_config.py 使用示範")
    print("這個檔案的作用是管理任務條件檢查的所有設定")
    
    # 基本使用
    demo_basic_usage()
    
    # 情境管理
    demo_scenario_management()
    
    # 真實世界場景
    demo_real_world_scenarios()
    
    # 配置檔案管理
    demo_config_file_management()
    
    print("\n🎉 示範完成！")
    print("\n💡 重點總結:")
    print("   1. task_condition_config.py 是一個配置管理中心")
    print("   2. 可以統一管理所有條件檢查相關的設定")
    print("   3. 設定會自動儲存，重啟後仍然有效")
    print("   4. 支援運行時動態調整")
    print("   5. 適合不同的使用場景和環境")

if __name__ == "__main__":
    main()
