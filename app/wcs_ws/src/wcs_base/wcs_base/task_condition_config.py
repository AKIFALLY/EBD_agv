"""
任務條件檢查配置管理器
提供統一的配置管理和模式切換功能
"""

import os
import json
from typing import Dict, Any, Optional
from dataclasses import dataclass, asdict


@dataclass
class TaskConditionConfig:
    """任務條件檢查配置"""
    real_time_mode: bool = True          # 是否使用即時查詢模式
    query_timeout: int = 30              # SQL 查詢超時時間（秒）
    max_iterations: int = 50            # 最大迭代次數
    enable_sql_validation: bool = True   # 是否啟用 SQL 驗證
    log_sql_queries: bool = False        # 是否記錄 SQL 查詢日誌
    cache_results: bool = False          # 是否快取查詢結果（未來功能）
    cache_ttl: int = 300                 # 快取存活時間（秒）


class TaskConditionConfigManager:
    """任務條件檢查配置管理器"""
    
    def __init__(self, config_file: Optional[str] = None):
        """
        初始化配置管理器
        
        Args:
            config_file: 配置檔案路徑，如果為 None 則使用預設路徑
        """
        self.config_file = config_file or self._get_default_config_path()
        self.config = TaskConditionConfig()
        self._load_config()
    
    def _get_default_config_path(self) -> str:
        """取得預設配置檔案路徑"""
        # 在 wcs_base 目錄下創建 config 資料夾
        base_dir = os.path.dirname(os.path.abspath(__file__))
        config_dir = os.path.join(base_dir, "config")
        os.makedirs(config_dir, exist_ok=True)
        return os.path.join(config_dir, "task_condition_config.json")
    
    def _load_config(self):
        """載入配置檔案"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    config_data = json.load(f)
                    # 更新配置，只更新存在的欄位
                    for key, value in config_data.items():
                        if hasattr(self.config, key):
                            setattr(self.config, key, value)
                print(f"✅ 載入配置檔案: {self.config_file}")
            else:
                # 如果配置檔案不存在，創建預設配置
                self._save_config()
                print(f"📝 創建預設配置檔案: {self.config_file}")
        except Exception as e:
            print(f"⚠️ 載入配置檔案失敗，使用預設配置: {e}")
    
    def _save_config(self):
        """儲存配置檔案"""
        try:
            config_data = asdict(self.config)
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=2, ensure_ascii=False)
            print(f"💾 配置已儲存到: {self.config_file}")
        except Exception as e:
            print(f"❌ 儲存配置檔案失敗: {e}")
    
    def get_config(self) -> TaskConditionConfig:
        """取得當前配置"""
        return self.config
    
    def update_config(self, **kwargs):
        """
        更新配置
        
        Args:
            **kwargs: 要更新的配置項目
        """
        for key, value in kwargs.items():
            if hasattr(self.config, key):
                setattr(self.config, key, value)
                print(f"🔧 更新配置 {key}: {value}")
            else:
                print(f"⚠️ 未知的配置項目: {key}")
        
        self._save_config()
    
    def set_real_time_mode(self, enabled: bool):
        """設定即時查詢模式"""
        self.update_config(real_time_mode=enabled)
        mode_name = "即時查詢" if enabled else "預存結果"
        print(f"🔄 切換到 {mode_name} 模式")
    
    def set_query_timeout(self, timeout: int):
        """設定查詢超時時間"""
        if timeout > 0:
            self.update_config(query_timeout=timeout)
        else:
            print(f"⚠️ 查詢超時時間必須大於 0")
    
    def set_max_iterations(self, max_iterations: int):
        """設定最大迭代次數"""
        if max_iterations > 0:
            self.update_config(max_iterations=max_iterations)
        else:
            print(f"⚠️ 最大迭代次數必須大於 0")
    
    def enable_sql_validation(self, enabled: bool):
        """啟用/停用 SQL 驗證"""
        self.update_config(enable_sql_validation=enabled)
    
    def enable_sql_logging(self, enabled: bool):
        """啟用/停用 SQL 查詢日誌"""
        self.update_config(log_sql_queries=enabled)
    
    def reset_to_defaults(self):
        """重置為預設配置"""
        self.config = TaskConditionConfig()
        self._save_config()
        print("🔄 配置已重置為預設值")
    
    def get_config_summary(self) -> Dict[str, Any]:
        """取得配置摘要"""
        config_dict = asdict(self.config)
        config_dict["mode_name"] = "即時查詢" if self.config.real_time_mode else "預存結果"
        config_dict["config_file"] = self.config_file
        return config_dict
    
    def print_config(self):
        """列印當前配置"""
        print("\n📋 當前任務條件檢查配置:")
        print("=" * 40)
        
        config_summary = self.get_config_summary()
        for key, value in config_summary.items():
            if key == "config_file":
                print(f"   配置檔案: {value}")
            elif key == "mode_name":
                print(f"   檢查模式: {value}")
            elif key == "real_time_mode":
                print(f"   即時查詢: {'啟用' if value else '停用'}")
            elif key == "query_timeout":
                print(f"   查詢超時: {value} 秒")
            elif key == "max_iterations":
                print(f"   最大迭代: {value} 次")
            elif key == "enable_sql_validation":
                print(f"   SQL 驗證: {'啟用' if value else '停用'}")
            elif key == "log_sql_queries":
                print(f"   SQL 日誌: {'啟用' if value else '停用'}")
            elif key == "cache_results":
                print(f"   結果快取: {'啟用' if value else '停用'}")
            elif key == "cache_ttl":
                print(f"   快取時間: {value} 秒")
        
        print("=" * 40)


# 全域配置管理器實例
_config_manager = None


def get_config_manager() -> TaskConditionConfigManager:
    """取得全域配置管理器實例"""
    global _config_manager
    if _config_manager is None:
        _config_manager = TaskConditionConfigManager()
    return _config_manager


def get_current_config() -> TaskConditionConfig:
    """取得當前配置"""
    return get_config_manager().get_config()


# 便利函數
def set_real_time_mode(enabled: bool):
    """設定即時查詢模式"""
    get_config_manager().set_real_time_mode(enabled)


def set_query_timeout(timeout: int):
    """設定查詢超時時間"""
    get_config_manager().set_query_timeout(timeout)


def set_max_iterations(max_iterations: int):
    """設定最大迭代次數"""
    get_config_manager().set_max_iterations(max_iterations)


def print_current_config():
    """列印當前配置"""
    get_config_manager().print_config()


if __name__ == "__main__":
    # 測試配置管理器
    print("🚀 測試任務條件檢查配置管理器")
    
    # 取得配置管理器
    config_mgr = get_config_manager()
    
    # 列印當前配置
    config_mgr.print_config()
    
    # 測試配置更新
    print("\n🔧 測試配置更新...")
    config_mgr.set_real_time_mode(False)
    config_mgr.set_query_timeout(60)
    config_mgr.set_max_iterations(50)
    
    # 列印更新後的配置
    config_mgr.print_config()
    
    # 重置配置
    print("\n🔄 重置配置...")
    config_mgr.reset_to_defaults()
    config_mgr.print_config()
    
    print("\n🎉 測試完成")
