#!/usr/bin/env python3
"""
KUKA 配置管理系統測試腳本
測試統一配置管理系統的各項功能
"""

import sys
import os
import tempfile
import shutil
from pathlib import Path

# 添加 RCS 模組路徑
sys.path.insert(0, str(Path(__file__).parent / "src" / "rcs" / "rcs"))

from kuka_config_manager import (
    KukaConfigManager, AGVConfig, AGVModel, AGVStatus,
    KukaAPIConfig, UnifiedFleetConfig
)


def test_config_manager():
    """測試配置管理器基本功能"""
    print("=== 測試配置管理器基本功能 ===")
    
    # 使用臨時目錄進行測試
    with tempfile.TemporaryDirectory() as temp_dir:
        print(f"使用臨時目錄: {temp_dir}")
        
        # 初始化配置管理器
        config_manager = KukaConfigManager(config_dir=temp_dir)
        
        # 測試配置摘要
        print("\n1. 配置摘要測試")
        summary = config_manager.get_config_summary()
        print(f"  總 AGV 數: {summary['total_agvs']}")
        print(f"  KUKA AGV 數: {summary['kuka_agvs']}")
        print(f"  CT AGV 數: {summary['ct_agvs']}")
        
        # 測試 AGV 查詢
        print("\n2. AGV 查詢測試")
        kuka_agvs = config_manager.get_kuka_agvs()
        print(f"  KUKA AGV: {[agv.name for agv in kuka_agvs]}")
        
        ct_agvs = config_manager.get_ct_agvs()
        print(f"  CT AGV: {[agv.name for agv in ct_agvs]}")
        
        # 測試特定 AGV 配置
        print("\n3. 特定 AGV 配置測試")
        for agv_name in ["KUKA001", "Cargo02", "Loader02"]:
            agv_config = config_manager.get_agv_config(agv_name)
            if agv_config:
                print(f"  {agv_name}: {agv_config.model.value}, ID={agv_config.id}")
            else:
                print(f"  {agv_name}: 未找到")
        
        # 測試配置驗證
        print("\n4. 配置驗證測試")
        errors = config_manager.validate_config()
        total_errors = sum(len(error_list) for error_list in errors.values())
        if total_errors == 0:
            print("  ✅ 配置驗證通過")
        else:
            print(f"  ❌ 發現 {total_errors} 個錯誤")
            for category, error_list in errors.items():
                if error_list:
                    print(f"    {category}: {error_list}")
        
        return config_manager


def test_agv_management(config_manager):
    """測試 AGV 管理功能"""
    print("\n=== 測試 AGV 管理功能 ===")
    
    # 測試新增 AGV
    print("\n1. 新增 AGV 測試")
    test_agv = AGVConfig(
        id=9999,
        name="TEST_AGV",
        model=AGVModel.KUKA400I,
        description="測試用 KUKA AGV",
        initial_x=1000.0,
        initial_y=2000.0,
        kuka_robot_id="9999",
        kuka_robot_type="KMP 400i diffDrive"
    )
    
    if config_manager.add_agv_config(test_agv):
        print("  ✅ AGV 新增成功")
        
        # 驗證新增的 AGV
        added_agv = config_manager.get_agv_config("TEST_AGV")
        if added_agv and added_agv.id == 9999:
            print("  ✅ AGV 資料正確")
        else:
            print("  ❌ AGV 資料不正確")
    else:
        print("  ❌ AGV 新增失敗")
    
    # 測試更新 AGV
    print("\n2. 更新 AGV 測試")
    updates = {
        'description': '測試用 KUKA AGV (已更新)',
        'initial_x': 1500.0,
        'battery_threshold_low': 25.0
    }
    
    if config_manager.update_agv_config("TEST_AGV", updates):
        print("  ✅ AGV 更新成功")
        
        # 驗證更新
        updated_agv = config_manager.get_agv_config("TEST_AGV")
        if (updated_agv and 
            updated_agv.description == updates['description'] and
            updated_agv.initial_x == updates['initial_x'] and
            updated_agv.battery_threshold_low == updates['battery_threshold_low']):
            print("  ✅ AGV 更新資料正確")
        else:
            print("  ❌ AGV 更新資料不正確")
    else:
        print("  ❌ AGV 更新失敗")
    
    # 測試移除 AGV
    print("\n3. 移除 AGV 測試")
    if config_manager.remove_agv_config("TEST_AGV"):
        print("  ✅ AGV 移除成功")
        
        # 驗證移除
        removed_agv = config_manager.get_agv_config("TEST_AGV")
        if removed_agv is None:
            print("  ✅ AGV 已正確移除")
        else:
            print("  ❌ AGV 未正確移除")
    else:
        print("  ❌ AGV 移除失敗")


def test_config_persistence(config_manager):
    """測試配置持久化"""
    print("\n=== 測試配置持久化 ===")
    
    # 測試配置儲存
    print("\n1. 配置儲存測試")
    original_summary = config_manager.get_config_summary()
    
    try:
        config_manager.save_config()
        print("  ✅ 配置儲存成功")
    except Exception as e:
        print(f"  ❌ 配置儲存失敗: {e}")
        return
    
    # 測試配置匯出
    print("\n2. 配置匯出測試")
    temp_export = Path(config_manager.config_dir) / "test_export.yaml"
    
    if config_manager.export_config(str(temp_export), 'yaml'):
        print("  ✅ YAML 匯出成功")
        
        if temp_export.exists():
            print("  ✅ 匯出檔案存在")
            
            # 檢查檔案內容
            with open(temp_export, 'r', encoding='utf-8') as f:
                content = f.read()
                if 'agvs:' in content and 'kuka_api:' in content:
                    print("  ✅ 匯出內容正確")
                else:
                    print("  ❌ 匯出內容不完整")
        else:
            print("  ❌ 匯出檔案不存在")
    else:
        print("  ❌ YAML 匯出失敗")
    
    # 測試 JSON 匯出
    temp_export_json = Path(config_manager.config_dir) / "test_export.json"
    
    if config_manager.export_config(str(temp_export_json), 'json'):
        print("  ✅ JSON 匯出成功")
    else:
        print("  ❌ JSON 匯出失敗")


def test_kuka_specific_features(config_manager):
    """測試 KUKA 特定功能"""
    print("\n=== 測試 KUKA 特定功能 ===")
    
    # 測試 KUKA API 配置
    print("\n1. KUKA API 配置測試")
    api_config = config_manager.config.kuka_api
    print(f"  Base URL: {api_config.base_url}")
    print(f"  Username: {api_config.username}")
    print(f"  Endpoints: {len(api_config.endpoints)} 個")
    
    # 測試 KUKA AGV 特定配置
    print("\n2. KUKA AGV 配置測試")
    kuka_agvs = config_manager.get_kuka_agvs()
    
    for agv in kuka_agvs:
        print(f"  {agv.name}:")
        print(f"    Robot ID: {agv.kuka_robot_id}")
        print(f"    Robot Type: {agv.kuka_robot_type}")
        print(f"    座標: ({agv.initial_x}, {agv.initial_y})")
    
    # 測試狀態映射
    print("\n3. 狀態映射測試")
    status_mapping = config_manager.config.kuka_fleet.status_mapping
    print(f"  狀態映射: {len(status_mapping)} 個")
    for ct_status, kuka_status in status_mapping.items():
        print(f"    {ct_status} -> {kuka_status}")


def test_ct_specific_features(config_manager):
    """測試 CT AGV 特定功能"""
    print("\n=== 測試 CT AGV 特定功能 ===")
    
    # 測試房間分派
    print("\n1. 房間分派測試")
    for room_id in [1, 2]:
        room_agvs = config_manager.get_agvs_by_room(room_id)
        print(f"  房間 {room_id}: {[agv.name for agv in room_agvs]}")
    
    # 測試 CT AGV 配置
    print("\n2. CT AGV 配置測試")
    ct_agvs = config_manager.get_ct_agvs()
    
    for agv in ct_agvs:
        room_text = f"房間{agv.ct_room_assignment}" if agv.ct_room_assignment else "房外"
        print(f"  {agv.name} ({agv.model.value}):")
        print(f"    分派: {room_text}")
        print(f"    能力: {', '.join(agv.ct_capabilities)}")
    
    # 測試分派規則
    print("\n3. 分派規則測試")
    dispatch_rules = config_manager.config.ct_fleet.room_dispatch_rules
    for rule_name, rule_config in dispatch_rules.items():
        print(f"  {rule_name}: {rule_config}")


def test_api_config_update(config_manager):
    """測試 API 配置更新"""
    print("\n=== 測試 API 配置更新 ===")
    
    # 記錄原始配置
    original_timeout = config_manager.config.kuka_api.timeout
    original_retries = config_manager.config.kuka_api.max_retries
    
    # 測試 API 配置更新
    print("\n1. KUKA API 配置更新測試")
    api_updates = {
        'timeout': 45.0,
        'max_retries': 5,
        'retry_delay': 2.0
    }
    
    if config_manager.update_kuka_api_config(api_updates):
        print("  ✅ API 配置更新成功")
        
        # 驗證更新
        updated_config = config_manager.config.kuka_api
        if (updated_config.timeout == 45.0 and
            updated_config.max_retries == 5 and
            updated_config.retry_delay == 2.0):
            print("  ✅ API 配置更新正確")
        else:
            print("  ❌ API 配置更新不正確")
    else:
        print("  ❌ API 配置更新失敗")
    
    # 恢復原始配置
    restore_updates = {
        'timeout': original_timeout,
        'max_retries': original_retries
    }
    config_manager.update_kuka_api_config(restore_updates)
    print("  ✅ 原始配置已恢復")


def main():
    """主測試函數"""
    print("KUKA 配置管理系統測試")
    print("=" * 50)
    
    try:
        # 基本功能測試
        config_manager = test_config_manager()
        
        # AGV 管理測試
        test_agv_management(config_manager)
        
        # 配置持久化測試
        test_config_persistence(config_manager)
        
        # KUKA 特定功能測試
        test_kuka_specific_features(config_manager)
        
        # CT AGV 特定功能測試
        test_ct_specific_features(config_manager)
        
        # API 配置更新測試
        test_api_config_update(config_manager)
        
        print("\n" + "=" * 50)
        print("✅ 所有測試完成")
        
        # 最終配置摘要
        print("\n=== 最終配置摘要 ===")
        final_summary = config_manager.get_config_summary()
        for key, value in final_summary.items():
            if isinstance(value, dict):
                print(f"{key}:")
                for sub_key, sub_value in value.items():
                    print(f"  {sub_key}: {sub_value}")
            else:
                print(f"{key}: {value}")
        
        return 0
        
    except Exception as e:
        print(f"\n❌ 測試過程中發生錯誤: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())