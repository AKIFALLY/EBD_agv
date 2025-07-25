#!/usr/bin/env python3
"""
KUKA 配置管理 CLI 工具
提供命令列介面來管理 KUKA 和 CT AGV 的配置
"""

import sys
import json
import argparse
from pathlib import Path
from typing import Dict, Any, Optional
from datetime import datetime

# 添加當前目錄到 Python 路徑
sys.path.insert(0, str(Path(__file__).parent))

from kuka_config_manager import KukaConfigManager, AGVConfig, AGVModel, AGVStatus


class KukaConfigCLI:
    """KUKA 配置管理 CLI"""
    
    def __init__(self):
        self.config_manager = KukaConfigManager()
    
    def list_agvs(self, args):
        """列出所有 AGV"""
        print("=== AGV 列表 ===")
        
        if args.type == 'kuka':
            agvs = self.config_manager.get_kuka_agvs()
            print(f"KUKA AGV ({len(agvs)} 台):")
        elif args.type == 'ct':
            agvs = self.config_manager.get_ct_agvs()
            print(f"CT AGV ({len(agvs)} 台):")
        else:
            agvs = list(self.config_manager.config.agvs.values())
            print(f"所有 AGV ({len(agvs)} 台):")
        
        if not agvs:
            print("  無 AGV 配置")
            return
        
        for agv in agvs:
            status_text = "啟用" if agv.enable else "停用"
            
            if agv.model == AGVModel.KUKA400I:
                detail = f"Robot ID: {agv.kuka_robot_id}, 座標: ({agv.initial_x:.0f}, {agv.initial_y:.0f})"
            else:
                room = f"房間{agv.ct_room_assignment}" if agv.ct_room_assignment else "房外"
                detail = f"分派: {room}, 能力: {', '.join(agv.ct_capabilities)}"
            
            print(f"  • {agv.name} ({agv.model.value}) - {status_text}")
            print(f"    ID: {agv.id}, {detail}")
            print(f"    描述: {agv.description}")
            print()
    
    def show_agv(self, args):
        """顯示特定 AGV 詳細資訊"""
        agv_config = self.config_manager.get_agv_config(args.name)
        
        if not agv_config:
            print(f"❌ AGV '{args.name}' 不存在")
            return
        
        print(f"=== AGV 詳細資訊: {args.name} ===")
        print(f"ID: {agv_config.id}")
        print(f"名稱: {agv_config.name}")
        print(f"型號: {agv_config.model.value}")
        print(f"描述: {agv_config.description}")
        print(f"狀態: {'啟用' if agv_config.enable else '停用'}")
        print(f"預設狀態: {agv_config.default_status.name}")
        
        print(f"\n位置資訊:")
        print(f"  初始 X: {agv_config.initial_x}")
        print(f"  初始 Y: {agv_config.initial_y}")
        print(f"  初始航向: {agv_config.initial_heading}°")
        
        print(f"\n性能參數:")
        print(f"  低電量門檻: {agv_config.battery_threshold_low}%")
        print(f"  危險電量門檻: {agv_config.battery_threshold_critical}%")
        print(f"  最大載重: {agv_config.max_payload} kg")
        print(f"  最大速度: {agv_config.max_speed} m/s")
        
        if agv_config.model == AGVModel.KUKA400I:
            print(f"\nKUKA 特定配置:")
            print(f"  Robot ID: {agv_config.kuka_robot_id}")
            print(f"  Robot Type: {agv_config.kuka_robot_type}")
        else:
            print(f"\nCT AGV 特定配置:")
            room_text = f"房間{agv_config.ct_room_assignment}" if agv_config.ct_room_assignment else "房外"
            print(f"  房間分派: {room_text}")
            print(f"  能力列表: {', '.join(agv_config.ct_capabilities)}")
    
    def add_agv(self, args):
        """新增 AGV 配置"""
        try:
            # 檢查 AGV 是否已存在
            if self.config_manager.get_agv_config(args.name):
                if not args.force:
                    print(f"❌ AGV '{args.name}' 已存在，使用 --force 覆蓋")
                    return
                print(f"⚠️ 覆蓋現有 AGV '{args.name}'")
            
            # 創建 AGV 配置
            agv_config = AGVConfig(
                id=args.id,
                name=args.name,
                model=AGVModel(args.model),
                description=args.description or f"{args.model} AGV",
                initial_x=args.x or 0.0,
                initial_y=args.y or 0.0,
                initial_heading=args.heading or 0.0,
                enable=not args.disable
            )
            
            # 設置特定類型配置
            if args.model == "KUKA400i":
                agv_config.kuka_robot_id = args.robot_id or str(args.id)
                agv_config.kuka_robot_type = args.robot_type or "KMP 400i diffDrive"
            else:
                if args.room:
                    agv_config.ct_room_assignment = args.room
                if args.capabilities:
                    agv_config.ct_capabilities = args.capabilities.split(',')
            
            # 新增配置
            if self.config_manager.add_agv_config(agv_config):
                print(f"✅ AGV '{args.name}' 配置已新增")
            else:
                print(f"❌ 新增 AGV '{args.name}' 失敗")
                
        except ValueError as e:
            print(f"❌ 參數錯誤: {e}")
        except Exception as e:
            print(f"❌ 新增 AGV 失敗: {e}")
    
    def update_agv(self, args):
        """更新 AGV 配置"""
        try:
            # 檢查 AGV 是否存在
            if not self.config_manager.get_agv_config(args.name):
                print(f"❌ AGV '{args.name}' 不存在")
                return
            
            # 準備更新資料
            updates = {}
            
            if args.description is not None:
                updates['description'] = args.description
            if args.x is not None:
                updates['initial_x'] = args.x
            if args.y is not None:
                updates['initial_y'] = args.y
            if args.heading is not None:
                updates['initial_heading'] = args.heading
            if args.enable is not None:
                updates['enable'] = args.enable
            if args.room is not None:
                updates['ct_room_assignment'] = args.room
            if args.robot_id is not None:
                updates['kuka_robot_id'] = args.robot_id
            if args.capabilities is not None:
                updates['ct_capabilities'] = args.capabilities.split(',')
            
            if not updates:
                print("❌ 沒有指定要更新的參數")
                return
            
            # 更新配置
            if self.config_manager.update_agv_config(args.name, updates):
                print(f"✅ AGV '{args.name}' 配置已更新")
                
                # 顯示更新的項目
                print("更新的項目:")
                for key, value in updates.items():
                    print(f"  - {key}: {value}")
            else:
                print(f"❌ 更新 AGV '{args.name}' 失敗")
                
        except Exception as e:
            print(f"❌ 更新 AGV 失敗: {e}")
    
    def remove_agv(self, args):
        """移除 AGV 配置"""
        # 檢查 AGV 是否存在
        if not self.config_manager.get_agv_config(args.name):
            print(f"❌ AGV '{args.name}' 不存在")
            return
        
        # 確認操作
        if not args.force:
            response = input(f"確定要移除 AGV '{args.name}' 嗎? (y/N): ")
            if response.lower() != 'y':
                print("操作已取消")
                return
        
        # 移除配置
        if self.config_manager.remove_agv_config(args.name):
            print(f"✅ AGV '{args.name}' 配置已移除")
        else:
            print(f"❌ 移除 AGV '{args.name}' 失敗")
    
    def show_config(self, args):
        """顯示系統配置"""
        if args.section == 'summary':
            summary = self.config_manager.get_config_summary()
            print("=== 配置摘要 ===")
            for key, value in summary.items():
                if isinstance(value, dict):
                    print(f"{key}:")
                    for sub_key, sub_value in value.items():
                        print(f"  {sub_key}: {sub_value}")
                else:
                    print(f"{key}: {value}")
        
        elif args.section == 'kuka_api':
            api_config = self.config_manager.config.kuka_api
            print("=== KUKA API 配置 ===")
            print(f"Base URL: {api_config.base_url}")
            print(f"Username: {api_config.username}")
            print(f"Password: {'*' * len(api_config.password)}")
            print(f"Timeout: {api_config.timeout}s")
            print(f"Max Retries: {api_config.max_retries}")
            print(f"Retry Delay: {api_config.retry_delay}s")
            
            print("\nAPI 端點:")
            for name, endpoint in api_config.endpoints.items():
                print(f"  {name}: {endpoint}")
        
        elif args.section == 'system':
            system_config = self.config_manager.config.system
            print("=== 系統配置 ===")
            for key, value in system_config.items():
                print(f"{key}: {value}")
        
        else:
            print("❌ 無效的配置區段，支援: summary, kuka_api, system")
    
    def validate_config(self, args):
        """驗證配置"""
        print("=== 配置驗證 ===")
        errors = self.config_manager.validate_config()
        
        total_errors = sum(len(error_list) for error_list in errors.values())
        
        if total_errors == 0:
            print("✅ 配置驗證通過")
            return
        
        print(f"❌ 發現 {total_errors} 個錯誤:")
        
        for category, error_list in errors.items():
            if error_list:
                print(f"\n{category.upper()} 錯誤:")
                for error in error_list:
                    print(f"  - {error}")
    
    def sync_database(self, args):
        """與資料庫同步"""
        print("=== 與資料庫同步 ===")
        
        if self.config_manager.sync_with_database():
            print("✅ 與資料庫同步成功")
        else:
            print("❌ 與資料庫同步失敗")
    
    def export_config(self, args):
        """匯出配置"""
        output_path = args.output or f"kuka_config_export_{datetime.now().strftime('%Y%m%d_%H%M%S')}.{args.format}"
        
        if self.config_manager.export_config(output_path, args.format):
            print(f"✅ 配置已匯出到: {output_path}")
        else:
            print("❌ 匯出配置失敗")


def main():
    """主函數"""
    parser = argparse.ArgumentParser(description="KUKA 配置管理 CLI 工具")
    subparsers = parser.add_subparsers(dest='command', help='可用命令')
    
    # list 命令
    list_parser = subparsers.add_parser('list', help='列出 AGV')
    list_parser.add_argument('--type', choices=['all', 'kuka', 'ct'], default='all', 
                           help='AGV 類型')
    list_parser.set_defaults(func=lambda cli, args: cli.list_agvs(args))
    
    # show 命令
    show_parser = subparsers.add_parser('show', help='顯示 AGV 詳細資訊')
    show_parser.add_argument('name', help='AGV 名稱')
    show_parser.set_defaults(func=lambda cli, args: cli.show_agv(args))
    
    # add 命令
    add_parser = subparsers.add_parser('add', help='新增 AGV 配置')
    add_parser.add_argument('name', help='AGV 名稱')
    add_parser.add_argument('id', type=int, help='AGV ID')
    add_parser.add_argument('model', choices=['Cargo', 'Loader', 'Unloader', 'KUKA400i'], 
                          help='AGV 型號')
    add_parser.add_argument('--description', help='描述')
    add_parser.add_argument('--x', type=float, help='初始 X 座標')
    add_parser.add_argument('--y', type=float, help='初始 Y 座標')
    add_parser.add_argument('--heading', type=float, help='初始航向')
    add_parser.add_argument('--disable', action='store_true', help='停用 AGV')
    add_parser.add_argument('--force', action='store_true', help='覆蓋現有配置')
    
    # KUKA 特定參數
    add_parser.add_argument('--robot-id', help='KUKA Robot ID')
    add_parser.add_argument('--robot-type', help='KUKA Robot Type')
    
    # CT AGV 特定參數
    add_parser.add_argument('--room', type=int, help='房間分派')
    add_parser.add_argument('--capabilities', help='能力列表 (逗號分隔)')
    add_parser.set_defaults(func=lambda cli, args: cli.add_agv(args))
    
    # update 命令
    update_parser = subparsers.add_parser('update', help='更新 AGV 配置')
    update_parser.add_argument('name', help='AGV 名稱')
    update_parser.add_argument('--description', help='描述')
    update_parser.add_argument('--x', type=float, help='初始 X 座標')
    update_parser.add_argument('--y', type=float, help='初始 Y 座標')
    update_parser.add_argument('--heading', type=float, help='初始航向')
    update_parser.add_argument('--enable', type=bool, help='啟用/停用 AGV')
    update_parser.add_argument('--room', type=int, help='房間分派')
    update_parser.add_argument('--robot-id', help='KUKA Robot ID')
    update_parser.add_argument('--capabilities', help='能力列表 (逗號分隔)')
    update_parser.set_defaults(func=lambda cli, args: cli.update_agv(args))
    
    # remove 命令
    remove_parser = subparsers.add_parser('remove', help='移除 AGV 配置')
    remove_parser.add_argument('name', help='AGV 名稱')
    remove_parser.add_argument('--force', action='store_true', help='不確認直接移除')
    remove_parser.set_defaults(func=lambda cli, args: cli.remove_agv(args))
    
    # config 命令
    config_parser = subparsers.add_parser('config', help='顯示系統配置')
    config_parser.add_argument('section', choices=['summary', 'kuka_api', 'system'], 
                             help='配置區段')
    config_parser.set_defaults(func=lambda cli, args: cli.show_config(args))
    
    # validate 命令
    validate_parser = subparsers.add_parser('validate', help='驗證配置')
    validate_parser.set_defaults(func=lambda cli, args: cli.validate_config(args))
    
    # sync 命令
    sync_parser = subparsers.add_parser('sync', help='與資料庫同步')
    sync_parser.set_defaults(func=lambda cli, args: cli.sync_database(args))
    
    # export 命令
    export_parser = subparsers.add_parser('export', help='匯出配置')
    export_parser.add_argument('--output', help='輸出檔案路徑')
    export_parser.add_argument('--format', choices=['yaml', 'json'], default='yaml', 
                             help='匯出格式')
    export_parser.set_defaults(func=lambda cli, args: cli.export_config(args))
    
    # 解析參數
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return 1
    
    # 執行命令
    try:
        cli = KukaConfigCLI()
        args.func(cli, args)
        return 0
    except KeyboardInterrupt:
        print("\n操作已中斷")
        return 1
    except Exception as e:
        print(f"❌ 執行失敗: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())