#!/usr/bin/env python3
"""
從 KUKA Fleet 同步容器資訊回 Rack 表

此工具從 KUKA Fleet Manager 查詢所有容器資訊，並更新到 RosAGV 的 Rack 表：
- KUKA containerCode → Rack.name（匹配）
- KUKA inMapStatus → Rack.is_in_map
- KUKA isCarry → Rack.is_carry
- KUKA nodeCode → Rack.location_id（需要透過 KukaNode 表映射）

使用方式：
    python3 sync_from_kuka_to_rack.py                    # 同步所有在地圖中的容器
    python3 sync_from_kuka_to_rack.py --all             # 同步所有容器
    python3 sync_from_kuka_to_rack.py --dry-run         # 預覽變更（不實際更新）
    python3 sync_from_kuka_to_rack.py --container 001   # 只同步特定容器
"""

import sys
import os
import argparse
from typing import List, Dict, Optional
from datetime import datetime

# 添加路徑
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../kuka_fleet_ws/src/kuka_fleet_adapter'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../db_proxy_ws/src/db_proxy'))

try:
    from kuka_fleet_adapter.kuka_api_client import KukaApiClient
except ImportError:
    print("❌ 錯誤：無法匯入 KukaApiClient")
    print("請確保在容器內執行此腳本，並已建置 kuka_fleet_ws")
    sys.exit(1)

try:
    from db_proxy.connection_pool_manager import ConnectionPoolManager
    from db_proxy.models.rack import Rack
    from db_proxy.models.agvc_kuka import KukaNode
    from sqlmodel import select
except ImportError:
    print("❌ 錯誤：無法匯入資料庫模組")
    print("請確保在容器內執行此腳本，並已建置 db_proxy_ws")
    sys.exit(1)


class KukaToRackSyncService:
    """從 KUKA 同步到 Rack 的服務"""

    # KUKA Fleet Manager 配置
    DEFAULT_BASE_URL = "http://192.168.10.3:10870"
    DEFAULT_USERNAME = "admin"
    DEFAULT_PASSWORD = "Admin"

    # 資料庫配置
    DEFAULT_DB_URL = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'

    def __init__(
        self,
        kuka_base_url: str = None,
        kuka_username: str = None,
        kuka_password: str = None,
        db_url: str = None,
        dry_run: bool = False
    ):
        """
        初始化同步服務

        Args:
            kuka_base_url: KUKA Fleet Manager URL
            kuka_username: KUKA 登入帳號
            kuka_password: KUKA 登入密碼
            db_url: PostgreSQL 連接字串
            dry_run: 是否為預覽模式（不實際更新）
        """
        # KUKA Client
        self.kuka_client = KukaApiClient(
            base_url=kuka_base_url or self.DEFAULT_BASE_URL,
            username=kuka_username or self.DEFAULT_USERNAME,
            password=kuka_password or self.DEFAULT_PASSWORD
        )

        # 資料庫連線池
        self.db_pool = ConnectionPoolManager(db_url or self.DEFAULT_DB_URL)

        self.dry_run = dry_run

    def get_kuka_containers(self, include_all: bool = False) -> List[Dict]:
        """
        查詢 KUKA 容器

        Args:
            include_all: 是否包括不在地圖中的容器

        Returns:
            容器列表
        """
        try:
            if include_all:
                response = self.kuka_client.container_query_all({})
            else:
                response = self.kuka_client.get_all_containers_in_map()

            if response.get("success"):
                return response.get("data", [])
            else:
                print(f"❌ 查詢 KUKA 容器失敗: {response.get('message', 'Unknown error')}")
                return []
        except Exception as e:
            print(f"❌ 查詢 KUKA 容器異常: {e}")
            return []

    def get_location_id_from_node_code(
        self,
        node_code: str,
        session
    ) -> Optional[int]:
        """
        將 KUKA nodeCode 映射到 location_id

        映射邏輯：
        1. KukaNode.uuid == node_code → 找到 KukaNode.id
        2. Location.node_id == KukaNode.id → 找到 Location.id

        Args:
            node_code: KUKA Node UUID (例如: "AlanACT-AlanSec1-3")
            session: SQLModel session

        Returns:
            location_id 或 None
        """
        if not node_code:
            return None

        try:
            # 查詢 KukaNode by uuid
            statement = select(KukaNode).where(KukaNode.uuid == node_code)
            kuka_node = session.exec(statement).first()

            if not kuka_node:
                print(f"   ⚠️ 找不到 KUKA Node: {node_code}")
                return None

            # 查詢 Location by node_id
            from db_proxy.models.agvc_location import Location
            statement = select(Location).where(Location.node_id == kuka_node.id)
            location = session.exec(statement).first()

            if not location:
                print(f"   ⚠️ 找不到 Location for node_id={kuka_node.id}")
                return None

            return location.id

        except Exception as e:
            print(f"   ❌ 映射 nodeCode 時發生錯誤: {e}")
            return None

    def sync_container_to_rack(
        self,
        kuka_container: Dict,
        session
    ) -> Dict[str, any]:
        """
        同步單個容器到 Rack 表

        Args:
            kuka_container: KUKA 容器資訊
            session: SQLModel session

        Returns:
            Dict 包含 success, action, changes
        """
        container_code = kuka_container.get("containerCode")
        if not container_code:
            return {
                "success": False,
                "action": "skip",
                "error": "Missing containerCode"
            }

        # 查詢對應的 Rack
        statement = select(Rack).where(Rack.name == container_code)
        rack = session.exec(statement).first()

        if not rack:
            return {
                "success": False,
                "action": "not_found",
                "error": f"Rack with name={container_code} not found"
            }

        # 記錄變更
        changes = []
        old_values = {}

        # 同步 is_in_map
        kuka_in_map = 1 if kuka_container.get("inMapStatus") == 1 else 0
        if rack.is_in_map != kuka_in_map:
            old_values["is_in_map"] = rack.is_in_map
            rack.is_in_map = kuka_in_map
            changes.append(f"is_in_map: {old_values['is_in_map']} → {kuka_in_map}")

        # 同步 is_carry
        kuka_is_carry = 1 if kuka_container.get("isCarry") == 1 else 0
        if rack.is_carry != kuka_is_carry:
            old_values["is_carry"] = rack.is_carry
            rack.is_carry = kuka_is_carry
            changes.append(f"is_carry: {old_values['is_carry']} → {kuka_is_carry}")

        # 同步 location_id (透過 nodeCode 映射) - 仅当容器在地图中时
        kuka_in_map = 1 if kuka_container.get("inMapStatus") == 1 else 0
        node_code = kuka_container.get("nodeCode")

        if kuka_in_map == 1 and node_code:
            location_id = self.get_location_id_from_node_code(node_code, session)
            if location_id and rack.location_id != location_id:
                old_values["location_id"] = rack.location_id
                rack.location_id = location_id
                changes.append(f"location_id: {old_values['location_id']} → {location_id} (nodeCode: {node_code})")
        elif kuka_in_map != 1 and node_code:
            # 容器不在地图中，跳过 location 同步
            print(f"   ℹ️ Rack {rack.name} 不在地图中 (inMapStatus={kuka_in_map})，跳过 location 更新 (nodeCode: {node_code})")

        # 如果有變更，提交
        if changes:
            if not self.dry_run:
                session.add(rack)
                session.commit()
                session.refresh(rack)
                action = "updated"
            else:
                action = "would_update"

            return {
                "success": True,
                "action": action,
                "changes": changes,
                "old_values": old_values,
                "rack_id": rack.id,
                "rack_name": rack.name
            }
        else:
            return {
                "success": True,
                "action": "no_change",
                "rack_id": rack.id,
                "rack_name": rack.name
            }

    def sync_all_containers(
        self,
        include_all: bool = False,
        container_filter: str = None
    ) -> Dict[str, any]:
        """
        同步所有容器

        Args:
            include_all: 是否包括不在地圖中的容器
            container_filter: 只同步特定容器

        Returns:
            Dict 包含統計資訊
        """
        print("=" * 60)
        print("🔄 從 KUKA Fleet 同步到 Rack 表")
        print("=" * 60)

        if self.dry_run:
            print("⚠️  預覽模式：不會實際更新資料庫")
            print()

        # 查詢 KUKA 容器
        print("🔍 查詢 KUKA Fleet 容器...")
        kuka_containers = self.get_kuka_containers(include_all=include_all)

        if not kuka_containers:
            print("❌ 沒有找到任何容器")
            return {
                "success": False,
                "total": 0,
                "error": "No containers found"
            }

        print(f"✅ 找到 {len(kuka_containers)} 個容器")
        print()

        # 過濾容器
        if container_filter:
            kuka_containers = [
                c for c in kuka_containers
                if c.get("containerCode") == container_filter
            ]
            print(f"🔍 過濾後: {len(kuka_containers)} 個容器")
            print()

        # 統計
        stats = {
            "total": len(kuka_containers),
            "updated": 0,
            "no_change": 0,
            "not_found": 0,
            "errors": 0,
            "details": []
        }

        # 同步每個容器
        print("🔄 開始同步...")
        print()

        with self.db_pool.get_session() as session:
            for i, container in enumerate(kuka_containers, 1):
                container_code = container.get("containerCode", "N/A")
                print(f"[{i}/{len(kuka_containers)}] 處理容器: {container_code}")

                # 同步
                result = self.sync_container_to_rack(container, session)

                # 更新統計
                action = result.get("action")
                if action in ("updated", "would_update"):
                    stats["updated"] += 1
                    print(f"   ✅ {'將會更新' if self.dry_run else '已更新'}:")
                    for change in result.get("changes", []):
                        print(f"      - {change}")
                elif action == "no_change":
                    stats["no_change"] += 1
                    print(f"   ℹ️  無需更新")
                elif action == "not_found":
                    stats["not_found"] += 1
                    print(f"   ⚠️  找不到對應的 Rack")
                else:
                    stats["errors"] += 1
                    print(f"   ❌ 錯誤: {result.get('error', 'Unknown')}")

                stats["details"].append(result)
                print()

        # 顯示摘要
        print("=" * 60)
        print("📊 同步摘要")
        print("=" * 60)
        print(f"總計: {stats['total']} 個容器")
        print(f"{'將會更新' if self.dry_run else '已更新'}: {stats['updated']} 個")
        print(f"無需更新: {stats['no_change']} 個")
        print(f"找不到: {stats['not_found']} 個")
        print(f"錯誤: {stats['errors']} 個")
        print("=" * 60)

        if self.dry_run and stats["updated"] > 0:
            print()
            print("💡 使用 --execute 執行實際更新")

        return stats


def main():
    """主程式入口"""
    parser = argparse.ArgumentParser(
        description='從 KUKA Fleet 同步容器資訊到 Rack 表',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
範例:
  %(prog)s                     # 同步所有在地圖中的容器
  %(prog)s --all              # 同步所有容器（包括不在地圖中的）
  %(prog)s --dry-run          # 預覽變更（不實際更新）
  %(prog)s --container 001    # 只同步特定容器
  %(prog)s --all --dry-run    # 預覽所有容器的變更
        """
    )

    parser.add_argument(
        '--all',
        action='store_true',
        help='同步所有容器（包括不在地圖中的）'
    )

    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='預覽模式：顯示將要進行的變更，但不實際更新資料庫'
    )

    parser.add_argument(
        '--container',
        help='只同步特定容器（containerCode）'
    )

    parser.add_argument(
        '--kuka-url',
        default='http://192.168.10.3:10870',
        help='KUKA Fleet Manager URL（預設: http://192.168.10.3:10870）'
    )

    parser.add_argument(
        '--kuka-username',
        default='admin',
        help='KUKA 登入帳號（預設: admin）'
    )

    parser.add_argument(
        '--kuka-password',
        default='Admin',
        help='KUKA 登入密碼（預設: Admin）'
    )

    parser.add_argument(
        '--db-url',
        default='postgresql+psycopg2://agvc:password@192.168.100.254/agvc',
        help='PostgreSQL 連接字串'
    )

    args = parser.parse_args()

    # 建立同步服務
    try:
        sync_service = KukaToRackSyncService(
            kuka_base_url=args.kuka_url,
            kuka_username=args.kuka_username,
            kuka_password=args.kuka_password,
            db_url=args.db_url,
            dry_run=args.dry_run
        )
    except Exception as e:
        print(f"❌ 初始化同步服務失敗: {e}")
        sys.exit(1)

    # 執行同步
    try:
        stats = sync_service.sync_all_containers(
            include_all=args.all,
            container_filter=args.container
        )

        # 根據結果返回適當的退出碼
        if stats["errors"] > 0:
            sys.exit(1)
        elif stats["not_found"] > 0:
            sys.exit(2)
        else:
            sys.exit(0)

    except KeyboardInterrupt:
        print("\n\n⚠️  操作已取消")
        sys.exit(130)
    except Exception as e:
        print(f"\n❌ 執行錯誤: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
