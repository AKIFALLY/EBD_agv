#!/usr/bin/env python3
"""
KUKA Fleet Container 查詢工具

用途：
- 查詢 KUKA Fleet Manager 中所有容器（Container）
- 顯示容器狀態和位置資訊
- 支援條件過濾和格式化輸出

使用方式：
    python3 query_kuka_containers.py                # 列出所有容器
    python3 query_kuka_containers.py --status in    # 只顯示在地圖中的容器
    python3 query_kuka_containers.py --json         # JSON 格式輸出
    python3 query_kuka_containers.py --code RACK001 # 查詢特定容器
"""

import sys
import os
import argparse
import json
from typing import List, Dict, Optional
from datetime import datetime

# 添加 kuka_fleet_adapter 路徑到 sys.path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../kuka_fleet_ws/src/kuka_fleet_adapter'))

try:
    from kuka_fleet_adapter.kuka_api_client import KukaApiClient
except ImportError:
    print("❌ 錯誤：無法匯入 KukaApiClient")
    print("請確保在容器內執行此腳本，並已建置 kuka_fleet_ws")
    sys.exit(1)


class KukaContainerQuery:
    """KUKA Container 查詢工具類別"""

    # KUKA Fleet Manager 配置
    DEFAULT_BASE_URL = "http://192.168.10.3:10870"
    DEFAULT_USERNAME = "admin"
    DEFAULT_PASSWORD = "Admin"

    def __init__(self, base_url: str = None, username: str = None, password: str = None, auto_login: bool = True):
        """初始化 KUKA API 客戶端"""
        base_url = base_url or self.DEFAULT_BASE_URL

        if auto_login:
            username = username or self.DEFAULT_USERNAME
            password = password or self.DEFAULT_PASSWORD
            self.client = KukaApiClient(base_url=base_url, username=username, password=password)
        else:
            self.client = KukaApiClient(base_url=base_url)

    def list_containers(self, include_out_of_map: bool = False) -> List[Dict]:
        """
        列出所有容器

        Args:
            include_out_of_map: 是否包括不在地圖中（離場）的容器

        Returns:
            容器列表，每個容器包含：
            - containerCode: 容器編號
            - nodeCode: 位置節點
            - inMapStatus: 在地圖狀態
        """
        try:
            # 根據參數選擇不同的 API
            if include_out_of_map:
                # 使用 containerQueryAll - 包括所有容器（入場+離場）
                response = self.client.container_query_all({})
            else:
                # 使用 containerQuery - 只包括入場的容器
                response = self.client.get_all_containers_in_map()

            if response.get("success"):
                return response.get("data", [])
            else:
                print(f"❌ 查詢失敗: {response.get('message', 'Unknown error')}")
                return []
        except Exception as e:
            print(f"❌ 查詢異常: {e}")
            return []

    def get_container(self, container_code: str) -> Optional[Dict]:
        """
        查詢特定容器

        Args:
            container_code: 容器編號

        Returns:
            容器資訊字典，如果不存在返回 None
        """
        try:
            # 使用 KukaApiClient 的便利方法
            response = self.client.get_container_by_code(container_code)
            if response.get("success"):
                data = response.get("data", [])
                # get_container_by_code 返回的是列表，取第一個元素
                if isinstance(data, list) and len(data) > 0:
                    return data[0]
                return None
            else:
                print(f"❌ 查詢失敗: {response.get('message', 'Unknown error')}")
                return None
        except Exception as e:
            print(f"❌ 查詢異常: {e}")
            return None

    def filter_containers(
        self,
        containers: List[Dict],
        status: Optional[str] = None,
        position: Optional[str] = None
    ) -> List[Dict]:
        """
        過濾容器列表

        Args:
            containers: 容器列表
            status: 狀態過濾 ('in'=在地圖中, 'out'=不在地圖中)
            position: 位置過濾 (KUKA Node Code)

        Returns:
            過濾後的容器列表
        """
        filtered = containers

        if status:
            if status == 'in':
                filtered = [c for c in filtered if c.get('inMapStatus') == 1]
            elif status == 'out':
                filtered = [c for c in filtered if c.get('inMapStatus') != 1]

        if position:
            filtered = [c for c in filtered if c.get('nodeCode') == position]

        return filtered

    def format_table(self, containers: List[Dict]) -> str:
        """
        格式化為表格輸出

        Args:
            containers: 容器列表

        Returns:
            格式化的表格字串
        """
        if not containers:
            return "📭 沒有找到容器"

        # 計算欄位寬度
        max_code_len = max(len(c.get('containerCode', '')) for c in containers)
        max_code_len = max(max_code_len, len('Container Code'))

        max_pos_len = max(len(c.get('nodeCode', 'N/A')) for c in containers)
        max_pos_len = max(max_pos_len, len('Position'))

        # 建立表格
        header = f"{'Container Code':<{max_code_len}}  {'Status':<8}  {'Position':<{max_pos_len}}"
        separator = "-" * len(header)

        lines = [header, separator]

        for container in containers:
            code = container.get('containerCode', 'N/A')
            node_code = container.get('nodeCode', 'N/A')
            in_map = container.get('inMapStatus', 0)
            status = '🟢 In Map' if in_map == 1 else '⚪ Out'

            line = f"{code:<{max_code_len}}  {status:<8}  {node_code:<{max_pos_len}}"
            lines.append(line)

        lines.append(separator)
        lines.append(f"總計: {len(containers)} 個容器")

        return '\n'.join(lines)

    def format_json(self, containers: List[Dict]) -> str:
        """
        格式化為 JSON 輸出

        Args:
            containers: 容器列表

        Returns:
            格式化的 JSON 字串
        """
        return json.dumps({
            "timestamp": datetime.now().isoformat(),
            "total": len(containers),
            "containers": containers
        }, indent=2, ensure_ascii=False)

    def format_simple(self, containers: List[Dict]) -> str:
        """
        簡單列表輸出（僅顯示容器編號）

        Args:
            containers: 容器列表

        Returns:
            容器編號列表
        """
        if not containers:
            return "📭 沒有找到容器"

        codes = [c.get('containerCode', 'N/A') for c in containers]
        return '\n'.join(codes)


def main():
    """主程式入口"""
    parser = argparse.ArgumentParser(
        description='KUKA Fleet Container 查詢工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
範例:
  %(prog)s                      # 列出所有在地圖中的容器
  %(prog)s --all                # 列出所有容器（包括不在地圖中的）
  %(prog)s --status in          # 只顯示在地圖中的容器
  %(prog)s --status out         # 只顯示不在地圖中的容器
  %(prog)s --json               # JSON 格式輸出
  %(prog)s --simple             # 簡單列表（僅容器編號）
  %(prog)s --code RACK001       # 查詢特定容器
  %(prog)s --position node-123  # 查詢特定位置的容器
  %(prog)s --all --json         # 查詢所有容器（JSON格式）
        """
    )

    parser.add_argument(
        '--all',
        action='store_true',
        help='查詢所有容器（包括不在地圖中的）。預設只查詢在地圖中的容器。'
    )

    parser.add_argument(
        '--status',
        choices=['in', 'out'],
        help='過濾容器狀態 (in=在地圖中, out=不在地圖中)'
    )

    parser.add_argument(
        '--position',
        help='過濾特定位置 (KUKA Node UUID)'
    )

    parser.add_argument(
        '--code',
        help='查詢特定容器編號'
    )

    parser.add_argument(
        '--json',
        action='store_true',
        help='JSON 格式輸出'
    )

    parser.add_argument(
        '--simple',
        action='store_true',
        help='簡單列表輸出（僅容器編號）'
    )

    parser.add_argument(
        '--no-login',
        action='store_true',
        help='不自動登入 KUKA Fleet'
    )

    args = parser.parse_args()

    # 建立查詢工具
    print("🔍 連接 KUKA Fleet Manager...")
    query = KukaContainerQuery(auto_login=not args.no_login)

    # 查詢容器
    if args.code:
        # 查詢特定容器
        print(f"🔍 查詢容器: {args.code}")
        container = query.get_container(args.code)
        if container:
            if args.json:
                print(json.dumps(container, indent=2, ensure_ascii=False))
            else:
                print(f"\n📦 容器: {container.get('containerCode')}")
                print(f"   位置: {container.get('nodeCode', 'N/A')}")
                print(f"   方向: {container.get('orientation', 'N/A')}°")
                print(f"   型號: {container.get('containerModelCode', 'N/A')}")
                in_map = container.get('inMapStatus', 0)
                print(f"   狀態: {'🟢 在地圖中' if in_map == 1 else '⚪ 不在地圖中'}")
                print(f"   空/滿: {'空' if container.get('emptyFullStatus') == 0 else '滿'}")
                print(f"   搬運中: {'是' if container.get('isCarry') == 1 else '否'}")
        else:
            print(f"❌ 找不到容器: {args.code}")
            sys.exit(1)
    else:
        # 列出所有容器
        if args.all:
            print("🔍 查詢所有容器（包括不在地圖中的）...")
        else:
            print("🔍 查詢在地圖中的容器...")
        containers = query.list_containers(include_out_of_map=args.all)

        # 過濾
        if args.status or args.position:
            print(f"🔍 套用過濾條件...")
            containers = query.filter_containers(
                containers,
                status=args.status,
                position=args.position
            )

        # 輸出
        print()
        if args.json:
            print(query.format_json(containers))
        elif args.simple:
            print(query.format_simple(containers))
        else:
            print(query.format_table(containers))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  操作已取消")
        sys.exit(130)
    except Exception as e:
        print(f"\n❌ 執行錯誤: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
