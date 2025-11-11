#!/usr/bin/env python3
"""
測試 AGV 資料表查詢

目的：
1. 查詢 agv 資料表
2. 找到對應 namespace 的 AGV ID
3. 驗證資料格式轉換
"""

import psycopg2
from psycopg2.extras import RealDictCursor
import json


class AGVTableTest:
    def __init__(self):
        # 資料庫連接配置
        self.db_config = {
            'host': '192.168.10.3',  # AGVC 電腦 IP
            'port': 5432,
            'database': 'agvc',
            'user': 'agvc',
            'password': 'password'
        }
        self.conn = None
        self.cursor = None

    def connect(self):
        """連接資料庫"""
        print("\n【測試 1】連接 PostgreSQL 資料庫")
        print(f"  Host: {self.db_config['host']}")
        print(f"  Port: {self.db_config['port']}")
        print(f"  Database: {self.db_config['database']}")

        try:
            self.conn = psycopg2.connect(**self.db_config)
            self.cursor = self.conn.cursor(cursor_factory=RealDictCursor)
            print("✅ 連接成功！")
            return True
        except Exception as e:
            print(f"❌ 連接失敗: {e}")
            return False

    def test_agv_table_exists(self):
        """測試 2: 檢查 agv 資料表是否存在"""
        print("\n【測試 2】檢查 agv 資料表")

        sql = """
        SELECT table_name
        FROM information_schema.tables
        WHERE table_schema = 'public' AND table_name = 'agv'
        """

        try:
            self.cursor.execute(sql)
            result = self.cursor.fetchall()
            if result:
                print(f"✅ agv 資料表存在")
                return True
            else:
                print(f"❌ agv 資料表不存在")
                return False
        except Exception as e:
            print(f"❌ 查詢失敗: {e}")
            return False

    def test_agv_table_structure(self):
        """測試 3: 查看 agv 資料表結構"""
        print("\n【測試 3】查看 agv 資料表結構")

        sql = """
        SELECT column_name, data_type, is_nullable
        FROM information_schema.columns
        WHERE table_schema = 'public' AND table_name = 'agv'
        ORDER BY ordinal_position
        """

        try:
            self.cursor.execute(sql)
            columns = self.cursor.fetchall()
            print(f"✅ agv 資料表有 {len(columns)} 個欄位：")
            for col in columns:
                print(f"  - {col['column_name']}: {col['data_type']} (nullable: {col['is_nullable']})")
            return True, columns
        except Exception as e:
            print(f"❌ 查詢失敗: {e}")
            return False, None

    def test_query_all_agvs(self):
        """測試 4: 查詢所有 AGV"""
        print("\n【測試 4】查詢所有 AGV")

        sql = """
        SELECT *
        FROM agv
        ORDER BY id
        """

        try:
            self.cursor.execute(sql)
            results = self.cursor.fetchall()
            print(f"✅ 查詢成功！共 {len(results)} 筆 AGV 資料")

            for i, agv in enumerate(results, 1):
                print(f"\n--- AGV {i} ---")
                for key, value in agv.items():
                    print(f"  {key}: {value} (type: {type(value).__name__})")

            return True, results
        except Exception as e:
            print(f"❌ 查詢失敗: {e}")
            import traceback
            traceback.print_exc()
            return False, None

    def test_query_by_name(self, agv_name):
        """測試 5: 根據名稱查詢 AGV"""
        print(f"\n【測試 5】根據名稱查詢 AGV: {agv_name}")

        sql = """
        SELECT id, name, status, battery_level, current_node_id,
               is_online, last_seen, description
        FROM agv
        WHERE name = %s
        """

        try:
            self.cursor.execute(sql, (agv_name,))
            result = self.cursor.fetchone()

            if result:
                print(f"✅ 找到 AGV！")
                print(f"\n查詢結果：")
                for key, value in result.items():
                    print(f"  {key}: {value} (type: {type(value).__name__})")
                return True, result
            else:
                print(f"⚠️ 找不到名稱為 '{agv_name}' 的 AGV")
                return False, None
        except Exception as e:
            print(f"❌ 查詢失敗: {e}")
            import traceback
            traceback.print_exc()
            return False, None

    def test_namespace_mapping(self):
        """測試 6: 測試 namespace 映射邏輯"""
        print("\n【測試 6】測試 namespace 映射")

        # 模擬不同的 namespace
        test_namespaces = [
            "loader02",
            "unloader02",
            "cargo_mover01"
        ]

        print("\n常見的 namespace 格式：")
        for ns in test_namespaces:
            print(f"  - {ns}")

        print("\n請確認您的 AGV 使用哪種命名方式")

    def simulate_data_conversion(self, agv_data):
        """測試 7: 模擬資料轉換為 ROS Message"""
        print("\n【測試 7】模擬資料轉換為 ROS Message")

        try:
            # 模擬 AGVMsg 的欄位（根據實際的 msg 定義）
            agv_msg = {}

            # 基本欄位轉換
            agv_msg['id'] = int(agv_data.get('id', 0))
            agv_msg['name'] = str(agv_data.get('name', ''))
            agv_msg['status'] = str(agv_data.get('status', ''))

            # 可選欄位
            battery_level = agv_data.get('battery_level')
            agv_msg['battery_level'] = float(battery_level) if battery_level is not None else 0.0

            current_node_id = agv_data.get('current_node_id')
            agv_msg['current_node_id'] = int(current_node_id) if current_node_id is not None else 0

            is_online = agv_data.get('is_online')
            agv_msg['is_online'] = bool(is_online) if is_online is not None else False

            description = agv_data.get('description')
            agv_msg['description'] = str(description) if description else ''

            print("✅ 資料轉換成功！")
            print(f"\n轉換後的資料：")
            for key, value in agv_msg.items():
                print(f"  {key}: {value} (type: {type(value).__name__})")

            return True, agv_msg
        except Exception as e:
            print(f"❌ 資料轉換失敗: {e}")
            import traceback
            traceback.print_exc()
            return False, None

    def close(self):
        """關閉連接"""
        if self.cursor:
            self.cursor.close()
        if self.conn:
            self.conn.close()
        print("\n🔌 資料庫連接已關閉")


def main():
    print("=" * 60)
    print("AGV 資料表查詢測試")
    print("=" * 60)

    test = AGVTableTest()

    try:
        # 測試 1: 連接資料庫
        if not test.connect():
            print("\n❌ 無法連接資料庫，測試中止")
            return

        # 測試 2: 檢查 agv 資料表
        if not test.test_agv_table_exists():
            print("\n❌ agv 資料表不存在")
            return

        # 測試 3: 查看資料表結構
        test.test_agv_table_structure()

        # 測試 4: 查詢所有 AGV
        success, agvs = test.test_query_all_agvs()

        if not success or not agvs:
            print("\n⚠️ 沒有 AGV 資料")
        else:
            # 測試 5: 根據名稱查詢
            print("\n" + "=" * 60)
            print("\n可用的 AGV 名稱：")
            for agv in agvs:
                print(f"  - {agv.get('name')} (ID: {agv.get('id')})")

            print("\n" + "=" * 60)
            agv_name = input("\n請輸入要查詢的 AGV 名稱（例如: loader02）: ").strip()

            if agv_name:
                success, agv_data = test.test_query_by_name(agv_name)
                if success and agv_data:
                    # 測試 7: 資料轉換
                    test.simulate_data_conversion(agv_data)

        # 測試 6: Namespace 映射說明
        test.test_namespace_mapping()

        print("\n" + "=" * 60)
        print("✅ 所有測試完成！")
        print("=" * 60)

        print("\n💡 下一步：")
        print("  1. 確認 AGV 的命名格式（例如: loader02, unloader02）")
        print("  2. 確認如何從 ROS namespace 映射到資料庫 name")
        print("  3. 實作 agvs_callback 的備援機制")

    except KeyboardInterrupt:
        print("\n\n🛑 測試中斷")
    except Exception as e:
        print(f"\n❌ 測試異常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        test.close()


if __name__ == '__main__':
    main()
