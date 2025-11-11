#!/usr/bin/env python3
"""
直接連接 PostgreSQL 資料庫測試

測試項目：
1. 直接連接資料庫
2. 查詢 task 資料表
3. 驗證資料格式
"""

import psycopg2
from psycopg2.extras import RealDictCursor
import json
from datetime import datetime


class DirectDatabaseTest:
    def __init__(self):
        # 資料庫連接配置
        # AGVC 電腦 IP: 192.168.10.3
        # PostgreSQL 透過 docker port mapping 開放 5432
        self.db_config = {
            'host': '192.168.10.3',  # AGVC 電腦的實際 IP
            'port': 5432,
            'database': 'agvc',
            'user': 'agvc',
            'password': 'password'
        }
        self.conn = None
        self.cursor = None

    def connect(self):
        """測試 1: 連接資料庫"""
        print("\n【測試 1】連接 PostgreSQL 資料庫")
        print(f"  Host: {self.db_config['host']}")
        print(f"  Port: {self.db_config['port']}")
        print(f"  Database: {self.db_config['database']}")
        print(f"  User: {self.db_config['user']}")

        try:
            self.conn = psycopg2.connect(**self.db_config)
            self.cursor = self.conn.cursor(cursor_factory=RealDictCursor)
            print("✅ 連接成功！")
            return True
        except Exception as e:
            print(f"❌ 連接失敗: {e}")
            return False

    def test_simple_query(self):
        """測試 2: 執行簡單查詢"""
        print("\n【測試 2】執行簡單查詢")

        sql = "SELECT 1 as test"
        print(f"SQL: {sql}")

        try:
            self.cursor.execute(sql)
            result = self.cursor.fetchall()
            print(f"✅ 查詢成功！")
            print(f"結果: {result}")
            return True
        except Exception as e:
            print(f"❌ 查詢失敗: {e}")
            return False

    def test_table_exists(self):
        """測試 3: 檢查 task 資料表是否存在"""
        print("\n【測試 3】檢查 task 資料表")

        sql = """
        SELECT table_name
        FROM information_schema.tables
        WHERE table_schema = 'public' AND table_name = 'task'
        """

        try:
            self.cursor.execute(sql)
            result = self.cursor.fetchall()
            if result:
                print(f"✅ task 資料表存在")
                return True
            else:
                print(f"❌ task 資料表不存在")
                return False
        except Exception as e:
            print(f"❌ 查詢失敗: {e}")
            return False

    def test_task_query(self, agv_id=None):
        """測試 4: 查詢 task 資料表"""
        print("\n【測試 4】查詢 task 資料表")

        if agv_id is not None:
            sql = """
            SELECT id, work_id, status_id, room_id, node_id,
                   name, description, agv_id, priority, parameters,
                   created_at, updated_at
            FROM task
            WHERE agv_id = %s AND status_id IN (1, 2, 3)
            ORDER BY priority DESC, created_at ASC
            """
            params = (agv_id,)
            print(f"查詢 AGV ID={agv_id} 的任務")
        else:
            sql = """
            SELECT id, work_id, status_id, room_id, node_id,
                   name, description, agv_id, priority, parameters,
                   created_at, updated_at
            FROM task
            WHERE status_id IN (1, 2, 3)
            ORDER BY priority DESC, created_at ASC
            LIMIT 10
            """
            params = None
            print(f"查詢前 10 筆進行中的任務")

        try:
            if params:
                self.cursor.execute(sql, params)
            else:
                self.cursor.execute(sql)

            results = self.cursor.fetchall()
            print(f"✅ 查詢成功！查詢到 {len(results)} 筆任務")

            for i, row in enumerate(results, 1):
                print(f"\n--- 任務 {i} ---")
                print(f"  ID: {row['id']}")
                print(f"  Name: {row['name']}")
                print(f"  Work ID: {row['work_id']}")
                print(f"  Status ID: {row['status_id']}")
                print(f"  AGV ID: {row['agv_id']}")
                print(f"  Priority: {row['priority']}")
                print(f"  Node ID: {row['node_id']}")
                print(f"  Parameters type: {type(row['parameters'])}")
                print(f"  Parameters: {row['parameters']}")
                print(f"  Created at: {row['created_at']}")
                print(f"  Updated at: {row['updated_at']}")

            return True, results
        except Exception as e:
            print(f"❌ 查詢失敗: {e}")
            import traceback
            traceback.print_exc()
            return False, None

    def test_data_conversion(self, task_data):
        """測試 5: 驗證資料類型轉換"""
        print("\n【測試 5】驗證資料類型轉換")

        try:
            # 模擬轉換為 ROS Message 格式
            task_msg = {}

            # 數值欄位
            task_msg['id'] = int(task_data['id'])
            task_msg['work_id'] = int(task_data['work_id'])
            task_msg['status_id'] = int(task_data['status_id'])
            task_msg['room_id'] = int(task_data['room_id'])
            task_msg['node_id'] = int(task_data['node_id'])
            task_msg['agv_id'] = int(task_data['agv_id'])

            # priority 是 uint8 (0-255)
            priority = task_data['priority']
            task_msg['priority'] = max(0, min(255, int(priority)))

            # 字串欄位
            task_msg['name'] = str(task_data['name'])
            task_msg['description'] = str(task_data['description']) if task_data['description'] else ''

            # parameters 轉換 (Dict/JSON → string)
            parameters = task_data['parameters']
            if parameters is None:
                task_msg['parameters'] = ''
            elif isinstance(parameters, str):
                task_msg['parameters'] = parameters
            elif isinstance(parameters, dict):
                task_msg['parameters'] = json.dumps(parameters)
            else:
                task_msg['parameters'] = str(parameters)

            # 時間戳轉換 (datetime → ISO string)
            created_at = task_data['created_at']
            updated_at = task_data['updated_at']

            if isinstance(created_at, datetime):
                task_msg['created_at'] = created_at.isoformat()
            else:
                task_msg['created_at'] = str(created_at) if created_at else ''

            if isinstance(updated_at, datetime):
                task_msg['updated_at'] = updated_at.isoformat()
            else:
                task_msg['updated_at'] = str(updated_at) if updated_at else ''

            print("✅ 資料類型轉換成功！")
            print(f"\n轉換後的資料：")
            for key, value in task_msg.items():
                print(f"  {key}: {value} (type: {type(value).__name__})")

            return True, task_msg
        except Exception as e:
            print(f"❌ 資料類型轉換失敗: {e}")
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
    print("PostgreSQL 直接連接測試")
    print("=" * 60)

    test = DirectDatabaseTest()

    try:
        # 測試 1: 連接資料庫
        if not test.connect():
            print("\n❌ 無法連接資料庫，測試中止")
            print("\n請確認以下事項：")
            print("  1. PostgreSQL 容器是否正常運行")
            print("  2. 網路連接是否正常 (192.168.100.254:5432)")
            print("  3. 資料庫憑證是否正確")
            return

        # 測試 2: 簡單查詢
        if not test.test_simple_query():
            print("\n❌ 簡單查詢失敗")
            return

        # 測試 3: 檢查 task 資料表
        if not test.test_table_exists():
            print("\n❌ task 資料表不存在")
            return

        # 測試 4: 查詢所有進行中的任務
        success, tasks = test.test_task_query(agv_id=None)
        if not success:
            print("\n⚠️ 查詢失敗或沒有任務資料")

        # 測試 5: 查詢特定 AGV 的任務
        print("\n" + "=" * 60)
        agv_id = input("請輸入要查詢的 AGV ID（留空跳過）: ").strip()
        if agv_id:
            success, tasks = test.test_task_query(agv_id=int(agv_id))

        # 測試 6: 資料類型轉換
        if tasks and len(tasks) > 0:
            test.test_data_conversion(tasks[0])

        print("\n" + "=" * 60)
        print("✅ 所有測試完成！")
        print("=" * 60)

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
