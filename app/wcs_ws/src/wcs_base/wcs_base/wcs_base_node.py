import rclpy
from rclpy.node import Node
from wcs_base.database_manager import DatabaseManager
from kuka_wcs.task_handler.rack_rotate_180 import RackRotate180Handler
from kuka_wcs.task_handler.empty_rack_to_boxout import EmptyRackToBoxoutHandler  # 空Rack搬運到出傳送箱
from kuka_wcs.task_handler.full_rack_to_manual_receive import FullRackToManualReceiveHandler  # 滿Rack搬運到人工收料區
from kuka_wcs.task_handler.ready_rack_to_boxin import ReadyRackToBoxinHandler  # 準備區料架送往入口傳送箱


class WCSBaseNode(Node):
    """WCS Base 主節點"""

    def __init__(self):
        super().__init__('wcs_base_node')
        self.get_logger().info("🚀 WCS Base Node 正在啟動...")
        # 初始化資料庫連線池
        db_url_agvc = self.declare_parameter(
            'db_url_agvc',
            'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
        ).value

        # 初始化資料庫管理器
        self.db_manager = DatabaseManager(self.get_logger(), db_url_agvc)



        # 初始化任務處理器
        self.rack_rotate_180_handler = RackRotate180Handler(self)
        self.empty_rack_to_boxout_handler = EmptyRackToBoxoutHandler(self)
        self.full_rack_to_manual_receive_handler = FullRackToManualReceiveHandler(self)
        self.ready_rack_to_boxin_handler = ReadyRackToBoxinHandler(self)
        # 創建定時器，每1秒執行一次任務處理 (參照 mission_select_state)
        self.task_timer = self.create_timer(1.0, self.cycle_process)

        # 創建定時器，每1.5秒執行一次任務處理 (參照 mission_select_state)
        self.task_timer_1_5 = self.create_timer(1.5, self.cycle1_5_process)
        
        # 創建定時器,每3秒執行一次
        self.cycle_3_timer = self.create_timer(3.0, self.cycle3_process)

        # 創建定時器，每10秒執行一次 rack_status 更新
        self.rack_status_timer = self.create_timer(10.0, self.cycle10_process)



        #初始化任務判斷物件列表
        # 檢查優先度由上往下排
        self.task_handler_list = [
            #RackRotate180Handler(self),
            #EmptyRackToBoxoutHandler(self),
            #FullRackToManualReceiveHandler(self),
            ReadyRackToBoxinHandler(self)
        ]

        self.get_logger().info("✅ WCS Base Node 啟動完成")

    def cycle_process(self):
        """定時處理任務 - 每1秒執行一次"""
        # 刷新所有資料表
        self.db_manager.refresh_all_tables()

        # 檢查是否有資料
        if self.db_manager.has_all_data():
            # 處理位置狀態
            self.db_manager.location_status_process()

            # 執行任務處理器
            for handler in self.task_handler_list:
                handler.execute()
        else:
            # 調試：檢查哪些資料表缺失
            missing_tables = []
            tables_to_check = [
                ('task_table', self.db_manager.task_table),
                ('task_id_list', self.db_manager.task_id_list),
                ('work_table', self.db_manager.work_table),
                ('location_table', self.db_manager.location_table),
                ('kuka_node_table', self.db_manager.kuka_node_table),
                ('carrier_table', self.db_manager.carrier_table),
                ('rack_table', self.db_manager.rack_table)
            ]

            for table_name, table_data in tables_to_check:
                if not table_data:
                    missing_tables.append(f"{table_name}({type(table_data).__name__}:{len(table_data) if table_data else 0})")

            if missing_tables:
                self.get_logger().info(f"❌ 缺失資料表: {', '.join(missing_tables)}")
            else:
                self.get_logger().info("⚠️ has_all_data() 回傳 False 但所有資料表都有資料")

    def cycle3_process(self):
        """定時處理任務 - 每3秒執行一次"""
        self.db_manager.refresh_periodic_tables()  # 讀取需要定期更新的資料表
        

    def cycle1_5_process(self):
        """定時處理任務 - 每1.5秒執行一次"""
        self.db_manager.rack_status_process()  # 更新rack狀態


    def cycle10_process(self):
        """定時處理任務 - 每10秒執行一次"""
        

    # 資料表操作方法已移至 DatabaseManager




def main(args=None):
    rclpy.init(args=args)
    try:
        node = WCSBaseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
