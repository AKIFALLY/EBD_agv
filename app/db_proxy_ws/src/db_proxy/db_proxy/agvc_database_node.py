import datetime
import json
from db_proxy.crud.task_crud import task_crud
from db_proxy.crud.rack_crud import rack_crud
# 新增 carrier_crud 匯入
from db_proxy.crud.carrier_crud import carrier_crud
from db_proxy.ros_converter import msg_to_model, model_to_msg
from db_proxy.agvc_logger_sub import AgvcLogger
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.service import Service
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.sql.db_install import initialize_default_data
# 只需匯入 SQLModel 與 model
from db_proxy.models import Task, Work, TaskStatus, ProcessSettings, Product, TrafficZone, Rack, Location, Eqp, AGV, Carrier, EqpSignal
from db_proxy_interfaces.srv import SqlQuery
from db_proxy_interfaces.srv import CarrierQuery
from db_proxy_interfaces.srv import RackQuery  # 匯入 Rack 服務
from db_proxy_interfaces.srv import EqpSignalQuery  # 匯入 EqpSignal 服務
from db_proxy_interfaces.msg import Carrier as CarrierMsg  # 匯入 Carrier 訊息
from db_proxy_interfaces.msg import EqpSignal as EqpSignalMsg  # 匯入 EqpSignal 訊息
from db_proxy_interfaces.srv import UpdateTask, UpdateRack
from db_proxy_interfaces.srv import GenericQuery  # 新增GenericQuery服務
# 新增 UpdateCarrier 服務匯入
from db_proxy_interfaces.srv import UpdateCarrier
from db_proxy_interfaces.msg import Tasks, Task as TaskMsg  # 匯入 Tasks 訊息
from db_proxy_interfaces.msg import Racks, Rack as RackMsg  # 匯入 Rack 訊息
from db_proxy_interfaces.msg import Works, Work as WorkMsg  # 匯入 Work 訊息
from db_proxy_interfaces.msg import Locations, Location as LocationMsg  # 匯入 Location 訊息
from db_proxy_interfaces.msg import Eqps, Eqp as EqpMsg  # 匯入 Eqp 訊息
from db_proxy_interfaces.msg import AGVs, AGV as AGVMsg  # 匯入 AGV 訊息
from db_proxy_interfaces.msg import Fetch, Tables  # 擷取請求(抓取資料表), 回應資料表
from sqlmodel import SQLModel, select, text
from sqlalchemy.orm import selectinload


# 需要先編譯 agv_interfaces 套件 (agv_ws)


# from db_proxy_interfaces.srv import AcquireTrafficArea, ReleaseTrafficArea, AddTrafficArea  # Define custom services


class AGVCDatabaseNode(Node):
    def __init__(self):
        super().__init__("agvc_database_node")
        self.get_logger().info("🚀 AGVC Database Node 已啟動")

        db_url_agvc = self.declare_parameter(
            'db_url_agvc',
            'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
        ).value
        self.get_logger().info(f"使用資料庫 URL: {db_url_agvc}")

        # 使用 SQLModel metadata 建立資料表
        self.pool_agvc = ConnectionPoolManager(
            db_url_agvc)  # SQLModel.metadata.create_all

        # self.logger = AgvcLogger(self, self.pool_agvc)

        # 初始化預設資料
        initialize_default_data(self.pool_agvc)

        # 定時發佈 Tasks, Racks, Works 資料
        self.pub_datas = [Work, Works, WorkMsg], [Task, Tasks, TaskMsg], [Rack, Racks, RackMsg], [
            # AGV 和 Carrier 沒有對應的 ROS message
            Location, Locations, LocationMsg], [Eqp, Eqps, EqpMsg], [AGV, AGVs, AGVMsg]
        # 初始化 publishers：以 ROS List Msg 作為 key，publisher 作為 value
        self.publisher_list = {
            ros_msg_list.__name__: self.create_publisher(
                ros_msg_list,
                f"/agvc/{ros_msg_list.__name__.lower()}",  # 自動生成 topic 名稱
                10
            )
            for _, ros_msg_list, _ in self.pub_datas
        }
        print("📌 publisher_list keys:", self.publisher_list.keys())

        # 新增: 用於控制發佈邏輯
        self.last_publish_time = {}
        self.publish_interval = 10.0  # 10 秒
        self.force_publish_flags = {
            ros_msg_list.__name__: False
            for _, ros_msg_list, _ in self.pub_datas
        }

        self.publish_all_table()

        self.generic_query_service = self.create_service(
            SqlQuery, "/agvc/sql_query", self.handle_sql_query)
        # 操作所有的SQL
        self.create_service(GenericQuery, "/agvc/generic_query", self.handle_generic_query)

        # 查詢特定條件的Carrier
        self.car_query_service = self.create_service(
            CarrierQuery, "/agvc/carrier_query", self.handle_carrier_query)
        # 查詢特定條件的Rack
        self.rack_service = self.create_service(
            RackQuery, "/agvc/rack_query", self.handle_rack_query)
        # 查詢特定條件的EqpSignal
        self.eqp_signal_service = self.create_service(
            EqpSignalQuery, "/agvc/eqp_signal_query", self.handle_eqp_signal_query)
        # 新增或更新task
        self.update_task = self.create_service(
            UpdateTask, "/agvc/update_task", self.handle_update_task)
        # 新增或更新rack
        self.update_rack = self.create_service(
            UpdateRack, "/agvc/update_rack", self.handle_update_rack)
        # 新增或更新carrier
        self.update_carrier = self.create_service(
            UpdateCarrier, "/agvc/update_carrier", self.handle_update_carrier)

        self.sub_fetch_request = self.create_subscription(
            Fetch, "/agvc/fetch", self.handle_fetch_tables, 10)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("⏰ 每秒查詢 資料表 的定時器已啟動")

    def handle_fetch_tables(self, request: Fetch):
        """處理擷取資料表的請求"""
        self.get_logger().info(f"📥 收到擷取資料表請求，回覆至: {request.response_to_topic}")

        # 初始化 publisher
        topic = request.response_to_topic
        publisher = self.create_publisher(Tables, topic, 10)

        # 預設空陣列，避免未定義錯誤
        works, tasks, racks, locations, eqps = [], [], [], [], []

        # 查詢對應資料
        if request.works:
            self.force_publish_flags['Works'] = True
            self.get_logger().info("🔍 查詢 Works 資料")
            works = self.query_all(Work, WorkMsg)
        if request.tasks:
            self.force_publish_flags['Tasks'] = True
            self.get_logger().info("🔍 查詢 Tasks 資料")
            tasks = self.query_all(Task, TaskMsg)
        if request.racks:
            self.force_publish_flags['Racks'] = True
            self.get_logger().info("🔍 查詢 Racks 資料")
            racks = self.query_all(Rack, RackMsg)
        if request.locations:
            self.force_publish_flags['Locations'] = True
            self.get_logger().info("🔍 查詢 Locations 資料")
            locations = self.query_all(Location, LocationMsg)
        if request.eqps:
            self.force_publish_flags['Eqps'] = True
            self.get_logger().info("🔍 查詢 Eqps 資料")
            eqps = self.query_all(Eqp, EqpMsg)

        # 整理成 Tables 回覆物件
        response = Tables()
        response.works = works
        response.tasks = tasks
        response.racks = racks
        response.locations = locations
        response.eqps = eqps
        response.success = True
        response.message = "✅ 資料表擷取成功"

        # 發佈
        publisher.publish(response)
        self.get_logger().info(f"📤 已將結果發佈至主題: {topic}")

        # 銷毀 publisher（釋放資源）
        self.destroy_publisher(publisher)
        self.get_logger().info(f"🗑️ 已銷毀 publisher：{topic}")

    def handle_generic_query(self, request, response):
        """通用 SQL 查詢/新增/更新/刪除服務"""
        try:
            with self.pool_agvc.get_session() as session:
                mode = request.mode.lower()
                table = request.table_name
                columns = list(request.columns)
                data = list(request.data)
                condition = request.condition

                if mode == 'select':
                    col_str = ", ".join(columns) if columns else "*"
                    sql = f"SELECT {col_str} FROM {table}"
                    if condition:
                        sql += f" WHERE {condition}"
                    result = session.execute(text(sql)).fetchall()

                    # ✅ 將每筆 row 轉成 JSON 字串（含 datetime）
                    response.results = [json.dumps(
                        dict(r._mapping), default=str, ensure_ascii=False) for r in result]
                    response.success = True
                    response.message = f"✅ 查詢成功，共 {len(response.results)} 筆"

                elif mode in ['insert', 'update']:
                    bind = {col: parse_value(val) for col, val in zip(columns, data)}
                    # bind = dict(zip(columns, data))
                    if mode == 'insert':
                        col_str = ", ".join(columns)
                        placeholder = ", ".join([f":{col}" for col in columns])
                        sql = f"INSERT INTO {table} ({col_str}) VALUES ({placeholder})"
                        strmessage = "✅ 資料插入成功"
                    else:
                        set_clause = ", ".join([f"{col} = :{col}" for col in columns])
                        sql = f"UPDATE {table} SET {set_clause}"
                        strmessage = "✅ 資料更新成功"
                        if condition:
                            sql += f" WHERE {condition}"

                    session.execute(text(sql), bind)
                    session.commit()
                    response.success = True
                    response.message = strmessage

                elif mode == 'delete':
                    sql = f"DELETE FROM {table}"
                    if condition:
                        sql += f" WHERE {condition}"
                    session.execute(text(sql))
                    session.commit()
                    response.success = True
                    response.message = "✅ 刪除成功"

                else:
                    response.success = False
                    response.message = f"❌ 不支援的 mode: {mode}"

        except Exception as e:
            response.success = False
            response.message = f"❌ 發生錯誤: {str(e)}"
            response.results = []
        return response

    def shutdown(self):
        """確保關閉連線池資源"""
        self.pool_agvc.shutdown()
        self.get_logger().info("🔻 agvc db node已關閉")

    # def handle_acquire_traffic_area(self, request, response):
    #    success = self.acquire_traffic_area(request.traffic_id, request.agv_id)
    #    response.success = success
    #    return response
#
    # def handle_release_traffic_area(self, request, response):
    #    success = self.release_traffic_area(request.traffic_id, request.agv_id)
    #    response.success = success
    #    return response
#
    # def handle_add_traffic_area(self, request, response):
    #    self.add_traffic_area(request.traffic_id, request.name, request.points)
    #    response.success = True
    #    return response
    def handle_carrier_query(self, request, response):
        """處理 Carrier 查詢請求"""
        try:
            self.get_logger().info(f"📥 Carrier 查詢請求: {request}")

            with self.pool_agvc.get_session() as session:
                carriers = []

                if request.id and request.id != 0:
                    # 查詢特定 ID 的 Carrier
                    carrier = session.exec(
                        select(Carrier)
                        .where(Carrier.id == request.id)).first()
                    if carrier:
                        carriers.append(carrier)
                else:
                    # 組建依條件查詢語句
                    sql_query = select(Carrier)

                    filters = []
                    if request.room_id and request.room_id != 0:
                        filters.append(Carrier.room_id == request.room_id)
                    if request.rack_id and request.rack_id != 0:
                        filters.append(Carrier.rack_id == request.rack_id)
                    if request.port_id and request.port_id != 0:
                        filters.append(Carrier.port_id == request.port_id)

                    # 支援 port_id 範圍查詢
                    if request.port_id_min and request.port_id_min != 0:
                        filters.append(Carrier.port_id >= request.port_id_min)
                    if request.port_id_max and request.port_id_max != 0:
                        filters.append(Carrier.port_id <= request.port_id_max)

                    if request.rack_index and request.rack_index != 0:
                        filters.append(Carrier.rack_index ==
                                       request.rack_index)
                    if request.status_id:
                        filters.append(Carrier.status_id == request.status_id)

                    if filters:
                        sql_query = sql_query.where(*filters)

                    # 👉 排序邏輯
                    if request.sort_order == 1:
                        sql_query = sql_query.order_by(
                            Carrier.rack_index.asc())
                    elif request.sort_order == 2:
                        sql_query = sql_query.order_by(
                            Carrier.updated_at.asc())
                    elif request.sort_order == -2:
                        sql_query = sql_query.order_by(
                            Carrier.updated_at.desc())
                    elif request.sort_order == 3:
                        sql_query = sql_query.order_by(
                            Carrier.created_at.asc())
                    elif request.sort_order == -3:
                        sql_query = sql_query.order_by(
                            Carrier.created_at.desc())
                    elif request.sort_order == -1:
                        sql_query = sql_query.order_by(
                            Carrier.rack_index.desc())
                    # sort_order == 0 或 None 就不排序

                    carriers = session.exec(sql_query).all()

            datas = []
            for item in carriers:
                item_msg = model_to_msg(item, CarrierMsg)
                datas.append(item_msg)
            response.datas = datas
            response.success = True
            response.message = f"✅ 查詢成功，共 {len(carriers)} 筆"

        except Exception as e:
            self.get_logger().error(f"❌ Carrier 查詢失敗: {e}")
            response.success = False
            response.message = str(e)
            response.datas = []

        return response

    def handle_rack_query(self, request, response):
        """處理 Rack 查詢請求"""
        try:
            self.get_logger().info(f"📥 Rack 查詢請求: {request}")

            with self.pool_agvc.get_session() as session:
                racks = []

                if request.id and request.id != 0:
                    # 查詢特定 ID 的 Rack
                    rack = session.exec(
                        select(Rack)
                        .where(Rack.id == request.id)).first()
                    if rack:
                        racks.append(rack)
                else:
                    # 組建依條件查詢語句
                    sql_query = select(Rack)

                    filters = []
                    if request.location_id and request.location_id != 0:
                        filters.append(Rack.location_id == request.location_id)
                    if request.product_id and request.product_id != 0:
                        filters.append(Rack.product_id == request.product_id)
                    if request.status_id and request.status_id != 0:
                        filters.append(Rack.status_id == request.status_id)
                    if request.direction and request.direction != 0:
                        filters.append(Rack.direction == request.direction)

                    if filters:
                        sql_query = sql_query.where(*filters)

                    racks = session.exec(sql_query).all()

            datas = []
            for item in racks:
                item_msg = model_to_msg(item, RackMsg)
                datas.append(item_msg)
            response.datas = datas
            response.success = True
            response.message = f"✅ 查詢成功，共 {len(racks)} 筆"

        except Exception as e:
            self.get_logger().error(f"❌ Rack 查詢失敗: {e}")
            response.success = False
            response.message = str(e)
            response.datas = []

        return response

    def handle_sql_query(self, request, response):
        try:
            self.get_logger().info(f"---- {request.query_string}")

            with self.pool_agvc.get_session() as session:
                results = session.exec(text(request.query_string)).all()
                self.get_logger().info(f"---- {results}")
                for r in results:
                    print(r, type(r))

                json_rows = [dict(r._mapping) for r in results]
                response.json_result = json.dumps(json_rows, default=str)

                # response.json_result = json.dumps(
                #    [dict(r) for r in results], default=str)
                response.success = True
                response.message = f"{len(results)} record(s) found"
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.json_result = "[]"
        return response

    def query_all(self, sql_model, ros_msg):
        """查詢所有資料並發佈成 ROS 訊息"""
        with self.pool_agvc.get_session() as session:
            items = session.exec(select(sql_model)).all()
        datas = []
        for item in items:
            item_msg = model_to_msg(item, ros_msg)
            datas.append(item_msg)
        return datas

    def handle_update_task(self, request, response):
        """處理新增或更新 Task 的請求"""
        new_task = msg_to_model(request.task, Task)

        # 自動處理時間戳欄位
        # 修正：使用 datetime.datetime.now() 或正確導入 datetime 類別
        from datetime import datetime, timezone
        current_time = datetime.now(timezone.utc)
        if new_task.id is None or new_task.id == 0:  # 新建任務
            new_task.id = None
            new_task.created_at = current_time
        new_task.updated_at = current_time

        try:
            with self.pool_agvc.get_session() as session:
                task_result = task_crud.create_or_update(session, new_task)
                task_msg = model_to_msg(task_result, TaskMsg)

                response.success = True
                response.message = "Task 設定完成"
                response.task = task_msg
                self.force_publish_flags['Tasks'] = True  # 設定旗標，強制發佈
                self.get_logger().info(f"task_crud:{task_result.model_dump()}")
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.task = None
        return response

    def handle_update_rack(self, request, response):
        """處理新增或更新 Task 的請求"""
        # 或從 ROS request 轉回 model
        new_rack: Rack = msg_to_model(request.rack, Rack)
        try:
            with self.pool_agvc.get_session() as session:
                rack_result = rack_crud.create_or_update(session, new_rack)

                # 資料庫回來的 RackModel 轉為 ROS message
                rack_msg: RackMsg = model_to_msg(rack_result, RackMsg)

                response.success = True
                response.message = "Task 設定完成"
                response.rack = rack_msg
                self.force_publish_flags['Racks'] = True  # 設定旗標，強制發佈
                self.get_logger().info(f"rack_crud:{rack_result}")
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.rack = None
        return response

    def handle_update_carrier(self, request, response):
        """處理新增或更新 Carrier 的請求"""
        # 從 ROS request 轉回 model
        self.get_logger().info(f"📥 收到 Carrier 更新請求: {request}")
        new_carrier: Carrier = msg_to_model(request.carrier, Carrier)
        # 確保 room_id 被設定
        new_carrier.room_id = None if new_carrier.room_id == 0 else new_carrier.room_id
        new_carrier.rack_id = None if new_carrier.rack_id == 0 else new_carrier.rack_id
        new_carrier.port_id = None if new_carrier.port_id == 0 else new_carrier.port_id
        new_carrier.rack_index = None if new_carrier.rack_index == 0 else new_carrier.rack_index
        try:
            with self.pool_agvc.get_session() as session:
                carrier_result = carrier_crud.create_or_update(
                    session, new_carrier)

                # 資料庫回來的 CarrierModel 轉為 ROS message
                carrier_msg: CarrierMsg = model_to_msg(
                    carrier_result, CarrierMsg)

                response.success = True
                response.message = "Carrier 設定完成"
                response.carrier = carrier_msg
                self.get_logger().info(f"carrier_crud:{carrier_result}")

                # 如果 carrier 是在 房間內的時候, 要去更新該房間的預烘的在席及出料狀態
                # 如果carrier的eqp是在預烘的時候,要自動去預烘訊號更新 在席以及出料
                # 如果carrier的eqp不在預烘的時候,要自動去預烘訊號更新 在席以及要料
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.carrier = None
        return response

    def handle_eqp_signal_query(self, request, response):
        """處理 EqpSignal 查詢請求"""
        try:
            self.get_logger().info(f"📥 EqpSignal 查詢請求: {request}")

            with self.pool_agvc.get_session() as session:
                eqp_signals = []

                if request.id and request.id != 0:
                    # 查詢特定 ID 的 EqpSignal
                    eqp_signal = session.exec(
                        select(EqpSignal)
                        .where(EqpSignal.id == request.id)).first()
                    if eqp_signal:
                        eqp_signals.append(eqp_signal)
                else:
                    # 組建依條件查詢語句
                    sql_query = select(EqpSignal)

                    filters = []
                    if request.eqp_id and request.eqp_id != 0:
                        filters.append(EqpSignal.eqp_id == request.eqp_id)
                    if request.name:
                        filters.append(EqpSignal.name == request.name)
                    if request.description:
                        filters.append(EqpSignal.description ==
                                       request.description)
                    if request.value:
                        filters.append(EqpSignal.value == request.value)
                    if request.type_of_value:
                        filters.append(EqpSignal.type_of_value ==
                                       request.type_of_value)

                    if filters:
                        sql_query = sql_query.where(*filters)

                    eqp_signals = session.exec(sql_query).all()

            datas = []
            for item in eqp_signals:
                item_msg = model_to_msg(item, EqpSignalMsg)
                datas.append(item_msg)
            response.datas = datas
            response.success = True
            response.message = f"✅ 查詢成功，共 {len(eqp_signals)} 筆"

        except Exception as e:
            self.get_logger().error(f"❌ EqpSignal 查詢失敗: {e}")
            response.success = False
            response.message = str(e)
            response.datas = []

        return response

    def timer_callback(self):
        """定時器回呼函式，每秒執行一次"""
        self.publish_all_table()

    def publish_all_table(self):
        """
        發佈所有資料表。
        - 如果 force_publish_flags 為 True，則立即發佈。
        - 否則，每 10 秒發佈一次。
        """
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        for sql_model, ros_msg_list, ros_msg in self.pub_datas:
            pub_key = ros_msg_list.__name__
            last_time = self.last_publish_time.get(pub_key, 0)
            force_publish = self.force_publish_flags.get(pub_key, False)

            # 檢查是否需要發佈
            if force_publish or (current_time - last_time >= self.publish_interval):
                # self.get_logger().info(f"📤 發佈 {sql_model.__name__} 資料 (強制: {force_publish})")
                datas = self.query_all(sql_model, ros_msg)
                self.publisher_list[pub_key].publish(
                    ros_msg_list(datas=datas))

                # 更新發佈時間並重設旗標
                self.last_publish_time[pub_key] = current_time
                if force_publish:
                    self.force_publish_flags[pub_key] = False


def parse_value(val: str):
    """將 ROS2 string[] 形式的值轉換為適合 SQL 的 Python 型別"""
    if val is None or val.strip().lower() in {"null", "__null__"}:
        return None
    if val.isdigit():
        return int(val)
    try:
        return float(val)
    except ValueError:
        pass
    try:
        return json.loads(val)  # 若是合法的 JSON 結構（物件或陣列）
    except json.JSONDecodeError:
        return val  # 保留原字串


def main(args=None):
    rclpy.init(args=args)

    # 使用 MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    node = AGVCDatabaseNode()

    # 將節點加入 Executor
    executor.add_node(node)

    try:
        executor.spin()  # 使用 Executor 來管理並發執行
    except KeyboardInterrupt:
        pass  # 捕獲 Ctrl+C 並且不輸出異常
    finally:
        node.shutdown()
        # rclpy.shutdown()


if __name__ == "__main__":
    main()
