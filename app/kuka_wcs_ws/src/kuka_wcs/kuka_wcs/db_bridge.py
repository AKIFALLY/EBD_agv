"""
数据库桥接层 - KUKA WCS 专用数据库操作封装

提供简化的数据库操作接口，供任务处理器使用
"""
from typing import List, Optional, Dict, Any
from sqlmodel import select, Session
from db_proxy.models import Location, Rack, Carrier, Task, Product, Room, Work
from db_proxy.utils.runtime_log_helper import TaskLogHelper
import json


class KukaWcsDbBridge:
    """KUKA WCS 数据库桥接类"""

    def __init__(self, logger):
        """
        初始化数据库桥接

        Args:
            logger: ROS2 logger 实例
        """
        self.logger = logger

    # ========== Location 查询 ==========

    def query_locations(self, session: Session, **kwargs) -> List[Location]:
        """
        查询 locations

        Args:
            session: 数据库 session
            **kwargs: 过滤条件
                - type: location 类型 (如 "room_outlet", "room_inlet")
                - id: location ID
                - ids: location ID 列表
                - node_id: node ID

        Returns:
            Location 对象列表
        """
        statement = select(Location)

        if 'type' in kwargs:
            statement = statement.where(Location.type == kwargs['type'])
        if 'id' in kwargs:
            statement = statement.where(Location.id == kwargs['id'])
        if 'ids' in kwargs:
            statement = statement.where(Location.id.in_(kwargs['ids']))
        if 'node_id' in kwargs:
            statement = statement.where(Location.node_id == kwargs['node_id'])

        return session.exec(statement).all()

    # ========== Rack 查询 ==========

    def query_racks(self, session: Session, **kwargs) -> List[Rack]:
        """
        查询 racks

        Args:
            session: 数据库 session
            **kwargs: 过滤条件
                - id: rack ID
                - location_id: location ID
                - is_in_map: 是否在地图上 (0/1)
                - is_carry: 是否被搬运中 (0/1)
                - is_docked: 是否已对接 (0/1)

        Returns:
            Rack 对象列表
        """
        statement = select(Rack)

        if 'id' in kwargs:
            statement = statement.where(Rack.id == kwargs['id'])
        if 'location_id' in kwargs:
            statement = statement.where(Rack.location_id == kwargs['location_id'])
        if 'is_in_map' in kwargs:
            statement = statement.where(Rack.is_in_map == kwargs['is_in_map'])
        if 'is_carry' in kwargs:
            statement = statement.where(Rack.is_carry == kwargs['is_carry'])
        if 'is_docked' in kwargs:
            statement = statement.where(Rack.is_docked == kwargs['is_docked'])

        return session.exec(statement).all()

    # ========== Carrier 查询 ==========

    def query_carriers(self, session: Session, **kwargs) -> List[Carrier]:
        """
        查询 carriers

        Args:
            session: 数据库 session
            **kwargs: 过滤条件
                - rack_id: rack ID
                - rack_index: rack 上的位置索引 (1-32)
                - status_id: carrier 状态 ID
                - room_id: room ID

        Returns:
            Carrier 对象列表
        """
        statement = select(Carrier)

        if 'rack_id' in kwargs:
            statement = statement.where(Carrier.rack_id == kwargs['rack_id'])
        if 'rack_index' in kwargs:
            statement = statement.where(Carrier.rack_index == kwargs['rack_index'])
        if 'status_id' in kwargs:
            statement = statement.where(Carrier.status_id == kwargs['status_id'])
        if 'room_id' in kwargs:
            statement = statement.where(Carrier.room_id == kwargs['room_id'])

        return session.exec(statement).all()

    # ========== 业务逻辑查询 ==========

    def get_rack_at_location(self, session: Session, location_id: int) -> Optional[Rack]:
        """
        获取指定 location 上的 rack

        Args:
            session: 数据库 session
            location_id: location ID

        Returns:
            Rack 对象或 None
        """
        racks = self.query_racks(session, location_id=location_id, is_in_map=1)
        return racks[0] if racks else None

    def get_product_by_id(self, session: Session, product_id: int) -> Optional[Product]:
        """
        根据 ID 获取产品信息

        Args:
            session: 数据库 session
            product_id: product ID

        Returns:
            Product 对象或 None
        """
        return session.get(Product, product_id)

    def get_room_by_id(self, session: Session, room_id: int) -> Optional[Room]:
        """
        根据 ID 获取房间信息

        Args:
            session: 数据库 session
            room_id: room ID

        Returns:
            Room 对象或 None
        """
        return session.get(Room, room_id)

    def check_rack_full(self, session: Session, rack: Rack, product: Product) -> Dict[str, Any]:
        """
        检查 rack 是否满载或尾批

        根据产品尺寸和 carrier_bitmap 判断：
        - S 尺寸: 32 slots (carrier_bitmap 全部填满 = 满载)
        - L 尺寸: 16 slots (carrier_bitmap 全部填满 = 满载)

        Args:
            session: 数据库 session
            rack: Rack 对象
            product: Product 对象

        Returns:
            {
                'is_full': bool,  # 是否满载
                'is_final_batch': bool,  # 是否尾批
                'filled_count': int,  # 已填充数量
                'total_slots': int,  # 总槽位数量
            }
        """
        # 根据产品尺寸确定总槽位数
        total_slots = 32 if product.dim == 'S' else 16

        # 解析 carrier_bitmap（JSON 字符串）
        try:
            carrier_bitmap = json.loads(rack.carrier_bitmap) if rack.carrier_bitmap else []
        except (json.JSONDecodeError, TypeError):
            self.logger.warn(f"Rack {rack.id} carrier_bitmap 解析失败: {rack.carrier_bitmap}")
            carrier_bitmap = []

        # 计算已填充数量
        filled_count = sum(1 for bit in carrier_bitmap if bit == 1)

        # 判断是否满载
        is_full = (filled_count == total_slots)

        # 判断是否尾批（这里简化实现，可根据业务需求调整）
        # 尾批逻辑：根据房间的尾批标志或其他业务规则判断
        is_final_batch = False  # TODO: 根据实际业务逻辑实现

        return {
            'is_full': is_full,
            'is_final_batch': is_final_batch,
            'filled_count': filled_count,
            'total_slots': total_slots,
        }

    def check_rack_side_status(self, session: Session, rack: Rack, side: str = 'A') -> Dict[str, Any]:
        """
        检查 rack 某一面的状态（用于旋转逻辑）

        Args:
            session: 数据库 session
            rack: Rack 对象
            side: 'A' 或 'B' 面

        Returns:
            {
                'is_empty': bool,  # 该面是否为空
                'is_full': bool,   # 该面是否满载
                'carriers': List[Carrier],  # 该面的 carriers
            }
        """
        # A 面: rack_index 1-16, B 面: rack_index 17-32
        if side == 'A':
            index_range = range(1, 17)
        else:  # side == 'B'
            index_range = range(17, 33)

        # 查询该面的 carriers
        carriers = []
        for idx in index_range:
            carrier_list = self.query_carriers(session, rack_id=rack.id, rack_index=idx)
            if carrier_list:
                carriers.extend(carrier_list)

        # 判断是否为空（没有任何 carrier）
        is_empty = (len(carriers) == 0)

        # 判断是否满载（所有位置都有 carrier，且状态不为空）
        occupied_count = sum(1 for c in carriers if c.status_id != 0)
        is_full = (occupied_count == 16)

        return {
            'is_empty': is_empty,
            'is_full': is_full,
            'carriers': carriers,
        }

    # ========== Task 创建 ==========

    def create_kuka_task(
        self,
        session: Session,
        work_id: int,
        nodes: List[str],
        rack_id: Optional[int] = None,
        room_id: Optional[int] = None,
        location_id: Optional[int] = None,
        rotation_angle: Optional[int] = None,
        **kwargs
    ) -> Task:
        """
        创建 KUKA 任务

        Args:
            session: 数据库 session
            work_id: 工作 ID (210001=move, 220001=rack_move, etc.)
            nodes: 节点列表 (node names)
            rack_id: rack ID（可选）
            room_id: room ID（可选，但建議提供）
            location_id: location ID（可选，但建議提供用於防重複）
            rotation_angle: 旋转角度（可选，如 180）
            **kwargs: 其他参数

        Returns:
            创建的 Task 对象
        """
        # 查询 work 表获取 name 和 description
        work = session.get(Work, work_id)
        if not work:
            self.logger.warn(f"Work ID {work_id} 不存在，使用預設值")
            task_name = f"KUKA任務 #{work_id}"
            task_description = "自動生成的 KUKA 任務"
        else:
            task_name = work.name
            task_description = work.description or work.name

        # 允許從 kwargs 覆蓋任務名稱和描述
        task_name = kwargs.get('name', task_name)
        task_description = kwargs.get('description', task_description)

        # 构建 parameters 字典
        parameters = {
            "model": "KUKA400i",  # 统一使用大寫（与 rcs 保持一致）
            "nodes": nodes,
        }

        if rotation_angle is not None:
            parameters["rotation_angle"] = rotation_angle

        # 合并其他参数
        parameters.update(kwargs.get('parameters', {}))

        # 创建任务
        task = Task(
            work_id=work_id,
            rack_id=rack_id,
            room_id=room_id,  # 明確設置 room_id
            location_id=location_id,  # 明確設置 location_id（用於防重複）
            status_id=1,  # PENDING
            name=task_name,  # 從 work 表查詢
            description=task_description,  # 從 work 表查詢
            parameters=parameters,
            priority=kwargs.get('priority', 5),
            notes=kwargs.get('notes'),
        )

        session.add(task)
        session.commit()
        session.refresh(task)

        # 記錄任務創建到 RuntimeLog
        TaskLogHelper.log_task_create_success(
            session=session,
            task_id=task.id,
            work_id=work_id,
            status_id=task.status_id,
            task_name=task.name,
            room_id=room_id,
            rack_id=rack_id,
            location_id=location_id,
            node_name="kuka_wcs"
        )
        session.commit()  # 提交 RuntimeLog

        self.logger.info(
            f"创建 KUKA 任务成功: Task ID={task.id}, Work ID={work_id}, Name={task_name}, "
            f"Room ID={room_id}, Location ID={location_id}, Rack ID={rack_id}"
        )
        return task

    # ========== Rack 更新 ==========

    def update_rack_status(
        self,
        session: Session,
        rack_id: int,
        **updates
    ) -> Optional[Rack]:
        """
        更新 rack 状态

        Args:
            session: 数据库 session
            rack_id: rack ID
            **updates: 要更新的字段
                - location_id: 新的 location ID
                - is_carry: 是否被搬运中
                - is_docked: 是否已对接
                - direction: 方向角度
                - 等等

        Returns:
            更新后的 Rack 对象或 None
        """
        rack = session.get(Rack, rack_id)
        if not rack:
            self.logger.warn(f"Rack {rack_id} 不存在")
            return None

        # 更新字段
        for key, value in updates.items():
            if hasattr(rack, key):
                setattr(rack, key, value)

        session.add(rack)
        session.commit()
        session.refresh(rack)

        self.logger.info(f"更新 Rack {rack_id} 成功: {updates}")
        return rack

    # ========== Waypoint 查詢 ==========

    def get_waypoint_nodes(
        self,
        session: Session,
        source_location_id: int,
        target_location_id: int
    ) -> List[int]:
        """
        獲取兩個 location 之間的路徑節點（包含 waypoint）

        查詢邏輯：
        1. 查詢 source_location 和 target_location
        2. 檢查兩者的 waypoint_node_id
        3. 如果都為空 → 返回 [source_location_id, target_location_id]
        4. 如果任一有值 → 返回 [source_location_id, waypoint_node_id, target_location_id]

        ⚠️ 注意：直接使用 location_id 作為路徑節點，不使用 location.node_id

        Args:
            session: 資料庫 session
            source_location_id: 出料位置 ID
            target_location_id: 要料位置 ID

        Returns:
            節點 ID 列表 [source_location_id, waypoint(可選), target_location_id]
        """
        # 查詢 source location
        source_location = session.get(Location, source_location_id)
        if not source_location:
            self.logger.warn(f"Location {source_location_id} 不存在")
            return []

        # 查詢 target location
        target_location = session.get(Location, target_location_id)
        if not target_location:
            self.logger.warn(f"Location {target_location_id} 不存在")
            return []

        # 檢查 waypoint_node_id（只使用 target 的 waypoint，若無則直接連接）
        waypoint_node_id = target_location.waypoint_node_id

        if waypoint_node_id:
            # 有 waypoint：直接使用 location_id 作為起點和終點
            nodes = [source_location_id, waypoint_node_id, target_location_id]
            self.logger.info(
                f"路徑節點（含 waypoint）: {nodes} "
                f"(Location {source_location_id} → waypoint {waypoint_node_id} → Location {target_location_id})"
            )
        else:
            # 無 waypoint，直接連接：使用 location_id
            nodes = [source_location_id, target_location_id]
            self.logger.info(
                f"路徑節點（無 waypoint）: {nodes} "
                f"(Location {source_location_id} → Location {target_location_id})"
            )

        return nodes
