"""
旋转处理器 - Rack 旋转逻辑（入口/出口）

根据 A/B 面状态自动旋转 rack 180 度

原 TAFL 流程:
- rack_rotation_room_inlet_aempty_bwork.yaml
- rack_rotation_room_outlet_afull_bempty.yaml
"""
from typing import List, Dict, Any
from sqlmodel import Session, select
from db_proxy.models import Task, Location
from .base_handler import BaseHandler


class RackRotationHandler(BaseHandler):
    """旋转处理器 - 每个 room 独立配置"""

    def __init__(self, node, room_id: int, config: Dict[str, Any]):
        """
        初始化旋转处理器

        Args:
            node: KukaWcsNode 实例
            room_id: 房间 ID
            config: 配置字典，包含以下键：
                - entrance_node_id: 入口节点 ID
                - exit_node_id: 出口节点 ID
                - rotation_nodes: 旋转路径节点列表 [起点, 旋转点, 终点]
                - priority: 任务优先级
        """
        super().__init__(node)
        self.room_id = room_id
        self.config = config

        # 从配置读取参数
        self.entrance_node_id = config['entrance_node_id']
        self.exit_node_id = config['exit_node_id']
        self.entrance_rotation_nodes = config['entrance_rotation_nodes']  # 入口旋转路径
        self.exit_rotation_nodes = config['exit_rotation_nodes']          # 出口旋转路径
        self.priority = config['priority']

        # 从全局配置读取（由调用者传入）
        self.work_id = config.get('work_id', 220001)  # KUKA_RACK_MOVE
        self.b_side_threshold = config.get('b_side_threshold', 12)  # B面至少12格
        self.a_side_threshold = config.get('a_side_threshold', 0)   # A面最多0格

        self.logger.info(
            f"初始化 RackRotationHandler: room_id={self.room_id}, "
            f"entrance_node={self.entrance_node_id}, exit_node={self.exit_node_id}, "
            f"entrance_rotation={self.entrance_rotation_nodes}, exit_rotation={self.exit_rotation_nodes}"
        )

    def check_and_create_tasks(self, session: Session) -> List[Task]:
        """
        检查 rack 是否需要旋转，创建旋转任务

        旋转条件：
        - B 面有载具（≥12格）且 A 面为空（=0格）

        Returns:
            创建的任务列表
        """
        created_tasks = []

        # 1. 查询入口和出口位置（使用配置的 node_id）
        entrance_locations = self.db.query_locations(session, node_id=self.entrance_node_id)
        exit_locations = self.db.query_locations(session, node_id=self.exit_node_id)

        all_locations = entrance_locations + exit_locations
        self.logger.debug(
            f"Room {self.room_id}: 找到 {len(entrance_locations)} 个入口位置, "
            f"{len(exit_locations)} 个出口位置"
        )

        # 2. 遍历每个位置，检查是否需要旋转
        for location in all_locations:
            task = self._check_location_and_create_task(session, location)
            if task:
                created_tasks.append(task)

        return created_tasks

    def _check_location_and_create_task(self, session: Session, location: Location) -> Task:
        """
        检查单个位置的 rack 是否需要旋转

        Args:
            session: 数据库 session
            location: Location 对象

        Returns:
            创建的任务或 None
        """
        # 查询该位置的 rack
        rack = self.db.get_rack_at_location(session, location.id)
        if not rack:
            return None

        self.logger.debug(f"检查位置 {location.name} (node_id={location.node_id}) 的 Rack {rack.id}")

        # 解析 carrier_bitmap 判断 A/B 面状态
        try:
            bitmap_hex = rack.carrier_bitmap or "00000000"
            # 移除可能的 0x 前缀并转大写
            bitmap_hex = bitmap_hex.replace("0x", "").replace("0X", "").upper()
            # 补齐到 8 位
            bitmap_hex = bitmap_hex.zfill(8)

            # 分离前4位(B面)和后4位(A面)
            # carrier_bitmap 格式: 前4位(高16-bit) = B面, 后4位(低16-bit) = A面
            b_side_hex = bitmap_hex[0:4]  # 前4位16进制 = B面
            a_side_hex = bitmap_hex[4:8]  # 后4位16进制 = A面

        except (ValueError, AttributeError) as e:
            self.logger.warn(
                f"Rack {rack.id} carrier_bitmap 解析失败: {rack.carrier_bitmap}, 错误: {e}"
            )
            return None

        # 判断是否需要旋转（根据位置类型应用不同条件）
        is_entrance = (location.node_id == self.entrance_node_id)
        is_exit = (location.node_id == self.exit_node_id)

        needs_rotation = False
        rotation_reason = ""

        if is_entrance:
            # 入口条件：需要同时检查 carrier_bitmap 和 direction
            # 条件1: B面有货(>0000) 且 A面空(=0000) 且 direction=90
            # 条件2: B面空(=0000) 且 A面有货(>0000) 且 direction=-90
            rack_direction = rack.direction if rack.direction is not None else 0

            # B面有货 = B面不为 0000
            b_side_has_cargo = (b_side_hex != "0000")
            # A面有货 = A面不为 0000
            a_side_has_cargo = (a_side_hex != "0000")

            condition1 = (b_side_has_cargo and not a_side_has_cargo and rack_direction == 90)
            condition2 = (not b_side_has_cargo and a_side_has_cargo and rack_direction == -90)

            needs_rotation = condition1 or condition2

            if condition1:
                rotation_reason = f"入口旋转条件1: B面有货({b_side_hex}), A面空(0000), direction=90°"
            elif condition2:
                rotation_reason = f"入口旋转条件2: B面空(0000), A面有货({a_side_hex}), direction=-90°"
            else:
                rotation_reason = f"入口不满足旋转条件: bitmap={b_side_hex}{a_side_hex}, direction={rack_direction}°"
        elif is_exit:
            # 出口条件：需要同时检查 carrier_bitmap 和 direction
            # 条件1: FFFF0000 且 direction=90
            # 条件2: 0000FFFF 且 direction=-90
            rack_direction = rack.direction if rack.direction is not None else 0

            condition1 = (b_side_hex == "FFFF" and a_side_hex == "0000" and rack_direction == 90)
            condition2 = (b_side_hex == "0000" and a_side_hex == "FFFF" and rack_direction == -90)

            needs_rotation = condition1 or condition2

            if condition1:
                rotation_reason = f"出口旋转条件1: B面满(FFFF), A面空(0000), direction=90°"
            elif condition2:
                rotation_reason = f"出口旋转条件2: B面空(0000), A面满(FFFF), direction=-90°"
            else:
                rotation_reason = f"出口不满足旋转条件: bitmap={b_side_hex}{a_side_hex}, direction={rack_direction}°"

        if not needs_rotation:
            rack_direction = rack.direction if rack.direction is not None else 0
            self.logger.debug(
                f"Rack {rack.id} 在位置 {location.name} 不需要旋转: "
                f"B面={b_side_hex}, A面={a_side_hex}, direction={rack_direction}°"
            )
            return None

        self.logger.info(
            f"✅ Rack {rack.id} 在位置 {location.name} 满足旋转条件: {rotation_reason}"
        )

        # ⚠️ 重要：先確定旋轉路徑（用於防重檢查）
        if is_entrance:
            rotation_path = self.entrance_rotation_nodes
            location_type = "入口"
            task_name = f"room{self.room_id}入口旋轉貨架"
        elif is_exit:
            rotation_path = self.exit_rotation_nodes
            location_type = "出口"
            task_name = f"room{self.room_id}出口旋轉貨架"
        else:
            # 理论上不应该到达这里
            self.logger.error(f"位置 {location.name} 既不是入口也不是出口")
            return None

        # 检查是否已存在任务（防重复）- 與 SystemAreaHandler 相同的防重機制
        # ⚠️ 关键：檢查 PENDING/READY_TO_EXECUTE/EXECUTING/ERROR 的任务，並比對路徑是否相同
        # 状态码说明：1=PENDING, 2=READY_TO_EXECUTE, 3=EXECUTING, 6=ERROR
        # ✅ 修复：包含 ERROR 状态防止重复创建失败任务
        import json

        existing_tasks = session.exec(
            select(Task).where(
                Task.location_id == location.id,   # 使用 location_id 防重複
                Task.work_id == self.work_id,       # 同一种工作类型
                Task.status_id.in_([1, 2, 3, 6])    # 包含 ERROR (6) 防止任务失败后重复创建
            )
        ).all()

        # 如果有任務，進一步檢查路徑是否相同
        if existing_tasks:
            for task in existing_tasks:
                try:
                    # 解析 parameters
                    if isinstance(task.parameters, str):
                        params = json.loads(task.parameters)
                    else:
                        params = task.parameters

                    # 檢查 nodes 是否相同
                    task_nodes = params.get('nodes', [])
                    if task_nodes == rotation_path:
                        self.logger.info(
                            f"⏭️  位置 {location.name} (location_id={location.id}) 已有相同路徑的未完成任務 "
                            f"(Task {task.id}, Status {task.status_id}, nodes={rotation_path})，跳過旋轉"
                        )
                        return None
                except (json.JSONDecodeError, TypeError) as e:
                    self.logger.warn(f"解析 Task {task.id} parameters 失敗: {e}")
                    continue

            # 有任務但路徑不同，可以創建
            self.logger.debug(
                f"防重檢查: Location {location.id} 有 {len(existing_tasks)} 個任務但路徑不同，允許創建"
            )

        # 创建旋转任务
        rotation_description = (
            f"Rack旋转: B面(前4位)={b_side_hex}, A面(后4位)={a_side_hex}, {rotation_reason}"
        )

        task = self.db.create_kuka_task(
            session=session,
            work_id=self.work_id,
            nodes=rotation_path,  # 动态生成的路径
            rack_id=rack.id,
            room_id=self.room_id,  # ⚠️ 明確設置 room_id = 2
            location_id=location.id,  # ⚠️ 使用 location_id 進行防重複檢查
            rotation_angle=180,
            priority=self.priority,  # 从配置读取
            name=task_name,  # 自定義任務名稱
            notes=rotation_description,
            parameters={
                "model": "KUKA400i",
                #"nodes":self.rotation_nodes,
                "rack_name": rack.name,
                "rack_id": rack.id,
                "location_name": location.name,
                "location_id": location.id,
                "node_id": location.node_id,  # 放在 parameters 供參考
                "b_side_hex": b_side_hex,
                "a_side_hex": a_side_hex,
                "rack_direction": rack.direction,  # 记录 rack 方向
                "rotation_reason": rotation_reason,
            }
        )

        self.logger.info(
            f"✅ 创建旋转任务: Rack {rack.id} 在{location_type} {location.name} (node_id={location.node_id}), "
            f"路径: {rotation_path}"
        )

        return task

    def check_and_mark_completed_tasks(self, session: Session) -> int:
        """
        检查并标记已完成的旋转任务

        完成条件（分别独立检查）：
        1. 入口(27): rack 空载 (carrier_bitmap=00000000)
        2. 出口(26): rack 满载 (carrier_bitmap=FFFFFFFF)

        Returns:
            标记为完成的任务数量
        """
        marked_count = 0

        # 检查条件配置
        completion_checks = [
            {
                'node_id': self.entrance_node_id,  # 27
                'expected_bitmap': '00000000',
                'description': '入口空载'
            },
            {
                'node_id': self.exit_node_id,      # 26
                'expected_bitmap': 'FFFFFFFF',
                'description': '出口满载'
            }
        ]

        for check in completion_checks:
            node_id = check['node_id']
            expected_bitmap = check['expected_bitmap']
            description = check['description']

            # 1. 查询该节点的 locations
            locations = self.db.query_locations(
                session,
                node_id=node_id
            )

            for location in locations:
                # 2. 获取该位置的 rack
                rack = self.db.get_rack_at_location(session, location.id)
                if not rack:
                    continue

                # 3. 检查 carrier_bitmap 是否匹配预期值
                bitmap_hex = (rack.carrier_bitmap or "00000000").replace("0x", "").replace("0X", "").upper().zfill(8)

                if bitmap_hex != expected_bitmap:
                    self.logger.debug(
                        f"位置 {location.name} (location_id={location.id}) bitmap不匹配: "
                        f"当前={bitmap_hex}, 预期={expected_bitmap}"
                    )
                    continue

                self.logger.debug(
                    f"检测到完成条件: Location {location.name} "
                    f"(location_id={location.id}), Rack {rack.id}, "
                    f"bitmap={bitmap_hex}, 条件={description}"
                )

                # 4. 查询该位置的执行中旋转任务
                executing_tasks = session.exec(
                    select(Task).where(
                        Task.location_id == location.id,
                        Task.work_id == self.work_id,        # 220001
                        Task.rack_id == rack.id,             # 验证 rack_id
                        Task.status_id.in_([2, 3])           # READY_TO_EXECUTE, EXECUTING
                    )
                ).all()

                if not executing_tasks:
                    self.logger.debug(
                        f"位置 {location.name} 没有执行中的旋转任务 "
                        f"(work_id={self.work_id}, rack_id={rack.id})"
                    )
                    continue

                # 5. 标记任务为完成
                for task in executing_tasks:
                    success = self._mark_task_completed(
                        session=session,
                        task=task,
                        reason=f"{description}: bitmap={bitmap_hex}"
                    )
                    if success:
                        marked_count += 1

        return marked_count
