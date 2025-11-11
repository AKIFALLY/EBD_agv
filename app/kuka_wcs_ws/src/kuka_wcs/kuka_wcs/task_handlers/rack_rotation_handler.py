"""
旋转处理器 - Rack 旋转逻辑（入口/出口）

根据 A/B 面状态自动旋转 rack 180 度

原 TAFL 流程:
- rack_rotation_room_inlet_aempty_bwork.yaml
- rack_rotation_room_outlet_afull_bempty.yaml
"""
from typing import List
from sqlmodel import Session, select
from db_proxy.models import Task
from .base_handler import BaseHandler


class RackRotationHandler(BaseHandler):
    """旋转处理器"""

    # 任务参数
    WORK_ID = 220001  # KUKA_RACK_MOVE（带旋转）
    PRIORITY = 7

    def check_and_create_tasks(self, session: Session) -> List[Task]:
        """
        检查 rack 是否需要旋转，创建旋转任务

        旋转条件：
        1. 房间入口：A 面空 + B 面有工作 → 旋转 180°
        2. 房间出口：A 面满 + B 面空 → 旋转 180°

        Returns:
            创建的任务列表
        """
        created_tasks = []

        # TODO: 实现旋转逻辑
        # 1. 查询 room_inlet 类型的 locations
        # inlet_locations = self.db.query_locations(session, type="room_inlet")

        # 2. 对每个 inlet，检查是否有 rack
        # 3. 检查 A/B 面状态
        # 4. 判断是否需要旋转
        # 5. 创建旋转任务

        # 示例代码（需要根据实际业务逻辑调整）:
        """
        for inlet_location in inlet_locations:
            rack = self.db.get_rack_at_location(session, inlet_location.id)
            if not rack:
                continue

            # 检查 A 面状态
            a_side_status = self.db.check_rack_side_status(session, rack, side='A')
            # 检查 B 面状态
            b_side_status = self.db.check_rack_side_status(session, rack, side='B')

            # 判断是否需要旋转
            should_rotate = (
                a_side_status['is_empty'] and not b_side_status['is_empty']
            )

            if should_rotate:
                # 创建旋转任务
                task = self.db.create_kuka_task(
                    session=session,
                    work_id=self.WORK_ID,
                    nodes=[
                        inlet_location.node_id,
                        inlet_location.rotation_node_id,  # 旋转节点
                        inlet_location.node_id
                    ],
                    rack_id=rack.id,
                    rotation_angle=180,
                    priority=self.PRIORITY,
                    notes="A面空+B面有工作，旋转180度",
                )
                created_tasks.append(task)
        """

        self.logger.debug(f"{self.get_handler_name()}: 旋转逻辑待实现")

        return created_tasks
