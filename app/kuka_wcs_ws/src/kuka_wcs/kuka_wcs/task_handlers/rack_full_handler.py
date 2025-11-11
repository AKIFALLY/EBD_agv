"""
满载搬运处理器 - 完成料架出口到人工收料区

当房间出口 Rack 满载时，搬运到人工收料区供人工取出产品

原 TAFL 流程: full_rack_outlet_to_manual_collection.yaml
简化版本：仅检查满载，使用 carrier_bitmap 判断
"""
from typing import List
from sqlmodel import Session, select
from db_proxy.models import Task, Location
from .base_handler import BaseHandler


class RackFullHandler(BaseHandler):
    """满载搬运处理器"""

    # 任务参数
    WORK_ID = 220001  # KUKA_RACK_MOVE
    PRIORITY = 8

    def check_and_create_tasks(self, session: Session) -> List[Task]:
        """
        检查房间出口 rack 是否满载或尾批，创建搬运任务

        Returns:
            创建的任务列表
        """
        created_tasks = []

        # 1. 查询所有房间出口位置
        outlet_locations = self.db.query_locations(session, type="room_outlet")
        self.logger.debug(f"找到 {len(outlet_locations)} 个房间出口位置")

        # 2. 遍历每个出口位置
        for outlet_location in outlet_locations:
            task = self._check_outlet_and_create_task(session, outlet_location)
            if task:
                created_tasks.append(task)

        return created_tasks

    def _check_outlet_and_create_task(self, session: Session, outlet_location: Location) -> Task:
        """
        检查单个出口位置并创建任务（如果需要）

        Args:
            session: 数据库 session
            outlet_location: 出口 location 对象

        Returns:
            创建的任务或 None
        """
        # 查询出口位置的 rack
        rack = self.db.get_rack_at_location(session, outlet_location.id)
        if not rack:
            return None

        self.logger.debug(f"出口 {outlet_location.name} 有 Rack {rack.id}")

        # 获取产品信息
        product = self.db.get_product_by_id(session, rack.product_id)
        if not product:
            self.logger.warn(f"Rack {rack.id} 的 product_id {rack.product_id} 不存在")
            return None

        # 根据产品尺寸确定容量
        # S 尺寸: 每面 16, 总共 32
        # L 尺寸: 每面 8, 总共 16
        if product.dim == 'S':
            max_total = 32
        else:  # 'L'
            max_total = 16

        # 解析 carrier_bitmap (8位16进制字符串，例如 "0001FFFF")
        # 低 16-bit (bit 0~15) = A面
        # 高 16-bit (bit 16~31) = B面
        try:
            bitmap_hex = rack.carrier_bitmap or "00000000"
            # 移除可能的 0x 前缀并转大写
            bitmap_hex = bitmap_hex.replace("0x", "").replace("0X", "").upper()
            # 补齐到 8 位
            bitmap_hex = bitmap_hex.zfill(8)
            # 转换为 32-bit 整数
            full_value = int(bitmap_hex, 16)

            # 分离 A 面和 B 面
            a_side = full_value & 0xFFFF         # 低 16-bit = A面 (索引 0~15)
            b_side = (full_value >> 16) & 0xFFFF  # 高 16-bit = B面 (索引 16~31)

            # 计算每面的载具数量 (bit count)
            a_side_count = bin(a_side).count('1')
            b_side_count = bin(b_side).count('1')
            total_count = a_side_count + b_side_count

        except (ValueError, AttributeError) as e:
            self.logger.warn(f"Rack {rack.id} carrier_bitmap 解析失败: {rack.carrier_bitmap}, 错误: {e}")
            a_side_count = 0
            b_side_count = 0
            total_count = 0

        # 判断是否满载
        is_full = (total_count >= max_total)

        # TODO: 实现尾批判断逻辑（房间内无可放载具时）
        is_last_batch = False  # 暂时设为 False，后续可扩展

        if not is_full:
            self.logger.debug(
                f"Rack {rack.id} 未满载: "
                f"total={total_count}/{max_total}, A面={a_side_count}, B面={b_side_count}"
            )
            return None

        # 满载时创建搬运任务
        move_reason = "Rack满载，制程完成送人工收料"
        move_description = f"{product.dim}尺寸 {total_count}/{max_total}个载具，满载送人工收料"

        self.logger.info(f"Rack {rack.id} 满足搬运条件: {move_reason}")

        # 查询人工收料区空闲位置
        free_location = self._find_free_collection_location(session)
        if not free_location:
            self.logger.warn("人工收料区无空闲位置，跳过创建任务")
            return None

        # 检查是否已存在未完成的搬运任务（避免重复）
        existing_tasks = session.exec(
            select(Task).where(
                Task.work_id == self.WORK_ID,
                Task.rack_id == rack.id,
                Task.status_id.in_([0, 1, 2, 3])  # 未完成的状态
            )
        ).all()

        if existing_tasks:
            self.logger.debug(f"Rack {rack.id} 已存在未完成的搬运任务，跳过")
            return None

        # 创建搬运任务
        task = self.db.create_kuka_task(
            session=session,
            work_id=self.WORK_ID,
            nodes=[outlet_location.node_id, free_location.node_id],
            rack_id=rack.id,
            priority=self.PRIORITY,
            notes=move_description,
            parameters={
                "rack_name": rack.name,
                "rack_id": rack.id,
                "from_location_name": outlet_location.name,
                "from_location_id": outlet_location.id,
                "to_location_name": free_location.name,
                "to_location_id": free_location.id,
                "room_id": outlet_location.room_id,
                "product_size": product.dim,
                "carrier_count": total_count,
                "max_capacity": max_total,
                "reason": move_reason,
                "is_full": is_full,
                "is_last_batch": is_last_batch,
                "a_side_count": a_side_count,
                "b_side_count": b_side_count,
            }
        )

        self.logger.info(
            f"✅ 创建满载搬运任务: Rack {rack.id} "
            f"从 {outlet_location.name} 到 {free_location.name}"
        )

        return task

    def _find_free_collection_location(self, session: Session) -> Location:
        """
        查找人工收料区空闲位置

        从数据库中查询所有名称以 "ManualReceiveArea_" 开头的 location，
        并返回第一个未占用的位置

        Args:
            session: 数据库 session

        Returns:
            空闲的 Location 对象或 None
        """
        # 查询所有人工收料区位置（名称以 "ManualReceiveArea_" 开头）
        locations = session.exec(
            select(Location).where(
                Location.name.like('ManualReceiveArea_%'),
                Location.location_status_id == 2  # 2 = 未占用
            ).order_by(Location.id)  # 按 ID 排序，优先使用较小的 ID
        ).all()

        if locations:
            self.logger.debug(f"找到 {len(locations)} 个空闲的人工收料区位置")
            return locations[0]  # 返回第一个空闲位置
        else:
            self.logger.warn("未找到空闲的人工收料区位置")
            return None
