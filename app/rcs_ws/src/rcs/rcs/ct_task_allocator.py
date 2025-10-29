"""
CT 任务分配器

根据 YAML 配置文件，实现 work_id → AGV 的智能分配逻辑

功能:
- 加载和解析 YAML 配置
- work_id → AGV 直接映射
- AGV 能力验证（房间限制、并发任务数等）
- 优先级覆盖
- 配置热重载

作者: AI Assistant
创建日期: 2025-10-21
"""

import os
import yaml
from typing import Dict, List, Optional, Tuple
from pathlib import Path


class CtTaskAllocator:
    """CT 任务分配器类"""

    def __init__(self, config_path: str, logger):
        """
        初始化任务分配器

        Args:
            config_path: YAML 配置文件路径
            logger: ROS logger 实例
        """
        self.config_path = config_path
        self.logger = logger
        self.config = {}
        self.last_mtime = 0.0

        # 加载初始配置
        self.load_config()

    def load_config(self) -> bool:
        """
        加载 YAML 配置文件

        Returns:
            bool: 加载成功返回 True，失败返回 False
        """
        try:
            # 检查文件是否存在
            if not os.path.exists(self.config_path):
                self.logger.error(f"配置文件不存在: {self.config_path}")
                return False

            # 读取 YAML 配置
            with open(self.config_path, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f)

            # 更新文件修改时间
            self.last_mtime = os.path.getmtime(self.config_path)

            # 验证配置结构
            if not self._validate_config():
                self.logger.error("配置文件格式验证失败")
                return False

            self.logger.info(
                f"成功加载配置文件: {self.config_path} "
                f"(版本: {self.config.get('version', 'unknown')})"
            )

            # 记录配置的 work_id 数量
            allocations = self.config.get('work_id_allocations', {})
            self.logger.info(f"已配置 {len(allocations)} 个 work_id 映射")

            return True

        except yaml.YAMLError as e:
            self.logger.error(f"YAML 解析错误: {e}")
            return False
        except Exception as e:
            self.logger.error(f"加载配置文件失败: {e}")
            return False

    def _validate_config(self) -> bool:
        """
        验证配置文件结构

        Returns:
            bool: 验证通过返回 True
        """
        required_keys = ['version', 'enabled', 'agv_capabilities', 'work_id_allocations']

        for key in required_keys:
            if key not in self.config:
                self.logger.error(f"配置文件缺少必要字段: {key}")
                return False

        return True

    def check_and_reload(self) -> bool:
        """
        检查配置文件是否有变更，如有则重新加载

        Returns:
            bool: 如果发生了重载返回 True
        """
        reload_config = self.config.get('reload_config', {})

        # 检查是否启用热重载
        if not reload_config.get('enabled', True):
            return False

        try:
            current_mtime = os.path.getmtime(self.config_path)

            # 文件有变更
            if current_mtime > self.last_mtime:
                if reload_config.get('log_on_reload', True):
                    self.logger.info(f"检测到配置文件变更，正在重新加载...")

                success = self.load_config()

                if success and reload_config.get('log_on_reload', True):
                    self.logger.info("配置文件重载成功")

                return success

        except Exception as e:
            self.logger.error(f"检查配置文件变更时出错: {e}")

        return False

    def allocate_task(self, task, available_agvs: List) -> Tuple[Optional[str], Optional[int]]:
        """
        为任务分配 AGV

        Args:
            task: Task 模型实例（包含 work_id, room_id, priority 等）
            available_agvs: 可用的 AGV 列表

        Returns:
            Tuple[Optional[str], Optional[int]]: (agv_name, priority_override)
            如果无法分配则返回 (None, None)
        """
        # 检查配置是否启用
        if not self.config.get('enabled', True):
            self.logger.debug("任务分配器已禁用")
            return None, None

        work_id = task.work_id
        allocations = self.config.get('work_id_allocations', {})

        # 查找 work_id 映射
        allocation = allocations.get(work_id)

        if allocation is None:
            # 未找到映射，检查默认分配策略
            return self._handle_unmapped_work_id(work_id, task, available_agvs)

        # 获取分配的 AGV 名称
        agv_name = allocation.get('agv_name')

        if not agv_name:
            self.logger.warning(f"work_id {work_id} 的配置缺少 agv_name")
            return None, None

        # 验证 AGV 是否可用
        if not self._validate_agv_for_task(agv_name, task, available_agvs):
            if self.config.get('debug', {}).get('log_skipped_tasks', True):
                self.logger.debug(
                    f"AGV {agv_name} 当前不可用，跳过任务 {task.id} (work_id={work_id})"
                )
            return None, None

        # 获取优先级覆盖
        priority_override = allocation.get('priority_override')

        # 记录分配决策（使用 DEBUG 級別，避免重複輸出）
        if self.config.get('debug', {}).get('log_allocation_decisions', True):
            self.logger.debug(
                f"任务 {task.id} (work_id={work_id}) 找到候選 AGV {agv_name} "
                f"(优先级: {priority_override if priority_override else task.priority})"
            )

        return agv_name, priority_override

    def _handle_unmapped_work_id(self, work_id, task, available_agvs) -> Tuple[Optional[str], Optional[int]]:
        """
        处理未映射的 work_id

        Args:
            work_id: 工作 ID
            task: 任务实例
            available_agvs: 可用 AGV 列表

        Returns:
            Tuple[Optional[str], Optional[int]]: (agv_name, priority)
        """
        default = self.config.get('default_allocation', {})

        # 记录未映射的 work_id
        if default.get('log_unmapped', True):
            self.logger.warning(
                f"work_id {work_id} 未在配置文件中定义映射 (任务 {task.id})"
            )

        # 检查是否启用默认分配
        if not default.get('enabled', False):
            return None, None

        # 使用默认 AGV
        fallback_agv = default.get('fallback_agv')

        if fallback_agv and self._validate_agv_for_task(fallback_agv, task, available_agvs):
            self.logger.info(f"使用默认 AGV {fallback_agv} 处理未映射的 work_id {work_id}")
            return fallback_agv, None

        return None, None

    def _validate_agv_for_task(self, agv_name: str, task, available_agvs: List) -> bool:
        """
        验证 AGV 是否可以执行该任务

        检查:
        1. AGV 是否在 available_agvs 列表中（idle状态）
        2. AGV 能力配置中是否 enabled
        3. 房间限制是否满足

        Args:
            agv_name: AGV 名称
            task: 任务实例
            available_agvs: 可用 AGV 列表

        Returns:
            bool: 验证通过返回 True
        """
        # 1. 检查 AGV 是否在可用列表中
        agv_in_list = any(agv.name == agv_name for agv in available_agvs)

        if not agv_in_list:
            return False

        # 2. 检查 AGV 能力配置
        capabilities = self.config.get('agv_capabilities', {})
        agv_cap = capabilities.get(agv_name)

        if not agv_cap:
            self.logger.warning(f"AGV {agv_name} 未在 agv_capabilities 中定义")
            # 未定义能力，默认允许（向后兼容）
            return True

        # 检查是否启用
        if not agv_cap.get('enabled', True):
            return False

        # 3. 检查房间限制
        allowed_rooms = agv_cap.get('rooms', [])

        # 空列表表示所有房间都允许
        if allowed_rooms and task.room_id not in allowed_rooms:
            if self.config.get('debug', {}).get('verbose_logging', False):
                self.logger.debug(
                    f"AGV {agv_name} 不允许在房间 {task.room_id} 执行任务 "
                    f"(允许的房间: {allowed_rooms})"
                )
            return False

        # 所有验证通过
        return True

    def get_agv_capability(self, agv_name: str) -> Optional[Dict]:
        """
        获取 AGV 的能力配置

        Args:
            agv_name: AGV 名称

        Returns:
            Dict: AGV 能力配置，如果不存在返回 None
        """
        capabilities = self.config.get('agv_capabilities', {})
        return capabilities.get(agv_name)

    def get_all_configured_agvs(self) -> List[str]:
        """
        获取所有已配置的 AGV 名称列表

        Returns:
            List[str]: AGV 名称列表
        """
        capabilities = self.config.get('agv_capabilities', {})
        return list(capabilities.keys())

    def get_work_id_allocation(self, work_id: int) -> Optional[Dict]:
        """
        获取指定 work_id 的分配配置

        Args:
            work_id: 工作 ID

        Returns:
            Dict: 分配配置，如果不存在返回 None
        """
        allocations = self.config.get('work_id_allocations', {})
        return allocations.get(work_id)

    def get_config_stats(self) -> Dict:
        """
        获取配置统计信息

        Returns:
            Dict: 包含各种统计信息的字典
        """
        return {
            'version': self.config.get('version', 'unknown'),
            'enabled': self.config.get('enabled', False),
            'total_work_id_mappings': len(self.config.get('work_id_allocations', {})),
            'total_agvs_configured': len(self.config.get('agv_capabilities', {})),
            'hot_reload_enabled': self.config.get('reload_config', {}).get('enabled', False),
            'config_path': self.config_path,
            'last_modified': self.last_mtime
        }
