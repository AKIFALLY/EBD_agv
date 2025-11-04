"""
CT 任務分配器

根據 YAML 配置文件，實現 work_id → AGV 的智能分配邏輯

功能:
- 加載和解析 YAML 配置
- work_id → AGV 直接映射
- AGV 能力驗證（房間限制、並發任務數等）
- 優先級覆蓋
- 配置熱重載

作者: AI Assistant
創建日期: 2025-10-21
"""

import os
import yaml
from typing import Dict, List, Optional, Tuple
from pathlib import Path


class CtTaskAllocator:
    """CT 任務分配器類"""

    def __init__(self, config_path: str, logger):
        """
        初始化任務分配器

        Args:
            config_path: YAML 配置文件路徑
            logger: ROS logger 實例
        """
        self.config_path = config_path
        self.logger = logger
        self.config = {}
        self.last_mtime = 0.0

        # 加載初始配置
        self.load_config()

    def load_config(self) -> bool:
        """
        加載 YAML 配置文件

        Returns:
            bool: 加載成功返回 True，失敗返回 False
        """
        try:
            # 檢查文件是否存在
            if not os.path.exists(self.config_path):
                self.logger.error(f"配置文件不存在: {self.config_path}")
                return False

            # 讀取 YAML 配置
            with open(self.config_path, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f)

            # 更新文件修改時間
            self.last_mtime = os.path.getmtime(self.config_path)

            # 驗證配置結構
            if not self._validate_config():
                self.logger.error("配置文件格式驗證失敗")
                return False

            self.logger.info(
                f"成功加載配置文件: {self.config_path} "
                f"(版本: {self.config.get('version', 'unknown')})"
            )

            # 記錄配置的 work_id 數量
            allocations = self.config.get('work_id_allocations', {})
            self.logger.info(f"已配置 {len(allocations)} 個 work_id 映射")

            return True

        except yaml.YAMLError as e:
            self.logger.error(f"YAML 解析錯誤: {e}")
            return False
        except Exception as e:
            self.logger.error(f"加載配置文件失敗: {e}")
            return False

    def _validate_config(self) -> bool:
        """
        驗證配置文件結構

        Returns:
            bool: 驗證通過返回 True
        """
        required_keys = ['version', 'enabled', 'agv_capabilities', 'work_id_allocations']

        for key in required_keys:
            if key not in self.config:
                self.logger.error(f"配置文件缺少必要字段: {key}")
                return False

        return True

    def check_and_reload(self) -> bool:
        """
        檢查配置文件是否有變更，如有則重新加載

        Returns:
            bool: 如果發生了重載返回 True
        """
        reload_config = self.config.get('reload_config', {})

        # 檢查是否啟用熱重載
        if not reload_config.get('enabled', True):
            return False

        try:
            current_mtime = os.path.getmtime(self.config_path)

            # 文件有變更
            if current_mtime > self.last_mtime:
                if reload_config.get('log_on_reload', True):
                    self.logger.info(f"檢測到配置文件變更，正在重新加載...")

                success = self.load_config()

                if success and reload_config.get('log_on_reload', True):
                    self.logger.info("配置文件重載成功")

                return success

        except Exception as e:
            self.logger.error(f"檢查配置文件變更時出錯: {e}")

        return False

    def allocate_task(self, task, available_agvs: List) -> Tuple[Optional[str], Optional[int]]:
        """
        為任務分配 AGV

        Args:
            task: Task 模型實例（包含 work_id, room_id, priority 等）
            available_agvs: 可用的 AGV 列表

        Returns:
            Tuple[Optional[str], Optional[int]]: (agv_name, priority_override)
            如果無法分配則返回 (None, None)
        """
        # 檢查配置是否啟用
        if not self.config.get('enabled', True):
            self.logger.debug("任務分配器已禁用")
            return None, None

        work_id = task.work_id
        allocations = self.config.get('work_id_allocations', {})

        # 查找 work_id 映射
        allocation = allocations.get(work_id)

        if allocation is None:
            # 未找到映射，檢查默認分配策略
            return self._handle_unmapped_work_id(work_id, task, available_agvs)

        # 獲取分配的 AGV 名稱
        agv_name = allocation.get('agv_name')

        if not agv_name:
            self.logger.warning(f"work_id {work_id} 的配置缺少 agv_name")
            return None, None

        # 驗證 AGV 是否可用
        if not self._validate_agv_for_task(agv_name, task, available_agvs):
            if self.config.get('debug', {}).get('log_skipped_tasks', True):
                self.logger.debug(
                    f"AGV {agv_name} 當前不可用，跳過任務 {task.id} (work_id={work_id})"
                )
            return None, None

        # 獲取優先級覆蓋
        priority_override = allocation.get('priority_override')

        # 記錄分配決策（使用 DEBUG 級別，避免重複輸出）
        if self.config.get('debug', {}).get('log_allocation_decisions', True):
            self.logger.debug(
                f"任務 {task.id} (work_id={work_id}) 找到候選 AGV {agv_name} "
                f"(優先級: {priority_override if priority_override else task.priority})"
            )

        return agv_name, priority_override

    def _handle_unmapped_work_id(self, work_id, task, available_agvs) -> Tuple[Optional[str], Optional[int]]:
        """
        處理未映射的 work_id

        Args:
            work_id: 工作 ID
            task: 任務實例
            available_agvs: 可用 AGV 列表

        Returns:
            Tuple[Optional[str], Optional[int]]: (agv_name, priority)
        """
        default = self.config.get('default_allocation', {})

        # 記錄未映射的 work_id
        if default.get('log_unmapped', True):
            self.logger.warning(
                f"work_id {work_id} 未在配置文件中定義映射 (任務 {task.id})"
            )

        # 檢查是否啟用默認分配
        if not default.get('enabled', False):
            return None, None

        # 使用默認 AGV
        fallback_agv = default.get('fallback_agv')

        if fallback_agv and self._validate_agv_for_task(fallback_agv, task, available_agvs):
            self.logger.info(f"使用默認 AGV {fallback_agv} 處理未映射的 work_id {work_id}")
            return fallback_agv, None

        return None, None

    def _validate_agv_for_task(self, agv_name: str, task, available_agvs: List) -> bool:
        """
        驗證 AGV 是否可以執行該任務

        檢查:
        1. AGV 是否在 available_agvs 列表中（idle狀態）
        2. AGV 能力配置中是否 enabled
        3. 房間限制是否滿足

        Args:
            agv_name: AGV 名稱
            task: 任務實例
            available_agvs: 可用 AGV 列表

        Returns:
            bool: 驗證通過返回 True
        """
        # 1. 檢查 AGV 是否在可用列表中
        agv_in_list = any(agv.name == agv_name for agv in available_agvs)

        if not agv_in_list:
            return False

        # 2. 檢查 AGV 能力配置
        capabilities = self.config.get('agv_capabilities', {})
        agv_cap = capabilities.get(agv_name)

        if not agv_cap:
            self.logger.warning(f"AGV {agv_name} 未在 agv_capabilities 中定義")
            # 未定義能力，默認允許（向後兼容）
            return True

        # 檢查是否啟用
        if not agv_cap.get('enabled', True):
            return False

        # 3. 檢查房間限制
        allowed_rooms = agv_cap.get('rooms', [])

        # 空列表表示所有房間都允許
        if allowed_rooms and task.room_id not in allowed_rooms:
            if self.config.get('debug', {}).get('verbose_logging', False):
                self.logger.debug(
                    f"AGV {agv_name} 不允許在房間 {task.room_id} 執行任務 "
                    f"(允許的房間: {allowed_rooms})"
                )
            return False

        # 所有驗證通過
        return True

    def get_agv_capability(self, agv_name: str) -> Optional[Dict]:
        """
        獲取 AGV 的能力配置

        Args:
            agv_name: AGV 名稱

        Returns:
            Dict: AGV 能力配置，如果不存在返回 None
        """
        capabilities = self.config.get('agv_capabilities', {})
        return capabilities.get(agv_name)

    def get_all_configured_agvs(self) -> List[str]:
        """
        獲取所有已配置的 AGV 名稱列表

        Returns:
            List[str]: AGV 名稱列表
        """
        capabilities = self.config.get('agv_capabilities', {})
        return list(capabilities.keys())

    def get_work_id_allocation(self, work_id: int) -> Optional[Dict]:
        """
        獲取指定 work_id 的分配配置

        Args:
            work_id: 工作 ID

        Returns:
            Dict: 分配配置，如果不存在返回 None
        """
        allocations = self.config.get('work_id_allocations', {})
        return allocations.get(work_id)

    def get_config_stats(self) -> Dict:
        """
        獲取配置統計信息

        Returns:
            Dict: 包含各種統計信息的字典
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
