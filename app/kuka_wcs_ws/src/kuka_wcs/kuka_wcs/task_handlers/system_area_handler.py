"""
系統區域監控處理器 - 監控各區域站點狀態

監控以下區域的站點狀態：
- 系統準備區
- 房間入口/出口
- 空架回收區
- 射出機作業區

狀態編碼：
- 0: 無架（empty）
- 2: 有任務佔用（task_occupied）
- 5: 空架（empty_rack，carrier_bitmap = 00000000）
- 6: 滿架（full_rack，carrier_bitmap = FFFFFFFF）
- 7: 部分載貨（partial_rack，非 00000000 也非 FFFFFFFF）
"""
from typing import List, Dict, Any
from sqlmodel import Session, select
from db_proxy.models import Task, Rack, Location
from .base_handler import BaseHandler
import time


class SystemAreaHandler(BaseHandler):
    """系統區域監控處理器"""

    def __init__(self, node, config: Dict[str, Any]):
        """
        初始化系統區域監控處理器

        Args:
            node: KukaWcsNode 實例
            config: 配置字典，包含以下鍵：
                - enabled: 是否啟用
                - scan_interval: 掃描間隔（秒）
                - areas: 區域定義字典
        """
        super().__init__(node)
        self.config = config
        self.scan_interval = config.get('scan_interval', 5.0)
        self.areas = config.get('areas', {})

        # 最後掃描時間（用於控制掃描頻率）
        self.last_scan_time = 0.0

        # 最新的區域狀態數據（供其他方法使用）
        self.latest_area_statuses = {}

        # 任務判斷結果（供 create_task 使用）
        self.pending_task_decisions = []

        # 收集所有需要監控的 location_ids
        self.all_location_ids = []
        for area_name, area_config in self.areas.items():
            location_ids = area_config.get('location_ids', [])
            self.all_location_ids.extend(location_ids)

        self.logger.info(
            f"✅ 初始化 SystemAreaHandler: "
            f"監控 {len(self.areas)} 個區域，共 {len(self.all_location_ids)} 個站點"
        )

        # 印出各區域詳細資訊
        for area_name, area_config in self.areas.items():
            location_ids = area_config.get('location_ids', [])
            description = area_config.get('description', area_name)
            self.logger.info(
                f"   - {description} ({area_name}): {len(location_ids)} 個站點 {location_ids}"
            )

    def check_and_create_tasks(self, session: Session) -> List[Task]:
        """
        檢查系統區域狀態並執行任務判斷

        每 scan_interval 秒執行一次完整掃描，包括：
        1. 掃描所有區域站點狀態
        2. 執行4個任務流程判斷
        3. 保存判斷結果到實例變數

        Args:
            session: 資料庫 session

        Returns:
            創建的任務列表（目前為空列表，待實作 create_task）
        """
        # 檢查是否到達掃描時間
        current_time = time.time()
        if current_time - self.last_scan_time < self.scan_interval:
            return []

        self.last_scan_time = current_time

        # 執行完整掃描
        area_statuses = self._scan_all_areas(session)

        # 保存到實例變數（供其他方法使用）
        self.latest_area_statuses = area_statuses

        # 印出掃描結果
        self._print_scan_results(area_statuses)

        # 執行任務流程判斷
        task_decisions = self._run_all_task_flow_checks()

        # 保存到實例變數（供 create_task 使用）
        self.pending_task_decisions = task_decisions

        # 印出任務判斷結果
        self._print_task_decisions(task_decisions)

        # 創建任務
        return self._create_all_tasks(session)

    def check_and_mark_completed_tasks(self, session: Session) -> int:
        """
        檢查並標記已完成的任務（目前不實作）

        Returns:
            標記為完成的任務數量（目前為 0）
        """
        return 0

    def _scan_all_areas(self, session: Session) -> Dict[str, Dict[int, Dict[str, Any]]]:
        """
        掃描所有區域的站點狀態

        Args:
            session: 資料庫 session

        Returns:
            區域狀態字典 {area_name: {location_id: {status, rack_id, ...}}}
        """
        area_statuses = {}

        for area_name, area_config in self.areas.items():
            location_ids = area_config.get('location_ids', [])
            area_statuses[area_name] = {}

            for location_id in location_ids:
                status_info = self._check_location_status(session, location_id)
                area_statuses[area_name][location_id] = status_info

        return area_statuses

    def _check_location_status(self, session: Session, location_id: int) -> Dict[str, Any]:
        """
        檢查單個 location 的狀態

        業務邏輯：
        1. 查詢該 location 是否有 rack（查 rack.location_id）
           - 如果沒有：result = 0
           - 如果有：取得 rack_id，繼續到步驟 2
        2. 檢查 rack.is_carry
           - 如果 is_carry = 1：result = 0（視為無架）
           - 如果 is_carry != 1：繼續到步驟 3
        3. 查詢 rack 的 carrier_bitmap
           - FFFFFFFF → result = 6（滿架）
           - 00000000 → result = 5（空架）
           - 其他 → result = 7（部分載貨）
        4. 查詢是否有任務使用該 rack（查 task.rack_id，且 status_id 不為完成）
           - 如果有 → result = 2（有任務佔用）

        Args:
            session: 資料庫 session
            location_id: location ID

        Returns:
            {
                'status': int,          # 狀態編碼 (0/2/5/6/7)
                'rack_id': int or None, # rack ID（如果有）
                'carrier_bitmap': str,  # carrier_bitmap（如果有）
                'task_count': int,      # 相關任務數量
            }
        """
        result = {
            'status': 0,
            'rack_id': None,
            'carrier_bitmap': None,
            'task_count': 0,
        }

        # 步驟 1: 查詢該 location 是否有 rack
        rack = self.db.get_rack_at_location(session, location_id)
        if not rack:
            # 無架
            result['status'] = 0
            return result

        # 有架，記錄 rack_id 和 carrier_bitmap
        result['rack_id'] = rack.id
        result['carrier_bitmap'] = rack.carrier_bitmap

        # 步驟 2: 檢查 rack.is_carry（新增邏輯）
        if rack.is_carry == 1:
            # is_carry = 1，視為無架（正在搬運中）
            result['status'] = 0
            return result

        # 步驟 3: 查詢 carrier_bitmap 狀態
        if rack.carrier_bitmap == "FFFFFFFF":
            # 滿架
            result['status'] = 6
        elif rack.carrier_bitmap == "00000000":
            # 空架
            result['status'] = 5
        else:
            # 部分載貨
            result['status'] = 7

        # 步驟 4: 查詢是否有任務使用該 rack（status_id 不為 4 COMPLETED）
        statement = select(Task).where(
            Task.rack_id == rack.id,
            Task.status_id != 4  # 不包括已完成的任務
        )
        tasks = session.exec(statement).all()
        result['task_count'] = len(tasks)

        if tasks:
            # 有任務佔用，覆蓋之前的狀態
            result['status'] = 2

        return result

    def _print_scan_results(self, area_statuses: Dict[str, Dict[int, Dict[str, Any]]]):
        """
        印出掃描結果（使用 logger.info）- 簡化版，僅顯示統計摘要

        Args:
            area_statuses: 區域狀態字典
        """
        # 狀態編碼對應文字
        status_text = {
            0: "無架",
            2: "任務佔用",
            5: "空架",
            6: "滿架",
            7: "部分載貨",
        }

        # 只記錄簡要統計，不顯示每個站點的詳細資訊
        for area_name, location_statuses in area_statuses.items():
            # 統計各狀態數量
            status_counts = {0: 0, 2: 0, 5: 0, 6: 0, 7: 0}
            for location_id, status_info in location_statuses.items():
                status = status_info['status']
                status_counts[status] = status_counts.get(status, 0) + 1

        # 註釋掉詳細輸出，減少日誌量
        # self.logger.debug(f"掃描完成: 監控 {len(area_statuses)} 個區域")

    # ========== 數據訪問輔助方法（供任務判斷使用） ==========

    def get_area_status(self, area_name: str) -> Dict[int, Dict[str, Any]]:
        """
        獲取指定區域的所有站點狀態

        Args:
            area_name: 區域名稱 (如 'system_prepare', 'room_entrance')

        Returns:
            該區域的站點狀態字典 {location_id: {status, rack_id, ...}}
            如果區域不存在，返回空字典
        """
        return self.latest_area_statuses.get(area_name, {})

    def get_location_status(self, location_id: int) -> Dict[str, Any]:
        """
        獲取指定站點的狀態

        Args:
            location_id: 站點 ID

        Returns:
            站點狀態字典 {status, rack_id, carrier_bitmap, task_count}
            如果站點不存在，返回 None
        """
        for area_name, locations in self.latest_area_statuses.items():
            if location_id in locations:
                return locations[location_id]
        return None

    def get_locations_by_status(self, area_name: str, status: int) -> List[int]:
        """
        獲取指定區域中符合特定狀態的所有站點 ID

        Args:
            area_name: 區域名稱
            status: 狀態編碼 (0=無架, 2=任務佔用, 5=空架, 6=滿架, 7=部分載貨)

        Returns:
            符合條件的 location_id 列表
        """
        area_data = self.latest_area_statuses.get(area_name, {})
        return [
            location_id
            for location_id, status_info in area_data.items()
            if status_info['status'] == status
        ]

    def count_by_status(self, area_name: str) -> Dict[int, int]:
        """
        統計指定區域各狀態的站點數量

        Args:
            area_name: 區域名稱

        Returns:
            狀態統計字典 {status: count}
        """
        area_data = self.latest_area_statuses.get(area_name, {})
        status_counts = {0: 0, 2: 0, 5: 0, 6: 0, 7: 0}

        for status_info in area_data.values():
            status = status_info['status']
            status_counts[status] = status_counts.get(status, 0) + 1

        return status_counts

    def has_available_location(self, area_name: str) -> bool:
        """
        檢查指定區域是否有可用站點（狀態為 0 無架）

        Args:
            area_name: 區域名稱

        Returns:
            True 如果有可用站點，否則 False
        """
        empty_locations = self.get_locations_by_status(area_name, 0)
        return len(empty_locations) > 0

    def get_full_racks_in_area(self, area_name: str) -> List[Dict[str, Any]]:
        """
        獲取指定區域中所有滿架的信息

        Args:
            area_name: 區域名稱

        Returns:
            滿架信息列表 [{location_id, rack_id, carrier_bitmap}, ...]
        """
        area_data = self.latest_area_statuses.get(area_name, {})
        full_racks = []

        for location_id, status_info in area_data.items():
            if status_info['status'] == 6:  # 滿架
                full_racks.append({
                    'location_id': location_id,
                    'rack_id': status_info['rack_id'],
                    'carrier_bitmap': status_info['carrier_bitmap']
                })

        return full_racks

    # ========== 任務判斷示例方法 ==========

    def _example_task_decision(self, session: Session) -> List[Task]:
        """
        任務判斷示例方法（僅供參考，尚未啟用）

        展示如何使用輔助方法進行任務判斷

        Returns:
            創建的任務列表
        """
        created_tasks = []

        # 示例 1: 檢查系統準備區是否有滿架需要搬運
        full_racks = self.get_full_racks_in_area('system_prepare')
        if full_racks:
            self.logger.info(f"系統準備區有 {len(full_racks)} 個滿架待搬運")
            # TODO: 創建搬運任務

        # 示例 2: 檢查空架回收區是否有可用空間
        if self.has_available_location('empty_rack_recycle'):
            empty_locations = self.get_locations_by_status('empty_rack_recycle', 0)
            self.logger.info(f"空架回收區有 {len(empty_locations)} 個可用位置: {empty_locations}")
            # TODO: 可以接收空架

        # 示例 3: 統計各區域狀態
        for area_name in ['system_prepare', 'room_entrance', 'room_exit']:
            counts = self.count_by_status(area_name)
            # 判斷邏輯...
            pass

        # 示例 4: 檢查特定站點狀態
        location_26_status = self.get_location_status(26)
        if location_26_status and location_26_status['status'] == 6:
            # 房間入口有滿架
            rack_id = location_26_status['rack_id']
            self.logger.info(f"房間入口(26)有滿架 Rack ID={rack_id}")
            # TODO: 創建旋轉或搬運任務

        return created_tasks

    # ========== 防重機制 ==========

    def _has_pending_task_to_location(self, location_id: int, nodes: List[int] = None) -> bool:
        """
        檢查指定 location 是否已有未完成任務

        防重機制：
        1. 檢查 Task.location_id 是否已有 PENDING/RUNNING/PAUSED 任務
        2. 如果提供 nodes，進一步檢查 parameters["nodes"] 是否完全相同

        Args:
            location_id: 要檢查的 location ID
            nodes: 路徑節點列表（可選），如提供則檢查是否有相同路徑的任務

        Returns:
            True: 已有未完成任務（且路徑相同），不應創建新任務
            False: 沒有未完成任務（或路徑不同），可以創建新任務
        """
        from db_proxy.models import Task
        from sqlmodel import select
        import json

        with self.node.db_pool.get_session() as session:
            statement = select(Task).where(
                Task.location_id == location_id,
                Task.status_id.in_([1, 2, 3])  # 1=PENDING, 2=RUNNING, 3=PAUSED
            )
            existing_tasks = session.exec(statement).all()

            if not existing_tasks:
                return False

            # 如果沒有提供 nodes，只要有任務就算重複
            if nodes is None:
                self.logger.debug(
                    f"防重檢查: Location {location_id} 已有 {len(existing_tasks)} 個未完成任務，跳過創建"
                )
                return True

            # 檢查是否有相同 nodes 的任務
            for task in existing_tasks:
                try:
                    # 解析 parameters
                    if isinstance(task.parameters, str):
                        params = json.loads(task.parameters)
                    else:
                        params = task.parameters

                    # 檢查 nodes 是否相同
                    task_nodes = params.get('nodes', [])
                    if task_nodes == nodes:
                        self.logger.debug(
                            f"防重檢查: Location {location_id} 已有相同路徑的任務 (Task ID={task.id}, nodes={nodes})，跳過創建"
                        )
                        return True
                except (json.JSONDecodeError, TypeError) as e:
                    self.logger.warn(f"解析 Task {task.id} parameters 失敗: {e}")
                    continue

            # 有任務但路徑不同，可以創建
            self.logger.debug(
                f"防重檢查: Location {location_id} 有 {len(existing_tasks)} 個任務但路徑不同，允許創建"
            )
            return False

    # ========== 實際任務判斷流程方法 ==========

    def _check_task_flow_1_room_exit_needs_empty_rack(self) -> dict:
        """
        流程1: 房間出口需要空料架

        判斷邏輯：
        - 要料: 房間出口(status=0)
        - 出料: a.房間入口(status=5) 若無 -> b.空架回收區(status=5)

        Returns:
            判斷結果字典或 None
            {
                'flow_name': '房間出口需要空料架',
                'target_location': int,  # 要料位置
                'source_location': int,  # 出料位置
                'source_area': str       # 出料區域名稱
            }
        """
        # 要料: 房間出口(status=0)
        target_locations = self.get_locations_by_status('room_exit', 0)
        if not target_locations:
            return None

        target_location = target_locations[0]  # 擇一位置

        # 防重檢查: 該位置是否已有未完成任務
        if self._has_pending_task_to_location(target_location):
            return None

        # 出料: a.房間入口(status=5)
        source_locations = self.get_locations_by_status('room_entrance', 5)
        if source_locations:
            return {
                'flow_name': '房間出口需要空料架',
                'target_location': target_location,
                'source_location': source_locations[0],
                'source_area': 'room_entrance'
            }

        # 出料: b.空架回收區(status=5)
        source_locations = self.get_locations_by_status('empty_rack_recycle', 5)
        if source_locations:
            return {
                'flow_name': '房間出口需要空料架',
                'target_location': target_location,
                'source_location': source_locations[0],
                'source_area': 'empty_rack_recycle'
            }

        return None

    def _check_task_flow_2_room_entrance_move_out_empty_rack(self) -> dict:
        """
        流程2: 房間入口需要移出空料架

        判斷邏輯：
        - 額外需求: 系統準備區(status=6 or 7)
        - 出料: 房間入口(status=5)
        - 要料: 空架回收區(status=0)

        Returns:
            判斷結果字典或 None
        """
        # 額外需求: 系統準備區(status=6 or 7)
        full_locations = self.get_locations_by_status('system_prepare', 6)
        partial_locations = self.get_locations_by_status('system_prepare', 7)

        if not (full_locations or partial_locations):
            return None  # 系統準備區沒有滿架或部分載貨

        # 出料: 房間入口(status=5)
        source_locations = self.get_locations_by_status('room_entrance', 5)
        if not source_locations:
            return None

        # 要料: 空架回收區(status=0)
        target_locations = self.get_locations_by_status('empty_rack_recycle', 0)
        if not target_locations:
            return None

        target_location = target_locations[0]  # 擇一位置

        # 防重檢查: 該位置是否已有未完成任務
        if self._has_pending_task_to_location(target_location):
            return None

        return {
            'flow_name': '房間入口需要移出空料架',
            'target_location': target_location,
            'source_location': source_locations[0],
            'source_area': 'room_entrance'
        }

    def _check_task_flow_3_room_entrance_needs_full_rack(self) -> dict:
        """
        流程3: 房間入口需要滿料架

        判斷邏輯：
        - 要料: 房間入口(status=0)
        - 出料: 系統準備區(status=6 or 7)

        Returns:
            判斷結果字典或 None
        """
        # 要料: 房間入口(status=0)
        target_locations = self.get_locations_by_status('room_entrance', 0)
        if not target_locations:
            return None

        target_location = target_locations[0]  # 擇一位置

        # 防重檢查: 該位置是否已有未完成任務
        if self._has_pending_task_to_location(target_location):
            return None

        # 出料: 系統準備區(status=6 or 7)
        full_locations = self.get_locations_by_status('system_prepare', 6)
        partial_locations = self.get_locations_by_status('system_prepare', 7)

        source_locations = full_locations + partial_locations
        if not source_locations:
            return None

        return {
            'flow_name': '房間入口需要滿料架',
            'target_location': target_location,
            'source_location': source_locations[0],
            'source_area': 'system_prepare'
        }

    def _check_task_flow_4_injection_work_needs_unload(self) -> dict:
        """
        流程4: 射出機作業區需要出料

        判斷邏輯：
        - 要料: 系統準備區(status=0)
        - 出料: 射出機作業區(status=6 or 7)

        Returns:
            判斷結果字典或 None
        """
        # 要料: 系統準備區(status=0)
        target_locations = self.get_locations_by_status('system_prepare', 0)
        if not target_locations:
            return None

        target_location = target_locations[0]  # 擇一位置

        # 防重檢查: 該位置是否已有未完成任務
        if self._has_pending_task_to_location(target_location):
            return None

        # 出料: 射出機作業區(status=6 or 7)
        full_locations = self.get_locations_by_status('injection_work', 6)
        partial_locations = self.get_locations_by_status('injection_work', 7)

        source_locations = full_locations + partial_locations
        if not source_locations:
            return None

        return {
            'flow_name': '射出機作業區需要出料',
            'target_location': target_location,
            'source_location': source_locations[0],
            'source_area': 'injection_work'
        }

    def _check_task_flow_5_room_exit_full_rack_to_recycle(self) -> dict:
        """
        流程5: 房間出口滿料移到回收區

        判斷邏輯：
        - 出料: 房間出口(status=6)  -- 只能是满架
        - 要料: 滿料回收區(status=0)

        Returns:
            判斷結果字典或 None
        """
        # 出料: 房間出口(status=6) -- 只能是满架
        source_locations = self.get_locations_by_status('room_exit', 6)
        if not source_locations:
            return None

        source_location = source_locations[0]  # 擇一位置

        # 要料: 滿料回收區(status=0)
        target_locations = self.get_locations_by_status('full_rack_recycle', 0)
        if not target_locations:
            return None

        target_location = target_locations[0]  # 擇一位置

        # 防重檢查: 該位置是否已有未完成任務
        if self._has_pending_task_to_location(target_location):
            return None

        return {
            'flow_name': '房間出口滿料移到回收區',
            'target_location': target_location,
            'source_location': source_location,
            'source_area': 'room_exit'
        }

    def _run_all_task_flow_checks(self) -> List[dict]:
        """
        執行所有任務流程判斷

        輪詢原則：要料需求擇一位置 -> 出料需求擇一位置成立則跳出輪詢
        最多判斷出五個流程同時成立

        額外規則：
        - 流程1成立時，跳過流程2（避免房間入口空架衝突）

        Returns:
            所有成立的流程列表
        """
        task_decisions = []

        # 流程1: 房間出口需要空料架
        flow_1 = self._check_task_flow_1_room_exit_needs_empty_rack()
        if flow_1:
            task_decisions.append(flow_1)

        # 流程2: 房間入口需要移出空料架
        # 額外規則：當流程1成立時跳過流程2（避免房間入口空架衝突）
        if not flow_1:
            flow_2 = self._check_task_flow_2_room_entrance_move_out_empty_rack()
            if flow_2:
                task_decisions.append(flow_2)
        else:
            self.logger.debug("流程1成立，跳過流程2判斷（避免房間入口空架衝突）")

        # 流程3: 房間入口需要滿料架
        flow_3 = self._check_task_flow_3_room_entrance_needs_full_rack()
        if flow_3:
            task_decisions.append(flow_3)

        # 流程4: 射出機作業區需要出料
        flow_4 = self._check_task_flow_4_injection_work_needs_unload()
        if flow_4:
            task_decisions.append(flow_4)

        # 流程5: 房間出口滿料移到回收區
        flow_5 = self._check_task_flow_5_room_exit_full_rack_to_recycle()
        if flow_5:
            task_decisions.append(flow_5)

        return task_decisions

    def _print_task_decisions(self, task_decisions: List[dict]):
        """
        印出任務判斷結果

        Args:
            task_decisions: 任務判斷結果列表
        """
        if not task_decisions:
            self.logger.info("📋 任務判斷: 目前無符合條件的流程")
            return

        self.logger.info("=" * 80)
        self.logger.info(f"📋 任務判斷結果 - 發現 {len(task_decisions)} 個符合條件的流程")
        self.logger.info("=" * 80)

        for idx, decision in enumerate(task_decisions, 1):
            flow_name = decision['flow_name']
            source_location = decision['source_location']
            target_location = decision['target_location']
            source_area = decision['source_area']

            # 獲取出料位置的詳細狀態
            source_status = self.get_location_status(source_location)
            source_rack_id = source_status['rack_id'] if source_status else None
            source_bitmap = source_status['carrier_bitmap'] if source_status else None

            self.logger.info(f"\n流程 {idx}: {flow_name}")
            self.logger.info(f"  ├─ 出料位置: Location {source_location} ({source_area})")
            if source_rack_id:
                self.logger.info(f"  │  └─ Rack ID={source_rack_id}, Bitmap={source_bitmap}")
            self.logger.info(f"  └─ 要料位置: Location {target_location}")

        self.logger.info("=" * 80)

    # ========== 任務創建方法 ==========

    def _create_task_from_decision(
        self,
        session: Session,
        decision: dict
    ) -> Task:
        """
        根據判斷結果創建 KUKA 搬運任務

        Args:
            session: 資料庫 session
            decision: 任務判斷結果字典
                {
                    'flow_name': str,
                    'source_location': int,
                    'target_location': int,
                    'source_area': str
                }

        Returns:
            創建的 Task 對象
        """
        source_location_id = decision['source_location']
        target_location_id = decision['target_location']
        flow_name = decision['flow_name']

        # 獲取出料位置的 rack_id
        source_status = self.get_location_status(source_location_id)
        if not source_status or not source_status['rack_id']:
            self.logger.error(f"出料位置 {source_location_id} 沒有 rack，無法創建任務")
            return None

        rack_id = source_status['rack_id']

        # 獲取路徑節點（包含 waypoint）
        nodes = self.db.get_waypoint_nodes(session, source_location_id, target_location_id)
        if not nodes:
            self.logger.error(f"無法獲取路徑節點: {source_location_id} → {target_location_id}")
            return None

        # 防重檢查: 檢查是否已有相同 location_id 和 nodes 的未完成任務
        if self._has_pending_task_to_location(target_location_id, nodes):
            self.logger.info(
                f"⏭️  跳過創建任務: Location {target_location_id} 已有相同路徑任務 (nodes={nodes})"
            )
            return None

        # 查詢 work 資訊（僅用於 description）
        from db_proxy.models import Work
        work_id = 220001  # KUKA_RACK_MOVE
        work = session.get(Work, work_id)

        # task.name 使用流程名稱
        task_name = flow_name

        # task.description 優先使用 work 表的描述
        if work and work.description:
            task_description = work.description
        else:
            task_description = f"{flow_name}"

        # 構建 parameters（其他自定義參數）
        custom_parameters = {
            "api": "submit_mission",
            "function": "rack_move",
            "missionType": "RACK_MOVE"
        }

        # ✅ 使用統一方法創建任務（會自動添加 model, nodes）
        task = self.db.create_kuka_task(
            session=session,
            work_id=work_id,
            nodes=nodes,
            rack_id=rack_id,
            room_id=2,  # ✅ 修正：明確設置為 room_id=2（原先為 None）
            location_id=target_location_id,
            priority=50,
            name=task_name,
            description=task_description,
            notes=f"[SystemAreaHandler] {flow_name}",
            parameters=custom_parameters  # 額外參數會自動合併
        )

        self.logger.info(
            f"✅ 創建任務成功: Task ID={task.id}, {flow_name}\n"
            f"   ├─ Work: {task_name} (ID={work_id})\n"
            f"   ├─ Rack ID: {rack_id}\n"
            f"   ├─ 出料位置: Location {source_location_id}\n"
            f"   ├─ 要料位置: Location {target_location_id}\n"
            f"   ├─ 路徑節點: {nodes}\n"
            f"   └─ Priority: {task.priority}"
        )

        return task

    def _create_all_tasks(self, session: Session) -> List[Task]:
        """
        根據所有判斷結果創建任務

        使用 self.pending_task_decisions 中保存的判斷結果

        Args:
            session: 資料庫 session

        Returns:
            創建的任務列表
        """
        created_tasks = []

        if not self.pending_task_decisions:
            return created_tasks

        self.logger.info("=" * 80)
        self.logger.info(f"🚀 開始創建任務 - 共 {len(self.pending_task_decisions)} 個流程")
        self.logger.info("=" * 80)

        for idx, decision in enumerate(self.pending_task_decisions, 1):
            try:
                task = self._create_task_from_decision(session, decision)
                if task:
                    created_tasks.append(task)
                    self.logger.info(f"[{idx}/{len(self.pending_task_decisions)}] 任務創建成功")
                else:
                    self.logger.warn(f"[{idx}/{len(self.pending_task_decisions)}] 任務創建失敗")
            except Exception as e:
                import traceback
                error_detail = ''.join(traceback.format_exception(type(e), e, e.__traceback__))
                self.logger.error(
                    f"[{idx}/{len(self.pending_task_decisions)}] 創建任務時發生錯誤: {e}\n{error_detail}"
                )

        self.logger.info("=" * 80)
        self.logger.info(f"✅ 任務創建完成 - 成功創建 {len(created_tasks)}/{len(self.pending_task_decisions)} 個任務")
        self.logger.info("=" * 80)

        return created_tasks
