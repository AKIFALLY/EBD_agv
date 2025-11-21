"""
資料庫操作封裝模組（不依賴 TAFL）
使用 SQLModel + ConnectionPoolManager + BaseCRUD
"""

from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.crud.base_crud import BaseCRUD
from db_proxy.models import Work, Task, AGV
from db_proxy.utils.runtime_log_helper import TaskLogHelper
from sqlmodel import select
from typing import Optional, List
import logging


class DatabaseHelper:
    """資料庫操作助手（不依賴 TAFL 系統）"""

    def __init__(self, db_url: str, logger: logging.Logger):
        """
        初始化資料庫助手

        Args:
            db_url: 資料庫連接字串
            logger: 日誌記錄器
        """
        self.logger = logger
        self.pool_manager = ConnectionPoolManager(db_url)

        # 建立 CRUD 實例
        self.work_crud = BaseCRUD(Work, id_column="id")
        self.task_crud = BaseCRUD(Task, id_column="id")
        self.agv_crud = BaseCRUD(AGV, id_column="id")

        self.logger.info("✅ DatabaseHelper 初始化完成")

    def get_work_by_id(self, work_id: int) -> Optional[Work]:
        """
        根據 ID 查詢 Work

        Args:
            work_id: Work ID

        Returns:
            Work 物件，若不存在則返回 None
        """
        try:
            with self.pool_manager.get_session() as session:
                work = self.work_crud.get_by_id(session, work_id)
                if work:
                    self.logger.debug(f"查詢到 Work: ID={work.id}, Name={work.name}")
                else:
                    self.logger.warn(f"Work ID {work_id} 不存在")
                return work
        except Exception as e:
            self.logger.error(f"❌ 查詢 Work {work_id} 失敗: {e}")
            return None

    def check_duplicate_task(self, work_id: int, room_id: int) -> bool:
        """
        檢查是否已有未完成的 Task（避免重複建立）

        Args:
            work_id: Work ID
            room_id: Room ID

        Returns:
            True 表示已有未完成的 Task，False 表示可以建立新 Task
        """
        try:
            with self.pool_manager.get_session() as session:
                # 查詢未完成的 Task
                # status_id not in [4=已完成, 54=已取消]
                statement = select(Task).where(
                    Task.work_id == work_id,
                    Task.room_id == room_id,
                    Task.status_id.not_in([4, 54])
                )
                existing_tasks = session.exec(statement).all()

                if existing_tasks:
                    self.logger.info(
                        f"⚠️ Work {work_id} 在 Room {room_id} 已有 {len(existing_tasks)} 個未完成的 Task"
                    )
                    return True
                else:
                    return False

        except Exception as e:
            self.logger.error(f"❌ 檢查重複 Task 失敗: {e}")
            # 出錯時保守處理，視為已存在（避免重複建立）
            return True

    def create_task(
        self,
        work_id: int,
        room_id: int,
        agv_type: str,
        work: Optional[Work] = None,
        work_name: str = "",
        **kwargs
    ) -> Optional[Task]:
        """
        建立新 Task

        Args:
            work_id: Work ID
            room_id: Room ID
            agv_type: AGV 類型（LOADER 或 UNLOADER）
            work: Work 物件（用於提取 parameters 中的 nodes）
            work_name: Work 名稱（用於 Task 名稱）
            **kwargs: 其他 Task 參數

        Returns:
            建立的 Task 物件，失敗則返回 None
        """
        try:
            with self.pool_manager.get_session() as session:
                # 從 work.parameters 中提取 node_id（如果有的話）
                node_id = kwargs.get('node_id', None)  # 預設為 None

                if work and work.parameters:
                    nodes = work.parameters.get('nodes', [])

                    # 如果 nodes 列表只有 1 個元素，提取第一個作為 node_id
                    if isinstance(nodes, list) and len(nodes) == 1:
                        node_id = nodes[0]
                        self.logger.info(
                            f"📍 從 Work {work_id} parameters 提取 node_id: {node_id}"
                        )
                    elif isinstance(nodes, list) and len(nodes) > 1:
                        # 2 個以上元素，暫時不處理（保留預設值）
                        self.logger.info(
                            f"📍 Work {work_id} 有 {len(nodes)} 個 nodes，暫時保留預設 node_id"
                        )

                # 準備 parameters 欄位（保存 agv_type 和 room_id 供 RCS 使用）
                task_parameters = {
                    "agv_type": agv_type,     # AGV 類型（LOADER/UNLOADER）
                    "room_id": room_id,       # 房間編號
                }

                # 如果 work 有 parameters，保留原有的 model 等欄位
                if work and work.parameters:
                    # 保留 work.parameters 中的其他欄位（如 model）
                    work_params = work.parameters.copy() if isinstance(work.parameters, dict) else {}
                    task_parameters.update(work_params)
                    # 確保 agv_type 和 room_id 不被覆蓋
                    task_parameters["agv_type"] = agv_type
                    task_parameters["room_id"] = room_id

                # 建立 Task 物件
                new_task = Task(
                    work_id=work_id,
                    room_id=room_id,
                    name=kwargs.get('name', f"{agv_type} Task - {work_name}"),
                    description=kwargs.get(
                        'description',
                        f"Auto-created from PLC DM for {agv_type}"
                    ),
                    status_id=kwargs.get('status_id', 1),  # 預設 PENDING
                    priority=kwargs.get('priority', 5),     # 預設優先級 5
                    agv_id=kwargs.get('agv_id', None),     # ✅ 預設為 None，由 RCS 動態分配
                    node_id=node_id,                       # 從 work.parameters.nodes 提取或預設值
                    location_id=node_id,                   # ✅ 同步填入 location_id（用於重複防護）
                    parameters=task_parameters             # ✅ 保存 agv_type 和 room_id 供 RCS 使用
                )

                # 使用 CRUD 建立
                created_task = self.task_crud.create(session, new_task)

                # 記錄任務創建到 RuntimeLog
                TaskLogHelper.log_task_create_success(
                    session=session,
                    task_id=created_task.id,
                    work_id=work_id,
                    status_id=created_task.status_id,
                    task_name=created_task.name,
                    room_id=room_id,
                    agv_type=agv_type,
                    location_id=node_id,
                    node_name="alan_room"
                )
                session.commit()  # 提交 RuntimeLog

                self.logger.info(
                    f"✅ 建立 Task 成功: "
                    f"Task ID={created_task.id}, "
                    f"Work ID={work_id}, "
                    f"Room ID={room_id}, "
                    f"AGV Type={agv_type}, "
                    f"Node ID={node_id}, "
                    f"Location ID={node_id}"
                )

                return created_task

        except Exception as e:
            self.logger.error(f"❌ 建立 Task 失敗: {e}")
            # 記錄錯誤到 RuntimeLog
            try:
                with self.pool_manager.get_session() as error_session:
                    TaskLogHelper.log_task_create_error(
                        session=error_session,
                        work_id=work_id,
                        error=str(e),
                        room_id=room_id,
                        node_name="alan_room"
                    )
                    error_session.commit()
            except:
                pass  # 忽略 RuntimeLog 提交失敗
            return None

    def get_agv_by_name(self, agv_name: str) -> Optional[AGV]:
        """
        根據名稱查詢 AGV（僅查詢啟用的 AGV）

        Args:
            agv_name: AGV 名稱（例如: loader02, unloader02）

        Returns:
            AGV 物件，若不存在或未啟用則返回 None
        """
        try:
            with self.pool_manager.get_session() as session:
                # 查詢 AGV：name 匹配且 enable = 1
                statement = select(AGV).where(
                    AGV.name == agv_name,
                    AGV.enable == 1
                )
                agv = session.exec(statement).first()

                if agv:
                    self.logger.debug(
                        f"查詢到 AGV: Name={agv.name}, ID={agv.id}, "
                        f"Model={agv.model}, Enable={agv.enable}"
                    )
                else:
                    self.logger.warn(
                        f"AGV '{agv_name}' 不存在或未啟用 (enable=1)"
                    )
                return agv

        except Exception as e:
            self.logger.error(f"❌ 查詢 AGV '{agv_name}' 失敗: {e}")
            return None

    def delete_completed_tasks(self, status_ids: List[int]) -> int:
        """
        刪除已完成或已取消的 Task

        Args:
            status_ids: 需要刪除的狀態 ID 列表（例如: [4, 54]）

        Returns:
            刪除的 Task 數量，失敗則返回 0
        """
        try:
            with self.pool_manager.get_session() as session:
                # 查詢需要刪除的 Task
                statement = select(Task).where(
                    Task.status_id.in_(status_ids)
                )
                tasks_to_delete = session.exec(statement).all()

                if not tasks_to_delete:
                    self.logger.debug(
                        f"沒有需要清理的 Task (status in {status_ids})"
                    )
                    return 0

                # 刪除 Task
                delete_count = 0
                for task in tasks_to_delete:
                    # 記錄刪除前的任務信息
                    task_id = task.id
                    work_id = task.work_id
                    status_id = task.status_id
                    task_name = task.name

                    # 刪除任務
                    session.delete(task)

                    # 記錄任務刪除到 RuntimeLog
                    TaskLogHelper.log_task_delete_success(
                        session=session,
                        task_id=task_id,
                        work_id=work_id,
                        status_id=status_id,
                        task_name=task_name,
                        node_name="alan_room"
                    )

                    delete_count += 1

                session.commit()

                self.logger.info(
                    f"🗑️ 已清理 {delete_count} 個 Task (status in {status_ids})"
                )
                return delete_count

        except Exception as e:
            self.logger.error(f"❌ 刪除已完成 Task 失敗: {e}")
            # 記錄錯誤到 RuntimeLog
            try:
                with self.pool_manager.get_session() as error_session:
                    # 無法確定具體是哪個任務失敗，記錄通用錯誤
                    TaskLogHelper.log_task_create_error(
                        session=error_session,
                        work_id=0,
                        error=f"批量刪除失敗: {str(e)}",
                        node_name="alan_room"
                    )
                    error_session.commit()
            except:
                pass  # 忽略 RuntimeLog 提交失敗
            return 0

    def shutdown(self):
        """關閉資料庫連線池"""
        try:
            self.pool_manager.shutdown()
            self.logger.info("✅ DatabaseHelper 已關閉連線池")
        except Exception as e:
            self.logger.error(f"❌ 關閉連線池失敗: {e}")
