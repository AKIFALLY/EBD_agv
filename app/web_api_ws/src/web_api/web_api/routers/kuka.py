# routers/kuka.py
import logging
from datetime import datetime, timezone
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional, Dict, Any
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Task
from db_proxy.crud.task_crud import task_crud
from sqlmodel import select

logger = logging.getLogger(__name__)


class MissionStateCallbackData(BaseModel):
    """Kuka 系統任務狀態回報的資料模型"""
    missionCode: str  # 任務代碼 ID (必填，最大長度32)
    viewBoardType: Optional[str] = None  # 任務類型
    containerCode: Optional[str] = None  # 容器代碼
    currentPosition: Optional[str] = None  # 容器當前位置
    slotCode: Optional[str] = None  # 所在槽位
    robotId: Optional[str] = None  # 執行任務的機器人 ID
    missionStatus: str  # 任務狀態 (必填)
    message: Optional[str] = None  # 補充說明
    missionData: Optional[Dict[str, Any]] = None  # 任務自訂資料


def create_kuka_router(db_pool: ConnectionPoolManager):
    """創建 Kuka API 路由器"""
    router = APIRouter(prefix="/interfaces/api/amr", tags=["Kuka"])

    @router.post("/missionStateCallback")
    async def mission_state_callback(data: MissionStateCallbackData):
        """
        接收 Kuka 系統的任務狀態回報

        將 Kuka 系統回報的狀態資訊存入 Task 的 parameters 欄位：
        - MOVE_BEGIN: 開始移動
        - ARRIVED: 到達任務節點
        - UP_CONTAINER: 升箱完成
        - DOWN_CONTAINER: 放下完成
        - ROLLER_RECEIVE: 滾筒上料完成
        - ROLLER_SEND: 滾筒下料完成
        - PICKER_RECEIVE: 料箱取料完成
        - PICKER_SEND: 料箱下料完成
        - FORK_UP: 叉車叉取完成
        - FORK_DOWN: 叉車放下完成
        - COMPLETED: 任務完成
        - CANCELED: 任務取消完成

        注意：此 API 只更新 parameters 欄位，不會修改 Task 的 status_id
        """
        logger.info(f"收到 Kuka 任務狀態回報: missionCode={data.missionCode}, "
                    f"missionStatus={data.missionStatus}, robotId={data.robotId}")

        try:
            with db_pool.get_session() as session:
                # 根據 missionCode 查找對應的任務
                # 使用新增的 mission_code 欄位進行查詢
                statement = select(Task).where(
                    Task.mission_code == data.missionCode)
                existing_task = session.exec(statement).first()

                if not existing_task:
                    logger.warning(f"找不到對應的任務: missionCode={data.missionCode}")
                    raise HTTPException(
                        status_code=404,
                        detail=f"Task not found for missionCode: {data.missionCode}"
                    )

                # 更新任務的參數，將 Kuka 回報的資訊存入 parameters 欄位
                current_params = existing_task.parameters or {}
                logger.info(f"原始 parameters: {current_params}")

                # 更新任務狀態相關資訊
                kuka_status_info = {
                    "kuka_mission_status": data.missionStatus,
                    "kuka_robot_id": data.robotId,
                    "kuka_container_code": data.containerCode,
                    "kuka_current_position": data.currentPosition,
                    "kuka_slot_code": data.slotCode,
                    "kuka_view_board_type": data.viewBoardType,
                    "kuka_message": data.message,
                    "kuka_mission_data": data.missionData,
                    "kuka_last_update": datetime.now(timezone.utc).isoformat()
                }
                logger.info(f"新增的 Kuka 狀態資訊: {kuka_status_info}")

                # 合併現有參數和新的 Kuka 狀態資訊
                current_params.update(kuka_status_info)
                logger.info(f"合併後的 parameters: {current_params}")

                # 簡化版本：只更新 parameters，不改變狀態
                # 狀態由 WCS 統一管理
                existing_task.parameters = dict(current_params)
                existing_task.updated_at = datetime.now(timezone.utc)
                logger.info(
                    f"更新任務 {existing_task.id} parameters: {existing_task.parameters}")
                logger.info(f"任務狀態保持: {existing_task.status_id} (由 WCS 統一管理)")

                # 標記 parameters 欄位為已修改（確保 SQLAlchemy 檢測到變化）
                from sqlalchemy.orm import attributes
                attributes.flag_modified(existing_task, "parameters")

                # 將更新後的物件添加到 session 並提交
                session.add(existing_task)
                session.commit()
                session.refresh(existing_task)

                updated_task = existing_task
                logger.info(f"提交後的 parameters: {updated_task.parameters}")

                logger.info(f"任務參數更新成功: task_id={updated_task.id}, "
                            f"missionStatus={data.missionStatus} (僅更新 parameters，不修改 status_id)")

                return {
                    "success": True,
                    "message": "Mission state callback processed successfully",
                    "task_id": updated_task.id,
                    "mission_code": data.missionCode,
                    "mission_status": data.missionStatus
                }

        except HTTPException:
            # 重新拋出 HTTP 異常
            raise
        except Exception as e:
            logger.exception(f"處理 Kuka 任務狀態回報時發生錯誤: {e}")
            raise HTTPException(
                status_code=500,
                detail=f"Error processing mission state callback: {str(e)}"
            )

    return router
