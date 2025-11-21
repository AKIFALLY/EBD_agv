# routers/kuka.py
import logging
from datetime import datetime, timezone
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional, Dict, Any
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Task, Rack
from db_proxy.models.modify_log import ModifyLog
from db_proxy.crud.task_crud import task_crud
from db_proxy.crud.agv_crud import agv_crud
from sqlmodel import select
from shared_constants.task_status import TaskStatus

logger = logging.getLogger(__name__)


def _normalize_direction(orientation: float) -> int:
    """将 KUKA orientation 规范化到 10 的倍数（保持原始正負號）"""
    return round(orientation / 10) * 10


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

    # KUKA 任務狀態到 TaskStatus 的映射
    KUKA_STATUS_MAPPING = {
        "MOVE_BEGIN": TaskStatus.EXECUTING,       # 3 - 開始移動
        "ARRIVED": TaskStatus.EXECUTING,          # 3 - 到達節點
        "UP_CONTAINER": TaskStatus.EXECUTING,     # 3 - 升箱完成
        "DOWN_CONTAINER": TaskStatus.EXECUTING,   # 3 - 放下完成
        "ROLLER_RECEIVE": TaskStatus.EXECUTING,   # 3 - 滾筒上料
        "ROLLER_SEND": TaskStatus.EXECUTING,      # 3 - 滾筒下料
        "PICKER_RECEIVE": TaskStatus.EXECUTING,   # 3 - 料箱取料
        "PICKER_SEND": TaskStatus.EXECUTING,      # 3 - 料箱下料
        "FORK_UP": TaskStatus.EXECUTING,          # 3 - 叉車叉取
        "FORK_DOWN": TaskStatus.EXECUTING,        # 3 - 叉車放下
        "COMPLETED": TaskStatus.COMPLETED,        # 4 - 任務完成
        "CANCELED": TaskStatus.CANCELLED,         # 54 - 任務取消
    }

    @router.post("/missionStateCallback")
    async def mission_state_callback(data: MissionStateCallbackData):
        """
        接收 Kuka 系統的任務狀態回報

        將 Kuka 系統回報的狀態資訊存入 Task 的 parameters 欄位，
        並根據 missionStatus 自動更新 Task 的 status_id：

        KUKA 狀態 → TaskStatus 映射：
        - MOVE_BEGIN → EXECUTING (3) - 開始移動
        - ARRIVED → EXECUTING (3) - 到達任務節點
        - UP_CONTAINER → EXECUTING (3) - 升箱完成
        - DOWN_CONTAINER → EXECUTING (3) - 放下完成
        - ROLLER_RECEIVE → EXECUTING (3) - 滾筒上料完成
        - ROLLER_SEND → EXECUTING (3) - 滾筒下料完成
        - PICKER_RECEIVE → EXECUTING (3) - 料箱取料完成
        - PICKER_SEND → EXECUTING (3) - 料箱下料完成
        - FORK_UP → EXECUTING (3) - 叉車叉取完成
        - FORK_DOWN → EXECUTING (3) - 叉車放下完成
        - COMPLETED → COMPLETED (4) - 任務完成
        - CANCELED → CANCELLED (54) - 任務取消完成

        狀態轉換驗證：
        - 避免倒退：已完成(4)或已取消(54)的任務不允許回到其他狀態
        - 標準流程：PENDING(1) → EXECUTING(3) → COMPLETED(4)
        - 取消流程：任何狀態 → CANCELLED(54)
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

                # ✅ 修改：即使沒有 task，仍然處理 rack 狀態更新
                if not existing_task:
                    logger.warning(
                        f"找不到對應的任務: missionCode={data.missionCode}, "
                        f"但仍會處理 rack 狀態更新 (containerCode={data.containerCode}, "
                        f"missionStatus={data.missionStatus})")
                else:
                    # 有 task 時才更新 task.parameters
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

                    # 更新 parameters
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

                # 處理容器頂升/放下狀態
                if data.missionStatus in ['UP_CONTAINER', 'DOWN_CONTAINER']:
                    if data.containerCode:
                        # 根據 containerCode 查找 Rack
                        rack_statement = select(Rack).where(Rack.name == data.containerCode)
                        rack = session.exec(rack_statement).first()

                        if rack:
                            if data.missionStatus == 'UP_CONTAINER':
                                # AGV 頂升容器
                                # ✅ 使用 robotId 查詢 AGV（解耦優化）
                                if data.robotId:
                                    try:
                                        agv = agv_crud.get_by_id(session, int(data.robotId))
                                        if agv:
                                            rack.agv_id = agv.id
                                            logger.info(
                                                f"✅ UP_CONTAINER: Rack {rack.name} (id={rack.id}) "
                                                f"picked up by AGV {agv.id} (robotId={data.robotId})")
                                        else:
                                            logger.warning(f"⚠️ AGV not found for robotId: {data.robotId}")
                                            # 如果有 task，使用 task.agv_id 作為 fallback
                                            if existing_task:
                                                rack.agv_id = existing_task.agv_id
                                                logger.info(f"使用 task.agv_id={existing_task.agv_id} 作為 fallback")
                                            else:
                                                logger.error("❌ 無法設置 agv_id：robotId 無效且沒有對應的 task")
                                    except (ValueError, TypeError) as e:
                                        logger.error(f"❌ Invalid robotId format: {data.robotId}, error: {e}")
                                        # 如果有 task，使用 task.agv_id 作為 fallback
                                        if existing_task:
                                            rack.agv_id = existing_task.agv_id
                                            logger.info(f"使用 task.agv_id={existing_task.agv_id} 作為 fallback")
                                        else:
                                            logger.error("❌ 無法設置 agv_id：robotId 格式無效且沒有對應的 task")
                                else:
                                    # 沒有 robotId
                                    if existing_task:
                                        logger.warning("⚠️ No robotId provided, using task.agv_id")
                                        rack.agv_id = existing_task.agv_id
                                    else:
                                        logger.error("❌ 無法設置 agv_id：沒有 robotId 且沒有對應的 task")

                                rack.is_carry = 1

                            elif data.missionStatus == 'DOWN_CONTAINER':
                                # AGV 放下容器
                                old_agv_id = rack.agv_id
                                rack.agv_id = None
                                rack.is_carry = 0

                                # 🆕 更新 direction（从 missionData 中获取 orientation）
                                if data.missionData and 'orientation' in data.missionData:
                                    try:
                                        orientation = float(data.missionData['orientation'])
                                        old_direction = rack.direction
                                        rack.direction = _normalize_direction(orientation)
                                        logger.info(
                                            f"📐 DOWN_CONTAINER: Rack {rack.name} direction 更新: "
                                            f"{old_direction}° → {rack.direction}° "
                                            f"(KUKA orientation: {orientation}°)")
                                    except (ValueError, TypeError) as e:
                                        logger.warning(f"⚠️ 无法解析 orientation: {data.missionData.get('orientation')}, error: {e}")

                                # ✅ 更新 location_id（位置同步）- 仅当容器在地图中时
                                if data.currentPosition and rack.is_in_map == 1:
                                    try:
                                        # 解析 currentPosition: "M001-A001-31" → location_id = 31
                                        location_id = int(data.currentPosition.split('-')[-1])
                                        old_location_id = rack.location_id
                                        rack.location_id = location_id
                                        logger.info(
                                            f"✅ DOWN_CONTAINER: Rack {rack.name} (id={rack.id}) "
                                            f"put down at location {location_id} "
                                            f"(was on AGV {old_agv_id}, location {old_location_id} → {location_id})")
                                    except (ValueError, IndexError) as e:
                                        logger.error(
                                            f"❌ Failed to parse location_id from currentPosition: "
                                            f"{data.currentPosition}, error: {e}")
                                elif data.currentPosition and rack.is_in_map != 1:
                                    logger.debug(
                                        f"DOWN_CONTAINER for Rack {rack.name}: 容器不在地图中 (is_in_map={rack.is_in_map})，"
                                        f"跳过 location 更新 (currentPosition={data.currentPosition})")
                                elif not data.currentPosition:
                                    logger.debug(
                                        f"DOWN_CONTAINER for Rack {rack.name} but no currentPosition provided")

                            # 觸發前端更新
                            session.add(rack)
                            ModifyLog.mark(session, "rack")
                            session.commit()
                            session.refresh(rack)
                        else:
                            logger.warning(
                                f"⚠️ Rack not found for containerCode: {data.containerCode}")
                    else:
                        logger.debug(
                            f"missionStatus={data.missionStatus} but no containerCode provided")

                # 根據是否有 task 返回不同的響應
                if existing_task:
                    logger.info(f"提交後的 parameters: {existing_task.parameters}")
                    logger.info(f"任務參數更新成功: task_id={existing_task.id}, "
                                f"missionStatus={data.missionStatus} (僅更新 parameters，不修改 status_id)")

                    return {
                        "success": True,
                        "message": "Mission state callback processed successfully",
                        "task_id": existing_task.id,
                        "mission_code": data.missionCode,
                        "mission_status": data.missionStatus
                    }
                else:
                    logger.info(f"Rack 狀態更新成功 (無對應 task): "
                                f"missionCode={data.missionCode}, "
                                f"missionStatus={data.missionStatus}, "
                                f"containerCode={data.containerCode}")

                    return {
                        "success": True,
                        "message": "Rack state updated (task not found)",
                        "mission_code": data.missionCode,
                        "mission_status": data.missionStatus,
                        "warning": "No corresponding task found, only rack state updated"
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
