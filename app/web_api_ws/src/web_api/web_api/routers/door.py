# routers/door.py
import logging
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

logger = logging.getLogger(__name__)


class DoorStateData(BaseModel):
    doorId: str


class DoorControlData(BaseModel):
    doorId: str
    isOpen: bool


def create_door_router(door_controller):
    router = APIRouter(prefix="/door", tags=["Door"])

    @router.post("/control")
    async def door_control(data: DoorControlData):
        logger.info(f"收到門控制指令: doorId={data.doorId}, isOpen={data.isOpen}")
        return await door_controller.async_control_door(int(data.doorId), data.isOpen)

    @router.post("/state")
    async def door_state(data: DoorStateData):
        logger.info(f"查詢門狀態: doorId={data.doorId}")
        return await door_controller.async_state_door(int(data.doorId))

    return router
