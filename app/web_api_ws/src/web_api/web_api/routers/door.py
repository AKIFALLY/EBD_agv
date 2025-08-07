# routers/door.py
import logging
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
import asyncio
from concurrent.futures import ThreadPoolExecutor

logger = logging.getLogger(__name__)

# Create a thread pool for blocking operations
executor = ThreadPoolExecutor(max_workers=5)


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
        try:
            # async_control_door is actually synchronous (fire-and-forget)
            # It sends command to PLC and returns immediately
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(
                executor,
                door_controller.async_control_door,
                int(data.doorId),
                data.isOpen
            )
            return {
                "success": True, 
                "doorId": data.doorId, 
                "action": "open" if data.isOpen else "close",
                "message": "控制指令已發送"
            }
        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e))
        except Exception as e:
            logger.error(f"門控制失敗: {e}")
            raise HTTPException(status_code=500, detail="門控制失敗")

    @router.post("/state")
    async def door_state(data: DoorStateData):
        logger.info(f"查詢門狀態: doorId={data.doorId}")
        try:
            # Use the synchronous state_door method in a thread pool
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(
                executor, 
                door_controller.state_door, 
                int(data.doorId)
            )
            return result
        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e))
        except Exception as e:
            logger.error(f"查詢門狀態失敗: {e}")
            raise HTTPException(status_code=500, detail="查詢門狀態失敗")

    return router
