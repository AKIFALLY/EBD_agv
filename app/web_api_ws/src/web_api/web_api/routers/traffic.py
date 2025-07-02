# routers/traffic.py
import logging
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from traffic_manager.traffic_controller import TrafficController


class TrafficData(BaseModel):
    trafficId: str
    agvId: str


def create_traffic_router(traffic_controller: TrafficController):
    logger = logging.getLogger(__name__)
    router = APIRouter(prefix="/traffic", tags=["Traffic"])

    @router.post("/acquire")
    def traffic_acquire(data: TrafficData):
        logger.info(f"請求取得交管區使用權: trafficId={data.trafficId}")
        try:
            success = traffic_controller.acquire_traffic_zone(
                int(data.trafficId), int(data.agvId))
            if success:
                return {"isAllow": True, "success": True}
            else:
                raise HTTPException(
                    status_code=500, detail="Failed to acquire traffic zone")
        except Exception as e:
            logger.exception("取得交管區使用權發生錯誤")
            raise HTTPException(
                status_code=500, detail=f"Error acquiring traffic zone: {e}")

    @router.post("/release")
    def traffic_release(data: TrafficData):
        logger.info(f"釋放交管區使用權: trafficId={data.trafficId}")
        try:
            success = traffic_controller.release_traffic_zone(
                int(data.trafficId), int(data.agvId))
            if success:
                return {"success": True}
            else:
                raise HTTPException(
                    status_code=500, detail="Failed to release traffic zone")
        except Exception as e:
            logger.exception("釋放交管區使用權發生錯誤")
            raise HTTPException(
                status_code=500, detail=f"Error releasing traffic zone: {e}")

    @router.post("/acquire_by_name")
    def traffic_acquire_by_name(data: TrafficData):
        logger.info(
            f"請求取得交管區使用權 (依名稱): trafficName={data.trafficId}, agvName={data.agvId}")
        try:
            success = traffic_controller.acquire_traffic_zone_by_name(
                data.trafficId, data.agvId)
            if success:
                return {"isAllow": True, "success": True}
            else:
                raise HTTPException(
                    status_code=500, detail="Failed to acquire traffic zone by name")
        except Exception as e:
            logger.exception("取得交管區使用權 (依名稱) 發生錯誤")
            raise HTTPException(
                status_code=500, detail=f"Error acquiring traffic zone by name: {e}")

    @router.post("/release_by_name")
    def traffic_release_by_name(data: TrafficData):
        logger.info(
            f"釋放交管區使用權 (依名稱): trafficName={data.trafficId}, agvName={data.agvId}")
        try:
            success = traffic_controller.release_traffic_zone_by_name(
                data.trafficId, data.agvId)
            if success:
                return {"success": True}
            else:
                raise HTTPException(
                    status_code=500, detail="Failed to release traffic zone by name")
        except Exception as e:
            logger.exception("釋放交管區使用權 (依名稱) 發生錯誤")
            raise HTTPException(
                status_code=500, detail=f"Error releasing traffic zone by name: {e}")

    return router
