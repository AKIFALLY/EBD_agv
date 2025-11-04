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
        logger.info(f"請求取得交管區使用權: trafficId={data.trafficId}, agvId={data.agvId}")
        try:
            # 驗證參數格式
            try:
                traffic_id = int(data.trafficId)
                agv_id = int(data.agvId)
            except ValueError:
                logger.warning(f"無效的參數格式: trafficId={data.trafficId}, agvId={data.agvId}")
                raise HTTPException(
                    status_code=400,
                    detail="無效的參數格式，trafficId 和 agvId 必須為數字"
                )

            # 嘗試獲取交管區
            result = traffic_controller.acquire_traffic_zone(traffic_id, agv_id)

            # 根據結果返回對應的響應
            if result["success"]:
                logger.info(f"成功獲取交管區: trafficId={traffic_id}, agvId={agv_id}")
                return {"isAllow": True, "success": True, "message": result.get("message")}

            # 處理失敗情況
            reason = result.get("reason", "unknown")
            message = result.get("message", "Failed to acquire traffic zone")

            if reason == "zone_not_found":
                logger.warning(f"交管區不存在: trafficId={traffic_id}")
                raise HTTPException(status_code=404, detail=message)
            elif reason == "zone_controlled":
                owner_agv_id = result.get("owner_agv_id")
                logger.warning(f"交管區已被占用: trafficId={traffic_id}, ownerAgv={owner_agv_id}")
                return {"isAllow": False, "success": False, "message": message ,"owner_agv_id": owner_agv_id}
                # raise HTTPException(
                #     status_code=409,
                #     detail={"message": message, "owner_agv_id": owner_agv_id}
                # )
            elif reason == "database_error":
                logger.error(f"資料庫錯誤: {message}")
                raise HTTPException(status_code=500, detail=message)
            else:
                logger.error(f"未知錯誤: {message}")
                raise HTTPException(status_code=500, detail=message)

        except HTTPException:
            # 重新拋出 HTTPException
            raise
        except Exception as e:
            logger.exception("取得交管區使用權發生未預期錯誤")
            raise HTTPException(
                status_code=500,
                detail=f"系統內部錯誤: {str(e)}"
            )

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
