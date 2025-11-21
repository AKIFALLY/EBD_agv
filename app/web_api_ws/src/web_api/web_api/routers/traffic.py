# routers/traffic.py
import logging
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from traffic_manager.traffic_controller import TrafficController


class TrafficData(BaseModel):
    trafficId: str
    agvId: str


class ForceReleaseData(BaseModel):
    trafficId: str


def create_traffic_router(traffic_controller: TrafficController):
    logger = logging.getLogger(__name__)
    router = APIRouter(prefix="/traffic", tags=["Traffic"])

    @router.post("/acquire")
    def traffic_acquire(data: TrafficData):
        # 記錄 request body
        logger.info(f"[交管API] 📥 收到請求: POST /traffic/acquire, body={{trafficId: {data.trafficId}, agvId: {data.agvId}}}")
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
                response_body = {"isAllow": True, "success": True, "message": result.get("message")}
                logger.info(f"[交管API] 📤 返回回應: status=200, body={response_body}")
                return response_body

            # 處理失敗情況
            reason = result.get("reason", "unknown")
            message = result.get("message", "Failed to acquire traffic zone")

            if reason == "zone_not_found":
                logger.warning(f"交管區不存在: trafficId={traffic_id}")
                raise HTTPException(status_code=404, detail=message)
            elif reason == "zone_controlled":
                owner_agv_id = result.get("owner_agv_id")
                logger.warning(f"交管區已被占用: trafficId={traffic_id}, ownerAgv={owner_agv_id}")
                response_body = {"isAllow": False, "success": False, "message": message, "owner_agv_id": owner_agv_id}
                logger.info(f"[交管API] 📤 返回回應: status=200, body={response_body}")
                return response_body
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
        # 記錄 request body
        logger.info(f"[交管API] 📥 收到釋放請求: POST /traffic/release, body={{trafficId: {data.trafficId}, agvId: {data.agvId}}}")
        try:
            success = traffic_controller.release_traffic_zone(
                int(data.trafficId), int(data.agvId))
            if success:
                response_body = {"success": True}
                logger.info(f"[交管API] 📤 返回釋放回應: status=200, body={response_body}")
                return response_body
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
            # 參數驗證
            if not data.trafficId or not data.trafficId.strip():
                logger.warning(f"無效的交管區名稱參數: trafficName={data.trafficId}")
                raise HTTPException(
                    status_code=400,
                    detail="無效的參數，trafficName 不能為空"
                )

            if not data.agvId or not data.agvId.strip():
                logger.warning(f"無效的 AGV 名稱參數: agvName={data.agvId}")
                raise HTTPException(
                    status_code=400,
                    detail="無效的參數，agvName 不能為空"
                )

            # 嘗試獲取交管區
            result = traffic_controller.acquire_traffic_zone_by_name(
                data.trafficId, data.agvId)

            # 根據結果返回對應的響應
            if result["success"]:
                logger.info(f"成功獲取交管區: trafficName={data.trafficId}, agvName={data.agvId}")
                return {"isAllow": True, "success": True, "message": result.get("message")}

            # 處理失敗情況
            reason = result.get("reason", "unknown")
            message = result.get("message", "Failed to acquire traffic zone by name")

            if reason == "zone_not_found":
                logger.warning(f"交管區不存在: trafficName={data.trafficId}")
                raise HTTPException(status_code=404, detail=message)
            elif reason == "agv_not_found":
                logger.warning(f"AGV 不存在: agvName={data.agvId}")
                raise HTTPException(status_code=404, detail=message)
            elif reason == "zone_disabled":
                logger.warning(f"交管區已禁用: trafficName={data.trafficId}")
                raise HTTPException(status_code=403, detail=message)
            elif reason == "zone_controlled":
                owner_agv_id = result.get("owner_agv_id")
                logger.warning(f"交管區已被占用: trafficName={data.trafficId}, ownerAgv={owner_agv_id}")
                return {"isAllow": False, "success": False, "message": message, "owner_agv_id": owner_agv_id}
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
            logger.exception("取得交管區使用權 (依名稱) 發生未預期錯誤")
            raise HTTPException(
                status_code=500,
                detail=f"系統內部錯誤: {str(e)}"
            )

    @router.post("/release_by_name")
    def traffic_release_by_name(data: TrafficData):
        logger.info(
            f"釋放交管區使用權 (依名稱): trafficName={data.trafficId}, agvName={data.agvId}")
        try:
            # 參數驗證
            if not data.trafficId or not data.trafficId.strip():
                logger.warning(f"無效的交管區名稱參數: trafficName={data.trafficId}")
                raise HTTPException(
                    status_code=400,
                    detail="無效的參數，trafficName 不能為空"
                )

            if not data.agvId or not data.agvId.strip():
                logger.warning(f"無效的 AGV 名稱參數: agvName={data.agvId}")
                raise HTTPException(
                    status_code=400,
                    detail="無效的參數，agvName 不能為空"
                )

            # 嘗試釋放交管區
            result = traffic_controller.release_traffic_zone_by_name(
                data.trafficId, data.agvId)

            # 根據結果返回對應的響應
            if result["success"]:
                logger.info(f"成功釋放交管區: trafficName={data.trafficId}, agvName={data.agvId}")
                return {"success": True, "message": result.get("message")}

            # 處理失敗情況
            reason = result.get("reason", "unknown")
            message = result.get("message", "Failed to release traffic zone by name")

            if reason == "zone_not_found":
                logger.warning(f"交管區不存在: trafficName={data.trafficId}")
                raise HTTPException(status_code=404, detail=message)
            elif reason == "agv_not_found":
                logger.warning(f"AGV 不存在: agvName={data.agvId}")
                raise HTTPException(status_code=404, detail=message)
            elif reason == "not_owner":
                logger.warning(f"無權釋放交管區: trafficName={data.trafficId}, agvName={data.agvId}")
                raise HTTPException(status_code=403, detail=message)
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
            logger.exception("釋放交管區使用權 (依名稱) 發生未預期錯誤")
            raise HTTPException(
                status_code=500,
                detail=f"系統內部錯誤: {str(e)}"
            )

    @router.post("/force_release")
    def traffic_force_release(data: ForceReleaseData):
        """
        強制釋放交管區使用權 (管理員功能)
        :param data: 包含 trafficId 的資料
        :return: 操作結果
        """
        logger.info(f"強制釋放交管區: trafficId={data.trafficId}")
        try:
            # 驗證參數格式
            try:
                traffic_id = int(data.trafficId)
            except ValueError:
                logger.warning(f"無效的參數格式: trafficId={data.trafficId}")
                raise HTTPException(
                    status_code=400,
                    detail="無效的參數格式，trafficId 必須為數字"
                )

            # 直接操作資料庫強制釋放交管區
            with traffic_controller.db_pool.get_session() as session:
                from db_proxy.models import TrafficZone
                from sqlmodel import select

                zone = session.exec(
                    select(TrafficZone).where(TrafficZone.id == traffic_id)
                ).first()

                # 檢查交管區是否存在
                if not zone:
                    logger.warning(f"交管區不存在: trafficId={traffic_id}")
                    raise HTTPException(
                        status_code=404,
                        detail=f"交管區 ID {traffic_id} 不存在"
                    )

                # 記錄原始擁有者資訊
                original_owner = zone.owner_agv_id
                original_status = zone.status

                # 強制釋放：設為 free 狀態，清除擁有者
                zone.status = "free"
                zone.owner_agv_id = None
                session.add(zone)
                session.commit()
                session.refresh(zone)

                logger.info(
                    f"[強制釋放交管區] 交管區 {traffic_id} - "
                    f"原狀態: {original_status}, 原擁有者: AGV {original_owner} → "
                    f"新狀態: free, 擁有者: None - 強制釋放成功"
                )

                return {
                    "success": True,
                    "message": "成功強制釋放交管區",
                    "zone_id": traffic_id,
                    "previous_owner": original_owner,
                    "previous_status": original_status
                }

        except HTTPException:
            # 重新拋出 HTTPException
            raise
        except Exception as e:
            logger.exception("強制釋放交管區發生未預期錯誤")
            raise HTTPException(
                status_code=500,
                detail=f"系統內部錯誤: {str(e)}"
            )

    return router
