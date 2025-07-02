from fastapi import APIRouter, HTTPException
from plc_proxy.plc_client import PlcClient
from pydantic import BaseModel
from typing import List


class ForceInput(BaseModel):
    device_type: str
    key: str


class SingleDataInput(BaseModel):
    device_type: str
    key: str
    value: str


class ContinuousDataInput(BaseModel):
    device_type: str
    start_key: str
    values: List[str]


def create_plc_router(plc_client: PlcClient):
    router = APIRouter(prefix="/plc", tags=["PLC"])

    @router.get("/get_data/{device_type}/{key}")
    async def get_data(device_type: str, key: str):
        try:
            rsp = plc_client.read_data(device_type, key)
            print(rsp)
            if not rsp.success:
                raise HTTPException(
                    status_code=500,
                    detail=f"Failed to reading data from PLC {rsp.message}",
                )
            if not rsp.value:
                raise HTTPException(
                    status_code=404, detail="Key not found in PLC")
            return {"key": key, "value": rsp.value}
        except Exception as e:
            raise HTTPException(
                status_code=500, detail=f"Error reading from PLC: {e}"
            )

    @router.get("/get_continuous_data/{device_type}/{start_key}/{count}")
    async def get_continuous_data(device_type: str, start_key: str, count: int):
        try:
            rsp = plc_client.read_continuous_data(
                device_type, start_key, count)
            print(rsp)
            if not rsp.success:
                raise HTTPException(
                    status_code=500,
                    detail=f"Failed to reading continuous data from PLC {rsp.message}",
                )
            if not rsp.values:
                raise HTTPException(
                    status_code=404, detail="Key not found in PLC")
            return {"start_key": start_key, "count": count, "values": rsp.values}
        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"Error reading continuous data from PLC: {e}",
            )

    @router.post("/set_data")
    async def set_data(data: SingleDataInput):
        try:
            rsp = plc_client.write_data(data.device_type, data.key, data.value)
            print(rsp)
            if not rsp.success:
                raise HTTPException(
                    status_code=500, detail=f"Failed to write to PLC {rsp.message}"
                )
            return {"message": "Data written to PLC successfully"}
        except Exception as e:
            raise HTTPException(
                status_code=500, detail=f"Error writing to PLC: {e}"
            )

    @router.post("/set_continuous_data")
    async def set_continuous_data(data: ContinuousDataInput):
        try:
            rsp = plc_client.write_continuous_data(
                data.device_type, data.start_key, data.values
            )
            print(rsp)
            if not rsp.success:
                raise HTTPException(
                    status_code=500,
                    detail=f"Failed to write continuous data to PLC {rsp.message}",
                )
            return {"message": "Continuous data written to PLC successfully"}
        except Exception as e:
            raise HTTPException(
                status_code=500, detail=f"Error writing continuous data to PLC: {e}"
            )

    @router.post("/force_on")
    async def force_on(data: ForceInput):
        try:
            rsp = plc_client.force_on(data.device_type, data.key)
            print(rsp)
            if not rsp.success:
                raise HTTPException(
                    status_code=500,
                    detail=f"Failed to force_on to PLC {rsp.message}",
                )
            return {"message": "force_on to PLC successfully"}
        except Exception as e:
            raise HTTPException(
                status_code=500, detail=f"Error force_on to PLC: {e}"
            )

    @router.post("/force_off")
    async def force_off(data: ForceInput):
        try:
            rsp = plc_client.force_off(data.device_type, data.key)
            print(rsp)
            if not rsp.success:
                raise HTTPException(
                    status_code=500,
                    detail=f"Failed to force_off to PLC {rsp.message}",
                )
            return {"message": "force_off to PLC successfully"}
        except Exception as e:
            raise HTTPException(
                status_code=500, detail=f"Error force_off to PLC: {e}"
            )

    return router
