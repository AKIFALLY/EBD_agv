import os
import signal
import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List
from ecs.plc_client import PlcClient


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


class ApiServer:
    def __init__(self):
        self.app = FastAPI()
        self.plc_client = PlcClient()

        # 註冊 API 端點
        self.setup_routes()

    def setup_routes(self):
        """定義 API 端點"""

        @self.app.get("/get_data/{device_type}/{key}")
        async def get_data(device_type: str, key: str):
            try:
                rsp = self.plc_client.read_data(device_type, key)
                print(rsp)
                if not rsp.success:
                    raise HTTPException(
                        status_code=500,
                        detail=f"Failed to reading data from PLC {rsp.message}",
                    )
                if not rsp.value:
                    raise HTTPException(status_code=404, detail="Key not found in PLC")
                return {"key": key, "value": rsp.value}
            except Exception as e:
                raise HTTPException(
                    status_code=500, detail=f"Error reading from PLC: {e}"
                )

        @self.app.get("/get_continuous_data/{device_type}/{start_key}/{count}")
        async def get_continuous_data(device_type: str, start_key: str, count: int):
            try:
                rsp = self.plc_client.read_continuous_data(
                    device_type, start_key, count
                )
                print(rsp)
                if not rsp.success:
                    raise HTTPException(
                        status_code=500,
                        detail=f"Failed to reading continuous data from PLC {rsp.message}",
                    )
                if not rsp.values:
                    raise HTTPException(status_code=404, detail="Key not found in PLC")
                return {"start_key": start_key, "count": count, "values": rsp.values}
            except Exception as e:
                raise HTTPException(
                    status_code=500,
                    detail=f"Error reading continuous data from PLC: {e}",
                )

        @self.app.post("/set_data")
        async def set_data(data: SingleDataInput):
            try:
                rsp = self.plc_client.write_data(data.device_type, data.key, data.value)
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

        @self.app.post("/set_continuous_data")
        async def set_continuous_data(data: ContinuousDataInput):
            try:
                rsp = self.plc_client.write_continuous_data(
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

        @self.app.post("/force_on")
        async def force_on(data: ForceInput):
            try:
                rsp = self.plc_client.force_on(data.device_type, data.key)
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

        @self.app.post("/force_off")
        async def force_off(data: ForceInput):
            try:
                rsp = self.plc_client.force_off(data.device_type, data.key)
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

        @self.app.get("/shutdown")
        async def shutdown():
            """關閉伺服器"""
            os.kill(os.getpid(), signal.SIGINT)
            return {"message": "Server is shutting down..."}

    def run(self):
        """啟動 API 伺服器"""
        uvicorn.run(self.app, host="0.0.0.0", port=8000)


def main():
    server = ApiServer()
    server.run()


if __name__ == "__main__":
    main()
