from fastapi import APIRouter, HTTPException
from typing import List
from opui.db import connection_pool
from db_proxy.models import AGV
from db_proxy.crud.agv_crud import agv_crud
from pydantic import BaseModel

router = APIRouter()


class AGVRequest(BaseModel):
    name: str
    description: str = None
    model: str
    x: float
    y: float
    heading: float
    last_node: int = None
    enable: int = 1


@router.get("/agvs", response_model=List[AGV])
def list_agvs():
    with connection_pool.get_session() as session:
        return agv_crud.get_all(session)


@router.get("/agvs/{agv_id}", response_model=AGV)
def get_agv(agv_id: int):
    with connection_pool.get_session() as session:
        agv = agv_crud.get_by_id(session, agv_id)
        if not agv:
            raise HTTPException(status_code=404, detail="AGV not found")
        return agv


@router.post("/agvs", response_model=AGV)
def create_agv(request: AGVRequest):
    with connection_pool.get_session() as session:
        agv = agv_crud.create(session, request.model_dump())
        return agv


@router.put("/agvs/{agv_id}", response_model=AGV)
def update_agv(agv_id: int, request: AGVRequest):
    with connection_pool.get_session() as session:
        updated = agv_crud.update(session, agv_id, request.model_dump())
        if not updated:
            raise HTTPException(status_code=404, detail="AGV not found")
        return updated


@router.delete("/agvs/{agv_id}")
def delete_agv(agv_id: int):
    with connection_pool.get_session() as session:
        success = agv_crud.delete(session, agv_id)
        if not success:
            raise HTTPException(status_code=404, detail="AGV not found")
        return {"message": "AGV deleted successfully"}
