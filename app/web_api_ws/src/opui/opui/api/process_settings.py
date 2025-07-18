from fastapi import APIRouter, HTTPException
from typing import List
from opui.database.operations import connection_pool
from db_proxy.models import ProcessSettings
from db_proxy.crud.process_settings_crud import process_settings_crud
from pydantic import BaseModel

router = APIRouter(tags=["ProcessSettings"])


class ProcessSettingsRequest(BaseModel):
    soaking_times: int
    description: str = None


@router.get("/process-settings", response_model=List[ProcessSettings])
def list_process_settings():
    with connection_pool.get_session() as session:
        return process_settings_crud.get_all(session)


@router.get("/process-settings/{ps_id}", response_model=ProcessSettings)
def get_process_settings(ps_id: int):
    with connection_pool.get_session() as session:
        ps = process_settings_crud.get_by_id(session, ps_id)
        if not ps:
            raise HTTPException(
                status_code=404, detail="ProcessSettings not found")
        return ps


@router.post("/process-settings", response_model=ProcessSettings)
def create_process_settings(request: ProcessSettingsRequest):
    with connection_pool.get_session() as session:
        new_ps = ProcessSettings()
        new_ps.soaking_times = request.soaking_times
        new_ps.description = request.description
        ps = process_settings_crud.create(session, new_ps)
        return ps


@router.put("/process-settings/{ps_id}", response_model=ProcessSettings)
def update_process_settings(ps_id: int, request: ProcessSettingsRequest):
    with connection_pool.get_session() as session:
        update_ps = ProcessSettings()
        update_ps.soaking_times = request.soaking_times
        update_ps.description = request.description
        update_ps.id = ps_id
        updated = process_settings_crud.update(session, ps_id, update_ps)
        if not updated:
            raise HTTPException(
                status_code=404, detail="ProcessSettings not found")
        return updated


@router.delete("/process-settings/{ps_id}")
def delete_process_settings(ps_id: int):
    with connection_pool.get_session() as session:
        success = process_settings_crud.delete(session, ps_id)
        if not success:
            raise HTTPException(
                status_code=404, detail="ProcessSettings not found")
        return {"message": "ProcessSettings deleted successfully"}
