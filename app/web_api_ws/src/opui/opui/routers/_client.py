# routers/client.py
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional
from opui.db import connection_pool  # ✅ 使用共用的 pool
from db_proxy.models import Client
from datetime import datetime, timezone
import logging
from sqlmodel import Session, select

router = APIRouter()
logger = logging.getLogger(__name__)


class MachineSetting(BaseModel):
    clientId: int
    machineId: int


@router.post("/api/apply-machine-id")
async def apply_machine_id(setting: MachineSetting):
    logger.info(f"收到射出機設定: {setting.machineId}")
    return {"status": "ok", "machineId": setting.machineId}


class ConnectRequest(BaseModel):
    clientId: int
    userAgent: Optional[str] = None


@router.post("/connect")
def connect(req: ConnectRequest):
    if not req.clientId:
        raise HTTPException(status_code=400, detail="clientId is required")

    session: Session = connection_pool.get_session()  # ✅ 改用共用連線池

    try:
        client = session.exec(select(Client)).where(
            Client.client_id == req.clientId).first()

        logger.info(f"client: {client}")
        if not client:
            new_client = Client(
                client_id=req.clientId,
                user_agent=req.userAgent,
                created_at=datetime.now(timezone.utc),
                updated_at=datetime.now(timezone.utc)
            )
            session.add(new_client)
            session.commit()
            session.refresh(new_client)
            return new_client

        client.user_agent = req.userAgent
        client.updated_at = datetime.now(timezone.utc)
        session.commit()
        return client
    except Exception as e:
        session.rollback()
        logger.error(f"Error in connect endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")
    finally:
        session.close()


@router.get("/clients/{client_id}")
def get_client(client_id: int):
    session: Session = connection_pool.get_session()
    try:
        client = session.exec(select(Client)).where(
            Client.client_id == client_id).first()
        if not client:
            raise HTTPException(status_code=404, detail="Client not found")
        return client
    finally:
        session.close()


@router.post("/clients")
def create_client(client: Client):
    session: Session = connection_pool.get_session()
    try:
        session.add(client)
        session.commit()
        session.refresh(client)
        return client
    except Exception as e:
        session.rollback()
        logger.error(f"Error creating client: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")
    finally:
        session.close()


@router.put("/clients/{client_id}")
def update_client(client_id: int, updated_client: Client):
    session: Session = connection_pool.get_session()
    try:
        client = session.exec(select(Client)).where(
            Client.client_id == client_id).first()
        if not client:
            raise HTTPException(status_code=404, detail="Client not found")

        for key, value in updated_client.dict().items():
            setattr(client, key, value)
        client.updated_at = datetime.now(timezone.utc)
        session.commit()
        return client
    except Exception as e:
        session.rollback()
        logger.error(f"Error updating client: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")
    finally:
        session.close()


@router.delete("/clients/{client_id}")
def delete_client(client_id: int):
    session: Session = connection_pool.get_session()
    try:
        client = session.exec(select(Client)).where(
            Client.client_id == client_id).first()
        if not client:
            raise HTTPException(status_code=404, detail="Client not found")

        session.delete(client)
        session.commit()
        return {"message": "Client deleted successfully"}
    except Exception as e:
        session.rollback()
        logger.error(f"Error deleting client: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")
    finally:
        session.close()
