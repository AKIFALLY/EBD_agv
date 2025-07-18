from fastapi import APIRouter, HTTPException
from typing import List
from opui.database.operations import connection_pool
from db_proxy.models import Node
from db_proxy.crud.node_crud import node_crud
from pydantic import BaseModel

router = APIRouter()


class NodeRequest(BaseModel):
    name: str
    description: str = None
    x: float
    y: float


@router.get("/nodes", response_model=List[Node])
def list_nodes():
    with connection_pool.get_session() as session:
        return node_crud.get_all(session)


@router.get("/nodes/{node_id}", response_model=Node)
def get_node(node_id: int):
    with connection_pool.get_session() as session:
        node = node_crud.get_by_id(session, node_id)
        if not node:
            raise HTTPException(status_code=404, detail="Node not found")
        return node


@router.post("/nodes", response_model=Node)
def create_node(request: NodeRequest):
    with connection_pool.get_session() as session:
        node = node_crud.create(session, request.model_dump())
        return node


@router.put("/nodes/{node_id}", response_model=Node)
def update_node(node_id: int, request: NodeRequest):
    with connection_pool.get_session() as session:
        updated = node_crud.update(session, node_id, request.model_dump())
        if not updated:
            raise HTTPException(status_code=404, detail="Node not found")
        return updated


@router.delete("/nodes/{node_id}")
def delete_node(node_id: int):
    with connection_pool.get_session() as session:
        success = node_crud.delete(session, node_id)
        if not success:
            raise HTTPException(status_code=404, detail="Node not found")
        return {"message": "Node deleted successfully"}
