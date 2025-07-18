from fastapi import APIRouter, HTTPException
from typing import List, Dict, Any, Optional
from opui.database.operations import connection_pool
from db_proxy.models import AGV, Task, TaskStatus
from db_proxy.crud.agv_crud import agv_crud
from db_proxy.crud.task_crud import task_crud
from pydantic import BaseModel

from sqlmodel import select

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





class TaskStatusResponse(BaseModel):
    """任務狀態回應模型"""
    success: bool
    message: str
    task_id: Optional[int] = None
    status: Optional[int] = None
    status_name: Optional[str] = None


class TaskQueryResponse(BaseModel):
    """任務查詢回應模型"""
    success: bool
    tasks: List[Dict[str, Any]]
    total: int


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


# ===== 任務監控相關 API (僅供內部監控使用) =====


@router.get("/tasks/{task_id}/status", response_model=TaskStatusResponse)
def get_task_status(task_id: int):
    """
    查詢任務狀態 API

    Args:
        task_id: 任務 ID

    Returns:
        TaskStatusResponse: 任務狀態資訊
    """
    try:
        with connection_pool.get_session() as session:
            task = task_crud.get_by_id(session, task_id)

            if not task:
                raise HTTPException(status_code=404, detail="任務不存在")

            return TaskStatusResponse(
                success=True,
                message="查詢成功",
                task_id=task.id,
                status=task.status_id,
                status_name=TaskStatus.get_name(task.status_id) if task.status_id else "未知"
            )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"查詢任務狀態失敗: {str(e)}")


@router.get("/tasks", response_model=TaskQueryResponse)
def list_tasks(
    status: Optional[int] = None,
    task_type: Optional[str] = None,
    limit: int = 50,
    offset: int = 0
):
    """
    查詢任務列表 API

    Args:
        status: 任務狀態篩選 (可選)
        task_type: 任務類型篩選 (可選)
        limit: 返回數量限制
        offset: 偏移量

    Returns:
        TaskQueryResponse: 任務列表
    """
    try:
        with connection_pool.get_session() as session:
            # 構建查詢條件
            query = select(Task)

            if status is not None:
                query = query.where(Task.status_id == status)

            if task_type:
                query = query.where(Task.parameters["task_type"].as_string() == task_type)

            # 按創建時間降序排列
            query = query.order_by(Task.created_at.desc())

            # 應用分頁
            query = query.offset(offset).limit(limit)

            tasks = session.exec(query).all()

            # 轉換為字典格式
            task_list = []
            for task in tasks:
                task_dict = task.model_dump()
                task_dict['status_name'] = TaskStatus.get_name(task.status_id) if task.status_id else "未知"
                task_list.append(task_dict)

            return TaskQueryResponse(
                success=True,
                tasks=task_list,
                total=len(task_list)
            )

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"查詢任務列表失敗: {str(e)}")
