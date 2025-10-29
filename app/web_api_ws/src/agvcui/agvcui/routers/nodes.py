#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AGVCUI 節點管理路由器
提供節點管理的 Web 界面 API 和頁面渲染
"""

from fastapi import APIRouter, Request, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.templating import Jinja2Templates
import httpx
import logging

logger = logging.getLogger(__name__)

# Web API 服務的內部 URL
# AGVCUI 和 web_api 都在同一個容器內運行，應該使用 localhost
API_BASE_URL = "http://localhost:8000"


def get_router(templates: Jinja2Templates) -> APIRouter:
    """創建並返回節點管理路由器"""
    router = APIRouter(
        prefix="/nodes",
        tags=["nodes"],
    )

    @router.get("/", response_class=HTMLResponse)
    async def nodes_page(request: Request):
        """節點管理頁面"""
        # 從請求中獲取當前用戶（如果有的話）
        from agvcui.middleware import get_current_user_from_request
        current_user = get_current_user_from_request(request)
        
        # 從 Web API 獲取節點狀態
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(f"{API_BASE_URL}/api/nodes/status")
                if response.status_code == 200:
                    node_data = response.json()
                else:
                    node_data = {"nodes": [], "agvs": []}
        except Exception as e:
            logger.error(f"Failed to fetch node status: {e}")
            node_data = {"nodes": [], "agvs": []}
        
        return templates.TemplateResponse(
            "nodes.html",
            {
                "request": request,
                "current_user": current_user,
                "nodes": node_data.get("nodes", []),
                "agvs": node_data.get("agvs", []),
                "page_title": "節點管理"
            }
        )

    @router.get("/api/status")
    async def get_all_node_status():
        """獲取所有節點狀態 (代理到 Web API)"""
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(f"{API_BASE_URL}/api/nodes/status")
                return JSONResponse(content=response.json(), status_code=response.status_code)
        except Exception as e:
            logger.error(f"Failed to fetch node status: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    @router.post("/api/node/{node_name}/start")
    async def start_node(node_name: str):
        """啟動節點 (代理到 Web API)"""
        try:
            # 增加超時時間為 30 秒，因為某些節點啟動需要較長時間
            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.post(f"{API_BASE_URL}/api/nodes/node/{node_name}/start")
                return JSONResponse(content=response.json(), status_code=response.status_code)
        except httpx.TimeoutException:
            logger.error(f"Timeout starting node {node_name}")
            raise HTTPException(status_code=504, detail=f"Timeout starting node {node_name}")
        except Exception as e:
            logger.error(f"Failed to start node {node_name}: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    @router.post("/api/node/{node_name}/stop")
    async def stop_node(node_name: str):
        """停止節點 (代理到 Web API)"""
        try:
            # 增加超時時間為 30 秒
            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.post(f"{API_BASE_URL}/api/nodes/node/{node_name}/stop")
                return JSONResponse(content=response.json(), status_code=response.status_code)
        except httpx.TimeoutException:
            logger.error(f"Timeout stopping node {node_name}")
            raise HTTPException(status_code=504, detail=f"Timeout stopping node {node_name}")
        except Exception as e:
            logger.error(f"Failed to stop node {node_name}: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    @router.post("/api/node/{node_name}/restart")
    async def restart_node(node_name: str):
        """重啟節點 (代理到 Web API)"""
        try:
            # 增加超時時間為 60 秒，因為重啟需要停止和啟動兩個操作
            async with httpx.AsyncClient(timeout=60.0) as client:
                response = await client.post(f"{API_BASE_URL}/api/nodes/node/{node_name}/restart")
                return JSONResponse(content=response.json(), status_code=response.status_code)
        except httpx.TimeoutException:
            logger.error(f"Timeout restarting node {node_name}")
            raise HTTPException(status_code=504, detail=f"Timeout restarting node {node_name}")
        except Exception as e:
            logger.error(f"Failed to restart node {node_name}: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    @router.get("/api/group/{group_name}")
    async def get_group_status(group_name: str):
        """獲取節點群組狀態 (代理到 Web API)"""
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(f"{API_BASE_URL}/api/nodes/group/{group_name}")
                return JSONResponse(content=response.json(), status_code=response.status_code)
        except Exception as e:
            logger.error(f"Failed to fetch group status {group_name}: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    @router.post("/api/group/{group_name}/start")
    async def start_group(group_name: str):
        """啟動節點群組 (代理到 Web API)"""
        try:
            async with httpx.AsyncClient() as client:
                response = await client.post(f"{API_BASE_URL}/api/nodes/group/{group_name}/start")
                return JSONResponse(content=response.json(), status_code=response.status_code)
        except Exception as e:
            logger.error(f"Failed to start group {group_name}: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    @router.post("/api/group/{group_name}/stop")
    async def stop_group(group_name: str):
        """停止節點群組 (代理到 Web API)"""
        try:
            async with httpx.AsyncClient() as client:
                response = await client.post(f"{API_BASE_URL}/api/nodes/group/{group_name}/stop")
                return JSONResponse(content=response.json(), status_code=response.status_code)
        except Exception as e:
            logger.error(f"Failed to stop group {group_name}: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    @router.get("/api/agv/{agv_name}")
    async def get_agv_status(agv_name: str):
        """獲取遠端 AGV 狀態 (代理到 Web API)"""
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(f"{API_BASE_URL}/api/nodes/agv/{agv_name}")
                return JSONResponse(content=response.json(), status_code=response.status_code)
        except Exception as e:
            logger.error(f"Failed to fetch AGV status {agv_name}: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    @router.post("/api/agv/{agv_name}/{action}")
    async def control_agv(agv_name: str, action: str):
        """控制遠端 AGV (代理到 Web API)"""
        if action not in ["start", "stop", "restart"]:
            raise HTTPException(status_code=400, detail=f"Invalid action: {action}")

        try:
            async with httpx.AsyncClient() as client:
                response = await client.post(f"{API_BASE_URL}/api/nodes/agv/{agv_name}/{action}")
                return JSONResponse(content=response.json(), status_code=response.status_code)
        except Exception as e:
            logger.error(f"Failed to {action} AGV {agv_name}: {e}")
            raise HTTPException(status_code=500, detail=str(e))

    return router