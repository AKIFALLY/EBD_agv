# machines.py - 射出机管理 API
"""
射出机管理 API 端点
提供射出机配置和 workspace 信息
"""
from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse
from fastapi.templating import Jinja2Templates
from agvcui.db import get_all_machines


def get_router(templates: Jinja2Templates) -> APIRouter:
    router = APIRouter()

    @router.get("/machines")
    async def machines_list(request: Request):
        """
        获取所有射出机配置

        返回包含 workspace_1 和 workspace_2 的完整机器配置列表
        """
        try:
            machines = get_all_machines()

            # 转换为 JSON 友好的格式
            machines_data = []
            for machine in machines:
                # 处理不同的数据类型（dict 或 SQLModel）
                if isinstance(machine, dict):
                    machine_dict = machine
                else:
                    machine_dict = {
                        "id": getattr(machine, 'id', None),
                        "name": getattr(machine, 'name', ''),
                        "enable": getattr(machine, 'enable', 0),
                        "workspace_1": getattr(machine, 'workspace_1', None) or [],
                        "workspace_2": getattr(machine, 'workspace_2', None) or [],
                        "parking_space_1": getattr(machine, 'parking_space_1', None),
                        "parking_space_2": getattr(machine, 'parking_space_2', None),
                        "parking_space_1_status": getattr(machine, 'parking_space_1_status', None),
                        "parking_space_2_status": getattr(machine, 'parking_space_2_status', None),
                        "description": getattr(machine, 'description', None)
                    }
                machines_data.append(machine_dict)

            return JSONResponse({
                "machines": machines_data
            })
        except Exception as e:
            import traceback
            traceback.print_exc()
            return JSONResponse(
                {"error": str(e), "details": traceback.format_exc()},
                status_code=500
            )

    return router
