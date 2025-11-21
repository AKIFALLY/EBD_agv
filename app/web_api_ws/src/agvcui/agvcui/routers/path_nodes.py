#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AGVCUI 路徑節點管理路由器
提供路徑節點（Path Nodes）和邊（Edges）的 CRUD API
用於 AGV 導航路徑規劃
"""

from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
import logging
from typing import Optional, Dict, Any, List
from pydantic import BaseModel
from datetime import datetime, timezone
import json

logger = logging.getLogger(__name__)


# ========== Pydantic 模型 ==========

class PathNodeCreate(BaseModel):
    """創建路徑節點的請求模型"""
    id: Optional[int] = None  # 允許用戶指定 ID，如果不指定則自動生成
    x: float  # 像素座標 (px)
    y: float  # 像素座標 (px)
    theta: float
    node_type_id: Optional[int] = None
    pgv: Optional[str] = None
    x_mm: Optional[float] = None  # 原始座標 (mm)
    y_mm: Optional[float] = None  # 原始座標 (mm)
    group_1: Optional[Dict[str, Any]] = None
    group_2: Optional[Dict[str, Any]] = None
    group_3: Optional[Dict[str, Any]] = None
    group_4: Optional[Dict[str, Any]] = None
    group_5: Optional[Dict[str, Any]] = None


class PathNodeUpdate(BaseModel):
    """更新路徑節點的請求模型"""
    x: Optional[float] = None  # 像素座標 (px)
    y: Optional[float] = None  # 像素座標 (px)
    theta: Optional[float] = None
    node_type_id: Optional[int] = None
    pgv: Optional[str] = None
    x_mm: Optional[float] = None  # 原始座標 (mm)
    y_mm: Optional[float] = None  # 原始座標 (mm)
    group_1: Optional[Dict[str, Any]] = None
    group_2: Optional[Dict[str, Any]] = None
    group_3: Optional[Dict[str, Any]] = None
    group_4: Optional[Dict[str, Any]] = None
    group_5: Optional[Dict[str, Any]] = None


class LabVIEWImportRequest(BaseModel):
    """LabVIEW 路徑檔匯入請求模型"""
    labview_data: List[Dict[str, Any]]


class LabVIEWImportResponse(BaseModel):
    """LabVIEW 路徑檔匯入回應模型"""
    imported_count: int
    edges_saved: int
    message: str


class NodeDiffItem(BaseModel):
    """節點差異項目"""
    id: int
    x: Optional[float] = None
    y: Optional[float] = None
    theta: Optional[float] = None
    x_mm: Optional[float] = None
    y_mm: Optional[float] = None
    node_type_id: Optional[int] = None
    name: Optional[str] = None
    description: Optional[str] = None
    pgv: Optional[str] = None


class ModifiedNodeDiff(BaseModel):
    """修改節點差異（新舊值對比）"""
    id: int
    old: NodeDiffItem
    new: NodeDiffItem


class LabVIEWPreviewResponse(BaseModel):
    """LabVIEW 匯入預覽回應模型"""
    new_nodes: List[NodeDiffItem]           # 新增的節點
    modified_nodes: List[ModifiedNodeDiff]  # 修改的節點（新舊對比）
    deleted_nodes: List[NodeDiffItem]       # 要刪除的節點
    total_edges: int                        # Edge 總數（會被清除）


class LabVIEWExecuteRequest(BaseModel):
    """LabVIEW 執行匯入請求模型"""
    labview_data: List[Dict[str, Any]]
    selected_operations: Dict[str, List[int]]  # {"add_node_ids": [...], "update_node_ids": [...], "delete_node_ids": [...]}


# ========== KUKA 同步相關模型 ==========

class KukaNodeDiffItem(BaseModel):
    """KUKA 節點差異項目"""
    id: int                              # nodeNumber
    node_uuid: str
    x: float                             # 像素座標
    y: float                             # 像素座標
    x_m: float                           # 原始座標（公尺）
    y_m: float                           # 原始座標（公尺）
    node_type_id: Optional[int] = None
    types: Optional[List[int]] = None    # KUKA 節點類型陣列


class KukaModifiedNodeDiff(BaseModel):
    """KUKA 修改節點差異"""
    id: int
    old: KukaNodeDiffItem
    new: KukaNodeDiffItem


class KukaEdgeDiffItem(BaseModel):
    """KUKA 邊差異項目"""
    from_id: int
    to_id: int
    weight: Optional[float] = 1.0
    edge_type: Optional[int] = 1  # 1=直線, 3=曲線


class KukaSyncPreviewResponse(BaseModel):
    """KUKA 同步預覽回應"""
    new_nodes: List[KukaNodeDiffItem]
    modified_nodes: List[KukaModifiedNodeDiff]
    deleted_nodes: List[KukaNodeDiffItem]
    new_edges: List[KukaEdgeDiffItem]
    deleted_edges: List[KukaEdgeDiffItem]
    missing_locations: List[int]          # 需要創建的 location ID
    kuka_data: Dict[str, Any]             # 完整的 KUKA API 回應資料


class KukaSyncExecuteRequest(BaseModel):
    """KUKA 同步執行請求"""
    kuka_data: Dict[str, Any]             # 完整的 KUKA API 回應
    selected_operations: Dict[str, List[int]]  # 使用者選擇
    create_locations: bool = True         # 是否自動創建 location


class KukaSyncExecuteResponse(BaseModel):
    """KUKA 同步執行回應"""
    nodes_added: int
    nodes_updated: int
    nodes_deleted: int
    edges_added: int
    edges_deleted: int
    locations_created: int
    message: str


def create_path_nodes_router(socket_instance=None) -> APIRouter:
    """創建並返回路徑節點管理路由器

    Args:
        socket_instance: Socket.IO 實例，用於廣播地圖更新事件
    """
    router = APIRouter(
        prefix="/path-nodes",
        tags=["path-nodes"],
    )

    # ========== 路徑節點 API 端點 ==========

    @router.get("/api/path-nodes")
    async def get_all_path_nodes():
        """獲取所有路徑節點

        使用 SQLModel ORM 查詢路徑節點
        """
        try:
            from agvcui.database import connection_pool, node_crud

            with connection_pool.get_session() as session:
                # 使用 CRUD 層查詢（與專案其他部分保持一致）
                nodes = node_crud.get_all(session)

                # 轉換為 dict（Pydantic 自動序列化）
                nodes_data = [node.model_dump() for node in nodes]

                # 處理日期時間格式
                for node_dict in nodes_data:
                    if node_dict.get('created_at'):
                        node_dict['created_at'] = node_dict['created_at'].isoformat()
                    if node_dict.get('updated_at'):
                        node_dict['updated_at'] = node_dict['updated_at'].isoformat()

                logger.info(f"Retrieved {len(nodes_data)} path nodes from database")
                return JSONResponse(content=nodes_data)

        except Exception as e:
            logger.error(f"Failed to fetch path nodes: {e}")
            raise HTTPException(status_code=500, detail=f"Failed to fetch nodes: {str(e)}")

    @router.get("/api/path-nodes/{node_id}")
    async def get_path_node(node_id: int):
        """獲取特定路徑節點

        使用 SQLModel ORM session.get()
        """
        try:
            from agvcui.database import connection_pool
            from db_proxy.models import Node

            with connection_pool.get_session() as session:
                # 使用 SQLModel ORM 直接查詢
                node = session.get(Node, node_id)

                if not node:
                    raise HTTPException(status_code=404, detail=f"Node {node_id} not found")

                # 轉換為 dict
                node_dict = node.model_dump()

                # 處理日期時間格式
                if node_dict.get('created_at'):
                    node_dict['created_at'] = node_dict['created_at'].isoformat()
                if node_dict.get('updated_at'):
                    node_dict['updated_at'] = node_dict['updated_at'].isoformat()

                return JSONResponse(content=node_dict)

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Failed to fetch path node {node_id}: {e}")
            raise HTTPException(status_code=500, detail=f"Failed to fetch node: {str(e)}")

    @router.post("/api/path-nodes")
    async def create_path_node(node_data: PathNodeCreate):
        """創建新的路徑節點

        使用 SQLModel ORM 創建節點
        """
        try:
            from agvcui.database import connection_pool
            from db_proxy.models import Node

            logger.info(f"Creating path node: {node_data.model_dump()}")

            with connection_pool.get_session() as session:
                now = datetime.now(timezone.utc)

                # 檢查是否提供了自定義 ID
                if node_data.id is not None:
                    # 使用 ORM 檢查 ID 是否已存在
                    existing = session.get(Node, node_data.id)
                    if existing:
                        raise HTTPException(status_code=400, detail=f"Node with ID {node_data.id} already exists")

                # 準備節點數據（處理 group 欄位的 JSON 序列化）
                node_dict = node_data.model_dump(exclude_none=True)

                # 處理 group 欄位 - 轉換為 JSON 字串
                for i in range(1, 6):
                    group_key = f'group_{i}'
                    if group_key in node_dict and node_dict[group_key] is not None:
                        node_dict[group_key] = json.dumps(node_dict[group_key], ensure_ascii=False)

                # 設置時間戳
                node_dict['created_at'] = now

                # 使用 SQLModel ORM 創建節點
                node = Node(**node_dict)
                session.add(node)
                session.commit()
                session.refresh(node)

                logger.info(f"Successfully created path node with ID: {node.id}")

                return JSONResponse(content={
                    "id": node.id,
                    "message": f"Node {node.id} created successfully",
                    **node_data.model_dump()
                }, status_code=201)

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Failed to create path node: {e}")
            raise HTTPException(status_code=500, detail=f"Failed to create node: {str(e)}")

    @router.put("/api/path-nodes/{node_id}")
    async def update_path_node(node_id: int, node_data: PathNodeUpdate):
        """更新路徑節點

        使用 SQLModel ORM 更新節點
        """
        try:
            from agvcui.database import connection_pool
            from db_proxy.models import Node

            logger.info(f"Updating path node {node_id}: {node_data.model_dump(exclude_none=True)}")

            # x, y 已經是像素座標 (px)，不需要重新計算
            update_data = node_data.model_dump(exclude_none=True)

            with connection_pool.get_session() as session:
                # 使用 ORM 查詢節點
                node = session.get(Node, node_id)

                if not node:
                    raise HTTPException(status_code=404, detail=f"Node {node_id} not found")

                if not update_data:
                    return JSONResponse(content={
                        "id": node_id,
                        "message": "No fields to update"
                    })

                # 使用 ORM 更新欄位
                for field_name, field_value in update_data.items():
                    # 如果是群組欄位，序列化為 JSON
                    if field_name.startswith('group_') and isinstance(field_value, dict):
                        setattr(node, field_name, json.dumps(field_value, ensure_ascii=False))
                    else:
                        setattr(node, field_name, field_value)

                # 更新時間戳
                node.updated_at = datetime.now(timezone.utc)

                session.add(node)
                session.commit()
                session.refresh(node)

                logger.info(f"Successfully updated path node {node_id}")

                return JSONResponse(content={
                    "id": node_id,
                    "message": f"Node {node_id} updated successfully"
                })

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Failed to update path node {node_id}: {e}")
            raise HTTPException(status_code=500, detail=f"Failed to update node: {str(e)}")

    @router.delete("/api/path-nodes/{node_id}")
    async def delete_path_node(node_id: int):
        """刪除路徑節點

        使用 SQLModel ORM 刪除節點
        """
        try:
            from agvcui.database import connection_pool
            from db_proxy.models import Node

            logger.info(f"Deleting path node {node_id}")

            with connection_pool.get_session() as session:
                # 使用 ORM 查詢節點
                node = session.get(Node, node_id)

                if not node:
                    raise HTTPException(status_code=404, detail=f"Node {node_id} not found")

                # 使用 ORM 刪除節點
                session.delete(node)
                session.commit()

                logger.info(f"Successfully deleted path node {node_id}")

                return JSONResponse(content={
                    "message": f"Node {node_id} deleted successfully"
                })

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Failed to delete path node {node_id}: {e}")
            raise HTTPException(status_code=500, detail=f"Failed to delete node: {str(e)}")

    @router.get("/api/path-edges")
    async def get_all_path_edges():
        """獲取所有路徑邊

        使用 SQLModel ORM 查詢路徑邊
        """
        try:
            from agvcui.database import connection_pool
            from db_proxy.models import Edge
            from sqlalchemy import select

            with connection_pool.get_session() as session:
                # 使用 SQLModel ORM 查詢
                statement = select(Edge).order_by(Edge.id)
                edges = session.scalars(statement).all()

                # 轉換為 dict
                edges_data = []
                for edge in edges:
                    edge_dict = edge.model_dump()

                    # 處理 name 預設值
                    if not edge_dict.get('name'):
                        edge_dict['name'] = f"{edge_dict['from_id']}-{edge_dict['to_id']}"

                    # 處理日期時間格式
                    if edge_dict.get('created_at'):
                        edge_dict['created_at'] = edge_dict['created_at'].isoformat()
                    if edge_dict.get('updated_at'):
                        edge_dict['updated_at'] = edge_dict['updated_at'].isoformat()

                    edges_data.append(edge_dict)

                logger.info(f"Retrieved {len(edges_data)} path edges from database")
                return JSONResponse(content=edges_data)

        except Exception as e:
            logger.error(f"Failed to fetch path edges: {e}")
            raise HTTPException(status_code=500, detail=f"Failed to fetch edges: {str(e)}")

    @router.post("/preview-labview-import")
    async def preview_labview_import(request: LabVIEWImportRequest) -> LabVIEWPreviewResponse:
        """預覽 LabVIEW 路徑匯入差異

        比較 LabVIEW 資料與資料庫現有節點的差異，返回：
        - new_nodes: 新增的節點（LabVIEW 有，資料庫沒有）
        - modified_nodes: 修改的節點（ID 相同但內容不同）
        - deleted_nodes: 刪除的節點（資料庫有，LabVIEW 沒有）
        """
        try:
            from agvcui.database import connection_pool, node_crud, edge_crud
            from db_proxy.models import calculate_pixel_coordinates

            logger.info(f"Starting LabVIEW preview with {len(request.labview_data)} nodes")

            # 查詢資料庫現有節點
            with connection_pool.get_session() as session:
                db_nodes = node_crud.get_all(session)
                db_edges = edge_crud.get_all(session)

            # 建立資料庫節點字典（以 ID 為 key）
            db_nodes_dict = {node.id: node for node in db_nodes}

            # 建立 LabVIEW 節點字典（以 TagNo 為 key）
            labview_nodes_dict = {}
            for labview_node in request.labview_data:
                node_id = int(labview_node.get('TagNo', 0))
                x_mm = float(labview_node.get('Tag_X', 0.0))
                y_mm = float(labview_node.get('Tag_Y', 0.0))
                x, y = calculate_pixel_coordinates(x_mm, y_mm)

                # CT Node 的 node_type_id 通常為 NULL（與資料庫實際情況一致）
                labview_nodes_dict[node_id] = {
                    'id': node_id,
                    'x': x,
                    'y': y,
                    'x_mm': x_mm,
                    'y_mm': y_mm,
                    'theta': 0.0,  # 簡化處理，實際可從 SHIFT 計算
                    'node_type_id': None
                }

            # 分類節點
            new_nodes = []
            modified_nodes = []
            deleted_nodes = []

            # 找出新增和修改的節點
            for node_id, labview_node in labview_nodes_dict.items():
                if node_id not in db_nodes_dict:
                    # 新增的節點
                    new_nodes.append(NodeDiffItem(**labview_node))
                else:
                    # 檢查是否修改
                    db_node = db_nodes_dict[node_id]
                    is_modified = False

                    # 精確比較關鍵欄位（不允許誤差）
                    if (db_node.x != labview_node['x'] or
                        db_node.y != labview_node['y'] or
                        db_node.x_mm != labview_node['x_mm'] or
                        db_node.y_mm != labview_node['y_mm']):
                        is_modified = True

                    if is_modified:
                        old_data = NodeDiffItem(
                            id=db_node.id,
                            x=db_node.x,
                            y=db_node.y,
                            theta=db_node.theta,
                            x_mm=db_node.x_mm,
                            y_mm=db_node.y_mm,
                            node_type_id=db_node.node_type_id,
                            name=db_node.name,
                            description=db_node.description,
                            pgv=db_node.pgv
                        )
                        new_data = NodeDiffItem(**labview_node)
                        modified_nodes.append(ModifiedNodeDiff(id=node_id, old=old_data, new=new_data))

            # 找出要刪除的節點（資料庫有，LabVIEW 沒有）
            for node_id, db_node in db_nodes_dict.items():
                if node_id not in labview_nodes_dict:
                    deleted_nodes.append(NodeDiffItem(
                        id=db_node.id,
                        x=db_node.x,
                        y=db_node.y,
                        theta=db_node.theta,
                        x_mm=db_node.x_mm,
                        y_mm=db_node.y_mm,
                        node_type_id=db_node.node_type_id,
                        name=db_node.name,
                        description=db_node.description,
                        pgv=db_node.pgv
                    ))

            logger.info(f"Preview result: {len(new_nodes)} new, {len(modified_nodes)} modified, {len(deleted_nodes)} deleted")

            return LabVIEWPreviewResponse(
                new_nodes=new_nodes,
                modified_nodes=modified_nodes,
                deleted_nodes=deleted_nodes,
                total_edges=len(db_edges)
            )

        except Exception as e:
            logger.error(f"LabVIEW preview failed: {e}")
            raise HTTPException(status_code=500, detail=f"預覽失敗: {str(e)}")

    @router.post("/import-labview")
    async def import_labview_paths(request: LabVIEWImportRequest) -> LabVIEWImportResponse:
        """LabVIEW 路徑檔匯入

        使用 SQLModel ORM 進行批量匯入
        """
        try:
            from agvcui.database import connection_pool
            # 匯入 ORM 模型
            from sqlalchemy import Row, select, delete
            from db_proxy.models import Edge, Node
            # 匯入座標轉換工具
            from db_proxy.models import calculate_pixel_coordinates

            logger.info(f"Starting LabVIEW import with {len(request.labview_data)} nodes")

            def convert_station_type(station_value):
                """轉換 Station 值到 node type"""
                station_map = {
                    0: "NONE",
                    1: "REST_AREA",      # 休息區
                    2: "CHARGING_AREA",  # 充電區
                    6: "TRANSPORT_POINT" # 搬運區
                }
                return station_map.get(station_value, "NONE")

            def convert_act_mode(act_list):
                """轉換 Act 陣列 index=2 的值到動作模式"""
                try:
                    logger.debug(f"convert_act_mode input: {act_list}, type: {type(act_list)}, len: {len(act_list) if isinstance(act_list, list) else 'N/A'}")
                    if isinstance(act_list, list) and len(act_list) > 2:
                        act_value = act_list[2]  # 取 index=2 的設定
                        logger.debug(f"convert_act_mode act_value: {act_value}")
                        if act_value == 12:
                            return "向量"
                        elif act_value == 16:
                            return "開門"
                        elif act_value == 17:
                            return "關門"
                        else:
                            return str(act_value)  # 其他值直接轉字串
                    logger.debug("convert_act_mode returning default: 0")
                    return "0"  # 預設值
                except (IndexError, TypeError) as e:
                    logger.debug(f"convert_act_mode exception: {e}")
                    return "0"

            def safe_get_index_2(arr, default_value=0.0):
                """安全取得陣列 index=2 的值"""
                try:
                    logger.debug(f"safe_get_index_2 input: {arr}, type: {type(arr)}, len: {len(arr) if isinstance(arr, list) else 'N/A'}")
                    if isinstance(arr, list) and len(arr) > 2:
                        value = float(arr[2])
                        logger.debug(f"safe_get_index_2 result: {value}")
                        return value
                    logger.debug(f"safe_get_index_2 returning default: {default_value}")
                    return default_value
                except (IndexError, TypeError, ValueError) as e:
                    logger.debug(f"safe_get_index_2 exception: {e}")
                    return default_value

            def safe_get_angle_index_2(arr, default_value=0.0):
                """安全取得陣列 index=2 的角度值並乘以 0.1 進行單位轉換"""
                try:
                    logger.debug(f"safe_get_angle_index_2 input: {arr}, type: {type(arr)}, len: {len(arr) if isinstance(arr, list) else 'N/A'}")
                    if isinstance(arr, list) and len(arr) > 2:
                        value = float(arr[2]) * 0.1  # 角度乘以 0.1
                        logger.debug(f"safe_get_angle_index_2 result: {value}")
                        return value
                    logger.debug(f"safe_get_angle_index_2 returning default: {default_value}")
                    return default_value
                except (IndexError, TypeError, ValueError) as e:
                    logger.debug(f"safe_get_angle_index_2 exception: {e}")
                    return default_value

            converted_nodes = []
            now = datetime.now(timezone.utc)

            # 轉換每個 LabVIEW 節點
            for labview_node in request.labview_data:
                try:
                    # 基本字段映射和單位轉換
                    node_id = int(labview_node.get('TagNo', 0))
                    x_mm = float(labview_node.get('Tag_X', 0.0)) * 1  # 原始座標 (mm)
                    y_mm = float(labview_node.get('Tag_Y', 0.0)) * 1  # 原始座標 (mm)
                    station = int(labview_node.get('Station', 0))
                    node_type = convert_station_type(station)

                    # 計算像素座標 (px)
                    x, y = calculate_pixel_coordinates(x_mm, y_mm)
                    logger.debug(f"Node {node_id}: original ({x_mm}, {y_mm}) mm -> pixel ({x}, {y}) px")

                    # 提取 CanToMoveSet (現在是陣列格式)
                    can_move_set_array = labview_node.get('CanToMoveSet', [])
                    logger.debug(f"CanToMoveSet array: {can_move_set_array}")

                    # 提取所有有效的 CanToMoveSet 項目（CanToMoveTag != 0）
                    valid_move_sets = []
                    if isinstance(can_move_set_array, list) and len(can_move_set_array) > 0:
                        for move_item in can_move_set_array:
                            if isinstance(move_item, dict) and move_item.get('CanToMoveTag', 0) != 0:
                                valid_move_sets.append(move_item)

                    logger.debug(f"Valid move sets: {len(valid_move_sets)} items")

                    # 構建多個 group 資料（最多 5 個）
                    group_data_list = []
                    default_shift_angle = 0.0  # 預設角度值

                    if valid_move_sets:
                        # 處理每個有效的移動設定
                        for i, move_set in enumerate(valid_move_sets[:5]):  # 最多處理 5 個
                            can_to_move_tag = move_set.get('CanToMoveTag', 0)
                            act_mode = convert_act_mode(move_set.get('Act', []))
                            speed = safe_get_index_2(move_set.get('Speed', []), 0.0)
                            shift_angle = safe_get_angle_index_2(move_set.get('SHIFT', []), 0.0)  # 使用角度轉換函數
                            safe_sensor = safe_get_index_2(move_set.get('SafeSensorSetting', []), 0.0)
                            pgv_value = move_set.get('PGV', 0)
                            weight_value = move_set.get('加權', 0)

                            group_data = {
                                "可移動點": can_to_move_tag,
                                "動作模式": act_mode,
                                "速度設定": speed,
                                "向量角度": shift_angle,
                                "區域防護": safe_sensor,
                                "PGV": pgv_value,
                                "加權": weight_value
                            }

                            group_data_list.append(group_data)

                            # 使用第一個有效項目的 SHIFT 角度作為節點角度
                            if i == 0:
                                default_shift_angle = shift_angle

                    else:
                        # 備用：從頂層取值（舊格式相容性）
                        act_mode = convert_act_mode(labview_node.get('Act', []))
                        speed = safe_get_index_2(labview_node.get('Speed', []), 0.0)
                        shift_angle = safe_get_angle_index_2(labview_node.get('SHIFT', []), 0.0)  # 使用角度轉換函數
                        safe_sensor = safe_get_index_2(labview_node.get('SafeSensorSetting', []), 0.0)

                        group_data = {
                            "可移動點": 0,
                            "動作模式": act_mode,
                            "速度設定": speed,
                            "向量角度": shift_angle,
                            "區域防護": safe_sensor,
                            "PGV": 0,
                            "加權": 0
                        }

                        group_data_list.append(group_data)
                        default_shift_angle = shift_angle

                    logger.debug(f"Created {len(group_data_list)} group data entries for node {node_id}")

                    converted_node = {
                        'id': node_id,
                        'x': x,          # 像素座標 (px)
                        'y': y,          # 像素座標 (px)
                        'x_mm': x_mm,    # 原始座標 (mm)
                        'y_mm': y_mm,    # 原始座標 (mm)
                        'theta': default_shift_angle,  # 使用第一個有效項目的 SHIFT 角度
                        'node_type_id': node_type,
                        'pgv': None,     # 預設值
                        'created_at': now
                    }

                    # 將 group 資料分配到不同的 group 欄位
                    for i, group_data in enumerate(group_data_list[:5]):  # 最多 5 個 group 欄位
                        converted_node[f'group_{i+1}'] = group_data

                    converted_nodes.append(converted_node)
                    logger.debug(f"Converted node {node_id}: {converted_node}")

                except Exception as e:
                    logger.warning(f"Failed to convert LabVIEW node: {labview_node}. Error: {e}")
                    continue

            if not converted_nodes:
                raise HTTPException(status_code=400, detail="No valid nodes found in LabVIEW data")

            # 三階段批量匯入到資料庫（在同一個 transaction 中完成）
            imported_count = 0
            edges_saved = 0
            deleted_nodes = 0
            deleted_edges = 0

            with connection_pool.get_session() as session:
                # === 第零階段：清除舊的 CT Node 和 Edge 資料 ===
                logger.info("清除舊的 CT Node 和 Edge 資料...")
                deleted_edges = session.exec(delete(Edge)).rowcount
                deleted_nodes = session.exec(delete(Node)).rowcount
                logger.info(f"已標記刪除 {deleted_nodes} 個節點, {deleted_edges} 個邊")

                # === 第一階段：處理所有節點 ===
                for node in converted_nodes:
                    try:
                        # 使用 SQLModel ORM merge 實現 UPSERT
                        # 動態建構 node 資料
                        node_params = {
                            'id': node['id'],
                            'x': node['x'],      # 像素座標 (px)
                            'y': node['y'],      # 像素座標 (px)
                            'x_mm': node['x_mm'],  # 原始座標 (mm)
                            'y_mm': node['y_mm'],  # 原始座標 (mm)
                            'theta': node['theta'],
                            'node_type_id': node['node_type_id'],
                            'pgv': node['pgv'],
                            'created_at': node['created_at'],
                            'updated_at': now
                        }

                        # 填充所有 group 欄位（動態支援多個 group）
                        for i in range(1, 6):  # group_1 到 group_5
                            group_key = f'group_{i}'
                            if node.get(group_key):
                                node_params[group_key] = json.dumps(node[group_key], ensure_ascii=False)
                            else:
                                node_params[group_key] = None

                        # 使用 ORM merge 實現 INSERT or UPDATE
                        node_obj = Node(**node_params)
                        session.merge(node_obj)

                        imported_count += 1

                    except Exception as e:
                        logger.error(f"Failed to upsert node {node['id']}: {e}")
                        continue

                logger.info(f"第一階段完成：匯入了 {imported_count} 個節點")

                # 提交第一階段的節點（確保節點先寫入 DB，避免第二階段的外鍵約束錯誤）
                session.commit()
                logger.info("✅ 第一階段已提交：所有節點已寫入資料庫")

                # === 第二階段：處理所有邊 ===
                for labview_node in request.labview_data:
                    try:
                        tag_no = int(labview_node.get('TagNo', 0))
                        can_move_set_array = labview_node.get('CanToMoveSet', [])

                        # 從新格式的 CanToMoveSet 陣列中提取所有有效的邊
                        if isinstance(can_move_set_array, list):
                            for move_item in can_move_set_array:
                                if not isinstance(move_item, dict):
                                    continue

                                can_to_move_tag = move_item.get('CanToMoveTag', 0)
                                if can_to_move_tag and can_to_move_tag != 0:  # 忽略無效來源
                                    from_tag = int(can_to_move_tag)
                                    edge_name = f"{from_tag}-{tag_no}"

                                    # 檢查邊是否已存在
                                    result = session.exec(select(Edge).where(Edge.name == edge_name)).first()
                                    if isinstance(result, Row):
                                        existing_edge = result[0]
                                    else:
                                        existing_edge = result

                                    if existing_edge:
                                        # 更新現有邊
                                        existing_edge.from_id = from_tag
                                        existing_edge.to_id = tag_no
                                        existing_edge.updated_at = now
                                    else:
                                        # 創建新邊
                                        new_edge = Edge(
                                            from_id=from_tag,
                                            to_id=tag_no,
                                            name=edge_name,
                                            weight=1.0,  # 預設權重
                                            created_at=now,
                                            updated_at=now
                                        )
                                        session.add(new_edge)

                                    edges_saved += 1
                                    logger.debug(f"處理邊: {edge_name} ({from_tag} -> {tag_no})")

                    except Exception as e:
                        logger.warning(f"Failed to process edge for node {labview_node.get('TagNo', 'unknown')}: {e}")
                        continue

                logger.info(f"第二階段完成：處理了 {edges_saved} 個邊（尚未提交）")

                # === 最終提交：一次性提交所有變更（刪除 + 節點 + 邊） ===
                logger.info(f"開始最終提交：刪除 {deleted_nodes} 個節點和 {deleted_edges} 個邊，新增 {imported_count} 個節點和 {edges_saved} 個邊")
                session.commit()
                logger.info("✅ 資料庫事務提交完成，所有變更已持久化")

            logger.info(f"Successfully imported {imported_count} nodes from LabVIEW")

            # 廣播地圖更新事件給所有連接的客戶端
            if socket_instance:
                try:
                    from agvcui.database import node_all, edge_all, kuka_node_all, kuka_edge_all, get_all_agvs
                    from fastapi.encoders import jsonable_encoder
                    import asyncio

                    nodes = node_all()
                    edges = edge_all()
                    kuka_nodes = kuka_node_all()
                    kuka_edges = kuka_edge_all()
                    agvs = get_all_agvs()

                    payload = {
                        "nodes": nodes,
                        "edges": edges,
                        "kukaNodes": kuka_nodes,
                        "kukaEdges": kuka_edges,
                        "agvs": agvs
                    }

                    # 使用 asyncio 執行異步廣播
                    asyncio.create_task(
                        socket_instance.emit("map_info", jsonable_encoder(payload))
                    )
                    logger.info("✅ 已廣播地圖更新事件給所有客戶端")
                except Exception as e:
                    logger.warning(f"Failed to broadcast map update: {e}")

            return LabVIEWImportResponse(
                imported_count=imported_count,
                edges_saved=edges_saved,
                message=f"成功匯入 {imported_count} 個節點和 {edges_saved} 個邊"
            )

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"LabVIEW import failed: {e}")
            raise HTTPException(status_code=500, detail=f"匯入失敗: {str(e)}")

    @router.post("/execute-labview-import")
    async def execute_labview_import(request: LabVIEWExecuteRequest) -> LabVIEWImportResponse:
        """執行 LabVIEW 路徑匯入（根據用戶選擇）

        根據用戶在預覽界面的選擇執行匯入操作：
        - 刪除選中的節點
        - 新增選中的節點
        - 更新選中的節點
        - 重建所有 Edge（全部清除後重建）
        """
        try:
            from agvcui.database import connection_pool
            from sqlalchemy import delete
            from db_proxy.models import Edge, Node, calculate_pixel_coordinates
            import asyncio
            from fastapi.encoders import jsonable_encoder

            logger.info("Starting LabVIEW execute import")

            selected_ops = request.selected_operations
            add_ids = set(selected_ops.get('add_node_ids', []))
            update_ids = set(selected_ops.get('update_node_ids', []))
            delete_ids = set(selected_ops.get('delete_node_ids', []))

            logger.info(f"Operations: add={len(add_ids)}, update={len(update_ids)}, delete={len(delete_ids)}")

            # 建立 LabVIEW 節點字典
            labview_nodes_dict = {}
            for labview_node in request.labview_data:
                node_id = int(labview_node.get('TagNo', 0))
                labview_nodes_dict[node_id] = labview_node

            imported_count = 0
            edges_saved = 0
            deleted_count = 0
            deleted_edges_count = 0

            with connection_pool.get_session() as session:
                # 階段 1：刪除選中的節點及其相關 Edge
                if delete_ids:
                    logger.info(f"嘗試刪除 {len(delete_ids)} 個節點及其相關 Edge...")
                    skipped_nodes = []

                    for node_id in delete_ids:
                        try:
                            # 先刪除涉及此節點的所有 Edge（from_id 或 to_id）
                            edges_deleted = session.exec(
                                delete(Edge).where(
                                    (Edge.from_id == node_id) | (Edge.to_id == node_id)
                                )
                            ).rowcount
                            deleted_edges_count += edges_deleted

                            # 再刪除節點本身
                            session.exec(delete(Node).where(Node.id == node_id))
                            session.commit()
                            deleted_count += 1
                            logger.debug(f"已刪除節點 {node_id} 及其 {edges_deleted} 條 Edge")
                        except Exception as e:
                            # 外鍵約束錯誤，跳過此節點
                            error_msg = str(e)
                            if 'ForeignKeyViolation' in error_msg or 'foreign key' in error_msg.lower():
                                skipped_nodes.append(node_id)
                                session.rollback()
                                logger.warning(f"節點 {node_id} 被其他表引用（machine/task），跳過刪除")
                            else:
                                session.rollback()
                                raise

                    logger.info(f"✅ 已刪除 {deleted_count} 個節點和 {deleted_edges_count} 條相關 Edge，跳過 {len(skipped_nodes)} 個節點（有外鍵引用）")

                # 階段 2：新增和更新選中的節點
                now = datetime.now(timezone.utc)
                for labview_node in request.labview_data:
                    node_id = int(labview_node.get('TagNo', 0))

                    # 跳過未選中的節點
                    if node_id not in add_ids and node_id not in update_ids:
                        continue

                    # 計算座標
                    x_mm = float(labview_node.get('Tag_X', 0.0))
                    y_mm = float(labview_node.get('Tag_Y', 0.0))
                    x, y = calculate_pixel_coordinates(x_mm, y_mm)

                    # 使用 merge 實現 UPSERT
                    # ⚠️ node_type_id 設為 None，因為 LabVIEW 的 Station 值不對應資料庫中的 node_type ID
                    node_obj = Node(
                        id=node_id,
                        x=x,
                        y=y,
                        x_mm=x_mm,
                        y_mm=y_mm,
                        theta=0.0,
                        node_type_id=None,
                        created_at=now,
                        updated_at=now
                    )
                    session.merge(node_obj)
                    imported_count += 1

                session.commit()
                logger.info(f"✅ 已新增/更新 {imported_count} 個節點")

                # 階段 3：智能更新 LabVIEW 檔案涉及節點的 Edge
                # 收集 LabVIEW 檔案中的所有節點 ID（排除已刪除的）
                labview_node_ids = set()
                for labview_node in request.labview_data:
                    node_id = int(labview_node.get('TagNo', 0))
                    if node_id not in delete_ids:
                        labview_node_ids.add(node_id)

                logger.info(f"LabVIEW 檔案包含 {len(labview_node_ids)} 個節點，準備更新其 Edge...")

                # 先刪除這些節點相關的舊 Edge（from_id 或 to_id 在 LabVIEW 節點中）
                if labview_node_ids:
                    from sqlalchemy import or_
                    old_edges_deleted = session.exec(
                        delete(Edge).where(
                            or_(
                                Edge.from_id.in_(labview_node_ids),
                                Edge.to_id.in_(labview_node_ids)
                            )
                        )
                    ).rowcount
                    logger.info(f"已刪除 {old_edges_deleted} 條舊 Edge（LabVIEW 節點相關）")

                # 重建 LabVIEW 檔案中節點的新 Edge
                for labview_node in request.labview_data:
                    tag_no = int(labview_node.get('TagNo', 0))

                    # 跳過已刪除的節點
                    if tag_no in delete_ids:
                        continue

                    can_move_set_array = labview_node.get('CanToMoveSet', [])
                    if isinstance(can_move_set_array, list):
                        for move_item in can_move_set_array:
                            if not isinstance(move_item, dict):
                                continue

                            can_to_move_tag = move_item.get('CanToMoveTag', 0)
                            if can_to_move_tag and can_to_move_tag != 0:
                                from_tag = int(can_to_move_tag)
                                # 確保 from 節點也未被刪除
                                if from_tag in delete_ids:
                                    continue

                                edge_name = f"{from_tag}-{tag_no}"
                                new_edge = Edge(
                                    from_id=from_tag,
                                    to_id=tag_no,
                                    name=edge_name,
                                    weight=1.0,
                                    created_at=now,
                                    updated_at=now
                                )
                                session.add(new_edge)
                                edges_saved += 1

                session.commit()
                logger.info(f"✅ 已重建 {edges_saved} 個新 Edge（LabVIEW 節點），保留了其他節點的 Edge")

            # 廣播地圖更新
            if socket_instance:
                try:
                    from agvcui.database import node_all, edge_all, kuka_node_all, kuka_edge_all, get_all_agvs

                    nodes = node_all()
                    edges = edge_all()
                    kuka_nodes = kuka_node_all()
                    kuka_edges = kuka_edge_all()
                    agvs = get_all_agvs()

                    payload = {
                        "nodes": nodes,
                        "edges": edges,
                        "kukaNodes": kuka_nodes,
                        "kukaEdges": kuka_edges,
                        "agvs": agvs
                    }

                    asyncio.create_task(
                        socket_instance.emit("map_info", jsonable_encoder(payload))
                    )
                    logger.info("✅ 已廣播地圖更新事件")
                except Exception as e:
                    logger.warning(f"Failed to broadcast map update: {e}")

            return LabVIEWImportResponse(
                imported_count=imported_count,
                edges_saved=edges_saved,
                message=f"成功處理 {imported_count} 個節點（刪除 {deleted_count} 個節點及 {deleted_edges_count} 條 Edge），重建 {edges_saved} 條新 Edge"
            )

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"LabVIEW execute import failed: {e}")
            raise HTTPException(status_code=500, detail=f"執行匯入失敗: {str(e)}")

    # ========== KUKA 同步 API 端點 ==========

    @router.post("/api/kuka-sync/preview", response_model=KukaSyncPreviewResponse)
    async def preview_kuka_sync():
        """預覽 KUKA 地圖同步差異

        流程：
        1. 從 KUKA Fleet API 獲取地圖數據
        2. 比對資料庫中的 kuka_node 和 kuka_edge
        3. 檢測新增、修改、刪除的節點和邊
        4. 檢查 location 表，找出缺失的記錄
        5. 返回差異報告
        """
        try:
            # 1. 調用 KUKA Fleet API
            import sys
            sys.path.append('/app/kuka_fleet_ws/install/kuka_fleet_adapter/lib/python3.12/site-packages')
            from kuka_fleet_adapter.kuka_api_client import KukaApiClient

            # 從配置讀取地圖資訊（目前使用硬編碼，之後可改為從配置檔讀取）
            map_code = "AlanACT"
            floor_number = "AlanSec1"

            try:
                client = KukaApiClient(
                    base_url='http://192.168.10.3:10870',
                    username='admin',
                    password='Admin'
                )
            except Exception as e:
                logger.error(f"Failed to create KUKA API client: {e}")
                raise HTTPException(status_code=503, detail=f"無法連接 KUKA Fleet API: {str(e)}")

            # 獲取地圖數據
            kuka_response = client.get_map_floor(map_code, floor_number)

            if not kuka_response.get('success'):
                raise HTTPException(status_code=500, detail=f"KUKA API 錯誤: {kuka_response.get('message', '未知錯誤')}")

            kuka_data = kuka_response.get('data', {})
            kuka_nodes_raw = kuka_data.get('nodes', [])
            kuka_edges_raw = kuka_data.get('edges', [])

            # 2. 建立 KUKA ID → nodeNumber 映射
            id_to_node_number = {
                node['id']: node['nodeNumber']
                for node in kuka_nodes_raw
            }

            # 3. 轉換 KUKA 節點為標準格式
            kuka_nodes_dict = {}
            for node in kuka_nodes_raw:
                node_number = node['nodeNumber']
                x_m = node['xCoordinate']
                y_m = node['yCoordinate']
                # 座標轉換：m → px（使用與現有系統一致的轉換公式）
                x_px = x_m * 1000 / 12.5
                y_px = y_m * 1000 / 12.5

                kuka_nodes_dict[node_number] = {
                    'id': node_number,
                    'node_uuid': node['nodeUuid'],
                    'x': x_px,
                    'y': y_px,
                    'x_m': x_m,
                    'y_m': y_m,
                    'node_type_id': node.get('types', [None])[0] if node.get('types') else node.get('nodeType'),
                    'types': node.get('types', [])
                }

            # 4. 查詢資料庫現有節點
            from agvcui.database import connection_pool
            from db_proxy.models.agvc_kuka import KukaNode, KukaEdge
            from db_proxy.models.agvc_location import Location

            with connection_pool.get_session() as session:
                # 獲取現有 KUKA 節點
                from sqlmodel import select
                db_kuka_nodes = session.exec(select(KukaNode)).all()
                db_nodes_dict = {node.id: node for node in db_kuka_nodes}

                # 5. 比對差異 - 節點
                new_nodes = []
                modified_nodes = []
                deleted_nodes = []

                # 新增和修改的節點
                for node_id, kuka_node in kuka_nodes_dict.items():
                    if node_id not in db_nodes_dict:
                        # 新增的節點
                        new_nodes.append(KukaNodeDiffItem(**kuka_node))
                    else:
                        # 檢查是否修改
                        db_node = db_nodes_dict[node_id]
                        is_modified = False

                        # 精確比較座標（誤差容限 0.01 像素）
                        if (abs(db_node.x - kuka_node['x']) > 0.01 or
                            abs(db_node.y - kuka_node['y']) > 0.01 or
                            db_node.uuid != kuka_node['node_uuid']):
                            is_modified = True

                        if is_modified:
                            old_item = KukaNodeDiffItem(
                                id=db_node.id,
                                node_uuid=db_node.uuid or "",
                                x=db_node.x,
                                y=db_node.y,
                                x_m=db_node.x / 1000 * 12.5,  # 反向轉換
                                y_m=db_node.y / 1000 * 12.5,
                                node_type_id=db_node.node_type_id,
                                types=[]
                            )
                            new_item = KukaNodeDiffItem(**kuka_node)
                            modified_nodes.append(
                                KukaModifiedNodeDiff(id=node_id, old=old_item, new=new_item)
                            )

                # 刪除的節點（資料庫有，KUKA 沒有）
                for node_id, db_node in db_nodes_dict.items():
                    if node_id not in kuka_nodes_dict:
                        deleted_nodes.append(KukaNodeDiffItem(
                            id=db_node.id,
                            node_uuid=db_node.uuid or "",
                            x=db_node.x,
                            y=db_node.y,
                            x_m=db_node.x / 1000 * 12.5,
                            y_m=db_node.y / 1000 * 12.5,
                            node_type_id=db_node.node_type_id,
                            types=[]
                        ))

                # 6. 處理邊的差異（忽略 edgeType 以避免誤判）
                kuka_edges_set = set()
                kuka_edges_dict = {}  # 用於保存 edgeType 資訊
                for edge in kuka_edges_raw:
                    begin_id = edge.get('beginNodeId')
                    end_id = edge.get('endNodeId')

                    # 將 KUKA 內部 ID 映射到 nodeNumber
                    if begin_id in id_to_node_number and end_id in id_to_node_number:
                        from_id = id_to_node_number[begin_id]
                        to_id = id_to_node_number[end_id]
                        kuka_edges_set.add((from_id, to_id))  # 移除 edgeType 比較
                        kuka_edges_dict[(from_id, to_id)] = edge.get('edgeType', 1)

                # 獲取資料庫現有邊
                db_kuka_edges = session.exec(select(KukaEdge)).all()
                db_edges_set = set((e.from_id, e.to_id) for e in db_kuka_edges)  # 移除硬編碼 edgeType

                # 計算新增和刪除的邊
                new_edges = [
                    KukaEdgeDiffItem(
                        from_id=f,
                        to_id=t,
                        weight=1.0,
                        edge_type=kuka_edges_dict.get((f, t), 1)
                    )
                    for f, t in (kuka_edges_set - db_edges_set)
                ]
                deleted_edges = [
                    KukaEdgeDiffItem(from_id=f, to_id=t, weight=1.0, edge_type=1)
                    for f, t in (db_edges_set - kuka_edges_set)
                ]

                # 7. 檢查 location 表（優化：只查詢 IDs，不載入整張表）
                existing_location_ids = set(session.exec(select(Location.id)).all())
                missing_locations = [
                    node_id for node_id in kuka_nodes_dict.keys()
                    if node_id not in existing_location_ids
                ]
                logger.info(f"📊 Location 檢查: 缺失 {len(missing_locations)} 個 location")

            logger.info(f"KUKA Sync Preview: {len(new_nodes)} new, {len(modified_nodes)} modified, {len(deleted_nodes)} deleted nodes")
            logger.info(f"KUKA Sync Preview: {len(new_edges)} new, {len(deleted_edges)} deleted edges")
            logger.info(f"KUKA Sync Preview: {len(missing_locations)} missing locations")

            return KukaSyncPreviewResponse(
                new_nodes=new_nodes,
                modified_nodes=modified_nodes,
                deleted_nodes=deleted_nodes,
                new_edges=new_edges,
                deleted_edges=deleted_edges,
                missing_locations=missing_locations,
                kuka_data=kuka_response  # 返回完整 KUKA API 回應，供 Execute 使用
            )

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"KUKA sync preview failed: {e}", exc_info=True)
            raise HTTPException(status_code=500, detail=f"預覽同步失敗: {str(e)}")

    @router.post("/api/kuka-sync/execute", response_model=KukaSyncExecuteResponse)
    async def execute_kuka_sync(request: KukaSyncExecuteRequest):
        """執行 KUKA 地圖同步（根據使用者選擇）

        流程：
        1. 刪除選中的節點和邊
        2. 新增/更新選中的節點
        3. 新增選中的邊
        4. 自動創建缺失的 location 記錄
        5. 廣播地圖更新事件
        """
        try:
            from agvcui.database import connection_pool
            from db_proxy.models.agvc_kuka import KukaNode, KukaEdge
            from db_proxy.models.agvc_location import Location, LocationStatus
            from sqlmodel import select, delete
            from datetime import datetime, timezone

            # 解析 KUKA 數據（處理兩種可能的結構）
            if 'data' in request.kuka_data:
                # 完整的 KUKA API 回應 {success, message, data: {nodes, edges}}
                kuka_data = request.kuka_data['data']
                logger.info("📊 Execute API - 數據結構: 完整 KUKA API 回應")
            else:
                # 直接傳遞的數據對象 {nodes, edges}
                kuka_data = request.kuka_data
                logger.info("📊 Execute API - 數據結構: 直接數據對象")

            kuka_nodes_raw = kuka_data.get('nodes', [])
            kuka_edges_raw = kuka_data.get('edges', [])

            logger.info(f"📊 Execute API - 收到數據: 節點數={len(kuka_nodes_raw)}, 邊數={len(kuka_edges_raw)}")

            if not kuka_nodes_raw:
                logger.warning("⚠️ Execute API - 警告：沒有收到任何節點資料！")
            if not kuka_edges_raw:
                logger.warning("⚠️ Execute API - 警告：沒有收到任何邊資料！")

            # 建立 KUKA ID → nodeNumber 映射
            id_to_node_number = {
                node['id']: node['nodeNumber']
                for node in kuka_nodes_raw
            }

            # 轉換 KUKA 節點為字典（以 nodeNumber 為 key）
            kuka_nodes_dict = {}
            for node in kuka_nodes_raw:
                node_number = node['nodeNumber']
                x_m = node['xCoordinate']
                y_m = node['yCoordinate']
                x_px = x_m * 1000 / 12.5
                y_px = y_m * 1000 / 12.5

                kuka_nodes_dict[node_number] = {
                    'id': node_number,
                    'node_uuid': node['nodeUuid'],
                    'x': x_px,
                    'y': y_px,
                    'node_type_id': node.get('types', [None])[0] if node.get('types') else node.get('nodeType')
                }

            # 解析使用者選擇
            add_node_ids = set(request.selected_operations.get('add_node_ids', []))
            update_node_ids = set(request.selected_operations.get('update_node_ids', []))
            delete_node_ids = set(request.selected_operations.get('delete_node_ids', []))

            # 統計
            nodes_added = 0
            nodes_updated = 0
            nodes_deleted = 0
            edges_added = 0
            edges_deleted = 0
            locations_created = 0

            with connection_pool.get_session() as session:
                now = datetime.now(timezone.utc)

                # ===== 階段 1: 刪除選中的節點和相關邊 =====
                if delete_node_ids:
                    logger.info(f"Deleting {len(delete_node_ids)} KUKA nodes...")

                    for node_id in delete_node_ids:
                        # 先刪除相關的邊
                        edges_deleted += session.exec(
                            delete(KukaEdge).where(
                                (KukaEdge.from_id == node_id) | (KukaEdge.to_id == node_id)
                            )
                        ).rowcount

                        # 再刪除節點
                        session.exec(delete(KukaNode).where(KukaNode.id == node_id))
                        nodes_deleted += 1

                    session.commit()
                    logger.info(f"✅ Deleted {nodes_deleted} nodes and {edges_deleted} edges")

                # ===== 階段 2: 新增/更新選中的節點 =====
                for node_number in kuka_nodes_raw:
                    node_id = node_number['nodeNumber']

                    # 跳過未選擇的節點
                    if node_id not in add_node_ids and node_id not in update_node_ids:
                        continue

                    # 建立/更新節點
                    kuka_node_data = kuka_nodes_dict[node_id]
                    kuka_node = KukaNode(
                        id=kuka_node_data['id'],
                        uuid=kuka_node_data['node_uuid'],
                        x=kuka_node_data['x'],
                        y=kuka_node_data['y'],
                        node_type_id=kuka_node_data['node_type_id'],
                        created_at=now,
                        updated_at=now
                    )

                    # 使用 merge 實現 UPSERT
                    session.merge(kuka_node)

                    if node_id in add_node_ids:
                        nodes_added += 1
                    if node_id in update_node_ids:
                        nodes_updated += 1

                session.commit()
                logger.info(f"✅ Added {nodes_added} nodes, Updated {nodes_updated} nodes")

                # ===== 階段 3: 重建邊 =====
                # 先刪除所有涉及更新節點的邊
                updated_node_ids = add_node_ids | update_node_ids
                if updated_node_ids:
                    from sqlmodel import or_
                    old_edges_deleted = session.exec(
                        delete(KukaEdge).where(
                            or_(
                                KukaEdge.from_id.in_(updated_node_ids),
                                KukaEdge.to_id.in_(updated_node_ids)
                            )
                        )
                    ).rowcount
                    logger.info(f"Deleted {old_edges_deleted} old edges for updated nodes")

                # 重新創建邊
                for edge in kuka_edges_raw:
                    begin_id = edge.get('beginNodeId')
                    end_id = edge.get('endNodeId')

                    # 映射到 nodeNumber
                    if begin_id in id_to_node_number and end_id in id_to_node_number:
                        from_id = id_to_node_number[begin_id]
                        to_id = id_to_node_number[end_id]

                        # 只處理涉及更新節點的邊
                        if from_id in updated_node_ids or to_id in updated_node_ids:
                            new_edge = KukaEdge(
                                from_id=from_id,
                                to_id=to_id,
                                name=f"{from_id}-{to_id}",  # 參考 LabVIEW Import 的命名格式
                                weight=1.0,
                                created_at=now,
                                updated_at=now
                            )
                            session.add(new_edge)
                            edges_added += 1

                session.commit()
                logger.info(f"✅ Added {edges_added} new edges")

                # ===== 階段 4: 創建缺失的 Location =====
                if request.create_locations:
                    # 檢查哪些節點缺少 location
                    locations = session.exec(select(Location)).all()
                    existing_location_ids = set(loc.id for loc in locations)

                    for node_id in add_node_ids:
                        if node_id not in existing_location_ids:
                            new_location = Location(
                                id=node_id,
                                node_id=node_id,
                                name=f"KUKA_{node_id}",
                                type="kuka_auto",
                                location_status_id=LocationStatus.UNOCCUPIED,  # UNOCCUPIED 已經是 int (值為 2)
                                created_at=now,
                                updated_at=now
                            )
                            session.add(new_location)
                            locations_created += 1

                    session.commit()
                    logger.info(f"✅ Created {locations_created} locations")

            # 廣播地圖更新
            if socket_instance:
                try:
                    from agvcui.database import node_all, edge_all, kuka_node_all, kuka_edge_all, get_all_agvs
                    from fastapi.encoders import jsonable_encoder
                    import asyncio

                    nodes = node_all()
                    edges = edge_all()
                    kuka_nodes = kuka_node_all()
                    kuka_edges = kuka_edge_all()
                    agvs = get_all_agvs()

                    payload = {
                        "nodes": nodes,
                        "edges": edges,
                        "kukaNodes": kuka_nodes,
                        "kukaEdges": kuka_edges,
                        "agvs": agvs
                    }

                    asyncio.create_task(
                        socket_instance.emit("map_info", jsonable_encoder(payload))
                    )
                    logger.info("✅ 已廣播地圖更新事件")
                except Exception as e:
                    logger.warning(f"Failed to broadcast map update: {e}")

            return KukaSyncExecuteResponse(
                nodes_added=nodes_added,
                nodes_updated=nodes_updated,
                nodes_deleted=nodes_deleted,
                edges_added=edges_added,
                edges_deleted=edges_deleted,
                locations_created=locations_created,
                message=f"成功同步：新增 {nodes_added} 節點、更新 {nodes_updated} 節點、刪除 {nodes_deleted} 節點、新增 {edges_added} 邊、刪除 {edges_deleted} 邊、創建 {locations_created} 位置"
            )

        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"KUKA sync execute failed: {e}", exc_info=True)
            raise HTTPException(status_code=500, detail=f"執行同步失敗: {str(e)}")

    return router
