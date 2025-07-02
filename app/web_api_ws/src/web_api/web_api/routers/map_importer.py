# routers/door.py
from datetime import datetime
import json
import logging
from zoneinfo import ZoneInfo
from fastapi import APIRouter, File, HTTPException, UploadFile
from pydantic import BaseModel
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import KukaNode, KukaEdge
from db_proxy.models import Node, Edge
from sqlalchemy import Row, select, delete

logger = logging.getLogger(__name__)


def kuka_unit_2_px(y, x):
    # m to mm to px 0.0125 m => 1 px
    return y * 1000 / 12.5, x * 1000 / 12.5


def ct_unit_2_px(y, x):
    # mm to px 12.5 mm => 1 px
    return y / 12.5, x / 12.5


def create_map_importer_router(pool_agvc: ConnectionPoolManager):
    router = APIRouter(prefix="/map_importer", tags=["Map Importer"])

    @router.post("/upload-kuka-map/")
    async def upload_kuka_map(file: UploadFile = File(...)):
        content = await file.read()
        data = json.loads(content)

        floor_list = data.get("floorList", [])
        total_nodes = 0
        total_edges = 0

        with pool_agvc.get_session() as session:
            try:
                taipei_timezone = ZoneInfo("Asia/Taipei")

                # 建立 nodeLabel 到 nodeNumber (kuka_node.id) 的映射
                node_label_to_id = {}  # nodeLabel -> kuka_node.id 的映射
                all_edges = []  # 暫存所有邊，等節點處理完後再處理

                for floor in floor_list:
                    floor_level = floor["floorLevel"]

                    # === 第一階段：處理所有節點，使用 nodeNumber 作為 kuka_node.id ===
                    for node in floor.get("nodeList", []):
                        node_label = node["nodeLabel"]
                        node_number = node.get("nodeNumber")
                        node_uuid = node.get("nodeUuid")

                        if node_number is None:
                            print(f"⚠️ 節點 {node_label} 沒有 nodeNumber，跳過")
                            continue

                        function_list = node.get("functionList", [])
                        node_type = None
                        if function_list and isinstance(function_list, list):
                            node_type = function_list[0].get("functionType")

                        y_px, x_px = kuka_unit_2_px(
                            node["yCoordinate"], node["xCoordinate"])

                        # 使用 nodeNumber 作為 kuka_node.id，並儲存 nodeUuid
                        session.merge(KukaNode(
                            id=node_number,
                            uuid=node_uuid,
                            node_type_id=node_type,
                            x=x_px,
                            y=y_px,
                            updated_at=datetime.now(taipei_timezone)
                        ))

                        # 建立 nodeLabel 到 nodeNumber 的映射
                        node_label_to_id[node_label] = node_number
                        total_nodes += 1
                        print(
                            f"儲存節點: {node_label} (nodeNumber: {node_number}, uuid: {node_uuid})")

                    # === 收集所有邊資料 ===
                    for edge in floor.get("edgeList", []):
                        all_edges.append(edge)

                # 提交所有節點
                session.commit()
                print(f"第一階段完成：儲存了 {total_nodes} 個節點")

                # === 第二階段：處理所有邊，根據 nodeLabel 映射到對應的 kuka_node.id ===
                for edge in all_edges:
                    begin_label = edge["beginNodeLabel"]
                    end_label = edge["endNodeLabel"]

                    # 根據 nodeLabel 查找對應的 kuka_node.id
                    begin_id = node_label_to_id.get(begin_label)
                    end_id = node_label_to_id.get(end_label)

                    if begin_id is None:
                        print(f"⚠️ 找不到節點 {begin_label} 對應的 nodeNumber")
                        continue

                    if end_id is None:
                        print(f"⚠️ 找不到節點 {end_label} 對應的 nodeNumber")
                        continue

                    # 創建邊，使用對應的 kuka_node.id
                    edge_name = f'{begin_id}-{end_id}'

                    result = session.exec(select(KukaEdge).where(
                        KukaEdge.name == edge_name)).first()
                    if isinstance(result, Row):
                        existing_edge = result[0]
                    else:
                        existing_edge = result

                    now = datetime.now(taipei_timezone)
                    if existing_edge:
                        existing_edge.from_id = begin_id
                        existing_edge.to_id = end_id
                        existing_edge.weight = edge["edgeWeight"]
                        existing_edge.updated_at = now
                    else:
                        new_edge = KukaEdge(
                            from_id=begin_id,
                            to_id=end_id,
                            weight=edge["edgeWeight"],
                            name=edge_name,
                            updated_at=now
                        )
                        session.add(new_edge)

                    total_edges += 1
                    print(
                        f"儲存邊: {begin_label}({begin_id}) -> {end_label}({end_id})")

                # 最終提交所有變更
                session.commit()

                return {
                    "message": "Upload successful",
                    "floors": len(floor_list),
                    "nodes_saved": total_nodes,
                    "edges_saved": total_edges,
                    "node_label_mapping": node_label_to_id
                }
            except Exception as e:
                print(f"error {e}")
                session.rollback()

                return {
                    "message": f"Upload failed {e}",
                }

    @router.post("/upload-ct-map/")
    async def upload_ct_map(file: UploadFile = File(...)):
        content = await file.read()
        json_data = json.loads(content)

        total_nodes = 0
        total_edges = 0

        with pool_agvc.get_session() as session:
            try:
                # === 第一段：建立或更新所有節點 ===
                for node in json_data:
                    tag_no = node["TagNo"]
                    tag_x = node["Tag_X"]
                    tag_y = node["Tag_Y"]

                    now = datetime.now(ZoneInfo("Asia/Taipei"))

                    y_px, x_px = ct_unit_2_px(tag_y, tag_x)
                    session.merge(Node(
                        id=tag_no,
                        x=y_px,
                        y=x_px,
                        created_at=now,
                        updated_at=now
                    ))

                    print(f"儲存節點: {tag_no}")
                    total_nodes += 1

                # 提前提交一次，確保節點先寫入 DB（避免 FK 失敗）
                session.commit()

                # === 第二段：建立或更新所有邊 ===
                for node in json_data:
                    tag_no = node["TagNo"]
                    for move in node.get("CanToMoveSet", []):
                        from_tag = move["CanToMoveTag"]
                        if from_tag == 0:
                            continue  # 忽略無效來源

                        edge_name = f"{from_tag}-{tag_no}"

                        result = session.exec(select(Edge).where(
                            Edge.name == edge_name)).first()
                        if isinstance(result, Row):
                            # 或 result["KukaEdge"] 如果你有 alias
                            existing_edge = result[0]
                        else:
                            existing_edge = result  # 已是 ORM instance

                        if existing_edge:
                            existing_edge.from_id = from_tag
                            existing_edge.to_id = tag_no
                            existing_edge.updated_at = now
                        else:
                            new_edge = Edge(
                                from_id=from_tag,
                                to_id=tag_no,
                                name=edge_name,
                                weight=1.0,
                                created_at=now,
                                updated_at=now
                            )
                            session.add(new_edge)

                        print(f"儲存邊: {edge_name}")
                        total_edges += 1

                # 最後再提交一次，將邊寫入 DB
                session.commit()
                return {
                    "message": "Upload successful",
                    "nodes_saved": total_nodes,
                    "edges_saved": total_edges
                }
            except Exception as e:
                print(f"error {e}")
                session.rollback()

                return {
                    "message": f"Upload failed {e}",
                }

    @router.delete("/delete-kuka-map")
    async def delete_kuka_map():
        with pool_agvc.get_session() as session:
            try:
                session.exec(delete(KukaEdge))
                session.exec(delete(KukaNode))
                # 清空資料表
                session.commit()

                return {
                    "message": "Delete successful"
                }
            except Exception as e:
                session.rollback()

                return {
                    "message": f"Delete failed {e}",
                }

    return router
