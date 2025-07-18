"""
18. KUKA 地圖初始化資料
依賴：節點類型
從 kuka_map.json 檔案自動匯入 KUKA 地圖資料到資料庫
"""

import json
import logging
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
from sqlalchemy import select, delete
from db_proxy.models import KukaNode, KukaEdge

logger = logging.getLogger(__name__)


def kuka_unit_2_px(y, x):
    """
    KUKA 單位轉換為像素
    m to mm to px 0.0125 m => 1 px
    """
    return y * 1000 / 12.5, x * 1000 / 12.5


def initialize_kuka_map(session):
    """
    從 kuka_map.json 檔案初始化 KUKA 地圖資料

    這個函數會：
    1. 讀取 kuka_map.json 檔案
    2. 解析地圖資料
    3. 匯入節點和邊到資料庫
    4. 提供詳細的錯誤處理和日誌記錄

    如果發生錯誤，不會中斷整個初始化流程
    """
    print("🗺️ 初始化 KUKA 地圖資料...")

    # 獲取 kuka_map.json 檔案路徑
    current_dir = Path(__file__).parent
    kuka_map_path = current_dir / "kuka_map.json"

    # 檢查檔案是否存在
    if not kuka_map_path.exists():
        print(f"⚠️  KUKA 地圖檔案不存在: {kuka_map_path}")
        print("   跳過 KUKA 地圖初始化")
        return

    try:
        # 讀取 JSON 檔案
        print(f"📖 讀取地圖檔案: {kuka_map_path}")
        with open(kuka_map_path, 'r', encoding='utf-8') as file:
            data = json.load(file)

        # 驗證 JSON 結構
        if 'floorList' not in data:
            print("❌ 地圖檔案格式錯誤：缺少 floorList")
            return

        floor_list = data.get("floorList", [])
        if not floor_list:
            print("⚠️  地圖檔案中沒有樓層資料")
            return

        print(f"📊 找到 {len(floor_list)} 個樓層")

        # 開始匯入處理
        total_nodes = 0
        total_edges = 0
        taipei_timezone = ZoneInfo("Asia/Taipei")

        # 建立 nodeLabel 到 nodeNumber (kuka_node.id) 的映射
        node_label_to_id = {}
        all_edges = []  # 暫存所有邊，等節點處理完後再處理

        print("🔄 第一階段：處理節點資料...")

        # === 第一階段：處理所有節點 ===
        for floor_idx, floor in enumerate(floor_list):
            floor_level = floor.get("floorLevel", floor_idx + 1)
            floor_name = floor.get("floorName", f"Floor {floor_level}")

            print(f"   處理樓層: {floor_name} (Level {floor_level})")

            for node in floor.get("nodeList", []):
                try:
                    node_label = node["nodeLabel"]
                    node_number = node.get("nodeNumber")
                    node_uuid = node.get("nodeUuid")

                    if node_number is None:
                        print(f"⚠️  節點 {node_label} 沒有 nodeNumber，跳過")
                        continue

                    # 解析節點類型
                    function_list = node.get("functionList", [])
                    node_type = None
                    if function_list and isinstance(function_list, list):
                        node_type = function_list[0].get("functionType")

                    # 座標轉換
                    y_px, x_px = kuka_unit_2_px(
                        node["yCoordinate"], node["xCoordinate"])

                    # 使用 nodeNumber 作為 kuka_node.id，並儲存 nodeUuid
                    # 不設置 created_at，讓 default_factory 處理
                    node_obj = KukaNode(
                        id=node_number,
                        uuid=node_uuid,
                        node_type_id=node_type,
                        x=x_px,
                        y=y_px
                    )
                    # 手動設置 updated_at
                    node_obj.updated_at = datetime.now(taipei_timezone)
                    session.merge(node_obj)

                    # 建立 nodeLabel 到 nodeNumber 的映射
                    node_label_to_id[node_label] = node_number
                    total_nodes += 1

                    if total_nodes % 10 == 0:  # 每10個節點顯示一次進度
                        print(f"   已處理 {total_nodes} 個節點...")

                except Exception as e:
                    print(f"❌ 處理節點 {node.get('nodeLabel', 'unknown')} 時發生錯誤: {e}")
                    continue

            # 收集所有邊資料
            for edge in floor.get("edgeList", []):
                all_edges.append(edge)

        # 提交所有節點
        session.commit()
        print(f"✅ 第一階段完成：成功儲存 {total_nodes} 個節點")

        print("🔄 第二階段：處理邊資料...")

        # === 第二階段：處理所有邊 ===
        for edge in all_edges:
            try:
                begin_label = edge["beginNodeLabel"]
                end_label = edge["endNodeLabel"]

                # 檢查節點是否存在於映射中
                if begin_label not in node_label_to_id:
                    print(f"⚠️  找不到起始節點: {begin_label}")
                    continue

                if end_label not in node_label_to_id:
                    print(f"⚠️  找不到結束節點: {end_label}")
                    continue

                begin_id = node_label_to_id[begin_label]
                end_id = node_label_to_id[end_label]
                edge_name = f"{begin_label}-{end_label}"

                # 檢查邊是否已存在
                existing_edge = session.exec(select(KukaEdge).where(
                    KukaEdge.name == edge_name)).first()

                if existing_edge:
                    # 邊已存在，跳過處理
                    # 這是正常情況，因為地圖資料通常已經匯入過
                    continue

                # 建立新邊
                now = datetime.now(taipei_timezone)
                new_edge = KukaEdge(
                    from_id=begin_id,
                    to_id=end_id,
                    weight=edge.get("edgeWeight", 1.0),
                    name=edge_name
                )
                # 手動設置 updated_at
                new_edge.updated_at = now
                session.add(new_edge)

                total_edges += 1

                if total_edges % 20 == 0:  # 每20個邊顯示一次進度
                    print(f"   已處理 {total_edges} 個邊...")

            except Exception as e:
                import traceback
                print(
                    f"❌ 處理邊 {edge.get('beginNodeLabel', 'unknown')}-{edge.get('endNodeLabel', 'unknown')} 時發生錯誤: {e}")
                print(f"   錯誤類型: {type(e).__name__}")
                print(f"   詳細錯誤:")
                traceback.print_exc()
                continue

        # 最終提交所有變更
        session.commit()

        print("=" * 50)
        print("✅ KUKA 地圖匯入完成！")
        print(f"📊 匯入統計:")
        print(f"   - 樓層數量: {len(floor_list)}")
        print(f"   - 節點數量: {total_nodes}")
        print(f"   - 邊數量: {total_edges}")
        print("=" * 50)

    except json.JSONDecodeError as e:
        print(f"❌ JSON 格式錯誤: {e}")
        print("   請檢查 kuka_map.json 檔案格式")
        session.rollback()

    except FileNotFoundError:
        print(f"❌ 找不到檔案: {kuka_map_path}")

    except Exception as e:
        print(f"❌ KUKA 地圖匯入失敗: {e}")
        print("   正在回滾資料庫變更...")
        session.rollback()

    finally:
        print("🔚 KUKA 地圖初始化程序結束")


def clear_kuka_map(session):
    """
    智能清除 KUKA 地圖資料
    跳過有外鍵約束參考的節點，只刪除沒有外鍵約束的節點和邊
    """
    try:
        print("🗑️ 開始智能清除 KUKA 地圖資料...")

        # 統計清除前的資料
        all_nodes = session.exec(select(KukaNode)).all()
        all_edges = session.exec(select(KukaEdge)).all()
        print(f"📊 清除前統計: {len(all_nodes)} 個節點, {len(all_edges)} 個邊")

        # 第一階段：清除所有邊（邊通常沒有被其他表參考）
        print("🔄 第一階段：清除邊資料...")
        edges_deleted = 0
        edges_skipped = 0

        for edge in all_edges:
            try:
                session.delete(edge)
                session.flush()  # 立即檢查約束
                edges_deleted += 1
            except Exception as e:
                session.rollback()
                edge_id = getattr(edge, 'id', 'unknown')
                edge_name = getattr(edge, 'name', f'ID:{edge_id}')
                print(f"   ⚠️  跳過邊 {edge_name}: {str(e)[:100]}...")
                edges_skipped += 1
                continue

        session.commit()
        print(f"   ✅ 邊清除完成: 刪除 {edges_deleted} 個, 跳過 {edges_skipped} 個")

        # 第二階段：智能清除節點
        print("🔄 第二階段：智能清除節點資料...")
        nodes_deleted = 0
        nodes_skipped = 0
        skipped_reasons = {}

        for node in all_nodes:
            try:
                session.delete(node)
                session.flush()  # 立即檢查約束
                nodes_deleted += 1

                if nodes_deleted % 10 == 0:
                    print(f"   已處理 {nodes_deleted + nodes_skipped} 個節點...")

            except Exception as e:
                session.rollback()
                nodes_skipped += 1

                # 分析跳過原因
                error_msg = str(e)
                if "foreign key constraint" in error_msg.lower():
                    # 提取約束名稱
                    import re
                    constraint_match = re.search(r'"([^"]*_fkey)"', error_msg)
                    if constraint_match:
                        constraint_name = constraint_match.group(1)
                        table_match = re.search(r'table "([^"]*)"', error_msg)
                        table_name = table_match.group(1) if table_match else "unknown"
                        reason = f"被 {table_name} 表參考 ({constraint_name})"
                    else:
                        reason = "外鍵約束"
                else:
                    reason = "其他約束"

                skipped_reasons[reason] = skipped_reasons.get(reason, 0) + 1

                if nodes_skipped <= 5:  # 只顯示前5個詳細錯誤
                    print(f"   ⚠️  跳過節點 {node.id}: {reason}")

                continue

        session.commit()
        print(f"   ✅ 節點清除完成: 刪除 {nodes_deleted} 個, 跳過 {nodes_skipped} 個")

        # 顯示跳過原因統計
        if skipped_reasons:
            print("\n📋 跳過原因統計:")
            for reason, count in skipped_reasons.items():
                print(f"   - {reason}: {count} 個節點")

        # 最終統計
        remaining_nodes = session.exec(select(KukaNode)).all()
        remaining_edges = session.exec(select(KukaEdge)).all()

        print("\n" + "=" * 50)
        print("✅ KUKA 地圖智能清除完成！")
        print(f"📊 清除統計:")
        print(f"   - 節點: 刪除 {nodes_deleted} 個, 跳過 {nodes_skipped} 個, 剩餘 {len(remaining_nodes)} 個")
        print(f"   - 邊: 刪除 {edges_deleted} 個, 跳過 {edges_skipped} 個, 剩餘 {len(remaining_edges)} 個")
        print("=" * 50)

        return {
            'nodes_deleted': nodes_deleted,
            'nodes_skipped': nodes_skipped,
            'edges_deleted': edges_deleted,
            'edges_skipped': edges_skipped,
            'skipped_reasons': skipped_reasons
        }

    except Exception as e:
        print(f"❌ 清除 KUKA 地圖資料失敗: {e}")
        session.rollback()
        raise
