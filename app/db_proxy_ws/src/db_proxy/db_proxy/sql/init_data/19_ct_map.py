"""
19. CT 地圖初始化資料
依賴：節點類型
從 20250509_pathtest.json 檔案自動匯入 CT 地圖資料到資料庫
"""

import json
import logging
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
from sqlalchemy import select, delete
from db_proxy.models import Node, Edge

logger = logging.getLogger(__name__)


def ct_unit_2_px(y, x):
    """
    CT 單位轉換為像素
    mm to px 12.5 mm => 1 px
    """
    return y / 12.5, x / 12.5


def initialize_ct_map(session):
    """
    從 20250509_pathtest.json 檔案初始化 CT 地圖資料

    這個函數會：
    1. 讀取 20250509_pathtest.json 檔案
    2. 解析地圖資料
    3. 匯入節點和邊到資料庫
    4. 提供詳細的錯誤處理和日誌記錄

    如果發生錯誤，不會中斷整個初始化流程
    """
    print("🗺️ 初始化 CT 地圖資料...")

    # 獲取 20250509_pathtest.json 檔案路徑
    current_dir = Path(__file__).parent
    ct_map_path = current_dir / "20250509_pathtest.json"

    # 檢查檔案是否存在
    if not ct_map_path.exists():
        print(f"⚠️  CT 地圖檔案不存在: {ct_map_path}")
        print("   跳過 CT 地圖初始化")
        return

    try:
        # 讀取 JSON 檔案
        print(f"📖 讀取地圖檔案: {ct_map_path}")
        with open(ct_map_path, 'r', encoding='utf-8') as file:
            json_data = json.load(file)

        # 驗證 JSON 結構
        if not isinstance(json_data, list):
            print("❌ 地圖檔案格式錯誤：應該是陣列格式")
            return

        if not json_data:
            print("⚠️  地圖檔案中沒有節點資料")
            return

        print(f"📊 找到 {len(json_data)} 個節點")

        # 開始匯入處理
        total_nodes = 0
        total_edges = 0
        taipei_timezone = ZoneInfo("Asia/Taipei")

        print("🔄 第一階段：處理節點資料...")

        # === 第一階段：建立或更新所有節點 ===
        for node in json_data:
            try:
                tag_no = node["TagNo"]
                tag_x = node["Tag_X"]
                tag_y = node["Tag_Y"]

                # 座標轉換
                y_px, x_px = ct_unit_2_px(tag_y, tag_x)

                # 檢查節點是否已存在
                existing_node = session.get(Node, tag_no)

                now = datetime.now(taipei_timezone)
                if existing_node:
                    # 更新現有節點
                    existing_node.x = y_px
                    existing_node.y = x_px
                    existing_node.updated_at = now
                else:
                    # 建立新節點
                    new_node = Node(
                        id=tag_no,
                        x=y_px,
                        y=x_px,
                        updated_at=now
                    )
                    session.add(new_node)

                total_nodes += 1

                if total_nodes % 10 == 0:  # 每10個節點顯示一次進度
                    print(f"   已處理 {total_nodes} 個節點...")

            except Exception as e:
                print(f"❌ 處理節點 {node.get('TagNo', 'unknown')} 時發生錯誤: {e}")
                continue

        # 提前提交一次，確保節點先寫入 DB（避免 FK 失敗）
        session.commit()
        print(f"✅ 第一階段完成：成功儲存 {total_nodes} 個節點")

        print("🔄 第二階段：處理邊資料...")

        # === 第二階段：建立或更新所有邊 ===
        for node in json_data:
            try:
                tag_no = node["TagNo"]
                can_to_move_set = node.get("CanToMoveSet", [])

                for move in can_to_move_set:
                    from_tag = move["CanToMoveTag"]
                    if from_tag == 0:
                        continue  # 忽略無效來源

                    edge_name = f"{from_tag}-{tag_no}"

                    # 檢查邊是否已存在
                    existing_edge = session.exec(select(Edge).where(
                        Edge.name == edge_name)).first()

                    now = datetime.now(taipei_timezone)
                    if existing_edge:
                        # 邊已存在，跳過處理
                        # 這是正常情況，因為地圖資料通常已經匯入過
                        continue

                    # 建立新邊
                    new_edge = Edge(
                        from_id=from_tag,
                        to_id=tag_no,
                        name=edge_name,
                        weight=1.0,
                        updated_at=now
                    )
                    session.add(new_edge)

                    total_edges += 1

                    if total_edges % 20 == 0:  # 每20個邊顯示一次進度
                        print(f"   已處理 {total_edges} 個邊...")

            except Exception as e:
                print(f"❌ 處理節點 {node.get('TagNo', 'unknown')} 的邊時發生錯誤: {e}")
                continue

        # 最後再提交一次，將邊寫入 DB
        session.commit()

        print("=" * 50)
        print("✅ CT 地圖匯入完成！")
        print(f"📊 匯入統計:")
        print(f"   - 節點數量: {total_nodes}")
        print(f"   - 邊數量: {total_edges}")
        print("=" * 50)

    except json.JSONDecodeError as e:
        print(f"❌ JSON 格式錯誤: {e}")
        print("   請檢查 20250509_pathtest.json 檔案格式")
        session.rollback()

    except FileNotFoundError:
        print(f"❌ 找不到檔案: {ct_map_path}")

    except Exception as e:
        print(f"❌ CT 地圖匯入失敗: {e}")
        print("   正在回滾資料庫變更...")
        session.rollback()

    finally:
        print("🔚 CT 地圖初始化程序結束")


def clear_ct_map(session):
    """
    智能清除 CT 地圖資料
    跳過有外鍵約束參考的節點，只刪除沒有外鍵約束的節點和邊
    """
    try:
        print("🗑️ 開始智能清除 CT 地圖資料...")

        # 統計清除前的資料
        all_nodes = session.exec(select(Node)).all()
        all_edges = session.exec(select(Edge)).all()
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
        remaining_nodes = session.exec(select(Node)).all()
        remaining_edges = session.exec(select(Edge)).all()

        print("\n" + "=" * 50)
        print("✅ CT 地圖智能清除完成！")
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
        print(f"❌ 清除 CT 地圖資料失敗: {e}")
        session.rollback()
        raise
