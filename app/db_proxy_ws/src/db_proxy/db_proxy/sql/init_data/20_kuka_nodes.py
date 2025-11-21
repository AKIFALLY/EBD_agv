"""
05. 節點初始化資料
依賴：節點類型
"""

from db_proxy.models.agvc_kuka import KukaNode
from ..db_install import insert_data_if_not_exists_name
from datetime import datetime, timezone

def initialize_kuka_nodes(session):
    """初始化節點資料"""
    print("🗺️ 初始化節點資料...")



    default_kuka_nodes = [
        {"id": 2, "uuid": "KUKA001", "node_type_id": 2, "name": "系統準備區1", "description": "人工將滿料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 3, "uuid": "KUKA002", "node_type_id": 2, "name": "系統準備區2", "description": "人工將滿料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 4, "uuid": "KUKA003", "node_type_id": 2, "name": "系統準備區3", "description": "人工將滿料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 5, "uuid": "KUKA004", "node_type_id": 2, "name": "系統準備區4", "description": "人工將滿料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 6, "uuid": "KUKA005", "node_type_id": 2, "name": "系統準備區5", "description": "人工將滿料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 7, "uuid": "KUKA006", "node_type_id": 2, "name": "系統準備區6", "description": "人工將滿料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 8, "uuid": "KUKA007", "node_type_id": 2, "name": "系統準備區7", "description": "人工將滿料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 9, "uuid": "KUKA008", "node_type_id": 2, "name": "系統準備區8", "description": "人工將滿料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},

        {"id": 11, "uuid": "KUKA031", "node_type_id": 2, "name": "空架放置區1", "description": "人工將空料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 12, "uuid": "KUKA032", "node_type_id": 2, "name": "空架放置區2", "description": "人工將空料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 13, "uuid": "KUKA033", "node_type_id": 2, "name": "空架放置區3", "description": "人工將空料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        
        {"id": 21, "uuid": "KUKA051", "node_type_id": 2, "name": "人工滿料回收區1", "description": "滿料貨架回收放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 22, "uuid": "KUKA052", "node_type_id": 2, "name": "人工滿料回收區2", "description": "滿料貨架回收放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        
        {"id": 71, "uuid": "KUKA071", "node_type_id": 2, "name": "NG回收區1", "description": "NG回收放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 72, "uuid": "KUKA072", "node_type_id": 2, "name": "NG回收區2", "description": "NG回收放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},

        # Room2 出入口節點 (2025-11-10 新增: 修復 location 20001/20002 的節點引用)
        {"id": 26, "uuid": "KUKA026", "node_type_id": 2, "name": "Room2出口(KUKA)", "description": "Room2出口(KUKA)",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 27, "uuid": "KUKA027", "node_type_id": 2, "name": "Room2入口(KUKA)", "description": "Room2入口(KUKA)",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        
        
    ]

    insert_data_if_not_exists_name(session, default_kuka_nodes, KukaNode)
    print("✅ KUKA節點資料初始化完成")



"""
class KukaNode(SQLModel, table=True):
    __tablename__ = "kuka_node"
    id: Optional[int] = Field(default=None, primary_key=True)
    uuid: Optional[str] = None
    node_type_id: Optional[int] = None
    name: Optional[str] = None
    description: Optional[str] = None
    x: float
    y: float
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))
    updated_at: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime(timezone=True), nullable=True))

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
"""