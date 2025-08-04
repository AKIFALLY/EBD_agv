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
        {"id": 11, "uuid": "KUKA001", "node_type_id": 2, "name": "系統準備區1", "description": "人工將滿料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 12, "uuid": "KUKA002", "node_type_id": 2, "name": "系統準備區2", "description": "人工將滿料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 13, "uuid": "KUKA003", "node_type_id": 2, "name": "系統準備區3", "description": "人工將滿料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 14, "uuid": "KUKA004", "node_type_id": 2, "name": "系統準備區4", "description": "人工將滿料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 15, "uuid": "KUKA005", "node_type_id": 2, "name": "系統準備區5", "description": "人工將滿料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 16, "uuid": "KUKA006", "node_type_id": 2, "name": "系統準備區6", "description": "人工將滿料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 17, "uuid": "KUKA007", "node_type_id": 2, "name": "系統準備區7", "description": "人工將滿料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 18, "uuid": "KUKA008", "node_type_id": 2, "name": "系統準備區8", "description": "人工將滿料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 31, "uuid": "KUKA031", "node_type_id": 2, "name": "空架放置區1", "description": "人工將空料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 32, "uuid": "KUKA032", "node_type_id": 2, "name": "空架放置區2", "description": "人工將空料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 33, "uuid": "KUKA033", "node_type_id": 2, "name": "空架放置區3", "description": "人工將空料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 34, "uuid": "KUKA034", "node_type_id": 2, "name": "空架放置區4", "description": "人工將空料貨架放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 51, "uuid": "KUKA051", "node_type_id": 2, "name": "人工滿料回收區1", "description": "滿料貨架回收放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 52, "uuid": "KUKA052", "node_type_id": 2, "name": "人工滿料回收區2", "description": "滿料貨架回收放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 53, "uuid": "KUKA053", "node_type_id": 2, "name": "人工滿料回收區3", "description": "滿料貨架回收放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 54, "uuid": "KUKA054", "node_type_id": 2, "name": "人工滿料回收區4", "description": "滿料貨架回收放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 55, "uuid": "KUKA055", "node_type_id": 2, "name": "人工滿料回收區5", "description": "滿料貨架回收放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 71, "uuid": "KUKA071", "node_type_id": 2, "name": "NG回收區1", "description": "NG回收放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 72, "uuid": "KUKA072", "node_type_id": 2, "name": "NG回收區2", "description": "NG回收放置處",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 91, "uuid": "KUKA091", "node_type_id": 2, "name": "Room2出口", "description": "Room2出口",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 92, "uuid": "KUKA092", "node_type_id": 2, "name": "Room2入口", "description": "Room2入口",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 20001, "uuid": "KUKA20001", "node_type_id": 2, "name": "Room2入口", "description": "Room2入口",
        "x": 0.0, "y": 0.0, "created_at": datetime.now(timezone.utc), "updated_at": None},
        {"id": 20002, "uuid": "KUKA20002", "node_type_id": 2, "name": "Room2入口", "description": "Room2入口",
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