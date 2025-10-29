"""
05. 節點初始化資料
依賴：節點類型
"""

from db_proxy.models import Node
from db_proxy.models.node import NodeTypeEnum, PGVEnum, ActionModeEnum
from ..db_install import insert_data_if_not_exists_name_and_not_exists_id


def initialize_nodes(session):
    """初始化節點資料"""
    print("🗺️ 初始化節點資料...")

    # 建立預設群組配置範例
    def create_default_group_config(movable_point=0.0, action_mode="向量", speed_setting=1.0, vector_angle=0.0, area_protection=0.5):
        return {
            "可移動點": movable_point,
            "動作模式": action_mode,
            "速度設定": speed_setting,
            "向量角度": vector_angle,
            "區域防護": area_protection
        }

    default_nodes = [
        # 基礎測試節點 (被 location 資料參考) - 更新為新格式
        {"id": 1, "x": 0.0, "y": 0.0, "theta": 0.0,
         "type": NodeTypeEnum.TRANSPORT_POINT, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config()},
        {"id": 2, "x": 1.0, "y": 1.0, "theta": 90.0,
         "type": NodeTypeEnum.TRANSPORT_POINT, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config()},

        # 停車區域節點
        {"id": 95, "x": 2.0, "y": 2.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 96, "x": 2.0, "y": 3.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 97, "x": 3.0, "y": 2.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 98, "x": 3.0, "y": 3.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 1005, "x": 4.0, "y": 2.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 1006, "x": 4.0, "y": 3.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 1007, "x": 5.0, "y": 2.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 1008, "x": 5.0, "y": 3.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},

        # 房間傳送箱節點
        {"id": 10001, "x": 10.0, "y": 1.0, "theta": 0.0,
         "type": NodeTypeEnum.TRANSPORT_POINT, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(action_mode="開門"),
         "group_2": create_default_group_config(action_mode="關門")},
        {"id": 10002, "x": 10.0, "y": 2.0, "theta": 180.0,
         "type": NodeTypeEnum.TRANSPORT_POINT, "pgv": PGVEnum.REAR,
         "group_1": create_default_group_config(action_mode="開門"),
         "group_2": create_default_group_config(action_mode="關門")},

        {"id": 20001, "x": 20.0, "y": 1.0, "theta": 0.0,
         "type": NodeTypeEnum.TRANSPORT_POINT, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(action_mode="開門"),
         "group_2": create_default_group_config(action_mode="關門")},
        {"id": 20002, "x": 20.0, "y": 2.0, "theta": 180.0,
         "type": NodeTypeEnum.TRANSPORT_POINT, "pgv": PGVEnum.REAR,
         "group_1": create_default_group_config(action_mode="開門"),
         "group_2": create_default_group_config(action_mode="關門")},

        # 充電區節點
        {"id": 201, "x": 21.0, "y": 1.0, "theta": 0.0,
         "type": NodeTypeEnum.CHARGING_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0, speed_setting=0.5)},

        # 設備節點範例 - 其他節點保持基本結構以簡化初始化
        {"id": 210, "x": 21.0, "y": 10.0, "theta": 0.0,
         "type": NodeTypeEnum.TRANSPORT_POINT, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config()},
        {"id": 211, "x": 21.0, "y": 11.0, "theta": 180.0,
         "type": NodeTypeEnum.TRANSPORT_POINT, "pgv": PGVEnum.REAR,
         "group_1": create_default_group_config()},

        # 系統準備區域節點
        {"id": 11, "x": 1.0, "y": 10.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 12, "x": 1.0, "y": 11.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},

        # 空車區域節點
        {"id": 31, "x": 3.0, "y": 10.0, "theta": 0.0,
         "type": NodeTypeEnum.NONE, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config()},
        {"id": 32, "x": 3.0, "y": 11.0, "theta": 0.0,
         "type": NodeTypeEnum.NONE, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config()},
    ]

    insert_data_if_not_exists_name_and_not_exists_id(session, default_nodes, Node)
    print("✅ 節點資料初始化完成")
