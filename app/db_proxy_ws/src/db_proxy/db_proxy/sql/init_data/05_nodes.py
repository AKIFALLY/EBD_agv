"""
05. 節點初始化資料
依賴：節點類型
"""

from db_proxy.models import Node
from db_proxy.models.node import NodeTypeEnum, PGVEnum, ActionModeEnum
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

        # 系統準備區域節點 (2025-10-29 更新: ID 2-9, 共8個)
        {"id": 2, "x": 1.0, "y": 10.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 3, "x": 1.0, "y": 10.5, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 4, "x": 1.0, "y": 11.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 5, "x": 1.0, "y": 11.5, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 6, "x": 1.0, "y": 12.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 7, "x": 1.0, "y": 12.5, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 8, "x": 1.0, "y": 13.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 9, "x": 1.0, "y": 13.5, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},

        # 系統空車停放區域節點 (2025-10-29 更新: ID 11-13, 共3個)
        {"id": 11, "x": 3.0, "y": 10.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 12, "x": 3.0, "y": 10.5, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 13, "x": 3.0, "y": 11.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},

        # 射出機停車位置節點 (2025-10-29 新增: 共8個)
        {"id": 14, "x": 5.0, "y": 10.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 15, "x": 5.0, "y": 10.5, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 23, "x": 5.0, "y": 11.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 25, "x": 5.0, "y": 11.5, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 44, "x": 5.0, "y": 12.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 45, "x": 5.0, "y": 12.5, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 46, "x": 5.0, "y": 13.0, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
        {"id": 47, "x": 5.0, "y": 13.5, "theta": 0.0,
         "type": NodeTypeEnum.REST_AREA, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config(movable_point=1.0)},
    ]

    insert_data_if_not_exists_name_and_not_exists_id(session, default_nodes, Node)
    print("✅ 節點資料初始化完成")
