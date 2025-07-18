from agv_base.base_context import BaseContext
from agv_base.states.state import State
from unloader_agv.robot_states.unloader_robot_parameter import UnloaderRobotParameter
from agv_base.robot import Robot


class RobotContext(BaseContext):
    """ 管理unloader robot 狀態機 """
    # STEP定義
    IDLE = 0

    CHECK_IDLE = 1
    CHECK_IDLE_CALLBACK = 11
    WRITE_CHG_PARA = 2
    WRITE_CHG_PARAMTER = 21
    CHECK_CHG_PARA = 3
    WRITE_PGNO = 4
    CHECK_PGNO = 5
    ACTING = 6
    FINISH = 99
    UPDATE_DATABASE = 100

    def __init__(self, initial_state: State):
        super().__init__(initial_state)

        # 一些Unloader Robot可能需要跨狀態使用的參數
        self.ocr = None
        self.mission = None
        self.rack_rotaion = None
        self.robot_parameter = UnloaderRobotParameter()
        self.robot = Robot(self.node, self.robot_parameter)
        self.selected_agv_port = 0  # 新增屬性，用於存儲選擇的 AGV車上 Port

        # BOXOUT狀態 (Unloader 主要處理 BOXOUT)
        self.boxout_up_both_empty = False
        self.boxout_down_both_empty = False
        self.boxout_up_left_empty = False
        self.boxout_up_right_empty = False
        self.boxout_down_left_empty = False
        self.boxout_down_right_empty = False

        self.boxout_up_lock = False
        self.boxout_down_lock = False
        self.boxout_buffer = 0  # 新增屬性，用於存儲 BOXOUT Buffer 的值

        # BOXOUT PORT狀態
        self.boxout_port1 = False  # BOXOUT Port 1 是否有貨物
        self.boxout_port2 = False  # BOXOUT Port 2 是否有貨物
        self.boxout_port3 = False  # BOXOUT Port 3 是否有貨物
        self.boxout_port4 = False  # BOXOUT Port 4 是否有貨物

        # AGV狀態
        self.agv_port1 = False  # AGV Port 1 是否有貨物
        self.agv_port2 = False  # AGV Port 2 是否有貨物
        self.agv_port3 = False  # AGV Port 3 是否有貨物
        self.agv_port4 = False  # AGV Port 4 是否有貨物

        # PRE_DRYER PORT狀態
        self.pre_dryer_port1 = False  # PRE_DRYER Port 1 是否有貨物
        self.pre_dryer_port2 = False  # PRE_DRYER Port 2 是否有貨物
        self.pre_dryer_port3 = False  # PRE_DRYER Port 3 是否有貨物
        self.pre_dryer_port4 = False  # PRE_DRYER Port 4 是否有貨物
        self.pre_dryer_port5 = False  # PRE_DRYER Port 5 是否有貨物
        self.pre_dryer_port6 = False  # PRE_DRYER Port 6 是否有貨物
        self.pre_dryer_port7 = False  # PRE_DRYER Port 7 是否有貨物
        self.pre_dryer_port8 = False  # PRE_DRYER Port 8 是否有貨物

        # OVEN PORT狀態
        self.oven_port1 = False  # OVEN Port 1 是否有貨物
        self.oven_port2 = False  # OVEN Port 2 是否有貨物
        self.oven_port3 = False  # OVEN Port 3 是否有貨物
        self.oven_port4 = False  # OVEN Port 4 是否有貨物
        self.oven_port5 = False  # OVEN Port 5 是否有貨物
        self.oven_port6 = False  # OVEN Port 6 是否有貨物
        self.oven_port7 = False  # OVEN Port 7 是否有貨物
        self.oven_port8 = False  # OVEN Port 8 是否有貨物

        # AGV TO ROBOT PARAMETERS
        # rack

        # GET_PORT
        self.get_unloader_agv_port_back = 1  # 修正參數名稱，對應 unloader_robot_parameter.py
        self.get_boxout_port = 1
        self.get_pre_dryer_port = 1  # 修正參數名稱，對應 unloader_robot_parameter.py
        self.get_oven_port = 1  # 修正參數名稱，對應 unloader_robot_parameter.py

        # GET_QUANTITY
        self.get_take_quantity = 0  # 新增屬性，用於存儲取貨數量

        self.carrier_id = [None, None]  # 存儲雙 port 的 carrier_id [min, max]
        self.get_room_id = 0

        # 新增 rack_id 屬性，用於存儲機架 ID
        self.rack_id = None

        # 新增 take_transfer_continue 屬性，用於判斷是否繼續傳送
        self.take_transfer_continue = False

        # 新增 select_boxout_port 屬性，用於存儲選擇的 BOXOUT Port
        self.select_boxout_port = 0

        # 新增 port_have_carrier 屬性，用於判斷 Port 是否有載具
        self.port_have_carrier = False

        # 新增 carrier_query_success 屬性，用於判斷載具查詢是否成功
        self.carrier_query_success = False

        # 新增 carrier_update_success 屬性，用於判斷載具更新是否成功
        self.carrier_update_success = False

        # 新增 robot_action_success 屬性，用於判斷機器人動作是否成功
        self.robot_action_success = False

        # 新增 vision_position_success 屬性，用於判斷視覺定位是否成功
        self.vision_position_success = False

        # 新增 hokuyo_write_completed 屬性，用於判斷 Hokuyo 寫入是否完成
        self.hokuyo_write_completed = False

        # 新增 hokuyo_busy_write_completed 屬性，用於判斷 Hokuyo busy 寫入是否完成
        self.hokuyo_busy_write_completed = False

    def update_port_parameters(self):
        """更新 UNLOADER PORT 並同步參數"""
        self.robot_parameter.unloader_agv_port_back = self.get_unloader_agv_port_back  # 更新 unloader_agv_port_back
        self.robot_parameter.boxout_port = self.get_boxout_port  # 更新 boxout_port
        self.robot_parameter.pre_dryer_port = self.get_pre_dryer_port  # 更新 pre_dryer_port
        self.robot_parameter.oven_port = self.get_oven_port  # 更新 oven_port
        self.robot_parameter.take_quantity = self.get_take_quantity  # 更新 take_quantity
        self.node.get_logger().debug(
            f"更新參數: unloader_agv_port_back={self.robot_parameter.unloader_agv_port_back}, "
            f"boxout_port={self.robot_parameter.boxout_port}, "
            f"pre_dryer_port={self.robot_parameter.pre_dryer_port}, "
            f"oven_port={self.robot_parameter.oven_port}, "
            f"take_quantity={self.robot_parameter.take_quantity}")
        self.robot_parameter.calculate_parameter()  # 更新參數
        self.robot.update_parameter()
