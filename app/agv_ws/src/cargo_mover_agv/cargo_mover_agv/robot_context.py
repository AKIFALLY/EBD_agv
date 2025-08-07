from agv_base.base_context import BaseContext
from agv_base.states.state import State
from cargo_mover_agv.robot_states.cargo_robot_parameter import CargoRobotParameter
from agv_base.robot import Robot
import json

class RobotContext(BaseContext):
    """ 管理robot 狀態機 """
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

        # 一些Robot可能需要跨狀態使用的參數
        self.ocr = None
        self.mission = None
        self.rack_rotaion = None
        self.robot_parameter = CargoRobotParameter()
        self.robot = Robot(self.node, self.robot_parameter)

        # BOXIN狀態
        self.boxin_up_both_empty = False
        self.boxin_down_both_empty = False
        self.boxin_up_left_empty = False
        self.boxin_up_right_empty = False
        self.boxin_down_left_empty = False
        self.boxin_down_right_empty = False

        self.boxin_up_lock = False
        self.boxin_down_lock = False
        self.boxin_buffer = None  # 新增屬性，用於存儲 BOXIN Buffer 的值

        self.boxin_number = 0

        # BOXIN PORT狀態
        self.boxin_port1 = False  # BOXIN Port 1 是否有貨物
        self.boxin_port2 = False  # BOXIN Port 2 是否有貨物
        self.boxin_port3 = False  # BOXIN Port 3 是否有貨物
        self.boxin_port4 = False  # BOXIN Port 4 是否有貨物

        # BOXOUT狀態
        self.boxout_up_both_have = False
        self.boxout_down_both_have = False
        self.boxout_up_left_have = False
        self.boxout_up_right_have = False
        self.boxout_down_left_have = False
        self.boxout_down_right_have = False
        self.boxout_up_lock = False
        self.boxout_down_lock = False
        self.boxout_number = 0
        self.boxout_last = 0  # 新增屬性，用於存儲 BOXOUT Buffer 的值
        self.boxout_buffer = None  # 新增屬性，用於存儲 BOXOUT Buffer 的值

        # AGV TO ROBOT PARAMETERS
        # rack

        # GET_BOX_PORT
        self.get_rack_side = 0  # 新增屬性，用於存儲 Rack Side 判斷結果
        self.get_rack_port = 1
        self.get_rack_nothing = False  # 新增屬性，用於存儲 Rack Port 是否為空
        self.rack_photo_up_or_down_buffer = None  # 新增屬性，用於存儲 Rack Photo Up or Down 的值
        self.get_boxin_port = 1

        self.get_boxout_port = 1

        self.carrier_id = 0
        self.get_room_id = 0

        # 新增屬性，用於存儲從 task.parameters 中取得的 rack_id
        params_raw = self.node.task.parameters
        if isinstance(params_raw, str) and params_raw.strip():
            try:
                params_dict = json.loads(params_raw)
                self.rack_id = params_dict.get("rack_id")
            except json.JSONDecodeError as e:
                self.node.get_logger().error(f"❌ JSON decode error: {e} for raw: {params_raw}")
                self.rack_id = None
        else:
            self.node.get_logger().warn("⚠️ parameters is empty or not a valid string")
            self.rack_id = None
          

        # take transfer 相關狀態
        self.take_transfer_continue = False  # take transfer 是否可以繼續

        # hokuyo_dms_8bit
        self.hokuyo1_receive_ready = False
        self.hokuyo1_receive_load_req = False
        self.hokuyo1_receive_unload_req = False

    def update_rack_box_port(self):
        """更新 RACK PORT 並同步參數"""
        self.robot_parameter.rack_port = self.get_rack_port  # 更新 rack_port
        self.robot_parameter.boxin_port = self.get_boxin_port  # 更新 box_port
        self.node.get_logger().debug(
            f"更新 RACK PORT: {self.get_rack_port}, BOXIN PORT: {self.get_boxin_port}")
        self.robot_parameter.boxout_port = self.get_boxout_port
        self.robot_parameter.calculate_parameter()  # 同步更新參數
        self.robot.update_parameter()
