from agv_base.base_context import BaseContext
from agv_base.states.state import State
from loader_agv.robot_states.loader_robot_parameter import LoaderRobotParameter
from agv_base.robot import Robot


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

        # Robot 狀態機參數
        self.robot_parameter = LoaderRobotParameter()
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
        self.boxin_buffer = 0  # 新增屬性，用於存儲 BOXIN Buffer 的值

        # BOXIN PORT狀態
        self.boxin_port1 = False  # BOXIN Port 1 是否有貨物
        self.boxin_port2 = False  # BOXIN Port 2 是否有貨物
        self.boxin_port3 = False  # BOXIN Port 3 是否有貨物
        self.boxin_port4 = False  # BOXIN Port 4 是否有貨物

        # AGV狀態
        self.agv_port1 = False  # AGV Port 1 是否有貨物
        self.agv_port2 = False  # AGV Port 2 是否有貨物
        self.agv_port3 = False  # AGV Port 3 是否有貨物
        self.agv_port4 = False  # AGV Port 4 是否有貨物

        # CLEANER PORT狀態
        self.cleaner_port1 = False  # CLEANER Port 1 是否有貨物
        self.cleaner_port2 = False  # CLEANER Port 2 是否有貨物
        self.cleaner_port3 = False  # CLEANER Port 3 是否有貨物
        self.cleaner_port4 = False  # CLEANER Port 4 是否有貨物

        # SOAKER PORT狀態
        self.soaker_port1 = False  # SOAKER Port 1 是否有貨物
        self.soaker_port2 = False  # SOAKER Port 2 是否有貨物
        self.soaker_port3 = False  # SOAKER Port 3 是否有貨物
        self.soaker_port4 = False  # SOAKER Port 4 是否有貨物
        self.soaker_port5 = False  # SOAKER Port 5 是否有貨物
        self.soaker_port6 = False  # SOAKER Port 6 是否有貨物

        # PRE_DRYER PORT狀態
        self.pre_dryer_port1 = False  # PRE_DRYER Port 1 是否有貨物
        self.pre_dryer_port2 = False  # PRE_DRYER Port 2 是否有貨物
        self.pre_dryer_port3 = False  # PRE_DRYER Port 3 是否有貨物
        self.pre_dryer_port4 = False  # PRE_DRYER Port 4 是否有貨物
        self.pre_dryer_port5 = False  # PRE_DRYER Port 5 是否有貨物
        self.pre_dryer_port6 = False  # PRE_DRYER Port 6 是否有貨物
        self.pre_dryer_port7 = False  # PRE_DRYER Port 7 是否有貨物
        self.pre_dryer_port8 = False  # PRE_DRYER Port 8 是否有貨物

        # AGV TO ROBOT PARAMETERS
        # rack

        # GET_PORT
        self.get_loader_agv_port_front = 1
        self.get_loader_agv_port_side = 1
        self.get_boxin_port = 1
        self.get_soaker_port = 1
        self.get_cleaner_port = 1
        self.get_pre_dryer_port = 1

        self.carrier_id = 0

        # take transfer 相關狀態
        self.take_transfer_continue = False  # take transfer 是否可以繼續

        # put cleaner 相關狀態
        self.put_cleaner_continue = False  # put cleaner 是否可以繼續

        # take cleaner 相關狀態
        self.take_cleaner_continue = False  # take cleaner 是否可以繼續

    def update_port_parameters(self):
        """更新 RACK PORT 並同步參數"""
        self.robot_parameter.loader_agv_port_front = self.get_loader_agv_port_front  # 更新 loader_agv_port_front
        self.robot_parameter.loader_agv_port_side = self.get_loader_agv_port_side  # 更新 loader_agv_port_side
        self.robot_parameter.boxin_port = self.get_boxin_port  # 更新 box_port
        self.robot_parameter.soaker_port = self.get_soaker_port  # 更新 soaker_port
        self.robot_parameter.cleaner_port = self.get_cleaner_port  # 更新 cleaner_port
        self.robot_parameter.pre_dryer_port = self.get_pre_dryer_port  # 更新 pre_dryer_port
        self.node.get_logger().debug(
            f"更新參數: loader_agv_port_front={self.robot_parameter.loader_agv_port_front}, "
            f"loader_agv_port_side={self.robot_parameter.loader_agv_port_side}, "
            f"boxin_port={self.robot_parameter.boxin_port}, "
            f"soaker_port={self.robot_parameter.soaker_port}, "
            f"cleaner_port={self.robot_parameter.cleaner_port}, "
            f"pre_dryer_port={self.robot_parameter.pre_dryer_port}"
        )
        # 同步更新參數
        self.robot_parameter.calculate_parameter()  # 同步更新參數
        self.robot.update_parameter()
