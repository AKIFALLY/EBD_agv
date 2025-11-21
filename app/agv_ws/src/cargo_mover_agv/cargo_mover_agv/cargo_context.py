from agv_base.base_context import BaseContext
from agv_base.states.state import State
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit


class CargoContext(BaseContext):
    """ 管理cargo agv狀態機 """

    # 覆写父类状态引用为 Cargo 专属版本（类变量）
    MissionSelectState = None
    WritePathState = None
    RunningState = None
    WaitRobotState = None
    # IdleState 不需要覆写（使用父类版本）

    def __init__(self, initial_state: State):
        super().__init__(initial_state)

        # 动态导入并赋值 Cargo 状态类
        from cargo_mover_agv.states.cargo_mission_select_state import CargoMissionSelectState
        from cargo_mover_agv.states.cargo_write_path_state import CargoWritePathState
        from cargo_mover_agv.states.cargo_running_state import CargoRunningState
        from cargo_mover_agv.states.cargo_wait_robot_state import CargoWaitRobotState

        # 设置类变量（所有 CargoContext 实例共享）
        CargoContext.MissionSelectState = CargoMissionSelectState
        CargoContext.WritePathState = CargoWritePathState
        CargoContext.RunningState = CargoRunningState
        CargoContext.WaitRobotState = CargoWaitRobotState

        # 一些cargo agv 可能需要跨狀態使用的參數
        # 初始化 rack_rotation 參數
        self.rack_rotation = False
        # 初始化 completed 參數
        self.completed = False
