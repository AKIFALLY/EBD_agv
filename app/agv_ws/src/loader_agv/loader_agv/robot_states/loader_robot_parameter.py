from agv_base.robot_parameter_abc import RobotParameterABC


class LoaderRobotParameter(RobotParameterABC):

    def __init__(self):   # 計算哪一個RACK PORT
        self.loader_agv_port_front = 0
        self.loader_agv_port_side = 0
        self.boxin_port = 0
        self.soaker_port = 0
        self.cleaner_port = 0
        self.pre_dryer_port = 0

    def update_parameter(self):
        print("❌❌❌❌❌❌❌❌❌✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌")

    def values(self):
        # self.update_parameter()
        return [

        ]
