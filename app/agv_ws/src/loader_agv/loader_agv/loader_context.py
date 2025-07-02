from agv_base.base_context import BaseContext
from agv_base.states.state import State


class LoaderContext(BaseContext):
    """ 管理cargo agv狀態機 """

    def __init__(self, initial_state: State):
        super().__init__(initial_state)

        


        # 一些cargo agv 可能需要跨狀態使用的參數
        # 初始化 rack_rotation 參數
        self.rack_rotation = False
        # 初始化 completed 參數
        self.completed = False

        