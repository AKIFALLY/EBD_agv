from agv_base.base_context import BaseContext
from agv_base.states.state import State


class LoaderContext(BaseContext):
    """ 管理 loader agv 狀態機 """

    def __init__(self, initial_state: State):
        super().__init__(initial_state)
