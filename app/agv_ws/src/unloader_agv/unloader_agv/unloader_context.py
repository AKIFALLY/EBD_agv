from agv_base.base_context import BaseContext
from agv_base.states.state import State


class UnloaderContext(BaseContext):
    """ 管理unloader agv狀態機 """

    def __init__(self, initial_state: State):
        super().__init__(initial_state)

        # 一些unloader agv 可能需要跨狀態使用的參數
        # 初始化 rack_rotation 參數
        self.rack_rotation = False
        # 初始化 completed 參數
        self.completed = False

        # Unloader AGV 特有的參數
        # 初始化 unload_completed 參數，用於判斷卸載是否完成
        self.unload_completed = False
        
        # 初始化 transfer_ready 參數，用於判斷傳送是否準備就緒
        self.transfer_ready = False
        
        # 初始化 boxout_ready 參數，用於判斷 BOXOUT 是否準備就緒
        self.boxout_ready = False
        
        # 初始化 dryer_ready 參數，用於判斷乾燥機是否準備就緒
        self.dryer_ready = False
        
        # 初始化 cooler_ready 參數，用於判斷冷卻機是否準備就緒
        self.cooler_ready = False
        
        # 初始化 agv_position_confirmed 參數，用於判斷 AGV 位置是否確認
        self.agv_position_confirmed = False
        
        # 初始化 unload_sequence_active 參數，用於判斷卸載序列是否啟動
        self.unload_sequence_active = False
        
        # 初始化 error_recovery_mode 參數，用於判斷是否處於錯誤恢復模式
        self.error_recovery_mode = False
