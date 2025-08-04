"""
KUKA 任務處理器模組

包含以下任務處理器：
1. EmptyRackToBoxoutHandler - 系統空料架區搬運到出口傳送箱處理器 (emptyarea_to_boxout_handler)
2. FullRackToManualReceiveHandler - 滿料架搬運到人工收料區處理器 (fullrack_to_receivearea_handler)
3. RackRotate180Handler - AGV 貨架180度旋轉處理器 (agv_rotate_handler)
4. ReadyRackToBoxinHandler - 準備料架搬運到入口傳送箱處理器 (readyrack_to_boxin_handler)
5. EmptyRackToBoxoutOrEmptyareaHandler - 空料架從入口傳送箱搬運處理器 (emptyrack_to_boxout_or_emptyarea_handler)
6. NgRackRecyclingHandler - NG架回收處理器 (保留原有)
7. ManualEmptyRackRecyclingHandler - 人工回收空料架搬運處理器 (recieverack_to_emptyarea_handler)

所有處理器都繼承自基礎任務處理器，並實作：
- check_condition(): 檢查任務條件
- insert_task(): 新增任務到 KUKA 系統
- check_insert_done(): 檢查任務是否完成新增
"""

from .emptyarea_to_boxout_handler import EmptyRackToBoxoutHandler
from .fullrack_to_receivearea_handler import FullRackToManualReceiveHandler  
from .agv_rotate_handler import RackRotate180Handler
from .readyrack_to_boxin_handler import ReadyRackToBoxinHandler
from .emptyrack_to_boxout_or_emptyarea_handler import EmptyRackToBoxoutOrEmptyareaHandler
from .ng_rack_recycling_handler import NgRackRecyclingHandler
from .recieverack_to_emptyarea_handler import ManualEmptyRackRecyclingHandler

__all__ = [
    'EmptyRackToBoxoutHandler',
    'FullRackToManualReceiveHandler', 
    'RackRotate180Handler',
    'ReadyRackToBoxinHandler',
    'EmptyRackToBoxoutOrEmptyareaHandler',
    'NgRackRecyclingHandler',
    'ManualEmptyRackRecyclingHandler'
]