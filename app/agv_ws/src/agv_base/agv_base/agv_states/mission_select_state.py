from agv_base.states.state import State
from db_proxy_interfaces.msg._tasks import Tasks
import json
from std_msgs.msg import String
from rclpy.node import Node
import requests


class MissionSelectState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        self.startpoint = None  # 起點
        self.endpoint = None  # 終點
        self.subscription = None
        self.task_list = []
        self.subscribed = False
        self.highest_priority_task = []
        self.count = 0  # 計數器，用於執行次數
        self.localMission = False  # 觸發Local端任務旗標
        # self.latest_tasks 已移至 node (全局共享)
        self.status_log_count = 0  # 狀態日誌計數器（每5秒輸出一次）

    def enter(self):
        self.node.get_logger().info("🎯 AGV 進入: Mission Select")

        # 建立 local mission timer
        self.locamissiontimer = self.node.create_timer(1.0, self.local_mission)

    def leave(self):
        self.node.get_logger().info("🚪 AGV 離開 Mission Select 狀態")
        # 全局訂閱不需移除，由 agv_node_base 管理
        self.locamissiontimer.cancel()  # 取消timer

    def handle(self, context):
        #self.node.get_logger().info("AGV Mission Select 狀態")
        #self.node.get_logger().info(f"Task列表:{self.latest_tasks}")

        if self.count > 30:
            self.count = 0

            # 🔍 診斷：檢查 latest_tasks 狀態
            self.node.get_logger().info(
                f"🔍 [診斷] latest_tasks 狀態: "
                f"exists={self.node.latest_tasks is not None}, "
                f"len={len(self.node.latest_tasks) if self.node.latest_tasks else 0}"
            )

            # 🔍 【新增】在檢查離開條件之前，先確保從 task table 中搜尋該 AGV 的任務資料
            if self.node.latest_tasks and len(self.node.latest_tasks) > 0:
                has_task = self._process_tasks(self.node.latest_tasks)
                if not has_task and not hasattr(self.node, 'task'):
                    # 沒有找到任務且 node.task 也不存在，記錄警告
                    self.node.get_logger().info("🔍 未找到屬於該 AGV 的任務")
                elif has_task:
                    self.node.get_logger().info(f"🔍 確認任務資料: task_id={getattr(self.node.task, 'id', 'None')}")

            # 🔒 【狀態轉換守衛】檢查 Base 層狀態，只有在 Auto 狀態時才允許 AGV 層狀態轉換
            if not self._is_base_auto_state():
                base_state_name = self.node.base_context.state.__class__.__name__
                self.node.get_logger().debug(
                    f"🔒 AGV 層狀態轉換被阻止: Base 層不在 Auto 狀態 (當前: {base_state_name})")
                self.count += 1
                return



            # 如果已經有路徑
            if self.node.agv_status.AGV_PATH:
                # ⚠️ 【改善】檢查是否有有效任務資料（task_id 不能為 0）
                # 注意：task 現在是 dict 格式
                task_id = self.node.task.get('id', 0) if isinstance(self.node.task, dict) else getattr(self.node.task, 'id', 0)
                task_status_id = self.node.task.get('status_id', 0) if isinstance(self.node.task, dict) else getattr(self.node.task, 'status_id', 0)

                # 🔍 檢查任務是否為完成狀態（5, 15, 25）
                from shared_constants.task_status import TaskStatus
                if TaskStatus.is_task_complete_status(task_status_id):
                    # 任務已完成，清除任務資料，繼續等待新任務
                    self.node.get_logger().info(
                        f"⚠️ 任務已完成 (task_id={task_id}, status={task_status_id})，"
                        f"清除任務資料並繼續等待新任務"
                    )
                    self.node.task = None
                    self.highest_priority_task = []
                    # 不進行狀態轉換，繼續在 MissionSelectState 等待
                elif hasattr(self.node, 'task') and self.node.task and task_id != 0:
                    self.node.get_logger().info(f"✅ AGV 已有路徑資料且有任務資料 (task_id={task_id})，離開 Mission Select 狀態")

                    context.set_state(context.RunningState(self.node))  # 切換狀態
                else:
                    self.node.get_logger().warn("⚠️ AGV 有路徑但無有效任務資料（task_id=0 或無任務），等待任務分配")

            # 當已取得任務後，可選擇自動取消訂閱（或等 leave() 處理）
            elif self.highest_priority_task:
                task = self.highest_priority_task
                from shared_constants.task_status import TaskStatus

                # 取得任務狀態和 ID（支援 dict 格式）
                task_status_id = task.get('status_id') if isinstance(task, dict) else getattr(task, 'status_id', 0)
                task_id = task.get('id') if isinstance(task, dict) else getattr(task, 'id', 0)

                # 檢查任務狀態：判斷是否為「任務開始」狀態
                if TaskStatus.is_task_start_status(task_status_id):
                    # status=1, 11, 13, 21 → 任務開始，進入 WritePathState 寫入路徑
                    self.node.get_logger().info(
                        f"✅ 任務開始 (status={task_status_id}): task_id={task_id}，進入 WritePathState"
                    )
                    context.set_state(context.WritePathState(self.node))
                elif task_status_id == TaskStatus.FROM_COMPLETE and not self.node.agv_status.AGV_PATH:
                    # status=3 (FROM_COMPLETE) 且無路徑 → From 完成，需寫入 To 路徑
                    self.node.get_logger().info(
                        f"✅ From 完成，準備 To 流程 (status={task_status_id}): task_id={task_id}，進入 WritePathState"
                    )
                    context.set_state(context.WritePathState(self.node))
                elif TaskStatus.is_task_executing_status(task_status_id) and not self.node.agv_status.AGV_PATH:
                    # status=2,4,12,14,22 且無路徑 → 進入 WritePathState 重算路徑
                    # 根據 status 決定目標端口
                    if task_status_id in (2, 12):
                        # FROM 執行中：目標是 from_port
                        target_port = task.get('from_port', '')
                        port_type = "from_port"
                    else:
                        # TO/PATH 執行中 (4, 14, 22)：目標是 to_port
                        target_port = task.get('to_port', '')
                        port_type = "to_port"

                    self.node.node_id, self.node.task_layer = self._get_node_id_from_port(target_port)

                    self.node.get_logger().info(
                        f"⚠️ 執行中狀態但無路徑 (status={task_status_id}): task_id={task_id}，"
                        f"目標節點: {self.node.node_id}, Layer: {self.node.task_layer} (使用 {port_type}={target_port})，重新計算路徑"
                    )
                    context.set_state(context.WritePathState(self.node))
                else:
                    # 其他情況：未預期狀態，記錄警告
                    self.node.get_logger().warn(
                        f"⚠️ 未預期的任務狀態 (status={task_status_id}): task_id={task_id}"
                    )

            # 如果HMI有設定Magic跟終點設定
            elif self.localMission and not self.node.agv_status.AGV_PATH:
                self.node.get_logger().info(
                    f"✅ HMI任務下達---  Magic:{self.node.agv_status.MAGIC}  Dest.:{self.node.agv_status.AGV_END_POINT}")
                context.set_state(context.WritePathState(self.node))  # 切換狀態

            # 【新增】Local 模式下有路徑但需要重新寫入路徑的情況
            # 條件：LOCAL=1, MAGIC>0, PATH=1 → 覆蓋現有路徑，重新進入 WritePathState
            elif self._should_rewrite_path_in_local_mode():
                self.node.get_logger().info(
                    f"✅ Local 模式路徑重寫---  Magic:{self.node.agv_status.MAGIC}  "
                    f"Dest.:{self.node.agv_status.AGV_END_POINT}  PATH=1 → 重新寫入路徑"
                )
                context.set_state(context.WritePathState(self.node))  # 切換狀態

        self.count += 1

    def _is_base_auto_state(self) -> bool:
        """
        檢查 Base 層是否在 Auto 狀態

        Returns:
            bool: Base 層在 Auto 狀態時返回 True，否則返回 False
        """
        from agv_base.states.auto_state import AutoState
        return isinstance(self.node.base_context.state, AutoState)

    def _should_rewrite_path_in_local_mode(self) -> bool:
        """
        檢查是否應在 Local 模式下重新寫入路徑

        條件：LOCAL=1, MAGIC>0, PATH=1
        當滿足這些條件時，即使已有路徑也需要重新進入 WritePathState

        Returns:
            bool: 需要重新寫入路徑時返回 True
        """
        local = self.node.agv_status.AGV_LOCAL if self.node.agv_status.AGV_LOCAL is not None else 0
        magic = self.node.agv_status.MAGIC if self.node.agv_status.MAGIC is not None else 0
        has_path = self.node.agv_status.AGV_PATH

        # 條件：LOCAL=1, MAGIC>0, PATH=1
        if local == 1 and magic > 0 and has_path:
            # 設定目標節點為 HMI 指定的終點
            end_point = self.node.agv_status.AGV_END_POINT if self.node.agv_status.AGV_END_POINT is not None else 0
            if end_point > 0:
                self.node.node_id = end_point
                return True

        return False

    def _get_node_id_from_port(self, to_port: str) -> tuple:
        """
        透過 to_port 查詢對應的 node_id 和 layer（用於 A* 路徑規劃）

        API: GET /api/v1/eqp_port/by-name/{name}
        回應格式: {"id": 0, "name": "string", "eqp_name": "string", "node": "string", "parameter": {"layer": "2"}, ...}

        Args:
            to_port: 目標端口名稱 (例如: "2011", "3021")

        Returns:
            tuple: (node_id, layer)，查詢失敗時返回 (0, 0)
        """
        if not to_port or to_port == 'na':
            self.node.get_logger().warn(f"⚠️ _get_node_id_from_port: to_port 為空或無效 ({to_port})")
            return 0, 0

        try:
            # 使用 eqp_port API 查詢
            url = f"{self.node.agvc_api_base_url}/api/v1/eqp_port/by-name/{to_port}"
            response = requests.get(url, timeout=5.0)

            if response.status_code == 200:
                result = response.json()
                node_id = 0
                layer = 0

                # 解析 node_id
                node_str = result.get('node', '')
                if node_str:
                    try:
                        node_id = int(node_str)
                    except ValueError:
                        self.node.get_logger().warn(
                            f"⚠️ _get_node_id_from_port: node 值無法轉換為整數 (node={node_str})"
                        )

                # 解析 layer（從 parameter 欄位）
                parameter_raw = result.get('parameter')
                if isinstance(parameter_raw, str):
                    try:
                        parameter = json.loads(parameter_raw) if parameter_raw else {}
                    except json.JSONDecodeError:
                        parameter = {}
                elif isinstance(parameter_raw, dict):
                    parameter = parameter_raw
                else:
                    parameter = {}

                layer_str = parameter.get('layer', '0')
                try:
                    layer = int(layer_str)
                except (ValueError, TypeError):
                    layer = 0

                self.node.get_logger().info(
                    f"✅ _get_node_id_from_port: to_port={to_port} → node_id={node_id}, layer={layer}"
                )
                return node_id, layer

            elif response.status_code == 404:
                self.node.get_logger().warn(
                    f"⚠️ _get_node_id_from_port: 查無 eqp_port (to_port={to_port})"
                )
                return 0, 0
            else:
                self.node.get_logger().error(
                    f"❌ _get_node_id_from_port: API 查詢失敗 HTTP {response.status_code}"
                )
                return 0, 0

        except requests.exceptions.Timeout:
            self.node.get_logger().warn(f"⚠️ _get_node_id_from_port: 查詢逾時 (to_port={to_port})")
            return 0, 0
        except requests.exceptions.ConnectionError:
            self.node.get_logger().warn(f"⚠️ _get_node_id_from_port: 連接失敗 ({self.node.agvc_api_base_url})")
            return 0, 0
        except Exception as e:
            self.node.get_logger().error(f"❌ _get_node_id_from_port: 查詢異常 - {e}")
            return 0, 0

    # tasks 改用 agv_node_base 的 Web API 輪詢取得

    def _process_tasks(self, tasks):
        """處理任務篩選邏輯

        篩選: agv_name 匹配 + status_id 為開始/執行中/過渡狀態
        """
        from shared_constants.task_status import TaskStatus

        # 篩選分配給本 AGV 且非完成狀態的任務
        running_tasks = [
            t for t in tasks
            if t.get('agv_name') == self.node.agv_name and
               (TaskStatus.is_task_start_status(t.get('status_id', 0)) or
                TaskStatus.is_task_executing_status(t.get('status_id', 0)) or
                t.get('status_id', 0) == TaskStatus.FROM_COMPLETE)
        ]

        self.node.get_logger().info(f"🔍 任務篩選: {len(tasks)} 筆 → {len(running_tasks)} 筆符合")

        if len(running_tasks) > 0:
            self.node.get_logger().info("⚠️ 有正在執行的任務")
            task = running_tasks[0]

            # 根據 status_id 決定使用 from_port 或 to_port 查詢 node_id
            # status=1 (FROM_TO_START), 11 (FROM_ONLY_START) → from_port
            # status=3 (FROM_COMPLETE), 13 (TO_ONLY_START), 21 (PATH_START) → to_port
            task_status_id = task.get('status_id', 0)
            from_port = task.get('from_port', '')
            to_port = task.get('to_port', '')

            if task_status_id in (1, 11):
                # 任務開始，目標是 From 位置
                target_port = from_port
                port_type = "from_port"
            else:
                # status=3, 13, 21 等，目標是 To 位置
                target_port = to_port
                port_type = "to_port"

            # 從 eqp_port API 取得 node_id 和 layer
            self.node.node_id, self.node.task_layer = self._get_node_id_from_port(target_port)

            self.highest_priority_task = task
            self.node.task = task  # 現在是 dict 格式

            self.node.get_logger().info(
                f"✅ 任務ID: {task.get('id')}, "
                f"WORK ID: {task.get('work_id')}, "
                f"Status: {task_status_id}, "
                f"優先級: {task.get('priority')}, "
                f"from_port: {from_port}, "
                f"to_port: {to_port}, "
                f"目標節點: {self.node.node_id}, Layer: {self.node.task_layer} (使用 {port_type}={target_port})"
            )
            return True  # 找到任務

        else:
            # ⚠️ 沒有找到已分配給此 AGV 的任務
            # 任務分配應由 RCS 負責，AGV 只接收已分配的任務
            self.node.get_logger().debug("📭 未查詢到已分配給此 AGV 的任務，等待 RCS 派發")
            return False  # 沒找到任務

    def local_mission(self):
        # self.node.get_logger().info(f"(magic={self.node.agv_status.MAGIC}) dest.={self.node.agv_status.AGV_END_POINT}")
        # 判斷是否AGV_HMI路徑任務,判斷nMagci,From,To是否值大於0
        # 安全檢查：確保 MAGIC 和 AGV_END_POINT 不是 None
        magic = self.node.agv_status.MAGIC if self.node.agv_status.MAGIC is not None else 0
        end_point = self.node.agv_status.AGV_END_POINT if self.node.agv_status.AGV_END_POINT is not None else 0
        auto = self.node.agv_status.AGV_Auto if self.node.agv_status.AGV_Auto is not None else 0
        local = self.node.agv_status.AGV_LOCAL if self.node.agv_status.AGV_LOCAL is not None else 0

        # 當 LOCAL 模式啟用時，印出條件狀態
        if local == 1:
            self.node.get_logger().info(
                f"🔧 [Local Mode] 條件檢查: "
                f"MAGIC={magic} (>0: {'✓' if magic > 0 else '✗'}), "
                f"END_POINT={end_point} (>0: {'✓' if end_point > 0 else '✗'}), "
                f"AUTO={auto} (=1: {'✓' if auto == 1 else '✗'})"
            )

        if magic > 0 and end_point > 0 and auto == 1 and local == 1:
            self.node.node_id = end_point
            self.localMission = True