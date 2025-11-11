from agv_base.states.state import State
from rclpy.node import Node
from plc_proxy.plc_client import PlcClient
from db_proxy.agvc_database_client import AGVCDatabaseClient
from shared_constants.task_status import TaskStatus

class WaitRobotState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.plc_client = PlcClient(node)
        self.agvdbclient = AGVCDatabaseClient(node)
        self.count = 0
        self.test = 0

        # 完成驗證相關變數
        self.completion_check_counter = 0  # 完成檢查計數器
        self.completion_verified = False  # 完成驗證標記
        self.completion_retry_count = 0  # 完成重試次數
        self.max_completion_retries = 5  # 最大重試次數
        self.completion_update_sent = False  # 是否已發送完成更新

    def enter(self):
        self.node.get_logger().info("🤖 AGV 進入: WaitRobot 狀態")

    def leave(self):
        self.node.get_logger().info("🚪 AGV 離開 WaitRobot 狀態")

    def handle(self, context):

        if self.count > 100:
            self.count = 0
            self.node.get_logger().info(f"🤖 AGV WaitRobot")
            self.test+=1

        self.count += 1

        # 🔍 檢查訂閱任務數量，若為 0 則回到 mission_select
        if len(self.node.latest_tasks) == 0:
            self.node.get_logger().warn("⚠️ 訂閱任務數量為 0，回到 mission select 狀態")
            from agv_base.agv_states.mission_select_state import MissionSelectState
            context.set_state(MissionSelectState(self.node))
            return

        # 🔍 完成驗證邏輯：如果已發送完成更新，立即開始驗證（之後每 5 秒重試）
        if self.completion_update_sent and not self.completion_verified:
            if self.completion_check_counter == 0 or self.completion_check_counter > 100:
                # 第一次立即執行（counter=0），之後每 100 個循環（約 5 秒）重試一次
                self.completion_check_counter = 1  # 重置為 1，避免下次立即觸發
                self._verify_task_completion_by_service(context)
            self.completion_check_counter += 1
            return  # 驗證期間不執行其他邏輯

        # 優先檢查：機器人完成（統一的完成檢查邏輯）
        if self._check_robot_completed(context):
            return  # 立即返回，避免繼續執行

        # 次要檢查：沒有路徑資料（備用機制）
        if not self.node.agv_status.AGV_PATH:
            self.node.get_logger().info("⚠️ AGV 在 WaitRobot 狀態下沒有路徑資料，回到 mission select 狀態")
            try:
                from agv_base.agv_states.mission_select_state import MissionSelectState
                context.set_state(MissionSelectState(self.node))
                return  # 立即返回，避免繼續執行後續邏輯
            except Exception as e:
                self.node.get_logger().error(f"❌狀態轉換失敗 (WaitRobot → MissionSelect 無路徑): {str(e)}")
                return

    def _check_robot_completed(self, context) -> bool:
        """
        檢查機器人是否完成工作（3層防禦）

        Returns:
            bool: 如果機器人已完成返回 True，否則返回 False
        """
        # 第1層：work_id=21 特殊手動路徑模式（最高優先級）
        # 條件：work_id=21（手動模式）且沒有路徑（手動操作已完成）
        if (self.node.task and
            self.node.task.work_id == 21 and
            not self.node.agv_status.AGV_PATH):
            self.node.get_logger().info(
                f"🎯 work_id=21 純手動路徑任務完成（task_id={self.node.task.id}）"
            )
            self._complete_task(context)
            return True

        # 第2層：明確的 robot_finished 標誌
        if self.node.robot_finished:
            self.node.get_logger().info(
                f"✅ 檢測到 robot_finished 標誌（work_id={self.node.task.work_id}）"
            )
            self._complete_task(context)
            return True

        # 第3層：PLC AGV_LD_COMPLETE 信號
        if self.node.agv_status.AGV_LD_COMPLETE:
            self.node.get_logger().info(
                f"✅ 檢測到 AGV_LD_COMPLETE 信號（work_id={self.node.task.work_id}）"
            )
            self._complete_task(context)
            return True

        return False

    def _complete_task(self, context):
        """執行任務完成邏輯（加入驗證機制）"""
        try:
            # 1. 更新任務狀態
            self.node.task.status_id = TaskStatus.COMPLETED
            self.agvdbclient.async_update_task(
                self.node.task,
                self.task_update_callback
            )

            # 2. 設置完成檢查標記（不立即離開，等待驗證）
            self.completion_update_sent = True
            self.completion_verified = False
            self.completion_retry_count = 0
            self.completion_check_counter = 0

            self.node.get_logger().info(
                f"📤 已發送任務完成更新 (task_id={self.node.task.id})，等待驗證..."
            )

        except Exception as e:
            self.node.get_logger().error(
                f"❌ 任務完成邏輯異常: {e}"
            )

    def _verify_task_completion_by_service(self, context):
        """使用 service 驗證任務是否真的完成（status=4）"""
        from db_proxy_interfaces.srv import SqlQuery
        import json

        # 建立 service client（如果還沒建立）
        if not hasattr(self, 'sql_query_client'):
            self.sql_query_client = self.node.create_client(SqlQuery, '/agvc/sql_query')

        # 等待 service 可用
        if not self.sql_query_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn("⚠️ SqlQuery service 不可用，無法驗證任務完成狀態")
            return

        # 查詢當前任務狀態（包含 agv_id 和 work_id 用於驗證）
        request = SqlQuery.Request()
        request.query_string = f"SELECT id, status_id, agv_id, work_id FROM task WHERE id = {self.node.task.id}"

        # 同步呼叫 service
        future = self.sql_query_client.call_async(request)
        future.add_done_callback(lambda f: self._handle_verification_response(f, context))

    def _handle_verification_response(self, future, context):
        """處理 service 查詢回應"""
        try:
            response = future.result()
            if not response.success:
                self.node.get_logger().error(f"❌ 任務狀態查詢失敗: {response.message}")
                return

            # 解析 JSON 結果
            import json
            result = json.loads(response.json_result)

            if not result or len(result) == 0:
                # 查不到任務資料，可能是任務已被刪除（表示已完成並清除）
                self.node.get_logger().info(
                    f"✅ 任務已不存在於資料庫 (task_id={self.node.task.id})，視為已完成"
                )
                self.completion_verified = True
                self.node.robot_finished = False

                # 轉換到 MissionSelect
                self.node.get_logger().info("✅ AGV 機器人已完成工作，回到 mission select 狀態")
                from agv_base.agv_states.mission_select_state import MissionSelectState
                context.set_state(MissionSelectState(self.node))
                return

            task_data = result[0]
            current_status = task_data.get('status_id')
            current_agv_id = task_data.get('agv_id')
            current_work_id = task_data.get('work_id')

            # 檢查任務資料完整性（agv_id 或 work_id 為 None 表示任務已被修改/刪除）
            if current_agv_id is None or current_work_id is None:
                self.node.get_logger().warn(
                    f"⚠️ 任務資料不完整，可能已被刪除或修改 (task_id={self.node.task.id})\n"
                    f"  - agv_id: {current_agv_id} (原為 {self.node.agv_id})\n"
                    f"  - work_id: {current_work_id} (原為 {self.node.task.work_id})\n"
                    f"  - 視為任務已結束，回到 mission select"
                )
                self.completion_verified = True
                self.node.robot_finished = False
                from agv_base.agv_states.mission_select_state import MissionSelectState
                context.set_state(MissionSelectState(self.node))
                return

            # 檢查任務是否仍屬於當前 AGV
            if current_agv_id != self.node.agv_id:
                self.node.get_logger().warn(
                    f"⚠️ 任務 AGV 已變更 (task_id={self.node.task.id}, "
                    f"原 agv_id={self.node.agv_id}, 現 agv_id={current_agv_id})，"
                    f"視為已完成，回到 mission select"
                )
                self.completion_verified = True
                self.node.robot_finished = False
                from agv_base.agv_states.mission_select_state import MissionSelectState
                context.set_state(MissionSelectState(self.node))
                return

            # 檢查任務 work_id 是否被修改（異常情況）
            if current_work_id != self.node.task.work_id:
                self.node.get_logger().warn(
                    f"⚠️ 任務 work_id 已變更 (task_id={self.node.task.id}, "
                    f"原 work_id={self.node.task.work_id}, 現 work_id={current_work_id})，"
                    f"這可能是資料異常，視為已完成，回到 mission select"
                )
                self.completion_verified = True
                self.node.robot_finished = False
                from agv_base.agv_states.mission_select_state import MissionSelectState
                context.set_state(MissionSelectState(self.node))
                return

            if current_status == TaskStatus.COMPLETED:
                # ✅ 驗證成功，可以離開
                self.node.get_logger().info(
                    f"✅ 任務完成已驗證 (task_id={self.node.task.id}, status=4)"
                )
                self.completion_verified = True

                # 重置機器人狀態
                self.node.robot_finished = False

                # 轉換到 MissionSelect
                self.node.get_logger().info("✅ AGV 機器人已完成工作，回到 mission select 狀態")
                from agv_base.agv_states.mission_select_state import MissionSelectState
                context.set_state(MissionSelectState(self.node))
            else:
                # ❌ 驗證失敗，重試
                self.completion_retry_count += 1
                self.node.get_logger().warn(
                    f"⚠️ 任務完成驗證失敗 (當前 status={current_status}，應為 4)\n"
                    f"  - task_id: {self.node.task.id}\n"
                    f"  - 重試: {self.completion_retry_count}/{self.max_completion_retries}"
                )

                if self.completion_retry_count < self.max_completion_retries:
                    # 重新發送更新
                    self.node.task.status_id = TaskStatus.COMPLETED
                    self.agvdbclient.async_update_task(
                        self.node.task,
                        self.task_update_callback
                    )
                    self.node.get_logger().info(f"🔄 重新發送任務完成更新 (第 {self.completion_retry_count} 次)")
                else:
                    self.node.get_logger().error(
                        f"❌ 任務完成更新失敗超過 {self.max_completion_retries} 次\n"
                        f"  - 停留在 WaitRobot 等待人工介入\n"
                        f"  - 請檢查 agvc_database_node 是否正常運行"
                    )
        except Exception as e:
            self.node.get_logger().error(f"❌ 處理驗證回應異常: {e}")

    def force_callback(self, response):
        if response.success:
            self.node.get_logger().info("✅ PLC force寫入成功")
        else:
            self.node.get_logger().warn("⚠️ PLC force寫入失敗")



    def task_update_callback(self,response):
        if response is None:
            self.node.get_logger().error("❌ 未收到任務更新的回應（可能逾時或錯誤）", throttle_duration_sec=1.0)
            return

        if response.success:
            self.node.get_logger().info(f"✅ 任務更新成功，訊息: {response.message}")
        else:
            self.node.get_logger().error(f"⚠️ 任務更新失敗，訊息: {response.message}")
