from agv_base.states.state import State
from rclpy.node import Node
from plc_proxy.plc_client import PlcClient
from db_proxy.agvc_database_client import AGVCDatabaseClient
from shared_constants.task_status import TaskStatus
from std_msgs.msg import String

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

        # OCR 相關變數（條件性訂閱：只有 cargo AGV）
        self.ocr_enabled = self._is_cargo_agv()
        self.latest_ocr_result = None
        self.ocr_received_time = None
        self.ocr_timeout_seconds = 10.0
        self._pending_ocr_verification = None  # 待驗證的 OCR 結果

        if self.ocr_enabled:
            # 建立 OCR 訂閱
            self.ocr_subscription = self.node.create_subscription(
                String,
                'sensor/ocr',  # namespace 會自動加上
                self._ocr_callback,
                10
            )
            self.node.get_logger().info("=" * 80)
            self.node.get_logger().info("📡 Cargo AGV OCR 整合已啟用")
            self.node.get_logger().info(f"   - 訂閱 Topic: {self.node.get_namespace()}/sensor/ocr")
            self.node.get_logger().info(f"   - 超時設定: {self.ocr_timeout_seconds} 秒")
            self.node.get_logger().info("=" * 80)

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

    # ==================== OCR 整合功能（Cargo AGV 專用）====================

    def _is_cargo_agv(self) -> bool:
        """
        判斷是否為 Cargo AGV

        Returns:
            bool: True 表示為 Cargo AGV，False 表示其他車型
        """
        namespace = self.node.get_namespace().lstrip('/')

        # 方法1：namespace 判斷（推薦）
        if 'cargo' in namespace.lower():
            return True

        # 方法2：節點名稱判斷
        node_name = self.node.get_name()
        if 'cargo' in node_name.lower():
            return True

        return False

    def _ocr_callback(self, msg: String):
        """
        OCR 資料回調處理

        Args:
            msg: OCR 識別結果 (std_msgs/String)
        """
        self.latest_ocr_result = msg.data
        self.ocr_received_time = self.node.get_clock().now()

        self.node.get_logger().info(f"📄 收到 OCR 識別結果: {msg.data}")

        # 記錄到任務日誌（未來可擴展寫入資料庫）
        self._log_ocr_to_task(msg.data)

        # 發送到 PLC/HMI 顯示
        self._send_ocr_to_plc(msg.data)

    def _log_ocr_to_task(self, ocr_result: str):
        """
        記錄 OCR 結果到任務日誌

        Args:
            ocr_result: OCR 識別結果
        """
        # TODO: 未來可擴展寫入資料庫
        # 例如：self.agvdbclient.async_update_task_log(...)

        self.node.get_logger().info(
            f"📝 OCR 已記錄: {ocr_result} (task_id={self.node.task.id if self.node.task else 'N/A'})"
        )

    def _send_ocr_to_plc(self, ocr_result: str):
        """
        發送 OCR 結果到 PLC DM 暫存器供 HMI 顯示

        Args:
            ocr_result: OCR 識別結果（最多 20 個字元）

        Note:
            將 OCR 字串轉換為 ASCII 碼陣列寫入 DM8000~DM8019
            每個 DM word 儲存 1 個字元的 ASCII 碼
        """
        try:
            # 將字串填充到 20 個字元（不足補空格）
            ocr_padded = ocr_result.ljust(20, ' ')[:20]

            # 轉換為 ASCII 碼陣列
            ascii_values = [str(ord(c)) for c in ocr_padded]

            # 寫入 PLC DM8000~DM8019（20 個 word）
            self.plc_client.async_write_continuous_data(
                device_type='DM',
                start_address='8000',
                values=ascii_values,
                callback=self._plc_ocr_write_callback
            )

            self.node.get_logger().info(
                f"📤 OCR 已發送到 PLC (DM8000~DM8019): {ocr_result}"
            )

            # ✅ 步驟2：解析房間 ID 並執行產品驗證
            room_id = self._extract_room_id_from_workid()
            if room_id is not None:
                self._query_room_products(room_id, ocr_result)

        except Exception as e:
            self.node.get_logger().error(f"❌ 發送 OCR 到 PLC 失敗: {e}")

    def _plc_ocr_write_callback(self, response):
        """PLC OCR 寫入回調"""
        if response.success:
            self.node.get_logger().info("✅ OCR PLC 寫入成功")
        else:
            self.node.get_logger().error(f"❌ OCR PLC 寫入失敗: {response.message}")

    def check_ocr_available(self) -> bool:
        """
        檢查是否有 OCR 資料（供 Robot 狀態調用）

        Returns:
            bool: True 表示有資料或已超時，False 表示等待中

        Note:
            超時後返回 True 並記錄警告，但不阻塞任務執行
        """
        if not self.ocr_enabled:
            # 非 cargo AGV，直接返回 True（不需要 OCR）
            return True

        if self.latest_ocr_result is None:
            if self.ocr_received_time is None:
                # 還沒開始等待 OCR
                return False

            # 檢查超時
            current_time = self.node.get_clock().now()
            elapsed = (current_time - self.ocr_received_time).nanoseconds / 1e9

            if elapsed > self.ocr_timeout_seconds:
                self.node.get_logger().warn(
                    f"⏰ OCR 超時 ({elapsed:.1f}秒 > {self.ocr_timeout_seconds}秒)，"
                    f"繼續執行任務"
                )
                return True  # 超時，不阻塞

            return False  # 等待中

        return True  # 有資料

    # ========================================================================
    # 🔍 產品驗證相關方法（Cargo AGV 專用）
    # ========================================================================

    def _extract_room_id_from_workid(self) -> int:
        """
        從 work_id 第1位數取得房間編號

        Returns:
            int | None: 房間編號，如果無法取得則返回 None

        Note:
            Work ID 格式：2060502 → 房間2
            房間編號 = work_id // 1000000（整除百萬）
        """
        if not self.node.task or not self.node.task.work_id:
            self.node.get_logger().warn("⚠️ 無任務或 work_id，跳過產品驗證")
            return None

        work_id = self.node.task.work_id
        room_id = work_id // 1000000  # 整除百萬取第1位

        self.node.get_logger().info(
            f"📍 Work ID 解析\n"
            f"  - work_id: {work_id}\n"
            f"  - room_id: {room_id}"
        )
        return room_id

    def _query_room_products(self, room_id: int, ocr_result: str):
        """
        查詢房間產品清單（非同步）

        Args:
            room_id: 房間編號
            ocr_result: OCR 識別結果

        Note:
            透過 SqlQuery service 查詢房間正在生產的產品名稱清單
            查詢 SQL: SELECT p.name FROM product p JOIN room r
                     ON p.process_settings_id = r.process_settings_id
                     WHERE r.id = {room_id}
        """
        from db_proxy_interfaces.srv import SqlQuery

        # 建立 service client（延遲初始化）
        if not hasattr(self, 'sql_query_client'):
            self.sql_query_client = self.node.create_client(
                SqlQuery, '/agvc/sql_query'
            )

        # 等待 service 可用
        if not self.sql_query_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error(
                "❌ SqlQuery service 不可用，無法驗證產品\n"
                "  - 寫入驗證 Fail"
            )
            self._write_verification_fail()
            return

        # 建立查詢請求
        request = SqlQuery.Request()
        request.query_string = f"""
            SELECT p.name
            FROM product p
            JOIN room r ON p.process_settings_id = r.process_settings_id
            WHERE r.id = {room_id}
        """

        # 儲存 OCR 結果供回調使用
        self._pending_ocr_verification = ocr_result

        self.node.get_logger().info(
            f"🔍 查詢房間產品清單\n"
            f"  - 房間: {room_id}\n"
            f"  - OCR: {ocr_result}"
        )

        # 非同步呼叫
        future = self.sql_query_client.call_async(request)
        future.add_done_callback(self._handle_product_verification_response)

    def _handle_product_verification_response(self, future):
        """
        處理產品查詢回應並執行比對

        Args:
            future: SQL 查詢 future 物件

        Note:
            比對 OCR 識別結果與資料庫產品名稱清單
            Pass → 寫入 MR7101=1, MR7102=0
            Fail → 寫入 MR7101=0, MR7102=1
        """
        import json

        try:
            response = future.result()

            # 檢查查詢是否成功
            if not response.success:
                self.node.get_logger().error(
                    f"❌ 產品查詢失敗: {response.message}\n"
                    f"  - 寫入驗證 Fail"
                )
                self._write_verification_fail()
                return

            # 解析查詢結果
            result = json.loads(response.json_result)

            # 檢查是否有產品資料
            if not result or len(result) == 0:
                self.node.get_logger().warn(
                    "⚠️ 查無產品資料\n"
                    f"  - 請確認房間配置是否正確\n"
                    f"  - 寫入驗證 Fail"
                )
                self._write_verification_fail()
                return

            # 取得產品名稱清單
            product_names = [row['name'] for row in result]
            ocr_result = self._pending_ocr_verification

            # 比對 OCR 與產品名稱
            if ocr_result in product_names:
                self.node.get_logger().info(
                    f"✅ 產品驗證通過\n"
                    f"  - OCR 識別: {ocr_result}\n"
                    f"  - 產品清單: {product_names}\n"
                    f"  - 寫入 Pass 到 PLC"
                )
                self._write_verification_pass()
            else:
                self.node.get_logger().error(
                    f"❌ 產品驗證失敗\n"
                    f"  - OCR 識別: {ocr_result}\n"
                    f"  - 產品清單: {product_names}\n"
                    f"  - 停留在 WaitRobot 等待人工處理\n"
                    f"  - 寫入 Fail 到 PLC"
                )
                self._write_verification_fail()

        except Exception as e:
            self.node.get_logger().error(
                f"❌ 驗證回應處理異常: {e}\n"
                f"  - 寫入驗證 Fail"
            )
            self._write_verification_fail()

    def _write_verification_pass(self):
        """
        寫入驗證 Pass 到 PLC

        Note:
            MR7101 = 1 (ON)  → Pass
            MR7102 = 0 (OFF) → Not Fail
        """
        self.plc_client.async_write_bit(
            device_type='MR',
            address='7101',
            value=1,  # ON
            callback=self._plc_pass_callback
        )
        self.plc_client.async_write_bit(
            device_type='MR',
            address='7102',
            value=0,  # OFF
            callback=self._plc_fail_callback
        )

    def _write_verification_fail(self):
        """
        寫入驗證 Fail 到 PLC

        Note:
            MR7101 = 0 (OFF) → Not Pass
            MR7102 = 1 (ON)  → Fail
        """
        self.plc_client.async_write_bit(
            device_type='MR',
            address='7101',
            value=0,  # OFF
            callback=self._plc_pass_callback
        )
        self.plc_client.async_write_bit(
            device_type='MR',
            address='7102',
            value=1,  # ON
            callback=self._plc_fail_callback
        )

    def _plc_pass_callback(self, response):
        """PLC Pass 位元寫入回調"""
        if response.success:
            self.node.get_logger().info("✅ MR7101 Pass 位元寫入成功")
        else:
            self.node.get_logger().error(
                f"❌ MR7101 寫入失敗: {response.message}"
            )

    def _plc_fail_callback(self, response):
        """PLC Fail 位元寫入回調"""
        if response.success:
            self.node.get_logger().info("✅ MR7102 Fail 位元寫入成功")
        else:
            self.node.get_logger().error(
                f"❌ MR7102 寫入失敗: {response.message}"
            )
