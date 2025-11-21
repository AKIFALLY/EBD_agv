"""
Cargo Mover AGV 專用的 WaitRobotState

整合 OCR 產品識別與 W,02 狀態控制：
1. 訂閱 sensor/ocr 話題取得產品識別結果
2. 定期讀取 W,02 狀態（每 1 秒）
3. W,02=0 時處理 OCR 資料，執行產品/制程驗證
4. 驗證結果寫入 W,02：
   - W,02=1: OCR 比對成功
   - W,02=2: OCR 比對失敗
   - W,02=3: 資料庫讀取失敗（服務不可用/查詢失敗/產品不存在/異常）
"""

from agv_base.agv_states.wait_robot_state import WaitRobotState
from rclpy.node import Node
from std_msgs.msg import String


class CargoWaitRobotState(WaitRobotState):
    """
    Cargo Mover AGV 專用的 WaitRobotState

    新增功能：
    - OCR 產品識別整合
    - W,02 狀態控制（讀取/寫入）
    - 產品制程適配性驗證
    - PLC 驗證結果輸出（W,02 單一接口：1=成功, 2=失敗, 3=異常）
    """

    def __init__(self, node: Node):
        super().__init__(node)
        self.plc_client = node.plc_client  # 引用 node 的 plc_client

        # OCR 相關變數
        self.ocr_enabled = True  # Cargo AGV 固定啟用 OCR
        self.latest_ocr_result = None
        self.ocr_received_time = None
        self.ocr_timeout_seconds = 10.0
        self._pending_ocr_verification = None  # 待驗證的 OCR 結果

        # W,02 狀態控制（Cargo AGV 專用）
        self.w02_value = None           # W,02 當前值
        self.w02_reading = False        # 是否正在讀取
        self.w02_read_counter = 0       # 讀取計數器
        self.w02_read_interval = 20     # 每 20 個循環讀取一次（1 秒）
        self.ocr_skip_log_counter = 0   # OCR 跳過日誌計數器
        self.ocr_skip_log_interval = 20 # 每 20 次跳過才輸出一次日誌

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
        """進入 WaitRobotState"""
        super().enter()
        self.node.get_logger().info("🤖 Cargo AGV 進入: WaitRobot 狀態 (OCR 已啟用)")

    def leave(self):
        """離開 WaitRobotState，清理 OCR 資源"""
        self.node.get_logger().info("🚪 Cargo AGV 離開 WaitRobot 狀態")

        # 清理 OCR subscription
        if hasattr(self, 'ocr_subscription') and self.ocr_subscription:
            try:
                self.node.destroy_subscription(self.ocr_subscription)
                self.ocr_subscription = None
                self.node.get_logger().info("✅ OCR subscription 已清理")
            except Exception as e:
                self.node.get_logger().warn(f"⚠️ OCR subscription 清理失敗: {e}")

        # 呼叫父類的 leave()
        super().leave()

    def handle(self, context):
        """
        覆寫 handle 方法，增加 W,02 定期讀取

        流程：
        1. 定期讀取 W,02 狀態（每 1 秒）
        2. 呼叫父類 handle() 處理基礎邏輯
        """
        # 🔍 定期讀取 W,02 狀態（僅 Cargo AGV）
        if not self.w02_reading:
            self.w02_read_counter += 1
            if self.w02_read_counter >= self.w02_read_interval:
                self.w02_read_counter = 0
                self._read_w02_status()

        # 呼叫父類 handle() 處理基礎邏輯
        super().handle(context)

    # ==================== OCR 整合功能（Cargo AGV 專用）====================

    def _ocr_callback(self, msg: String):
        """
        OCR 資料回調處理

        Args:
            msg: OCR 識別結果 (std_msgs/String)
        """
        # ⚠️ 檢查 W,02 狀態：只在 W,02=1 時不處理（其他狀態都處理）
        if self.w02_value == 1:
            self.ocr_skip_log_counter += 1
            if self.ocr_skip_log_counter >= self.ocr_skip_log_interval:
                self.ocr_skip_log_counter = 0
                self.node.get_logger().info(
                    f"⏸️ W,02=1（已驗證通過），已跳過 {self.ocr_skip_log_interval} 次 OCR 處理"
                )
            return

        # 重置跳過計數器（開始處理 OCR）
        self.ocr_skip_log_counter = 0

        # ✅ 清除前後空白和不可見字元
        raw_ocr = msg.data
        cleaned_ocr = raw_ocr.strip()

        self.latest_ocr_result = cleaned_ocr
        self.ocr_received_time = self.node.get_clock().now()

        self.node.get_logger().info(
            f"📄 收到 OCR 識別結果\n"
            f"  - 原始: '{raw_ocr}' (len={len(raw_ocr)})\n"
            f"  - 清理: '{cleaned_ocr}' (len={len(cleaned_ocr)})"
        )

        # 記錄到任務日誌（未來可擴展寫入資料庫）
        self._log_ocr_to_task(cleaned_ocr)

        # 直接執行制程驗證（不再寫入 DM8000）
        room_id = self._extract_room_id_from_workid()
        if room_id is not None:
            self._verify_product_process_match(room_id, cleaned_ocr)

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

    def check_ocr_available(self) -> bool:
        """
        檢查是否有 OCR 資料（供 Robot 狀態調用）

        Returns:
            bool: True 表示有資料或已超時，False 表示等待中

        Note:
            超時後返回 True 並記錄警告，但不阻塞任務執行
        """
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

    def _verify_product_process_match(self, room_id: int, ocr_result: str):
        """
        驗證產品制程與房間制程是否匹配（非同步）

        Args:
            room_id: 房間編號
            ocr_result: OCR 識別的產品名稱

        Workflow:
            1. 查詢產品的 process_settings_id（通過 product.name）
            2. 查詢房間的 process_settings_id（通過 room.id）
            3. 比對兩者是否相等
            4. 寫入 W,02 結果：
               - 成功: W,02=1 (Pass)
               - 失敗: W,02=2 (Fail)
               - 異常: W,02=3 (服務不可用/查詢失敗/產品不存在/異常)

        Note:
            透過 SqlQuery service 查詢產品和房間的制程信息
            查詢 SQL:
                SELECT p.process_settings_id, r.process_settings_id,
                       ps_product.soaking_times, ps_room.soaking_times, ...
                FROM product p CROSS JOIN room r ...
                WHERE p.name = '{ocr_result}' AND r.id = {room_id}
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
                "❌ SqlQuery service 不可用，無法驗證制程\n"
                "  - 寫入 W,02=3 (資料庫服務不可用)"
            )
            self._write_w02_value(3, "DB Service Unavailable")
            return

        # 建立查詢請求：查詢產品和房間的制程信息
        request = SqlQuery.Request()
        request.query_string = f"""
            SELECT
                p.process_settings_id AS product_process_id,
                p.name AS product_name,
                p.size AS product_size,
                r.process_settings_id AS room_process_id,
                r.name AS room_name,
                ps_product.soaking_times AS product_soaking_times,
                ps_product.description AS product_process_desc,
                ps_room.soaking_times AS room_soaking_times,
                ps_room.description AS room_process_desc
            FROM product p
            CROSS JOIN room r
            LEFT JOIN process_settings ps_product ON p.process_settings_id = ps_product.id
            LEFT JOIN process_settings ps_room ON r.process_settings_id = ps_room.id
            WHERE TRIM(p.name) = TRIM('{ocr_result}') AND r.id = {room_id}
        """

        # 儲存 OCR 結果供回調使用
        self._pending_ocr_verification = ocr_result

        self.node.get_logger().info(
            f"🔍 驗證產品制程匹配\n"
            f"  - 房間: {room_id}\n"
            f"  - OCR: {ocr_result}"
        )

        # 非同步呼叫
        future = self.sql_query_client.call_async(request)
        future.add_done_callback(self._handle_product_verification_response)

    def _handle_product_verification_response(self, future):
        """
        處理制程驗證查詢回應並執行比對

        Args:
            future: SQL 查詢 future 物件

        Note:
            比對產品的 process_settings_id 與房間的 process_settings_id
            - 制程匹配 → 寫入 W,02=1 (Pass)
            - 制程不匹配 → 寫入 W,02=2 (Fail)
            - 查詢失敗/產品不存在/異常 → 寫入 W,02=3
        """
        import json

        try:
            response = future.result()

            # 檢查查詢是否成功
            if not response.success:
                self.node.get_logger().error(
                    f"❌ 制程驗證查詢失敗: {response.message}\n"
                    f"  - 寫入 W,02=3 (查詢失敗)"
                )
                self._write_w02_value(3, "DB Query Failed")
                return

            # 解析查詢結果
            result = json.loads(response.json_result)

            # 檢查是否找到產品和房間資料
            if not result or len(result) == 0:
                self.node.get_logger().error(
                    f"❌ 產品不存在於系統中\n"
                    f"  - OCR 識別: {self._pending_ocr_verification}\n"
                    f"  - 請確認產品是否已在資料庫中註冊\n"
                    f"  - 寫入 W,02=3 (產品不存在)"
                )
                self._write_w02_value(3, "Product Not Found")
                return

            # 取得制程信息
            data = result[0]
            product_process_id = data.get('product_process_id')
            room_process_id = data.get('room_process_id')
            product_name = data.get('product_name')
            product_size = data.get('product_size')
            room_name = data.get('room_name')
            product_soaking = data.get('product_soaking_times')
            product_process_desc = data.get('product_process_desc')
            room_soaking = data.get('room_soaking_times')
            room_process_desc = data.get('room_process_desc')

            # 🔍 關鍵變更：比對制程 ID 而非產品名稱
            if product_process_id == room_process_id:
                # ✅ 制程匹配：寫入 PLC Pass
                self.node.get_logger().info(
                    f"✅ 制程驗證通過\n"
                    f"  - OCR 識別: {self._pending_ocr_verification}\n"
                    f"  - 產品信息: {product_name} ({product_size})\n"
                    f"  - 產品制程: ID={product_process_id}, 泡藥{product_soaking}次 ({product_process_desc})\n"
                    f"  - 房間信息: {room_name}\n"
                    f"  - 房間制程: ID={room_process_id}, 泡藥{room_soaking}次 ({room_process_desc})\n"
                    f"  - 制程匹配，允許進入\n"
                    f"  - 寫入 W,02=1 (Pass)"
                )
                # 寫入 W,02=1（成功）
                self._write_w02_value(1, "Pass")
            else:
                # ❌ 制程不匹配：寫入 W,02=2（失敗）
                self.node.get_logger().error(
                    f"❌ 制程驗證失敗\n"
                    f"  - OCR 識別: {self._pending_ocr_verification}\n"
                    f"  - 產品信息: {product_name} ({product_size})\n"
                    f"  - 產品需求: 制程{product_process_id}（泡藥{product_soaking}次 - {product_process_desc}）\n"
                    f"  - 房間信息: {room_name}\n"
                    f"  - 房間能力: 制程{room_process_id}（泡藥{room_soaking}次 - {room_process_desc}）\n"
                    f"  - 制程不匹配，禁止進入\n"
                    f"  - 寫入 W,02=2 (Fail)"
                )
                # 寫入 W,02=2（失敗）
                self._write_w02_value(2, "Fail")

        except Exception as e:
            # ❌ 異常處理：寫入 W,02=3
            self.node.get_logger().error(
                f"❌ 驗證回應處理異常: {e}\n"
                f"  - 寫入 W,02=3 (異常)"
            )
            self._write_w02_value(3, "Exception")

    # ========================================================================
    # 🔧 W,02 狀態控制相關方法（Cargo AGV 專用）
    # ========================================================================

    def _read_w02_status(self):
        """
        持續讀取 W,02 狀態（Cargo AGV 專用）

        Note:
            使用非同步讀取，不阻塞主循環
            每 10 個循環讀取一次（約 0.5 秒）
        """
        self.w02_reading = True
        self.plc_client.async_read_data(
            device_type='W',
            address='2',
            callback=self._w02_read_callback
        )

    def _w02_read_callback(self, response):
        """
        W,02 讀取回調

        Args:
            response: PLC 讀取回應

        Note:
            更新 self.w02_value 供 OCR callback 使用
        """
        self.w02_reading = False
        if response.success:
            self.w02_value = int(response.value)
            self.node.get_logger().debug(f"📡 W,02 狀態: {self.w02_value}")
        else:
            self.node.get_logger().warn(f"⚠️ W,02 讀取失敗: {response.message}")

    def _write_w02_value(self, value: int, reason: str):
        """
        寫入 W,02 狀態（Cargo AGV 專用）

        Args:
            value: 寫入值
                - 1: OCR 比對成功（Pass）
                - 2: OCR 比對失敗（Fail）
                - 3: 資料庫讀取失敗（服務不可用/查詢失敗/產品不存在/異常）
            reason: 原因標籤（用於日誌記錄）

        Note:
            使用非同步寫入，不阻塞主循環
            PLC 會在後續流程中將 W,02 重置為 0，觸發下一輪 OCR 判斷
        """
        self.plc_client.async_write_data(
            device_type='W',
            address='2',
            value=str(value),
            callback=lambda res: self._w02_write_callback(res, value, reason)
        )

    def _w02_write_callback(self, response, value: int, reason: str):
        """
        W,02 寫入回調

        Args:
            response: PLC 寫入回應
            value: 寫入的值
            reason: 原因標籤

        Note:
            寫入成功後會同步更新 self.w02_value
        """
        if response.success:
            self.w02_value = value  # 更新本地狀態
            self.node.get_logger().info(f"✅ W,02 寫入成功: {value} ({reason})")
        else:
            self.node.get_logger().error(f"❌ W,02 寫入失敗: {response.message}")
