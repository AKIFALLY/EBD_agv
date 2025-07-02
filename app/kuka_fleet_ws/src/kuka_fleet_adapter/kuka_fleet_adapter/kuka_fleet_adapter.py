import rclpy
import time
from rclpy.node import Node
from kuka_fleet_adapter.kuka_api_client import KukaApiClient
# 假設您有自定義的 message
# from your_interfaces.msg import RobotStatus, ContainerStatus


class KukaFleetAdapter:
    # AGV 狀態常數
    # |status | 机器人状态 1-离场；2-离线；3-空闲；4-任务中；5-充电中；6-更新中；7-异常 robot status 1-Removed;2-Offline;3-idle;4-running;5-charging;6-updating;7-error   |integer(int32)  |    |
    STATUS_REMOVED = 1  # 離場
    STATUS_OFFLINE = 2  # 離線
    STATUS_IDLE = 3  # 空閒
    STATUS_RUNNING = 4  # 任務中
    STATUS_CHARGING = 5  # 充電中
    STATUS_UPDATING = 6  # 更新中
    STATUS_ERROR = 7  # 錯誤

    MISSION_MOVE = "MOVE"  # 移動
    MISSION_RACK_MOVE = "RACK_MOVE"  # 搬運

    MAP_LAYOUT_DISTRICT = "test-test1"

    def __init__(self, node: Node):
        """
        初始化 Kuka Fleet Adapter。
        這個類別本身不是一個 ROS 節點，而是作為一個擴充功能，
        由一個現有的 ROS 節點來持有和使用。

        :param node: 持有此 adapter 的 ROS 2 Node 物件。
        """
        self.node = node
        self.logger = node.get_logger()
        self.logger.info("Kuka Fleet Adapter 擴充功能初始化...")

        # 透過傳入的 node 來宣告和取得參數
        self.node.declare_parameter(
            'api_base_url', 'http://192.168.11.206:10870')
        self.node.declare_parameter('api_username', 'admin')
        self.node.declare_parameter('api_password', 'Admin')
        self.node.declare_parameter(
            'query_cycle_time', 0.1)  # 0.1秒查詢一次kuka車輛及容器狀態
        self.node.declare_parameter(
            'timer_period', 0.05)  # 0.05秒 Monitor 的間隔時間

        base_url = self.node.get_parameter(
            'api_base_url').get_parameter_value().string_value
        username = self.node.get_parameter(
            'api_username').get_parameter_value().string_value
        password = self.node.get_parameter(
            'api_password').get_parameter_value().string_value
        self.cycle_time = self.node.get_parameter(
            'query_cycle_time').get_parameter_value().double_value

        self.timer_period = self.node.get_parameter(
            'timer_period').get_parameter_value().double_value  # seconds
        # 初始化 Kuka API Client
        self.api_client = KukaApiClient(
            base_url=base_url, username=username, password=password)
        self.query_timer = None
        self.is_querying = False
        self.last_query_time = 0
        self.query_timeout = 5  # 5 seconds timeout

        if not self.api_client.token:
            self.logger.error("Kuka API 登入失敗，Adapter 將不會啟動。")
        else:
            self.logger.info("Kuka API 登入成功。")
            # 建立 Publisher (未來可取消註解並使用自定義 message)
            # self.robot_status_publisher = self.node.create_publisher(RobotStatus, 'kuka/robot_status', 10)
            # self.container_status_publisher = self.node.create_publisher(ContainerStatus, 'kuka/container_status', 10)

    def start_monitoring(self):
        """啟動週期性狀態監控。"""
        if self.api_client and self.api_client.token:
            if self.query_timer is None:
                self.query_timer = self.node.create_timer(
                    self.timer_period, self.monitor_robot_and_container)
                self.logger.info(
                    f"Kuka Fleet Adapter 已啟動，每 {self.timer_period} 秒檢查一次。")
            else:
                self.logger.warn("監控已經在執行中。")
        else:
            self.logger.error("無法啟動監控，因為 API 未成功登入。")

    def stop_monitoring(self):
        """停止週期性狀態監控。"""
        if self.query_timer:
            self.query_timer.cancel()
            self.query_timer = None
            self.logger.info("Kuka Fleet Adapter 已停止。")

    def monitor_robot_and_container(self):
        now = time.time()

        if self.is_querying:
            if now - self.last_query_time > self.query_timeout:
                self.logger.warn(f"查詢超時 (超過 {self.query_timeout} 秒)，重設查詢狀態。")
                self.is_querying = False
            else:
                # Still waiting for the previous query to complete
                return
        delta_time = now - self.last_query_time

        if (delta_time > self.cycle_time):
            # self.logger.info(f"查:{delta_time:.2f}")
            self.is_querying = True
            self.last_query_time = now
            self.logger.debug(f"開始查詢機器人與容器資訊...{delta_time}")

            try:
                # 查詢機器人資訊
                robot_res = self.api_client.robot_query({})
                robots = []
                if robot_res.get("success") and "data" in robot_res:
                    robots = robot_res["data"]
                    # self.logger.info(f"成功查詢到 {len(robots)} 台機器人資訊。")
                    for robot in robots:
                        # self.logger.debug(
                        #    f"  - Robot: {robot.get('robotId')}, Status: {robot.get('status')}, Battery: {robot.get('batteryLevel')}%")
                        pass
                else:
                    self.logger.warn(
                        f"無法查詢 robot 資訊: {robot_res.get('message', '未知錯誤')}")

                self.on_robot_query_complete(robots)

                # 查詢容器資訊
                container_res = self.api_client.container_query_all({})
                containers = []
                if container_res.get("success") and "data" in container_res:
                    containers = container_res["data"]
                    # self.logger.info(f"成功查詢到 {len(containers)} 個容器資訊。")
                    for container in containers:
                        self.logger.debug(
                            f"  - Container: {container.get('containerCode')}, Status: {container.get('inMapStatus')}")
                        pass
                else:
                    self.logger.warn(
                        f"無法查詢容器資訊: {container_res.get('message', '未知錯誤')}")

                self.on_container_query_complete(containers)
            finally:
                self.is_querying = False

    def on_robot_query_complete(self, robots):
        """
        當機器人資訊查詢完成時的 callback。
        持有此 adapter 的節點 (例如 rcs_core) 可以在這裡掛載處理邏輯。
        """
        self.logger.debug(
            f"[Callback] Robot query complete. Robot count: {len(robots)}")
        # 例如: self.node.some_method_to_update_robots(robots)

    def on_container_query_complete(self, containers):
        """
        當容器資訊查詢完成時的 callback。
        持有此 adapter 的節點 (例如 rcs_core) 可以在這裡掛載處理邏輯。
        """
        self.logger.debug(
            f"[Callback] Container query complete. Container count: {len(containers)}")
        # 例如: self.node.some_method_to_update_containers(containers)

    def select_agv(self, status: int = None, robot_id: str = None):
        """
        根據指定的 status 與 filter 查詢並過濾 AGV。
        :param status: 狀態常數 (建議用 KukaFleetAdapter.STATUS_IDLE 等)
        :param filter: 機器id列表
        :return: 過濾後的 AGV 列表
        """
        query = {}
        if robot_id:
            query = {
                "robotId": robot_id
                # "robotType": robot_type
            }
        res = self.api_client.robot_query(query)
        agvs = []
        if not status:
            return agvs
        if res.get("success") and "data" in res:
            agvs = [agv for agv in res["data"] if agv.get("status") == status]
            # 將 robotId 設為 agv['id']，方便外部統一取用
            for agv in agvs:
                agv["id"] = agv.get("robotId")
        else:
            self.logger.warn(f"查詢 AGV 失敗: {res.get('message', '未知錯誤')}")
        return agvs

    def workflow(self, workflow: str, robot_id: int, kuka_mission_code: str):
        """
        執行指定的 workflow
        """
        kuka_mission = {
            "orgId": "Ching-Tech",
            "requestId": kuka_mission_code,
            "missionCode": kuka_mission_code,
            "missionType": self.MISSION_MOVE,
            "robotModels": [
                "KMP 400i diffDrive"
            ],
            "robotIds": [
                robot_id
            ],
            "robotType": "LIFT",
            "templateCode": workflow,
            "missionData": []  # <--- 用你組裝的 mission_data
        }

        res = self.api_client.submit_mission(kuka_mission)

        return res

    def move(self, nodes: list, robot_id: int, kuka_mission_code: str):
        """
        根據指定的 points 移動
        nodes: 可以是 int list 或 str list
        """
        mission_data = []
        for idx, node in enumerate(nodes):
            if isinstance(node, str):
                position = node  # 字串傳入當作是(node uuid)
            else:
                position = self.MAP_LAYOUT_DISTRICT + \
                    "-" + str(node)  # 整數傳入當作是 nodeNumber
            mission_data.append({
                "sequence": idx + 1,
                "position": position,
                "type": "NODE_POINT",
                "passStrategy": "AUTO"  # passStrategy 當前任務結束後放行策略 是必填的，值可以是 AUTO, MANUAL
            })

        kuka_mission = {
            "orgId": "Ching-Tech",
            "requestId": kuka_mission_code,
            "missionCode": kuka_mission_code,
            "missionType": self.MISSION_MOVE,
            "robotModels": [
                "KMP 400i diffDrive"
            ],
            "robotIds": [
                robot_id
            ],
            "robotType": "LIFT",
            "priority": 1,
            "missionData": mission_data
        }

        res = self.api_client.submit_mission(kuka_mission)

        return res

    def rack_move(self, nodes: list, robot_id: int, kuka_mission_code: str):
        """
        根據指定的 points 移動
        nodes: 可以是 int list 或 str list
        """
        mission_data = []
        for idx, node in enumerate(nodes):
            if isinstance(node, str):
                position = node
            else:
                position = self.MAP_LAYOUT_DISTRICT + "-" + str(node)
            mission_data.append({
                "sequence": idx + 1,
                "position": position,
                "type": "NODE_POINT",
                "passStrategy": "AUTO"  # passStrategy 當前任務結束後放行策略 是必填的，值可以是 AUTO, MANUAL
            })

        kuka_mission = {
            "orgId": "Ching-Tech",
            "requestId": kuka_mission_code,
            "missionCode": kuka_mission_code,
            "missionType": self.MISSION_RACK_MOVE,
            "robotModels": [
                "KMP 400i diffDrive"
            ],
            "robotIds": [
                robot_id
            ],
            "robotType": "LIFT",
            "priority": 1,
            "missionData": mission_data
        }

        res = self.api_client.submit_mission(kuka_mission)

        return res


def main(args=None):
    """
    這個 main 函數僅用於示範如何使用 KukaFleetAdapter。
    在實際應用中，您應該在自己的節點 (例如 rcs_core.py) 中
    建立 KukaFleetAdapter 的實例。
    """
    rclpy.init(args=args)

    # 建立一個範例節點
    example_node = rclpy.create_node('kuka_adapter_demo_node')

    # 將節點傳遞給 Adapter
    adapter = KukaFleetAdapter(example_node)

    # 啟動監控
    adapter.start_monitoring()

    try:
        # 使用 spin 來讓計時器和 ROS 事件循環保持運作
        rclpy.spin(example_node)
    except KeyboardInterrupt:
        example_node.get_logger().info("節點被手動關閉。")
    finally:
        # 清理
        adapter.stop_monitoring()
        example_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
