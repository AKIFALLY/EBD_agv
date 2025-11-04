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

        # 優先處理 robot_finished，因為需要執行路徑刪除等清理工作
        if self.node.robot_finished or self.node.agv_status.AGV_LD_COMPLETE:
            # robot_finished=True 時，無論是否有路徑都要執行完成流程
            self.node.task.status_id = TaskStatus.COMPLETED # 已完成 (AGV-任務已完成)
            self.agvdbclient.async_update_task(self.node.task,self.task_update_callback)  # 更新任務狀態為執行中

            self.node.get_logger().info("✅ AGV 機器人已完成工作，回到mission select 狀態")
            self.node.robot_finished = False # 重置機器人完成狀態

            self.node.get_logger().info("🗑️ ForceON MR7008 刪除路徑資料")
            self.plc_client.async_force_on('MR', '7008', self.force_callback)     #PLC寫入路徑

            try:
                from agv_base.agv_states.mission_select_state import MissionSelectState
                context.set_state(MissionSelectState(self.node))
                return  # 任務完成後立即返回
            except Exception as e:
                self.node.get_logger().error(f"❌狀態轉換失敗 (WaitRobot → MissionSelect robot完成): {str(e)}")
                return

        # 只有在 robot_finished=False 且沒有路徑時才跳轉回 mission_select
        if not self.node.agv_status.AGV_PATH:
            self.node.get_logger().info("⚠️ AGV 在 WaitRobot 狀態下沒有路徑資料，回到 mission select 狀態")
            try:
                from agv_base.agv_states.mission_select_state import MissionSelectState
                context.set_state(MissionSelectState(self.node))
                return  # 立即返回，避免繼續執行後續邏輯
            except Exception as e:
                self.node.get_logger().error(f"❌狀態轉換失敗 (WaitRobot → MissionSelect 無路徑): {str(e)}")
                return
            


        

    def force_callback(self, response):
        if response.success:
            self.node.get_logger().info("✅ PLC force寫入成功")
        else:
            self.node.get_logger().warn("⚠️ PLC force寫入失敗")



    def task_update_callback(self,response):
        if response is None:
            print("❌ 未收到任務更新的回應（可能逾時或錯誤）")
            return

        if response.success:
            print(f"✅ 任務更新成功，訊息: {response.message}")
        else:
            print(f"⚠️ 任務更新失敗，訊息: {response.message}")