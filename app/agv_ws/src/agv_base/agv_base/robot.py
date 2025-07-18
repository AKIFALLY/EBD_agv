from agv_base.robot_parameter_abc import RobotParameterABC
from cargo_mover_agv.robot_states.cargo_robot_parameter import CargoRobotParameter
import rclpy
from rclpy.node import Node
from plc_interfaces.srv import ReadData
from plc_proxy.plc_client import PlcClient


class Robot():

    # PGNO定義(AGV TO ROBOT)
    CHG_PARA = "40000"
    PHOTO_RACK_UP = "40001"
    PHOTO_RACK_DOWN = "40002"
    PHOTO_BOX_IN = "40003"
    PHOTO_BOX_OUT = "40004"
    PHOTO_CLEANER = "40006"
    PHOTO_SOAKER = "40007"
    PHOTO_PRE_DRYER = "40008"
    PHOTO_OVEN = "40009"

    # PGNO定義(ROBOT TO AGV)
    IDLE = "50000"

    # ACTION定義
    ACTION_FROM_TO = "1"
    ACTION_FROM = "2"
    ACTION_TO = "3"

    # POSITION定義
    NONE_POSITION = "00"
    RACK_IN_POSITION = "01"
    RACK_OUT_POSITION = "02"
    BOX_IN_POSITION = "03"
    BOX_OUT_POSITION = "04"
    AGV_POSITION = "05"
    AGV_POSITION_SIDE = "55"
    CLEANER_POSITION = "06"
    SOAKER_POSITION = "07"
    PRE_DRYER_POSITION = "08"
    OVEN_POSITION = "09"

    # Carrier Status Constants
    CARRIER_STATUS_IDLE = 1                    # 空閒
    CARRIER_STATUS_IN_USE = 2                  # 使用中
    CARRIER_STATUS_FAULT = 3                   # 故障
    CARRIER_STATUS_PENDING = 4                 # 待處理
    CARRIER_STATUS_PROCESSING = 5              # 處理中
    CARRIER_STATUS_NG = 6                      # NG
    CARRIER_STATUS_MAINTENANCE = 7             # 維護中
    CARRIER_STATUS_COMPLETED = 8               # 已完成
    CARRIER_STATUS_ENTER_BOXIN_TRANSFER = 101  # 進入入口傳送箱
    CARRIER_STATUS_PREPARE_ENTER_CLEANER = 301  # 準備進入清洗機處理中
    CARRIER_STATUS_ENTER_CLEANER = 302         # 進入清洗機處理中
    CARRIER_STATUS_CLEANER_COMPLETED = 303     # 清洗機處理完成
    CARRIER_STATUS_PREPARE_ENTER_SOAKER = 401  # 準備進入強化機處理中
    CARRIER_STATUS_ENTER_SOAKER = 402          # 進入強化機處理中
    CARRIER_STATUS_SOAKER_COMPLETED = 403      # 強化機處理完成
    CARRIER_STATUS_PREPARE_ENTER_PRE_DRYER = 501  # 準備進入預烘乾機處理中
    CARRIER_STATUS_ENTER_PRE_DRYER = 502       # 進入預烘乾機處理中
    CARRIER_STATUS_PRE_DRYER_COMPLETED = 503   # 預烘乾機處理完成
    CARRIER_STATUS_PREPARE_ENTER_OVEN = 601    # 準備進入烘乾機處理中
    CARRIER_STATUS_ENTER_OVEN = 602            # 進入烘乾機處理中
    CARRIER_STATUS_OVEN_COMPLETED = 603        # 烘乾機處理完成
    CARRIER_STATUS_PREPARE_ENTER_BOXOUT_TRANSFER = 201  # 準備進入出入口傳送箱
    CARRIER_STATUS_ENTER_BOXOUT_TRANSFER = 202  # 進入出入口傳送箱

    def __init__(self,  node: Node, parameter: RobotParameterABC):
        self.node = node
        self.plc_client = PlcClient(node)
        self.parameter_start_address = "10.D"
        self.parameter = parameter

        # 讀取 PGNO 相關變數
        self.read_step = 0
        self.read_pgno_response = None
        self.read_pgno_success = False
        self.read_pgno_failed = False

        # 讀取 Error Number 相關變數
        self.read_error_number_response = None
        self.read_error_number_success = False
        self.read_error_number_failed = False

        # 讀取狀態追蹤
        self.pgno_read_requested = False
        self.error_number_read_requested = False

        # 其他操作相關變數
        self.write_step = 0
        self.update_response = None
        self.update_parameter_step = 0
        self.update_parameter_response = None
        self.update_parameter_success = False
        self.update_parameter_failed = False
        self.update_pgno_success = False
        self.update_pgno_failed = False

    def read_pgno_callback(self, response):
        """PGNO 讀取回調函數"""
        # 設定 PGNO 相關狀態
        self.read_pgno_success = response.success
        self.read_pgno_failed = not response.success
        self.read_pgno_response = response
        self.pgno_read_requested = False

        # 新增錯誤處理日誌
        if not response.success:
            error_msg = f"❌ read_pgno_callback 讀取失敗"
            if hasattr(response, 'message') and response.message:
                error_msg += f" - 錯誤訊息: {response.message}"
            if hasattr(response, 'error_code') and response.error_code:
                error_msg += f" - 錯誤代碼: {response.error_code}"
            self.node.get_logger().error(error_msg)
        else:
            # 新增成功處理日誌
            success_msg = f"✅ read_pgno_callback 讀取成功: 值為 {response.value}"
            self.node.get_logger().info(success_msg)

        # 檢查是否兩個讀取都完成
        if not self.pgno_read_requested and not self.error_number_read_requested:
            self.read_step = 0
            # 記錄 Error Number 資訊（如果有的話）
            if (self.read_error_number_response is not None and
                self.read_error_number_response.success and
                    self.read_error_number_response.value != "0"):
                self.node.get_logger().info(
                    f"🔍 Robot Error Number: {self.read_error_number_response.value}")

    def read_error_number_callback(self, response):
        """Error Number 讀取回調函數"""
        self.read_error_number_success = response.success
        self.read_error_number_failed = not response.success
        self.read_error_number_response = response
        self.error_number_read_requested = False

        # 新增錯誤處理日誌
        if not response.success:
            error_msg = f"❌ read_error_number_callback 讀取失敗"
            if hasattr(response, 'message') and response.message:
                error_msg += f" - 錯誤訊息: {response.message}"
            if hasattr(response, 'error_code') and response.error_code:
                error_msg += f" - 錯誤代碼: {response.error_code}"
            self.node.get_logger().error(error_msg)
        else:
            # 新增成功處理日誌
            success_msg = f"✅ read_error_number_callback 讀取成功: 值為 {response.value}"
            self.node.get_logger().info(success_msg)

        # 檢查是否兩個讀取都完成
        if not self.pgno_read_requested and not self.error_number_read_requested:
            self.read_step = 0
            # 記錄 Error Number 資訊（如果有的話）
            if (response.success and response.value != "0"):
                self.node.get_logger().info(f"🔍 Robot Error Number: {response.value}")

    def update_pgno_callback(self, response):
        self.update_pgno_success = response.success
        self.update_pgno_failed = not response.success
        self.update_pgno_response = response
        self.write_step = 0

    def update_parameter_callback(self, response):
        self.update_parameter_success = response.success
        self.update_parameter_failed = not response.success
        self.update_parameter_step = 0

    def read_robot_status(self):
        """同時讀取 PGNO 和 Error Number 的方法"""
        self.node.get_logger().info("Robot PGNO 和 Error Number 狀態更新中")

        # 設備類型和地址
        device_type = 'W'
        pgno_address = "100"
        error_number_address = "104"

        match self.read_step:
            case 0:
                # 同時發起兩個讀取請求
                self.node.get_logger().info("發送 PGNO 和 Error Number 讀取請求")

                # 讀取 PGNO
                self.pgno_read_requested = True
                self.plc_client.async_read_data(
                    device_type=device_type,
                    address=pgno_address,
                    callback=self.read_pgno_callback
                )

                # 讀取 Error Number
                self.error_number_read_requested = True
                self.plc_client.async_read_data(
                    device_type=device_type,
                    address=error_number_address,
                    callback=self.read_error_number_callback
                )

                self.read_step = 1

            case 1:
                self.node.get_logger().info("等待 PGNO 和 Error Number 讀取完成")

    def update_pgno(self, write_data):
        update_pgno_device_type = 'W'  # 假設寫入的是EM裝置
        update_pgno_address = "0"  # 假設寫入的是地址0
        match self.write_step:
            case 0:
                self.node.get_logger().info("Send Update PGNO")
                self.plc_client.async_write_data(
                    device_type=update_pgno_device_type, address=update_pgno_address, value=write_data, callback=self.update_pgno_callback)
                self.write_step = 1
            case 1:
                self.node.get_logger().info("等待Update PGNO")

    def update_parameter(self):
        """更新參數"""
        update_parameter_device_type = 'W'  # 假設寫入的是W裝置
        parameter = self.parameter.values()
        match self.update_parameter_step:
            case 0:
                self.plc_client.async_write_continuous_data(
                    device_type=update_parameter_device_type, start_address=self.parameter_start_address, values=parameter, callback=self.update_parameter_callback)
                self.update_parameter_step = 1

            case 1:
                self.node.get_logger().info("等待更新參數")

    # def write_pgno(self, command: str) -> bool:
#
    #    self.plc_client.write_data(
    #        device_type='EM', address="0", value=command)
    #    self.node.get_logger().info(f"Command sent: {command}")
#
    #    readback = self.plc_client.read_data(
    #        device_type='EM', address="0")
#
    #    return readback.value == command

    # def sync_parameter(self) -> bool:
    #    parameter = self.parameter.values()
    #    self.plc_client.write_continuous_data(
    #        device_type='EM', start_address=self.parameter_start_address, values=parameter)
    #    self.node.get_logger().info(f"Command_Continuous sent: {parameter}")
#
    #    readback = self.plc_client.read_continuous_data(
    #        device_type='EM', start_address="1000.D", count=len(parameter))
#
    #    return readback.values == parameter
#
    # def read_pgno(self) -> str:
    #    result = self.plc_client.read_data(
    #        device_type='EM', address="256")
#
    #    return result.value
#
    # def make_commmand(self, command_tpye: str, _from: str, _to: str):
    #    return f"{command_tpye}{_from}{_to}"
#
    # def cleanup(self):
    #    self.plc_client.destroy()
#

# def main():
#    rclpy.init()
#    robot = Robot(Node(node_name="test_robot", namespace="/cargo02"))
#    try:
#        command = robot.make_commmand("1", "02", "03")
#        robot.send_robot_command(command)
#
#        result = robot.read_robot_command()
#        print(
#            f"Read value: {result.value}" if result else "Failed to read PLC data")
#
#    finally:
#        robot.cleanup()
#
#
# if __name__ == '__main__':
#    main()
