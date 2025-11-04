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
    CARRIER_STATUS_PREPARE_ENTER_PRE_DRYER = 501  # 準備進入預烘機處理中
    CARRIER_STATUS_ENTER_PRE_DRYER = 502       # 進入預烘機處理中
    CARRIER_STATUS_PRE_DRYER_COMPLETED = 503   # 預烘機處理完成
    CARRIER_STATUS_PREPARE_ENTER_OVEN = 601    # 準備進入烤箱處理中
    CARRIER_STATUS_ENTER_OVEN = 602            # 進入烤箱處理中
    CARRIER_STATUS_OVEN_COMPLETED = 603        # 烤箱處理完成
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

        # 讀取 Robot Parameter 相關變數
        self.read_robot_parameter_step = 0
        self.read_robot_parameter_response = None
        self.read_robot_parameter_success = False
        self.read_robot_parameter_failed = False
        self.robot_parameter_read_requested = False

        # Robot Parameter 個別屬性 (W110~W12F)
        self.w110 = None
        self.w111 = None
        self.w112 = None
        self.w113 = None
        self.w114 = None
        self.w115 = None
        self.w116 = None
        self.w117 = None
        self.w118 = None
        self.w119 = None
        self.w11a = None
        self.w11b = None
        self.w11c = None
        self.w11d = None
        self.w11e = None
        self.w11f = None
        self.w120 = None
        self.w121 = None
        self.w122 = None
        self.w123 = None
        self.w124 = None
        self.w125 = None
        self.w126 = None
        self.w127 = None
        self.w128 = None
        self.w129 = None
        self.w12a = None
        self.w12b = None
        self.w12c = None
        self.w12d = None
        self.w12e = None
        self.w12f = None

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
            #self.node.get_logger().info(success_msg)

        # 檢查是否兩個讀取都完成
        if not self.pgno_read_requested and not self.error_number_read_requested:
            self.read_step = 0
            # 記錄 Error Number 資訊（如果有的話）
            if (self.read_error_number_response is not None and
                self.read_error_number_response.success and
                    self.read_error_number_response.value != "0"):
                pass
                #self.node.get_logger().info(
                #    f"🔍 Robot Error Number: {self.read_error_number_response.value}")

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
            #self.node.get_logger().info(success_msg)

        # 檢查是否兩個讀取都完成
        if not self.pgno_read_requested and not self.error_number_read_requested:
            self.read_step = 0
            # 記錄 Error Number 資訊（如果有的話）
            if (response.success and response.value != "0"):
                #self.node.get_logger().info(f"🔍 Robot Error Number: {response.value}")
                pass

    def read_robot_parameter_callback(self, response):
        """Robot Parameter 讀取回調函數"""
        self.read_robot_parameter_success = response.success
        self.read_robot_parameter_failed = not response.success
        self.read_robot_parameter_response = response
        self.robot_parameter_read_requested = False

        # 新增錯誤處理日誌
        if not response.success:
            error_msg = f"❌ read_robot_parameter_callback 讀取失敗"
            if hasattr(response, 'message') and response.message:
                error_msg += f" - 錯誤訊息: {response.message}"
            if hasattr(response, 'error_code') and response.error_code:
                error_msg += f" - 錯誤代碼: {response.error_code}"
            self.node.get_logger().error(error_msg)
        else:
            # 更新個別屬性 (W110~W12F)
            if len(response.values) >= 32:
                self.w110 = response.values[0]
                self.w111 = response.values[1]
                self.w112 = response.values[2]
                self.w113 = response.values[3]
                self.w114 = response.values[4]
                self.w115 = response.values[5]
                self.w116 = response.values[6]
                self.w117 = response.values[7]
                self.w118 = response.values[8]
                self.w119 = response.values[9]
                self.w11a = response.values[10]
                self.w11b = response.values[11]
                self.w11c = response.values[12]
                self.w11d = response.values[13]
                self.w11e = response.values[14]
                self.w11f = response.values[15]
                self.w120 = response.values[16]
                self.w121 = response.values[17]
                self.w122 = response.values[18]
                self.w123 = response.values[19]
                self.w124 = response.values[20]
                self.w125 = response.values[21]
                self.w126 = response.values[22]
                self.w127 = response.values[23]
                self.w128 = response.values[24]
                self.w129 = response.values[25]
                self.w12a = response.values[26]
                self.w12b = response.values[27]
                self.w12c = response.values[28]
                self.w12d = response.values[29]
                self.w12e = response.values[30]
                self.w12f = response.values[31]

            # 新增成功處理日誌
            success_msg = f"✅ read_robot_parameter_callback 讀取成功: {len(response.values)} 個參數"
            self.node.get_logger().info(success_msg)

        self.read_robot_parameter_step = 0

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
        #self.node.get_logger().info("Robot PGNO 和 Error Number 狀態更新中")

        # 設備類型和地址
        device_type = 'W'
        pgno_address = "100"
        error_number_address = "104"

        match self.read_step:
            case 0:
                # 同時發起兩個讀取請求
                #self.node.get_logger().info("發送 PGNO 和 Error Number 讀取請求")

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

    def read_robot_parameter(self):
        """讀取 Robot Parameter (從 W110 開始連續讀取 32 word)"""
        device_type = 'W'
        address = '110'
        count = 32

        match self.read_robot_parameter_step:
            case 0:
                self.robot_parameter_read_requested = True
                self.plc_client.async_read_continuous_data(
                    device_type=device_type,
                    start_address=address,
                    count=count,
                    callback=self.read_robot_parameter_callback
                )
                self.read_robot_parameter_step = 1

            case 1:
                self.node.get_logger().info("等待 Robot Parameter 讀取完成")

    def check_parameter_match(self, expected_values: dict) -> tuple[bool, str]:
        """
        檢查讀取的參數是否符合預期

        Args:
            expected_values: {屬性名: 預期值}
                            例如 {'w114': 1, 'w115': 2}
                            屬性名使用小寫 (w110-w12f)

        Returns:
            (是否全部匹配, 訊息)
            - True: 所有參數都匹配
            - False: 有參數不匹配或尚未讀取

        Example:
            >>> match, msg = robot.check_parameter_match({'w114': 1, 'w115': 2})
            >>> if not match:
            >>>     logger.warn(f"參數檢查: {msg}")
        """
        if not self.read_robot_parameter_success:
            return (False, "參數尚未讀取成功")

        mismatches = []
        for attr_name, expected_value in expected_values.items():
            actual_value = getattr(self, attr_name, None)
            if actual_value is None:
                mismatches.append(f"{attr_name.upper()}: 未讀取到值")
            elif actual_value != expected_value:
                mismatches.append(
                    f"{attr_name.upper()}: 預期={expected_value}, 實際={actual_value}"
                )

        if mismatches:
            return (False, "; ".join(mismatches))

        return (True, "所有參數匹配")

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
