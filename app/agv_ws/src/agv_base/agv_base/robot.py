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
    CLEANER_POSISION = "06"
    SOAKER_POSISION = "07"
    PRE_DRYER_POSISION = "08"
    OVEN_POSISION = "09"

    def __init__(self,  node: Node, parameter: RobotParameterABC):
        self.node = node
        self.plc_client = PlcClient(node)
        self.parameter_start_address = "2A.D"
        self.parameter = parameter
        self.read_step = 0
        self.read_pgno_response = None
        self.write_step = 0
        self.update_response = None
        self.update_parameter_step = 0
        self.update_parameter_response = None
        self.update_parameter_success = False
        self.update_parameter_failed = False
        self.read_pgno_success = False
        self.read_pgno_failed = False
        self.update_pgno_success = False
        self.update_pgno_failed = False

    def read_pgno_callback(self, response):
        self.read_pgno_success = response.success
        self.read_pgno_failed = not response.success
        self.read_pgno_response = response
        self.read_step = 0

    def update_pgno_callback(self, response):
        self.update_pgno_success = response.success
        self.update_pgno_failed = not response.success
        self.update_pgno_response = response
        self.write_step = 0

    def update_parameter_callback(self, response):
        self.update_parameter_success = response.success
        self.update_parameter_failed = not response.success
        self.update_parameter_step = 0

    def read_pgno(self):
        self.node.get_logger().info("Robot PGNO狀態更新中")
        read_pgno_device_type = 'W'  # 假設讀取的是EM裝置
        read_pgno_address = "100"  # 假設讀取的是地址256
        match self.read_step:
            case 0:
                self.node.get_logger().info("Send Read PGNO")
                self.plc_client.async_read_data(
                    device_type=read_pgno_device_type, address=read_pgno_address, callback=self.read_pgno_callback)
                self.read_step = 1

            case 1:
                self.node.get_logger().info("等待 Read PGNO")

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
