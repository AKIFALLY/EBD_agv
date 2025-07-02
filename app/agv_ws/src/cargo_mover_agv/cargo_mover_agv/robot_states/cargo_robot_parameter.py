from agv_base.robot_parameter_abc import RobotParameterABC


class CargoRobotParameter(RobotParameterABC):
    RACK_MARK_OFS_X = -356
    RACK_MARK_OFS_Y = 0
    RACK_MARK_OFS_Z = 730
    RACK_HOME_X = 0
    RACK_HOME_Y = 0
    RACK_HOME_Z = 0
    RACK_POS1_Y = 531.9
    RACK_POS1_Z = -230
    RACK_FORK_GET_HEIGHT = 47
    RACK_FORK_GET_DEEP = 236
    RACK_FORK_PUT_HEIGHT = 47
    RACK_FORK_PUT_DEEP = 236
    RACK_POS1_X = -120
    RACK_POS2_X = 0
    RACK_POS2_Y = 0
    RACK_POS2_Z = 600
    RACK_POS2_RZ = 0
    BOXIN_HOME_X = 0
    BOXIN_HOME_Y = 0
    BOXIN_HOME_Z = 0
    BOXIN_POS1_Y = 190
    BOXIN_POS1_Z = 350
    BOXIN_FORK_GET_HEIGHT = 47
    BOXIN_FORK_GET_DEEP = 318
    BOXIN_FORK_PUT_HEIGHT = 47
    BOXIN_FORK_PUT_DEEP = 318
    BOXIN_POS1_X = -120
    BOXIN_POS2_X = 0
    BOXIN_POS2_Y = 0
    BOXIN_POS2_Z = 0
    NONE_SPARE = 0
    BOXOUT_HOME_X = 0
    BOXOUT_HOME_Y = 0
    BOXOUT_HOME_Z = 0
    BOXOUT_POS1_Y = 0
    BOXOUT_POS1_Z = 0
    BOXOUT_OFS_Y = 0
    BOXOUT_OFS_Z = 0
    BOXOUT_FORK_GET_HEIGHT = 0
    BOXOUT_FORK_GET_DEEP = 0
    BOXOUT_FORK_PUT_HEIGHT = 0
    BOXOUT_FORK_PUT_DEEP = 0
    BOXOUT_POS1_X = 0
    BOXOUT_POS2_X = 0
    BOXOUT_POS2_Y = 0
    BOXOUT_POS2_Z = 0

    def __init__(self):   # 計算哪一個RACK PORT

        self.rack_home_x = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_HOME_X)  # noqa
        self.rack_home_y = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_HOME_Y)  # noqa
        self.rack_home_z = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_HOME_Z)  # noqa
        self.rack_pos1_x = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_POS1_X)  # noqa
        self.rack_pos1_y = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_POS1_Y)  # noqa
        self.rack_pos1_z = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_POS1_Z)  # noqa
        self.rack_pos2_x = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_POS2_X)  # noqa
        self.rack_pos2_y = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_POS2_Y)  # noqa
        self.rack_pos2_z = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_POS2_Z)  # noqa
        self.rack_pos2_rz = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_POS2_RZ)  # noqa

        self.rack_ofs_y = self.float_to_32bit_decimal_string(0)  # noqa
        self.rack_ofs_z = self.float_to_32bit_decimal_string(0)  # noqa
        self.rack_mark_ofs_x = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_MARK_OFS_X)  # noqa
        self.rack_mark_ofs_y = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_MARK_OFS_Y)  # noqa
        self.rack_mark_ofs_z = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_MARK_OFS_Z)  # noqa
        self.rack_fork_get_height = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_FORK_GET_HEIGHT)  # noqa
        self.rack_fork_get_deep = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_FORK_GET_DEEP)  # noqa
        self.rack_fork_put_height = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_FORK_PUT_HEIGHT)  # noqa
        self.rack_fork_put_deep = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_FORK_PUT_DEEP)  # noqa

        self.boxin_home_x = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_HOME_X)  # noqa
        self.boxin_home_y = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_HOME_Y)  # noqa
        self.boxin_home_z = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_HOME_Z)  # noqa
        self.boxin_pos1_x = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_POS1_X)  # noqa
        self.boxin_pos1_y = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_POS1_Y)  # noqa
        self.boxin_pos1_z = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_POS1_Z)  # noqa
        self.boxin_pos2_x = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_POS2_X)  # noqa
        self.boxin_pos2_y = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_POS2_Y)  # noqa
        self.boxin_pos2_z = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_POS2_Z)  # noqa
        self.boxin_ofs_y = self.float_to_32bit_decimal_string(0)  # noqa
        self.boxin_ofs_z = self.float_to_32bit_decimal_string(0)  # noqa
        self.boxin_fork_get_height = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_FORK_GET_HEIGHT)  # noqa
        self.boxin_fork_get_deep = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_FORK_GET_DEEP)  # noqa
        self.boxin_fork_put_height = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_FORK_PUT_HEIGHT)  # noqa
        self.boxin_fork_put_deep = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_FORK_PUT_DEEP)  # noqa

        self.boxout_home_x = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_HOME_X)  # noqa
        self.boxout_home_y = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_HOME_Y)  # noqa
        self.boxout_home_z = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_HOME_Z)  # noqa
        self.boxout_pos1_y = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_POS1_Y)  # noqa
        self.boxout_pos1_z = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_POS1_Z)  # noqa
        self.boxout_ofs_y = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_OFS_Y)  # noqa
        self.boxout_ofs_z = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_OFS_Z)  # noqa
        self.boxout_fork_get_height = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_FORK_GET_HEIGHT)  # noqa
        self.boxout_fork_get_deep = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_FORK_GET_DEEP)  # noqa
        self.boxout_fork_put_height = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_FORK_PUT_HEIGHT)  # noqa
        self.boxout_fork_put_deep = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_FORK_PUT_DEEP)  # noqa
        self.boxout_pos1_x = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_POS1_X)  # noqa
        self.boxout_pos2_x = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_POS2_X)  # noqa
        self.boxout_pos2_y = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_POS2_Y)  # noqa
        self.boxout_pos2_z = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_POS2_Z)  # noqa

        self.none_spare = self.float_to_32bit_decimal_string(0)  # noqa

        self.rack_port = 0
        self.boxin_port = 0
        self.boxout_port = 0

    def update_parameter(self):
        print("❌❌❌❌❌❌❌❌❌✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌❌")
        if 1 <= self.rack_port <= 16:
            row = ((self.rack_port-1) // 4)  # 整數部分
            column = (self.rack_port-1) % 4  # 餘數
        elif 16 <= self.rack_port <= 32:
            row = ((self.rack_port-17) // 4)  # 整數部分
            column = (self.rack_port-17) % 4  # 餘數
        self.rack_ofs_y = self.float_to_32bit_decimal_string(-354*column)
        self.rack_ofs_z = self.float_to_32bit_decimal_string(-325*row)

        row = ((self.boxin_port-1) // 2)  # 整數部分
        column = (self.boxin_port-1) % 2  # 餘數
        self.boxin_ofs_y = self.float_to_32bit_decimal_string(-380*column)
        self.boxin_ofs_z = self.float_to_32bit_decimal_string(-600*row)

        row = ((self.boxout_port-1) // 2)  # 整數部分
        column = (self.boxout_port-1) % 2  # 餘數
        self.boxout_ofs_y = self.float_to_32bit_decimal_string(-380*column)
        self.boxout_ofs_z = self.float_to_32bit_decimal_string(-600*row)

        # rack
        self.rack_home_x = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_HOME_X)  # noqa
        self.rack_home_y = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_HOME_Y)  # noqa
        self.rack_home_z = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_HOME_Z)  # noqa
        self.rack_pos1_x = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_POS1_X)  # noqa
        self.rack_pos1_y = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_POS1_Y)  # noqa
        self.rack_pos1_z = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_POS1_Z)  # noqa

        if 1 <= self.rack_port <= 8 or 17 <= self.rack_port <= 24:
            self.rack_pos2_x = self.float_to_32bit_decimal_string(0)  # noqa
            self.rack_pos2_y = self.float_to_32bit_decimal_string(0)  # noqa
            self.rack_pos2_z = self.float_to_32bit_decimal_string(0)  # noqa
            self.rack_pos2_rz = self.float_to_32bit_decimal_string(0)  # noqa
            self.rack_mark_ofs_x = self.float_to_32bit_decimal_string(0)  # noqa
            self.rack_mark_ofs_y = self.float_to_32bit_decimal_string(0)  # noqa
            self.rack_mark_ofs_z = self.float_to_32bit_decimal_string(0)  # noqa
        else:
            self.rack_pos2_x = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_POS2_X)  # noqa
            self.rack_pos2_y = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_POS2_Y)  # noqa
            self.rack_pos2_z = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_POS2_Z)  # noqa
            self.rack_pos2_rz = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_POS2_RZ)  # noqa
            self.rack_mark_ofs_x = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_MARK_OFS_X)  # noqa
            self.rack_mark_ofs_y = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_MARK_OFS_Y)  # noqa
            self.rack_mark_ofs_z = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_MARK_OFS_Z)  # noqa

        self.rack_fork_get_height = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_FORK_GET_HEIGHT)  # noqa
        self.rack_fork_get_deep = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_FORK_GET_DEEP)  # noqa
        self.rack_fork_put_height = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_FORK_PUT_HEIGHT)  # noqa
        self.rack_fork_put_deep = self.float_to_32bit_decimal_string(CargoRobotParameter.RACK_FORK_PUT_DEEP)  # noqa

        self.boxin_home_x = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_HOME_X)  # noqa
        self.boxin_home_y = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_HOME_Y)  # noqa
        self.boxin_home_z = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_HOME_Z)  # noqa
        self.boxin_pos1_x = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_POS1_X)  # noqa
        self.boxin_pos1_y = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_POS1_Y)  # noqa
        self.boxin_pos1_z = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_POS1_Z)  # noqa
        self.boxin_pos2_x = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_POS2_X)  # noqa
        self.boxin_pos2_y = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_POS2_Y)  # noqa
        self.boxin_pos2_z = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_POS2_Z)  # noqa
        self.boxin_fork_get_height = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_FORK_GET_HEIGHT)  # noqa
        self.boxin_fork_get_deep = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_FORK_GET_DEEP)  # noqa
        self.boxin_fork_put_height = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_FORK_PUT_HEIGHT)  # noqa
        self.boxin_fork_put_deep = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXIN_FORK_PUT_DEEP)  # noqa

        self.boxout_home_x = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_HOME_X)  # noqa
        self.boxout_home_y = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_HOME_Y)  # noqa
        self.boxout_home_z = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_HOME_Z)  # noqa
        self.boxout_pos1_y = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_POS1_Y)  # noqa
        self.boxout_pos1_z = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_POS1_Z)  # noqa

        self.boxout_fork_get_height = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_FORK_GET_HEIGHT)  # noqa
        self.boxout_fork_get_deep = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_FORK_GET_DEEP)  # noqa
        self.boxout_fork_put_height = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_FORK_PUT_HEIGHT)  # noqa
        self.boxout_fork_put_deep = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_FORK_PUT_DEEP)  # noqa
        self.boxout_pos1_x = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_POS1_X)  # noqa
        self.boxout_pos2_x = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_POS2_X)  # noqa
        self.boxout_pos2_y = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_POS2_Y)  # noqa
        self.boxout_pos2_z = self.float_to_32bit_decimal_string(CargoRobotParameter.BOXOUT_POS2_Z)  # noqa

        self.none_spare = self.float_to_32bit_decimal_string(0)  # noqa

    def values(self):
        # self.update_parameter()
        return [
            self.rack_mark_ofs_x,
            self.rack_mark_ofs_y,
            self.rack_mark_ofs_z,
            self.rack_home_x,
            self.rack_home_y,
            self.rack_home_z,
            self.rack_pos1_y,
            self.rack_pos1_z,
            self.rack_ofs_y,
            self.rack_ofs_z,
            self.rack_fork_get_height,
            self.rack_fork_get_deep,
            self.rack_fork_put_height,
            self.rack_fork_put_deep,
            self.rack_pos1_x,
            self.rack_pos2_x,
            self.rack_pos2_y,
            self.rack_pos2_z,
            self.rack_pos2_rz,
            self.boxin_home_x,
            self.boxin_home_y,
            self.boxin_home_z,
            self.boxin_pos1_y,
            self.boxin_pos1_z,
            self.boxin_ofs_y,
            self.boxin_ofs_z,
            self.boxin_fork_get_height,
            self.boxin_fork_get_deep,
            self.boxin_fork_put_height,
            self.boxin_fork_put_deep,
            self.boxin_pos1_x,
            self.boxin_pos2_x,
            self.boxin_pos2_y,
            self.boxin_pos2_z,
            self.none_spare,
            self.boxout_home_x,
            self.boxout_home_y,
            self.boxout_home_z,
            self.boxout_pos1_y,
            self.boxout_pos1_z,
            self.boxout_ofs_y,
            self.boxout_ofs_z,
            self.boxout_fork_get_height,
            self.boxout_fork_get_deep,
            self.boxout_fork_put_height,
            self.boxout_fork_put_deep,
            self.boxout_pos1_x,
            self.boxout_pos2_x,
            self.boxout_pos2_y,
            self.boxout_pos2_z,
        ]
