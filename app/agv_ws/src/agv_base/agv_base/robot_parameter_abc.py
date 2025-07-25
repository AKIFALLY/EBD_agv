from abc import ABC, abstractmethod
import struct


class RobotParameterABC(ABC):
    """ 定義機器人參數的抽象基類 """

    @staticmethod
    def float_to_32bit_decimal_string(value: float) -> str:
        """Convert a float to a 32-bit decimal string and pad to 10 characters."""
        pack = struct.pack("<f", value)  # little-endian float
        int_pack = int.from_bytes(pack, byteorder="little")
        return str(int_pack).zfill(10)  # write continuous data 讀取時需要10位數

    @abstractmethod
    def calculate_parameter(self):
        pass

    @abstractmethod
    def values(self):
        pass
