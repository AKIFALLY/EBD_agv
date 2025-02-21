import struct


class PlcBytes(bytearray):
    def to_int(self) -> int:
        length = len(self)
        if length == 2:
            fmt = "<h"  # 2 bytes (short)
        elif length == 4:
            fmt = "<i"  # 4 bytes (int)
        elif length == 8:
            fmt = "<q"  # 8 bytes (long long)
        else:
            raise ValueError(f"Unsupported length {length}, must be 2, 4, or 8 bytes")
        return struct.unpack(fmt, bytes(self))[0]

    def to_float(self) -> float:
        if len(self) != 4:
            raise ValueError("Float requires exactly 4 bytes")
        return struct.unpack("<f", bytes(self))[0]

    def to_string(self, encoding="ascii") -> str:
        return self.decode(encoding, errors="ignore")

    def to_bools(self) -> list:
        """將 PlcBytes 轉換為布林陣列"""
        bools = []
        for byte in self:
            for i in range(8):  # 每個 byte 有 8 個 bit
                bools.append((byte >> i) & 1 == 1)
        return bools

    @classmethod
    def from_int(cls, value: int, length: int = 4) -> "PlcBytes":
        """從整數轉換為 PlcBytes,長度可以是 2, 4, 8 (預設 4)"""
        if length == 2:
            fmt = "<H"
        elif length == 4:
            fmt = "<I"
        elif length == 8:
            fmt = "<Q"
        else:
            raise ValueError("Length must be 2, 4, or 8 bytes")
        return cls(struct.pack(fmt, value))

    @classmethod
    def from_float(cls, value: float) -> "PlcBytes":
        """從浮點數轉換為 PlcBytes (4 bytes)"""
        return cls(struct.pack("<f", value))

    @classmethod
    def from_string(
        cls, value: str, length: int = None, encoding="ascii"
    ) -> "PlcBytes":
        """從字串轉換為 PlcBytes,可以指定長度,不足則補 0"""
        raw_bytes = value.encode(encoding)
        if length is not None:
            if len(raw_bytes) > length:
                raw_bytes = raw_bytes[:length]  # 截斷
            else:
                raw_bytes = raw_bytes.ljust(length, b"\x00")  # 補 0
        return cls(raw_bytes)

    @classmethod
    def from_bools(cls, bools: list) -> "PlcBytes":
        """從布林陣列轉換為 PlcBytes"""
        byte_array = []
        byte = 0
        for i, value in enumerate(bools):
            if value:
                byte |= 1 << (i % 8)
            if (i + 1) % 8 == 0 or i == len(bools) - 1:
                byte_array.append(byte)
                byte = 0
        return cls(byte_array)

    def set_bit(self, base_address: int, bit_index: int, value: bool) -> None:
        """設置特定 bit 為 0 或 1"""
        byte_index = base_address + (bit_index // 8)
        bit_position = bit_index % 8
        if value:
            self[byte_index] |= 1 << bit_position
        else:
            self[byte_index] &= ~(1 << bit_position)

    def get_bit(self, base_address: int, bit_index: int) -> bool:
        """讀取特定 bit"""
        byte_index = base_address + (bit_index // 8)
        bit_position = bit_index % 8
        return (self[byte_index] >> bit_position) & 0x01


if __name__ == "__main__":

    # 測試從布林陣列轉換
    bools = [True, False, True, True, False, False, True, True]
    b4 = PlcBytes.from_bools(bools)
    print(b4)  # 將布林陣列轉換為 PlcBytes
    print(b4.to_bools())  # 再轉回布林陣列

    # 設置與讀取 bit 值
    b4.set_bool(0, 3, False)  # 設置第 3 位為 False
    print(b4.get_bool(0, 3))  # 讀取第 3 位的值
