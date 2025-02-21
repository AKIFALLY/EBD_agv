class StringToNumberConverter:
    @staticmethod
    def convert(value: str):
        # 嘗試將字串轉換為整數
        try:
            return int(value)
        except ValueError:
            pass

        # 嘗試將字串轉換為浮點數
        try:
            return float(value)
        except ValueError:
            pass

        # 如果無法轉換，則回傳錯誤提示
        raise ValueError(f"Cannot convert '{value}' to a number")

    @staticmethod
    def convert_array(values: list):
        return [StringToNumberConverter.convert(value) for value in values]

    @staticmethod
    def convert_to_bytes(value: int):
        # 檢查是否是正整數且在0到65535範圍內
        if 0 <= value <= 65535:
            # 將正整數轉換為2個字節
            return value.to_bytes(2, byteorder="little")
        else:
            raise ValueError("Value must be between 0 and 65535")

    @staticmethod
    def bytes_to_ascii(byte_data: bytes):
        # 確保字節資料是可解碼的
        return byte_data.decode("ascii", errors="ignore")  # 忽略不可解碼的字符

    @staticmethod
    def convert_numbers_to_string(values: list):
        # 將每個整數轉換為字節，然後將其轉換為對應的ASCII字元
        result = ""
        for value in values:
            # 先將字串轉換為整數
            value = int(value)
            byte_data = StringToNumberConverter.convert_to_bytes(value)
            result += StringToNumberConverter.bytes_to_ascii(byte_data)
        return result


def main():
    # "16711", "22064", "12544",
    values = ["18241", "12374", "49", "0", "0", "18241", "0"]
    converter = StringToNumberConverter()

    try:
        # 將數字轉換為字串
        result_string = converter.convert_numbers_to_string(values)
        print(
            f"Resulting string: {result_string} {len(result_string.replace("\x00", ""))}"
        )
    except ValueError as e:
        print(e)


if __name__ == "__main__":
    main()
