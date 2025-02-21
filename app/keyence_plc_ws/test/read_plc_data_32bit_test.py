from keyence_plc.keyence_plc_com import KeyencePlcCom, PLC_END_MARKER
# 假設 KeyencePlcCom 在 keyence_plc_com.py 檔案中


def read_dm7600_10_words(plc):
    """讀取 DM7600 開始的 10 個 word"""
    response = plc.send_command(f"RDS DM7600 100{PLC_END_MARKER}")
    if response:
        print(f"讀取結果: {response}")

        data_list = response.split()  # 解析 PLC 傳回的數據

        # 兩個 16-bit 數值組合成一個 32-bit
        for i in range(0, len(data_list), 2):
            int32_list = []
            low = int(data_list[i])  # 低 16-bit
            high = int(data_list[i + 1])  # 高 16-bit

            # 轉成 bytes
            low_bytes = low.to_bytes(2, "little")
            high_bytes = high.to_bytes(2, "little")
            # 合併成 4 bytes
            merged_bytes = low_bytes + high_bytes

            # 轉換成 32-bit 整數
            int32_value = int.from_bytes(merged_bytes, "little")
            int32_list.append(int32_value)
            print("32-bit 整數結果:", int32_list)

    else:
        print("讀取失敗")


if __name__ == "__main__":
    plc = KeyencePlcCom("192.168.0.103", 8501)

    print("\n=== 讀取 DM7600 開始的 100 個 word ===")
    read_dm7600_10_words(plc)

    plc.disconnect()
