import struct

def int32_to_2_words(value):
    """
    將一個 32-bit 整數轉為兩個 16-bit 字串（高位在前）
    """
    packed = struct.pack('<i', value)  # 小端序 4-byte int
    low_word, high_word = struct.unpack('<HH', packed)
    return [str(low_word),str(high_word)]

def words_to_int32(high_str, low_str):
    high = int(high_str)
    low = int(low_str)
    packed = struct.pack('<HH', low, high)
    return struct.unpack('<i', packed)[0]