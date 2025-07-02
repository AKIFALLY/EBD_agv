import gpiod
import time

#需要安裝 python3-libgpiod 和 gpiod 套件
# sudo apt-get install python3-libgpiod
# sudo apt-get install gpiod 

GPIOCHIP = "/dev/gpiochip0"

# DI / DO GPIO 代號
DI_PINS = [17, 19, 20, 21, 22, 23, 24, 18]  # DI0 ~ DI7
DO_PINS = [25, 26, 27, 28, 29, 30, 31, 32]  # DO0 ~ DO7

# 自訂 GPIO 名稱
CUSTOM_GPIO_NAMES = [
    "DI0", "DI1", "DI2", "DI3", "DI4", "DI5", "DI6", "DI7",
    "DO0", "DO1", "DO2", "DO3", "DO4", "DO5", "DO6", "DO7"
]


def get_gpio_names():
    """ 取得 GPIO 名稱 """
    chip = gpiod.Chip(GPIOCHIP)
    
    print("GPIO Line Information:")
    
    # 查詢 DI / DO 的名稱
    for i, pin in enumerate(DI_PINS):
        line = chip.get_line(pin)
        name = line.name() or "(Unnamed)"
        print(f"GPIO {pin}: {name} (Custom: {CUSTOM_GPIO_NAMES[i]})")

    for i, pin in enumerate(DO_PINS):
        line = chip.get_line(pin)
        name = line.name() or "(Unnamed)"
        print(f"GPIO {pin}: {name} (Custom: {CUSTOM_GPIO_NAMES[i + 8]})")


def main():
    """ 主程式 """
    get_gpio_names()

    chip = gpiod.Chip(GPIOCHIP)

    # 設定 DI (輸入)
    di_lines = [chip.get_line(pin) for pin in DI_PINS]
    for line in di_lines:
        line.request(consumer="gpio_example", type=gpiod.LINE_REQ_DIR_IN)

    # 設定 DO (輸出)
    do_lines = [chip.get_line(pin) for pin in DO_PINS]
    for line in do_lines:
        line.request(consumer="gpio_example", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])

    while True:
        # 讀取 DI 狀態
        values = [line.get_value() for line in di_lines]
        print("DI:", " ".join(map(str, values)))

        # 設定 DO (輸出 DI 的反相值)
        for line, value in zip(do_lines, values):
            line.set_value(value)  # 直接輸出值

        time.sleep(1)  # 每秒更新一次


if __name__ == "__main__":
    main()
