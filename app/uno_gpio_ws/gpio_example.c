#include <stdio.h>
#include <gpiod.h>
#include <unistd.h>

//需安裝 libgpiod-dev 套件 (sudo apt install libgpiod-dev) 和 gpiod 套件 (sudo apt install gpiod)
//編譯指令: gcc -o gpio_example gpio_example.c -lgpiod

#define GPIOCHIP "/dev/gpiochip0"

// DI / DO GPIO 代號
const int DI_PINS[8] = {17, 19, 20, 21, 22, 23, 24, 18}; // DI0 ~ DI7
const int DO_PINS[8] = {25, 26, 27, 28, 29, 30, 31, 32}; // DO0 ~ DO7

// 自訂 GPIO 名稱
const char *custom_gpio_names[8 + 8] = {
    "DI0", "DI1", "DI2", "DI3", "DI4", "DI5", "DI6", "DI7",
    "DO0", "DO1", "DO2", "DO3", "DO4", "DO5", "DO6", "DO7"
};

void getGpioName() {
    struct gpiod_chip *chip;
    struct gpiod_line *line;
    const char *name;

    chip = gpiod_chip_open(GPIOCHIP);
    if (!chip) {
        perror("Failed to open GPIO chip");
        return;
    }

    printf("GPIO Line Information:\n");

    // 查詢 DI / DO 的名稱
    for (int i = 0; i < 8; i++) {
        // **查詢 DI**
        line = gpiod_chip_get_line(chip, DI_PINS[i]);
        name = gpiod_line_name(line);
        printf("GPIO %d: %s (Custom: %s)\n", DI_PINS[i], name ? name : "(Unnamed)", custom_gpio_names[i]);
    }

    for (int i = 0; i < 8; i++) {
        // **查詢 DO**
        line = gpiod_chip_get_line(chip, DO_PINS[i]);
        name = gpiod_line_name(line);
        printf("GPIO %d: %s (Custom: %s)\n", DO_PINS[i], name ? name : "(Unnamed)", custom_gpio_names[i + 8]);
    }

    gpiod_chip_close(chip);
}


int main() {
    struct gpiod_chip *chip;
    struct gpiod_line *di_lines[8], *do_lines[8];
    int values[8];
    
    getGpioName();

    // 開啟 GPIO 晶片
    chip = gpiod_chip_open(GPIOCHIP);
    if (!chip) {
        perror("Failed to open GPIO chip");
        return 1;
    }

    // 設定 DI (輸入)
    for (int i = 0; i < 8; i++) {
        di_lines[i] = gpiod_chip_get_line(chip, DI_PINS[i]);
        if (!di_lines[i] || gpiod_line_request_input(di_lines[i], "gpio_example") < 0) {
            perror("Failed to set DI as input");
            gpiod_chip_close(chip);
            return 1;
        }
    }

    // 設定 DO (輸出)
    for (int i = 0; i < 8; i++) {
        do_lines[i] = gpiod_chip_get_line(chip, DO_PINS[i]);
        if (!do_lines[i] || gpiod_line_request_output(do_lines[i], "gpio_example", 0) < 0) {
            perror("Failed to set DO as output");
            gpiod_chip_close(chip);
            return 1;
        }
    }

    while (1) {
        // 讀取 DI 狀態
        printf("DI: ");
        for (int i = 0; i < 8; i++) {
            values[i] = gpiod_line_get_value(di_lines[i]);
            printf("%d ", values[i]);
        }
        printf("\n");

        // 設定 DO (輸出 DI 的反相值)
        for (int i = 0; i < 8; i++) {
            gpiod_line_set_value(do_lines[i], values[i]); // 反相輸出
        }

        sleep(1); // 每秒更新一次
    }

    // 釋放 GPIO
    for (int i = 0; i < 8; i++) {
        gpiod_line_release(di_lines[i]);
        gpiod_line_release(do_lines[i]);
    }
    gpiod_chip_close(chip);

    return 0;
}