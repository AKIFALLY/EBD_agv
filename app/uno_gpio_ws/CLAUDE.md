# CLAUDE.md

## 系統概述
UNO GPIO工作空間 - 專為研華UNO-137工業電腦設計的GPIO控制專案，提供數位輸入/輸出(DI/DO)控制功能。

**⚠️ 特殊說明**: 此為獨立GPIO控制專案，非ROS 2工作空間，使用系統原生libgpiod庫。

## 核心功能
- **GPIO硬體控制**: 8個數位輸入(DI0-DI7) + 8個數位輸出(DO0-DO7)
- **雙語言支援**: C語言和Python實作
- **即時監控**: 每秒讀取DI狀態並輸出到DO
- **硬體對應**: 研華UNO-137 GPIO線路對應

## 技術架構
```
硬體層: 研華UNO-137 /dev/gpiochip0
介面層: libgpiod (C) + python3-libgpiod (Python)
應用層: gpio_example.c/py (即時DI/DO控制)
```

## 核心檔案
- `gpio_example.c`: C語言GPIO控制實作 (完整功能)
- `gpio_example.py`: Python GPIO控制實作 (完整功能)  
- `gpio_example`: 編譯後的C執行檔
- `README.md`: 詳細技術文檔和使用指南

## GPIO硬體對應
```c
// DI/DO GPIO 線路對應 (Linux Kernel 6.11)
DI_PINS[8] = {17, 19, 20, 21, 22, 23, 24, 18};  // DI0-DI7
DO_PINS[8] = {25, 26, 27, 28, 29, 30, 31, 32};  // DO0-DO7
```

## 主要API
### C語言API
```c
#include <gpiod.h>
struct gpiod_chip *chip = gpiod_chip_open("/dev/gpiochip0");
struct gpiod_line *line = gpiod_chip_get_line(chip, 17);
gpiod_line_request_input(line, "consumer");
int value = gpiod_line_get_value(line);
```

### Python API  
```python
import gpiod
chip = gpiod.Chip("/dev/gpiochip0")
line = chip.get_line(17)
line.request(consumer="app", type=gpiod.LINE_REQ_DIR_IN)
value = line.get_value()
```

## 執行方式
```bash
# C語言版本
gcc -o gpio_example gpio_example.c -lgpiod
sudo ./gpio_example

# Python版本  
sudo python3 gpio_example.py

# 命令列工具
sudo gpioget 0 17    # 讀取DI0
sudo gpioset 0 25=1  # 設定DO0為高電位
```

## 系統需求
- **硬體**: 研華UNO-137工業電腦
- **作業系統**: Ubuntu 24.04, Linux Kernel 6.11+
- **套件依賴**: libgpiod-dev, gpiod, python3-libgpiod

## 整合關係
- **AGV系統整合**: 可整合至agv_ws提供硬體I/O控制
- **獨立運作**: 不依賴ROS 2環境，可獨立執行
- **系統層級**: 直接操作硬體GPIO，需sudo權限

## 開發注意事項
1. **權限要求**: GPIO操作需要sudo權限
2. **硬體特定**: GPIO編號對應特定硬體平台
3. **核心相依**: GPIO編號可能隨核心版本變動
4. **非ROS工作空間**: 使用標準Linux GPIO介面，非ROS 2架構

## 故障排除
- **權限錯誤**: 使用sudo執行或加入gpio群組
- **套件缺失**: 安裝libgpiod-dev, python3-libgpiod
- **硬體不存在**: 檢查/dev/gpiochip0存在性
- **線路佔用**: 檢查其他程序是否占用GPIO

## 未來規劃
- ROS 2套件整合 (建立標準ROS節點)
- GPIO中斷處理支援
- Web介面整合至web_api_ws
- AGV系統硬體I/O整合