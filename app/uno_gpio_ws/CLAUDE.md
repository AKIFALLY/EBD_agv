# uno_gpio_ws CLAUDE.md

## 📚 Context Loading
@docs-ai/context/system/rosagv-overview.md
@docs-ai/context/system/dual-environment.md
@docs-ai/context/system/technology-stack.md
@docs-ai/operations/development/docker-development.md
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/operations/maintenance/troubleshooting.md

## 🎯 適用場景
- AGV 車載系統的 GPIO 硬體控制
- 研華 UNO-137 工業電腦的數位 I/O 操作
- C 語言和 Python 的 GPIO 程式開發
- 工業設備的數位輸入/輸出整合

## 📋 模組概述

**uno_gpio_ws** 是專為研華 UNO-137 工業電腦設計的 GPIO 控制專案，提供數位輸入/輸出 (DI/DO) 控制功能。作為 AGV 車載系統的硬體控制組件，實現了基於現代 libgpiod 介面的 GPIO 操作。

**⚠️ 重要**: 此為獨立 GPIO 控制專案，非標準 ROS 2 工作空間，使用系統原生 libgpiod 庫直接操作硬體。所有容器開發相關操作請參考 @docs-ai/operations/development/docker-development.md

### 核心特色
- **現代 GPIO 介面**: 使用 libgpiod 取代傳統 sysfs 方式
- **雙語言實作**: C 語言和 Python 完整範例
- **即時 I/O 控制**: 8 個 DI 和 8 個 DO 通道同步控制
- **硬體特化**: 專為研華 UNO-137 工業電腦設計
- **獨立運行**: 不依賴 ROS 2 環境，可獨立執行

### 業務價值
- **硬體整合**: 為 AGV 系統提供底層 GPIO 控制能力
- **工業級穩定**: 基於 libgpiod 現代介面確保穩定性
- **開發友好**: 提供 C 和 Python 兩種語言參考實作
- **即插即用**: 編譯後可直接在目標硬體上運行

## 🏗️ 系統架構

### 技術架構層次
```
硬體層: 研華 UNO-137 /dev/gpiochip0 (Linux Kernel 6.11)
    ↓
介面層: libgpiod (C API) + python3-libgpiod (Python API)
    ↓
應用層: gpio_example.c/py (DI/DO 即時控制範例)
    ↓
整合層: 可整合至 AGV 車載系統 (agv_ws)
```

### 專案結構
```
uno_gpio_ws/
├── README.md                    # 詳細技術文檔 (510行)
├── gpio_example.c               # C 語言完整實作
├── gpio_example.py              # Python 完整實作
├── gpio_example                 # 編譯後的 C 執行檔
├── README.txt.txt               # 原始技術說明
└── CLAUDE.md                    # 模組文檔
```

## 🔧 核心組件

### 1. C 語言實作 (gpio_example.c)
**完整的 libgpiod C API 使用範例**

#### GPIO 硬體配置 (來自實際代碼)
```c
#define GPIOCHIP "/dev/gpiochip0"

// DI/DO GPIO 代號 (來自 gpio_example.c 第 11-12 行)
const int DI_PINS[8] = {17, 19, 20, 21, 22, 23, 24, 18}; // DI0-DI7
const int DO_PINS[8] = {25, 26, 27, 28, 29, 30, 31, 32}; // DO0-DO7

// 自訂 GPIO 名稱 (來自 gpio_example.c 第 15-18 行)
const char *custom_gpio_names[16] = {
    "DI0", "DI1", "DI2", "DI3", "DI4", "DI5", "DI6", "DI7",
    "DO0", "DO1", "DO2", "DO3", "DO4", "DO5", "DO6", "DO7"
};
```

#### GPIO 初始化流程 (來自實際代碼)
```c
// GPIO 晶片開啟 (來自 gpio_example.c 第 60 行)
chip = gpiod_chip_open(GPIOCHIP);
if (!chip) {
    perror("Failed to open GPIO chip");
    return 1;
}

// DI 設定為輸入 (來自 gpio_example.c 第 67-74 行)
for (int i = 0; i < 8; i++) {
    di_lines[i] = gpiod_chip_get_line(chip, DI_PINS[i]);
    if (!di_lines[i] || gpiod_line_request_input(di_lines[i], "gpio_example") < 0) {
        perror("Failed to set DI as input");
        gpiod_chip_close(chip);
        return 1;
    }
}

// DO 設定為輸出 (來自 gpio_example.c 第 77-84 行)
for (int i = 0; i < 8; i++) {
    do_lines[i] = gpiod_chip_get_line(chip, DO_PINS[i]);
    if (!do_lines[i] || gpiod_line_request_output(do_lines[i], "gpio_example", 0) < 0) {
        perror("Failed to set DO as output");
        gpiod_chip_close(chip);
        return 1;
    }
}
```

### 2. Python 實作 (gpio_example.py)
**基於 python3-libgpiod 的完整範例**

#### Python GPIO 配置 (來自實際代碼)
```python
import gpiod
import time

GPIOCHIP = "/dev/gpiochip0"

# DI/DO GPIO 代號 (來自 gpio_example.py 第 11-12 行)
DI_PINS = [17, 19, 20, 21, 22, 23, 24, 18]  # DI0 ~ DI7
DO_PINS = [25, 26, 27, 28, 29, 30, 31, 32]  # DO0 ~ DO7

# 自訂 GPIO 名稱 (來自 gpio_example.py 第 15-18 行)
CUSTOM_GPIO_NAMES = [
    "DI0", "DI1", "DI2", "DI3", "DI4", "DI5", "DI6", "DI7",
    "DO0", "DO1", "DO2", "DO3", "DO4", "DO5", "DO6", "DO7"
]
```

#### GPIO 控制邏輯 (來自實際代碼)
```python
def main():
    """主程式 (來自 gpio_example.py 第 39 行)"""
    chip = gpiod.Chip(GPIOCHIP)

    # 設定 DI (輸入) (來自 gpio_example.py 第 46-48 行)
    di_lines = [chip.get_line(pin) for pin in DI_PINS]
    for line in di_lines:
        line.request(consumer="gpio_example", type=gpiod.LINE_REQ_DIR_IN)

    # 設定 DO (輸出) (來自 gpio_example.py 第 51-53 行)
    do_lines = [chip.get_line(pin) for pin in DO_PINS]
    for line in do_lines:
        line.request(consumer="gpio_example", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])

    while True:
        # 讀取 DI 狀態 (來自 gpio_example.py 第 57-58 行)
        values = [line.get_value() for line in di_lines]
        print("DI:", " ".join(map(str, values)))

        # 設定 DO (輸出 DI 的值) (來自 gpio_example.py 第 61-62 行)
        for line, value in zip(do_lines, values):
            line.set_value(value)  # 直接輸出值

        time.sleep(1)  # 每秒更新一次
```

## 📊 硬體規格

### 研華 UNO-137 GPIO 對應
| 功能 | GPIO 編號 | 自訂名稱 | 描述 |
|------|-----------|----------|------|
| DI0-DI7 | 17,19,20,21,22,23,24,18 | DI0-DI7 | 數位輸入通道 |
| DO0-DO7 | 25,26,27,28,29,30,31,32 | DO0-DO7 | 數位輸出通道 |

### 系統需求
- **硬體平台**: 研華 UNO-137 工業電腦
- **作業系統**: Ubuntu 24.04, Linux Kernel 6.11+
- **GPIO 介面**: /dev/gpiochip0
- **權限需求**: sudo 或 gpio 群組權限

## 🚀 快速開始
@docs-ai/operations/development/docker-development.md

### 系統套件安裝
```bash
# 安裝必要套件
sudo apt update
sudo apt install -y libgpiod-dev gpiod python3-libgpiod gcc

# 驗證安裝
gpiodetect
python3 -c "import gpiod; print('✅ python3-libgpiod 可用')"
```

### 編譯和執行
```bash
# 進入專案目錄 (容器內)
cd /app/uno_gpio_ws

# C 語言版本
gcc -o gpio_example gpio_example.c -lgpiod
sudo ./gpio_example

# Python 版本
sudo python3 gpio_example.py

# 使用命令列工具
sudo gpioget 0 17    # 讀取 DI0
sudo gpioset 0 25=1  # 設定 DO0 為高電位
```

詳細的容器環境操作、開發環境設定請參考 @docs-ai/operations/development/docker-development.md

## 💡 核心 API

### libgpiod C API 關鍵函數
```c
// 基本 GPIO 操作流程
struct gpiod_chip *chip = gpiod_chip_open("/dev/gpiochip0");
struct gpiod_line *line = gpiod_chip_get_line(chip, pin_number);

// 設定為輸入
gpiod_line_request_input(line, "consumer_name");
int value = gpiod_line_get_value(line);

// 設定為輸出
gpiod_line_request_output(line, "consumer_name", initial_value);
gpiod_line_set_value(line, new_value);

// 資源清理
gpiod_line_release(line);
gpiod_chip_close(chip);
```

### python3-libgpiod API 模式
```python
# 基本 GPIO 操作流程
import gpiod

chip = gpiod.Chip("/dev/gpiochip0")
line = chip.get_line(pin_number)

# 設定為輸入
line.request(consumer="app_name", type=gpiod.LINE_REQ_DIR_IN)
value = line.get_value()

# 設定為輸出
line.request(consumer="app_name", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
line.set_value(new_value)
```

## 🔄 系統整合

### 在 RosAGV 中的定位
```
AGV 車載系統 (agv_ws)
    ↓ 硬體控制需求
uno_gpio_ws (GPIO 控制層)
    ↓ libgpiod 介面
研華 UNO-137 硬體 (/dev/gpiochip0)
```

### 整合關係
- **AGV 系統整合**: 可整合至 agv_ws 提供硬體 I/O 控制
- **獨立運行**: 不依賴 ROS 2 環境，可獨立執行測試
- **系統層級**: 直接操作硬體 GPIO，需要 sudo 權限
- **硬體特化**: 專為研華 UNO-137 工業電腦設計

## ⚙️ 實際使用範例

### 1. 基本 GPIO 測試
```bash
# 檢查 GPIO 資訊
gpioinfo | grep -E "(17|19|20|21|22|23|24|18|25|26|27|28|29|30|31|32)"

# 測試單一 GPIO
sudo gpioget 0 17  # 讀取 DI0 狀態
sudo gpioset 0 25=1  # 設定 DO0 為高電位

# 批量操作
sudo gpioset 0 25=1 26=1 27=1 28=1  # DO0-DO3 全部設為高電位
```

### 2. 整合到其他專案
```python
# 在其他 Python 專案中使用 (基於實際代碼架構)
import sys
sys.path.append('/app/uno_gpio_ws')

# 參考 gpio_example.py 的實作模式
import gpiod

class UnoGPIOController:
    def __init__(self):
        self.chip = gpiod.Chip("/dev/gpiochip0")
        self.di_pins = [17, 19, 20, 21, 22, 23, 24, 18]  # 實際 DI 對應
        self.do_pins = [25, 26, 27, 28, 29, 30, 31, 32]  # 實際 DO 對應
        self._setup_gpio()
    
    def _setup_gpio(self):
        # 基於實際代碼的初始化模式
        self.di_lines = [self.chip.get_line(pin) for pin in self.di_pins]
        for line in self.di_lines:
            line.request(consumer="agv_controller", type=gpiod.LINE_REQ_DIR_IN)
        
        self.do_lines = [self.chip.get_line(pin) for pin in self.do_pins]
        for line in self.do_lines:
            line.request(consumer="agv_controller", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
```

## 🚨 故障排除
@docs-ai/operations/maintenance/troubleshooting.md
@docs-ai/operations/maintenance/system-diagnostics.md

### uno_gpio_ws 特定問題

#### GPIO 權限問題
```bash
# 症狀: "Permission denied" 或無法存取 GPIO
# 解決: 使用 sudo 執行
sudo python3 gpio_example.py
sudo ./gpio_example

# 或將使用者加入 gpio 群組 (如果存在)
sudo usermod -a -G gpio $USER
# 重新登入後生效
```

#### 硬體檢測問題
```bash
# 檢查 GPIO 晶片是否存在
ls -la /dev/gpiochip*

# 檢查特定 GPIO 是否可用
gpioinfo | grep -E "(17|19|20|21|22|23|24|18|25|26|27|28|29|30|31|32)"

# 檢查核心模組
lsmod | grep gpio
```

#### 套件相關問題
```bash
# libgpiod 套件未安裝
sudo apt install libgpiod-dev gpiod python3-libgpiod

# 驗證 Python 套件
python3 -c "import gpiod; print('✅ OK')"
```

通用的系統診斷方法、容器環境故障排除請參考上方 docs-ai 連結。

## 💡 開發注意事項

### 核心限制
1. **權限要求**: GPIO 操作需要 sudo 權限或適當群組權限
2. **硬體特定**: GPIO 編號對應研華 UNO-137 特定硬體平台
3. **核心相依**: GPIO 編號可能隨 Linux 核心版本變動，需定期驗證
4. **非 ROS 工作空間**: 使用標準 Linux GPIO 介面，非 ROS 2 架構
5. **即時性考量**: 直接硬體操作，適合即時 I/O 控制需求

### 最佳實踐
- **錯誤處理**: 參考實際代碼中的完整錯誤檢查機制
- **資源管理**: 確保 GPIO 線路正確釋放 (gpiod_line_release)
- **執行緒安全**: 多執行緒環境下注意 GPIO 存取同步
- **硬體驗證**: 部署前先用命令列工具驗證 GPIO 可用性

## 🔗 交叉引用

### 相關模組
- **AGV 控制系統**: `app/agv_ws/src/agv_base/CLAUDE.md` - 可整合 GPIO 控制功能
- **感測器處理**: `app/sensorpart_ws/CLAUDE.md` - 可結合硬體 I/O 狀態

### 通用指導
- **容器開發環境**: @docs-ai/operations/development/docker-development.md
- **雙環境架構**: @docs-ai/context/system/dual-environment.md
- **技術棧說明**: @docs-ai/context/system/technology-stack.md

### 運維支援
- **系統診斷工具**: @docs-ai/operations/maintenance/system-diagnostics.md
- **故障排除流程**: @docs-ai/operations/maintenance/troubleshooting.md
- **統一工具系統**: @docs-ai/operations/tools/unified-tools.md