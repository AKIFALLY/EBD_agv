研華建議使用套件libgpiod
sudo apt update
sudo apt install -y libgpiod-dev gpiod

c語言範例 編譯執行
gcc -o gpio_example gpio_example.c -lgpiod
sudo ./gpio_example

要用python寫的話要 libgpiod python 套件
sudo apt install python3-libgpiod

command line
gpioget
gpioset
gpioinfo
gpiodetect
uno 137 的 di/do 在 chip 0 的 17~32
IO對應編號(Kernal 6.11)
line x
DI 0~7   17 19 20 21 22 23 24 18
DO 0~7   25 26 27 28 29 30 31 32

讀 gpioget 0 17
寫 gpioset 0 25=1
寫 gpioset 0 25=0

原始方法 研華下載 platform SDK 安裝 libEAPI 後
可由libEAPI控制 sysfs方式控制
至/sys/class/gpio執行  (529~544) 目前linux kernal 6.11 版的編號 kernal換的話會變動
echo 529 > export
echo 530 > export
...
echo 544 > export

讀數值
cat gpio529/value
讀In或Out
cat gpio529/direction
寫數值
echo 1 > gpio529/value
echo 0 > gpio529/value

移除
echo 529 > unexport

IO對應編號(Kernal 6.11)
gpio x
DI 0~7   529 531 532 533 534 535 536 530
DO 0~7   537 538 539 540 541 542 543 544