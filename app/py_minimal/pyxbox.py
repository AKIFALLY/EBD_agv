import pygame
import os

os.environ["SDL_AUDIODRIVER"] = "dummy"  # 避免音訊驅動問題
pygame.init()
pygame.joystick.init()

joystick = None  # 預設搖桿為 None

while True:

    if pygame.joystick.get_count() > 0:
        # 如果搖桿被插入，則初始化搖桿

        if joystick is None:  # 只有在偵測到新設備時才初始化
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"偵測到手把: {joystick.get_name()}")

        pygame.event.pump()  # 更新事件

        # 🎮 讀取搖桿按鍵
        a_button = joystick.get_button(0)
        b_button = joystick.get_button(1)
        x_button = joystick.get_button(2)
        y_button = joystick.get_button(3)

        # 🎮 讀取方向鍵 (D-Pad)
        hat = joystick.get_hat(0)  # D-Pad，回傳 (x, y)

        # 🎮 讀取搖桿軸
        axis_x = joystick.get_axis(0)  # 左搖桿 X 軸
        axis_y = joystick.get_axis(1)  # 左搖桿 Y 軸
        right_x = joystick.get_axis(2) # 右搖桿 X 軸
        right_y = joystick.get_axis(3) # 右搖桿 Y 軸

        # 🎮 讀取搖桿按鍵
        l1_button = joystick.get_button(4)
        r1_button = joystick.get_button(5)
        l2_button = joystick.get_button(6)
        r2_button = joystick.get_button(7)

        select_button = joystick.get_button(8)
        start_button = joystick.get_button(9)

        print(f"方向鍵: 左右: {hat[0]}, 上下: {hat[1]} | 左搖桿 X: {axis_x:.2f}, Y: {axis_y:.2f} | 右搖桿 X: {right_x:.2f}, Y: {right_y:.2f} | A: {a_button}, B: {b_button}, X: {x_button}, Y: {y_button}| L1: {l1_button}, L2: {l2_button}, R1: {r1_button}, R2: {r2_button}| Select: {select_button}, Start: {start_button}")

        pygame.time.wait(50)  # 降低 CPU 負擔
    else:
        # 如果搖桿被拔除，則清除搖桿變數
        if joystick is not None:
            joystick = None  # 清除搖桿變數，讓它能夠重新偵測
            print("❌ 手把已拔除，等待重新連接...")

        pygame.time.wait(1000)  # 降低 CPU 負擔

        # **手動刷新 Joystick 狀態**
        pygame.joystick.quit()  # 釋放舊的 Joystick 狀態
        pygame.joystick.init()  # 重新初始化
        print(f"手把連接...? {pygame.joystick.get_count() > 0}")
        
