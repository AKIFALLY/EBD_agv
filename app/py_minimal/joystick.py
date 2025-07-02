import pygame
import os
os.environ["SDL_AUDIODRIVER"] = "dummy"
os.environ["XDG_RUNTIME_DIR"] = "/tmp"

# 初始化 pygame 和 joystick 模組
pygame.init()
pygame.joystick.init()

# 檢查是否有偵測到手把
if pygame.joystick.get_count() == 0:
    print("未偵測到手把，請確認是否正確連接")
    exit()

# 取得手把
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"偵測到手把: {joystick.get_name()}")

# 儲存上一個狀態以偵測變化
last_state = {}

def has_changed(name, current):
    if name not in last_state or last_state[name] != current:
        last_state[name] = current
        return True
    return False

try:
    while True:
        pygame.event.pump()  # 更新事件

        axis_x = round(joystick.get_axis(0), 2)
        axis_y = round(joystick.get_axis(1), 2)
        right_x = round(joystick.get_axis(3), 2)
        right_y = round(joystick.get_axis(4), 2)
        trigger_l2 = round(joystick.get_axis(2), 2)
        trigger_r2 = round(joystick.get_axis(5), 2)

        button_l1 = joystick.get_button(4)
        button_r1 = joystick.get_button(5)
        a_button = joystick.get_button(0)
        b_button = joystick.get_button(1)
        x_button = joystick.get_button(2)
        y_button = joystick.get_button(3)

        hat = joystick.get_hat(0)

        status = {
            "DPad_X": hat[0],
            "DPad_Y": hat[1],
            "Axis_X": axis_x,
            "Axis_Y": axis_y,
            "Right_X": right_x,
            "Right_Y": right_y,
            "L2": trigger_l2,
            "R2": trigger_r2,
            "L1": button_l1,
            "R1": button_r1,
            "A": a_button,
            "B": b_button,
            "X": x_button,
            "Y": y_button
        }

        # 只在狀態變化時輸出
        if any(has_changed(k, v) for k, v in status.items()):
            print(
                f"方向鍵: 左右: {hat[0]}, 上下: {hat[1]} | 左搖桿 X: {axis_x:.2f}, Y: {axis_y:.2f} | 右搖桿 X: {right_x:.2f}, Y: {right_y:.2f}"
                f" | L1: {button_l1}, R1: {button_r1} | L2: {trigger_l2:.2f}, R2: {trigger_r2:.2f}"
                f" | A: {a_button}, B: {b_button}, X: {x_button}, Y: {y_button}"
            )

except KeyboardInterrupt:
    print("\n結束程式")
    pygame.quit()
