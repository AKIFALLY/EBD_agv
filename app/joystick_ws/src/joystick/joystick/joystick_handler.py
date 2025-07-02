import pygame
import os
import threading
import time


class JoystickHandler:
    # 定義搖桿DPAD常量
    DPAD_UP = (0, 1)
    DPAD_DOWN = (0, -1)
    DPAD_LEFT = (-1, 0)
    DPAD_RIGHT = (1, 0)
    DPAD_UP_LEFT = (-1, 1)
    DPAD_UP_RIGHT = (1, 1)
    DPAD_DOWN_LEFT = (-1, -1)
    DPAD_DOWN_RIGHT = (1, -1)
    DPAD_CENTER = (0, 0)

    # 定義軸常量
    L_X_AXIS = 0
    L_Y_AXIS = 1
    R_X_AXIS = 2
    R_Y_AXIS = 3

    _instance = None  # 單例模式
    _initialized = False  # 確保 Pygame 只初始化一次
    _joystick = None
    _running = False
    _thread = None
    _callbacks = {}  # 按鈕回調
    _axis_callbacks = {}  # 軸回調
    _dpad_callback = None
    _prev_button_state = {}  # 上一次按鈕狀態
    _prev_hat_state = (0, 0)  # 上一次 D-Pad 狀態
    _prev_axis_state = [0, 0, 0, 0]  # 上一次軸的狀態

    @classmethod
    def init(cls):
        """初始化 Pygame 和搖桿"""
        if not cls._initialized:
            os.environ["SDL_AUDIODRIVER"] = "dummy"  # 避免音訊驅動問題
            pygame.init()
            pygame.joystick.init()
            cls._initialized = True
            print("🎮 Pygame 初始化完成")

        cls._init_joystick()

    @classmethod
    def _init_joystick(cls):
        """偵測並初始化搖桿"""
        pygame.joystick.quit()  # 清除舊的搖桿
        pygame.joystick.init()  # 重新初始化
        if pygame.joystick.get_count() > 0:
            cls._joystick = pygame.joystick.Joystick(0)
            cls._joystick.init()
            print(f"✅ 偵測到搖桿: {cls._joystick.get_name()}")
        else:
            cls._joystick = None
            print("⚠️ 沒有偵測到搖桿")

    @classmethod
    def register_button_callback(cls, button, callback):
        """註冊按鈕事件回調"""
        cls._callbacks[button] = callback

    @classmethod
    def unregister_button_callback(cls, button):
        """解除按鈕事件回調"""
        if button in cls._callbacks:
            del cls._callbacks[button]

    @classmethod
    def register_axis_callback(cls, axis, callback):
        """註冊軸事件回調"""
        cls._axis_callbacks[axis] = callback

    @classmethod
    def unregister_axis_callback(cls, axis):
        """解除軸事件回調"""
        if axis in cls._axis_callbacks:
            del cls._axis_callbacks[axis]

    @classmethod
    def register_dpad_callback(cls, callback):
        """註冊 D-Pad 方向鍵回調"""
        cls._dpad_callback = callback

    @classmethod
    def unregister_button_callback(cls, button):
        """解除按鈕回調註冊"""
        if button in cls._callbacks:
            del cls._callbacks[button]

    @classmethod
    def unregister_axis_callback(cls, axis):
        """解除軸回調註冊"""
        if axis in cls._axis_callbacks:
            del cls._axis_callbacks[axis]

    @classmethod
    def unregister_dpad_callback(cls):
        """解除 D-Pad 回調註冊"""
        cls._dpad_callback = None

    @classmethod
    def unregister_dpad_callback(cls):
        """解除 D-Pad 方向鍵回調"""
        cls._dpad_callback = None

    @classmethod
    def start(cls, polling_rate=50):
        """啟動搖桿監聽 (獨立執行緒)"""
        return
        if cls._running:
            return

        cls._running = True
        cls._thread = threading.Thread(
            target=cls._joystick_loop, args=(polling_rate,))
        cls._thread.daemon = True
        cls._thread.start()

    @classmethod
    def stop(cls):
        """停止搖桿監聽"""
        cls._running = False
        if cls._thread and cls._thread.is_alive():
            if threading.current_thread() != cls._thread:
                cls._thread.join(timeout=1.0)
            else:
                print("⚠️ 無法在 joystick 執行緒內 join 自己，略過 join()")

    @classmethod
    def _joystick_loop(cls, polling_rate):
        """搖桿監聽迴圈"""
        # while cls._running:
        try:
            if pygame.joystick.get_count() == 0:
                if cls._joystick:
                    print("❌ 搖桿已拔除，等待重新連接...")
                cls._joystick = None
                time.sleep(1)
                # cls._init_joystick()
                cls.init()
                return
                # continue

            if cls._joystick is None or not cls._joystick.get_init():
                print("⚠️ 搖桿未初始化，跳過此次讀取")
                time.sleep(1)
                cls.init()
                return
                # continue

            # 更新事件
            pygame.event.pump()  # 更新事件
            # if cls._joystick is None:
            #    time.sleep(0.5)  # 加一點緩衝時間
            #    continue

            # 讀取按鈕狀態
            button_state = {
                "a_button": cls._joystick.get_button(0),
                "b_button": cls._joystick.get_button(1),
                "x_button": cls._joystick.get_button(2),
                "y_button": cls._joystick.get_button(3),
                "l1_button": cls._joystick.get_button(4),
                "r1_button": cls._joystick.get_button(5),
                "l2_button": cls._joystick.get_button(6),
                "r2_button": cls._joystick.get_button(7),
                "select_button": cls._joystick.get_button(8),
                "start_button": cls._joystick.get_button(9),
            }

            # 讀取軸的狀態 (L_X_AXIS, L_Y_AXIS, R_X_AXIS, R_Y_AXIS)
            axis_state = [
                cls._joystick.get_axis(cls.L_X_AXIS),  # L_X_AXIS
                cls._joystick.get_axis(cls.L_Y_AXIS),  # L_Y_AXIS
                cls._joystick.get_axis(cls.R_X_AXIS),  # R_X_AXIS
                cls._joystick.get_axis(cls.R_Y_AXIS),  # R_Y_AXIS
            ]

            # 讀取 D-Pad 狀態
            hat_state = cls._joystick.get_hat(0)

        except pygame.error as e:
            print(f"⚠️ Joystick Error: {e}，將重新初始化搖桿")
            cls._joystick = None
            time.sleep(1)
            return
            # continue

        # 檢查按鈕狀態變化
        for button, state in button_state.items():
            if cls._prev_button_state.get(button) != state:
                action = "pressed" if state else "released"
                if button in cls._callbacks:
                    cls._callbacks[button](button, action)
                cls._prev_button_state[button] = state  # 更新狀態

        # 檢查軸狀態變化
        for i, state in enumerate(axis_state):
            # 限制小數精度並設定閾值
            state = round(state, 2)
            if abs(state) < 0.05:
                state = 0  # 小於0.05視為0

            if state != cls._prev_axis_state[i]:
                if i in cls._axis_callbacks:
                    cls._axis_callbacks[i](i, state)
                cls._prev_axis_state[i] = state  # 更新軸狀態

        # 檢查 D-Pad 狀態變化
        if hat_state != cls._prev_hat_state and cls._dpad_callback:
            cls._dpad_callback(hat_state)
            cls._prev_hat_state = hat_state  # 更新 D-Pad 狀態

        # pygame.time.wait(polling_rate)  # 控制輪詢速度


# 測試
if __name__ == "__main__":
    def button_event(button, action):
        print(f"🕹️ {button} {action}")

    def axis_event(axis, value):
        print(f"🎮 軸 {axis} 變化: {value}")

    def dpad_event(hat_state):
        print(f"🎮 D-Pad 移動: x={hat_state[0]}, y={hat_state[1]}")

    # 初始化搖桿
    JoystickHandler.init()

    # 註冊按鈕和軸事件
    JoystickHandler.register_button_callback("a_button", button_event)
    JoystickHandler.register_button_callback("b_button", button_event)
    JoystickHandler.register_button_callback("x_button", button_event)
    JoystickHandler.register_button_callback("y_button", button_event)

    JoystickHandler.register_button_callback("l1_button", button_event)
    JoystickHandler.register_button_callback("l2_button", button_event)
    JoystickHandler.register_button_callback("r1_button", button_event)
    JoystickHandler.register_button_callback("r2_button", button_event)

    JoystickHandler.register_button_callback("select_button", button_event)
    JoystickHandler.register_button_callback("start_button", button_event)

    JoystickHandler.register_axis_callback(
        JoystickHandler.L_X_AXIS, axis_event)  # 註冊 L_X_AXIS
    JoystickHandler.register_axis_callback(
        JoystickHandler.L_Y_AXIS, axis_event)  # 註冊 L_Y_AXIS
    JoystickHandler.register_axis_callback(
        JoystickHandler.R_X_AXIS, axis_event)  # 註冊 R_X_AXIS
    JoystickHandler.register_axis_callback(
        JoystickHandler.R_Y_AXIS, axis_event)  # 註冊 R_Y_AXIS
    JoystickHandler.register_dpad_callback(dpad_event)

    # 開始監聽搖桿
    JoystickHandler.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        JoystickHandler.stop()
        print("🛑 結束")
