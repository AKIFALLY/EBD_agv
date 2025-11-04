from agv_base.context_abc import ContextABC
from agv_base.event import Event
from agv_base.states.state import State
from agv_interfaces.msg import AgvStateChange
from rclpy.node import Node


class BaseContext(ContextABC):
    """ 實作完整邏輯的狀態機 Context """

    def __init__(self, initial_state: State):
        self.node = initial_state.node
        self.state = initial_state
        self.last_state = initial_state
        self.state.enter()

        ns = self.node.get_namespace()
        if ns:
            self.agv_id = ns.strip("/").split("/")[-1]
        else:
            self.agv_id = None

        self._publisher = self.node.create_publisher(
            AgvStateChange, "/agv/state_change", 10)

        self.on_state_changing = Event(
            name="on_state_changing", print_event=True)
        self.on_state_changed = Event(name="on_state_changed")
        self.before_handle = Event(name="before_handle")
        self.after_handle = Event(name="after_handle")

        # 綁定 event 來發佈訊息
        self.on_state_changed += self._publish_state_change


    def _publish_state_change(self, old_state, new_state):
        msg = AgvStateChange()
        msg.agv_id = self.agv_id

        # 🔍 [DEBUG] 詳細類型信息 - 用於診斷 context_name 問題
        class_name = self.__class__.__name__
        module_name = self.__class__.__module__
        self.node.get_logger().info(
            f"🔍 [DEBUG] Context 類型檢查:\n"
            f"   - type(self): {type(self)}\n"
            f"   - __class__.__name__: {class_name}\n"
            f"   - __class__.__module__: {module_name}\n"
            f"   - agv_id: {msg.agv_id}")

        msg.context_name = class_name
        msg.from_state = old_state.__class__.__name__
        msg.to_state = new_state.__class__.__name__
        msg.timestamp = self.node.get_clock().now().to_msg()

        self.node.get_logger().info(
            f"🔍 [DEBUG] 消息內容:\n"
            f"   - msg.context_name: {msg.context_name}\n"
            f"   - msg.from_state: {msg.from_state}\n"
            f"   - msg.to_state: {msg.to_state}")

        self._publisher.publish(msg)
        self.node.get_logger().info(
            f"📣 發佈狀態變更: {msg.agv_id} {msg.from_state} → {msg.to_state}")

        
    def set_state(self, state: State):
        self.on_state_changing(self.state, state)
        self.state.leave()
        self.last_state = self.state
        self.state = state
        self.state.enter()
        self.on_state_changed(self.last_state, self.state)

    def handle(self):
        self.before_handle(self.state)
        self.state.handle(self)
        self.after_handle(self.state)
