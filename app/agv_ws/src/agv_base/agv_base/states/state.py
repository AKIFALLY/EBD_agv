from abc import ABC, abstractmethod
from agv_base.context_abc import ContextABC
from rclpy.node import Node


class State(ABC):
    """ 狀態基礎類別，定義狀態的進入和離開方法 """

    def __init__(self, node: Node):
        self.node = node  # 保存 ROS 2 節點
        self.subscriptions = []

    @abstractmethod
    def handle(self, context: ContextABC):
        """ 根據接收到的數據決定是否切換狀態 """
        pass

    @abstractmethod
    def enter(self):
        """ 狀態進入時執行的邏輯 """
        pass

    @abstractmethod
    def leave(self):
        """ 狀態離開時執行的邏輯 """
        pass

    def create_subscription(self, msg_type, topic, callback):
        """ 創建訂閱並將其保存 """
        sub = self.node.create_subscription(msg_type, topic, callback, 10)
        self.subscriptions.append(sub)

    def remove_subscription(self):
        """ 移除所有訂閱 """
        for sub in self.subscriptions:
            self.node.destroy_subscription(sub)
        self.subscriptions.clear()
