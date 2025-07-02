from abc import ABC, abstractmethod


class ContextABC(ABC):
    """ 定義狀態機 Context 應具備的方法 """

    @abstractmethod
    def set_state(self, state):
        pass

    def handle(self):
        pass
