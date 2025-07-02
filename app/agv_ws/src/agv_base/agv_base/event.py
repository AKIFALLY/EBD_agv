import asyncio
import inspect

class Event:
    def __init__(self, name=None, print_event=False, pass_exception=True):
        self._handlers = []
        self.name = name or "UnnamedEvent"
        self.print_event = print_event
        self.pass_exception = pass_exception

    def __iadd__(self, handler):
        self._handlers.append(handler)
        return self

    def __isub__(self, handler):
        self._handlers.remove(handler)
        return self

    def clear(self):
        self._handlers.clear()

    def __call__(self, *args, **kwargs):
        if self.print_event:
            print(f"[Event] {self.name} triggered with args={args} kwargs={kwargs}")
        for handler in self._handlers:
            try:
                if inspect.iscoroutinefunction(handler):
                    asyncio.create_task(handler(*args, **kwargs))  # 非同步處理器
                else:
                    handler(*args, **kwargs)
            except Exception as e:
                if self.pass_exception:
                    print(f"[Event] Error in handler {handler}: {e}")
                else:
                    raise
