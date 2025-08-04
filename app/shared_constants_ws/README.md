# shared_constants_ws

共享常數工作空間，提供 AGV 和 AGVC 兩個環境共用的常數定義。

## 主要功能

- **TaskStatus 常數**: 統一的任務狀態定義
- **跨環境支援**: AGV 和 AGVC 都能使用
- **解決依賴問題**: 避免跨容器依賴衝突

## 快速使用

```python
from shared_constants.task_status import TaskStatus

# 使用狀態常數
if task.status_id == TaskStatus.PENDING:
    print("任務待處理")
```

更多詳細資訊請參考 [CLAUDE.md](CLAUDE.md)。