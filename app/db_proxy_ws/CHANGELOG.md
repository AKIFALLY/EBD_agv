# Changelog - db_proxy_ws

本文件记录 db_proxy_ws（数据库代理工作空间）的所有重要变更。

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.0.0/)，
版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/)。

## [Unreleased]

## [2025-11-18] - Task 模型优化

### 移除 (Removed)
- **Task.type 字段**: 移除已废弃的 `type` 字段
  - **背景**: 该字段仅在已弃用的 TAFL WCS 系统中使用
  - **当前系统**: KUKA WCS 系统完全不依赖此字段，功能已被 `work_id` 取代
  - **影响范围**:
    - 数据库表结构变更：删除 `task.type` 列
    - Task 模型定义更新：移除 `type: Optional[str] = None`
    - 已弃用测试标记：tafl_wcs_ws 中的测试文件已添加弃用警告
  - **兼容性**: 不影响当前活跃的 KUKA WCS 业务逻辑

### 新增 (Added)
- **数据库迁移脚本**: `db_proxy/sql/migrations/001_drop_task_type_column.py`
  - 支持 upgrade（删除字段）和 downgrade（恢复字段）操作
  - 包含完整的错误处理和状态检查

### 测试 (Testing)
- ✅ 所有核心功能测试通过：
  - Task 模型定义验证
  - 数据库连接测试
  - 任务 CRUD 操作（创建、查询、更新、删除）
- ✅ 数据库迁移成功执行
- ✅ 字段移除后系统功能正常

### 技术细节
- **迁移日期**: 2025-11-18
- **数据库影响**: 仅 schema 变更，无数据丢失（type 字段数据为空）
- **代码引用**: 所有 `Task.type` 引用仅存在于已弃用的 tafl_wcs_ws 测试中
- **回滚支持**: 迁移脚本支持完整回滚（downgrade）

### 文件变更清单
```
修改:
  - app/db_proxy_ws/src/db_proxy/db_proxy/models/agvc_task.py (移除 type 字段定义)
  - app/tafl_wcs_ws/src/tafl_wcs/test/test_loader_flows.py (添加弃用警告)
  - app/tafl_wcs_ws/src/tafl_wcs/test/test_unloader_flows.py (添加弃用警告)
  - app/tafl_wcs_ws/src/tafl_wcs/test/test_loader_port_rules.py (添加弃用警告)

新增:
  - app/db_proxy_ws/src/db_proxy/db_proxy/sql/migrations/001_drop_task_type_column.py (迁移脚本)
  - app/db_proxy_ws/CHANGELOG.md (本文件)
  - agents/test_task_type_removal.py (功能测试脚本)
```

## [历史版本]

### 2025-09 - Machine 模型工作区字段更新
- 新增 `workspace_1` 和 `workspace_2` 数组字段
- 支持射出机工作区配置

### 2025 早期版本
- 初始化 db_proxy_ws 工作空间
- 实现 ConnectionPoolManager 连线池管理
- 建立 32 个 SQLModel 数据模型
- 实现 21 个 CRUD 操作层
- 集成 ROS 2 服务接口

---

**维护者**: RosAGV 开发团队
**最后更新**: 2025-11-18
