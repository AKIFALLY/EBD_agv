# ROS2 管理函数标准化模板

## 📋 概述

本文档提供了创建高质量、可靠的服务管理函数的标准模板和指南。所有模板基于 `manage_web_api_launch` 的最佳实践。

## 🎯 设计目标

1. **可靠性**：每次都能正常启动和停止
2. **幂等性**：多次执行不会重复启动
3. **自动修复**：自动清理端口冲突和残留进程
4. **完整追踪**：记录所有相关进程 PID
5. **详细反馈**：失败时提供明确的诊断信息

## 📝 标准化模板

### 完整模板（ROS2 Launch 服务）

```bash
# =============================================================================
# 服务名称管理函数
# =============================================================================
#
# 功能说明：
#   管理 XXX 服务的启动、停止、重启和状态检查
#
# 依赖检查：
#   - 工作空间：xxx_ws
#   - 配置文件：/app/config/xxx_config.yaml
#   - 端口：8XXX（如果有）
#
# 使用方式：
#   manage_xxx_service start   - 启动服务
#   manage_xxx_service stop    - 停止服务
#   manage_xxx_service restart - 重启服务
#   manage_xxx_service status  - 查看服务状态
#
# =============================================================================

manage_xxx_service() {
    local LOG_FILE="/tmp/xxx_service.log"
    local PID_FILE="/tmp/xxx_service.pid"

    case "$1" in
        start)
            # ========== 阶段 1: 启动前检查 ==========

            # 1.1 检查是否已运行（幂等性）
            if [ -f "$PID_FILE" ]; then
                local all_running=true
                while read pid; do
                    if ! kill -0 $pid 2>/dev/null; then
                        all_running=false
                        break
                    fi
                done < "$PID_FILE"

                if [ "$all_running" = true ]; then
                    echo "✅ XXX Service 已经在运行中"
                    return 0
                else
                    echo "⏳ 检测到过时的 PID 文件，正在清理..."
                    rm -f "$PID_FILE"
                fi
            fi

            # 1.2 检查工作空间是否已建置
            if [ ! -d "/app/xxx_ws/install" ]; then
                echo "⚠️ 警告: xxx_ws 未建置，请先执行: build_ws xxx_ws"
            fi

            # 1.3 检查依赖文件
            if [ ! -f "/app/config/xxx_config.yaml" ]; then
                echo "❌ 配置文件不存在: /app/config/xxx_config.yaml"
                return 1
            fi

            # ========== 阶段 2: 端口冲突检查和清理（如果服务需要端口） ==========

            local port_conflict=false
            local required_ports=(8000 8001)  # 根据实际需要修改

            for port in "${required_ports[@]}"; do
                if ss -tulnp 2>/dev/null | grep -q ":$port "; then
                    echo "⚠️ 端口 $port 被占用，尝试清理..."

                    # 提取 PID 并终止
                    local pids=$(ss -tulnp 2>/dev/null | grep ":$port " | grep -oP 'pid=\K[0-9]+' | sort -u)

                    if [ -n "$pids" ]; then
                        for pid in $pids; do
                            echo "   终止占用端口 $port 的进程 PID: $pid"
                            kill -9 $pid 2>/dev/null
                        done
                        sleep 1

                        # 再次检查端口是否释放
                        if ss -tulnp 2>/dev/null | grep -q ":$port "; then
                            echo "❌ 端口 $port 清理失败"
                            port_conflict=true
                        else
                            echo "✅ 端口 $port 已清理"
                        fi
                    fi
                fi
            done

            # 如果有端口冲突无法解决，返回错误
            if [ "$port_conflict" = true ]; then
                echo "❌ 端口冲突无法解决，启动失败"
                echo ""
                echo "💡 建议："
                echo "   1. 检查端口占用: ss -tulnp | grep -E '(8000|8001)'"
                echo "   2. 手动停止服务: manage_xxx_service stop"
                echo "   3. 重新启动服务: manage_xxx_service start"
                return 1
            fi

            # ========== 阶段 3: 启动服务 ==========

            echo "🚀 启动 XXX Service..."

            # 启动服务（根据实际情况选择其中一种）
            # 选项 A: 使用 ros2 launch
            nohup bash -c "source /app/setup.bash && agvc_source > /dev/null 2>&1 && ros2 launch xxx_ws xxx_launch.py" > "$LOG_FILE" 2>&1 &

            # 选项 B: 使用 ros2 run
            # nohup bash -c "source /app/setup.bash && agvc_source > /dev/null 2>&1 && ros2 run xxx_package xxx_node --ros-args -r __ns:=/agvc --params-file /app/config/xxx_config.yaml" > "$LOG_FILE" 2>&1 &

            local PARENT_PID=$!

            # 记录父进程
            echo $PARENT_PID > "$PID_FILE"

            # 等待子进程启动
            sleep 5

            # 找出所有子进程并记录
            local CHILD_PIDS=$(pgrep -P $PARENT_PID)
            if [ -n "$CHILD_PIDS" ]; then
                for pid in $CHILD_PIDS; do
                    echo $pid >> "$PID_FILE"
                done
            fi

            # 记录实际的服务进程（针对 launch 文件）
            sleep 2
            local service_patterns=("xxx_server" "xxx_node")
            for pattern in "${service_patterns[@]}"; do
                local SERVICE_PID=$(pgrep -f "$pattern" | head -n1)
                if [ -n "$SERVICE_PID" ]; then
                    if ! grep -q "^$SERVICE_PID$" "$PID_FILE" 2>/dev/null; then
                        echo $SERVICE_PID >> "$PID_FILE"
                    fi
                fi
            done

            # ========== 阶段 4: 验证启动 ==========

            # 4.1 验证父进程
            if ! kill -0 $PARENT_PID 2>/dev/null; then
                echo "❌ XXX Service 启动失败（父进程未运行）"
                echo ""
                echo "💡 诊断建议："
                echo "   1. 查看日志: tail -f $LOG_FILE"
                echo "   2. 检查工作空间建置: ls /app/xxx_ws/install"
                echo "   3. 重新建置: cd /app/xxx_ws && colcon build --packages-select xxx_package"
                return 1
            fi

            echo "✅ XXX Service 已启动"
            echo "   记录的 PID: $(cat $PID_FILE | tr '\n' ' ')"

            # 4.2 验证 ROS2 节点注册（针对 ROS2 节点）
            if verify_ros2_node_startup "/agvc/xxx_node" 15; then
                echo "✅ ROS2 节点已注册"
            else
                echo "⚠️ ROS2 节点注册超时"
                echo ""
                echo "💡 建议："
                echo "   1. 查看日志: tail -f $LOG_FILE"
                echo "   2. 检查 Zenoh Router 状态: manage_zenoh status"
                return 1
            fi

            # 4.3 验证端口开启（如果服务需要端口）
            for port in "${required_ports[@]}"; do
                if wait_for_port_with_retry $port 30; then
                    echo "✅ 端口 $port 已开启"
                else
                    echo "❌ 端口 $port 等待超时"
                    echo ""
                    echo "💡 建议："
                    echo "   1. 查看日志: tail -f $LOG_FILE"
                    echo "   2. 检查配置: cat /app/config/xxx_config.yaml"
                    return 1
                fi
            done

            echo "✅ XXX Service 启动完成"
            ;;

        stop)
            echo "⏳ 停止 XXX Service..."

            # ========== 阶段 1: 优雅停止 ==========
            if [ -f "$PID_FILE" ]; then
                local PIDS=$(tac "$PID_FILE")  # 反向读取（先子后父）
                for pid in $PIDS; do
                    if kill -0 $pid 2>/dev/null; then
                        echo "   停止进程 PID: $pid"
                        kill $pid 2>/dev/null  # SIGTERM
                    fi
                done

                sleep 3  # 等待优雅退出

                # ========== 阶段 2: 强制终止 ==========
                for pid in $PIDS; do
                    if kill -0 $pid 2>/dev/null; then
                        echo "   强制终止 PID: $pid"
                        kill -9 $pid 2>/dev/null  # SIGKILL
                    fi
                done

                rm -f "$PID_FILE"
            fi

            # ========== 阶段 3: 备用清理（无 PID 文件时） ==========
            if [ ! -f "$PID_FILE" ]; then
                echo "🚨 PID 文件未找到，检查相关进程..."
                local service_patterns=("xxx_launch" "xxx_server" "xxx_node")
                local found_process=false

                for pattern in "${service_patterns[@]}"; do
                    if pgrep -f "$pattern" > /dev/null; then
                        echo "   发现进程: $pattern"
                        pkill -f "$pattern"
                        found_process=true
                    fi
                done

                if [ "$found_process" = true ]; then
                    sleep 2
                    echo "   相关进程已停止"
                fi
            fi

            # ========== 阶段 4: 残留进程清理 ==========
            echo "🔍 检查并清理残留进程..."
            local found_residual=false
            local service_patterns=("xxx_launch" "xxx_server" "xxx_node")

            for pattern in "${service_patterns[@]}"; do
                if pgrep -f "$pattern" > /dev/null; then
                    echo "   发现残留进程: $pattern"
                    pkill -9 -f "$pattern" 2>/dev/null
                    found_residual=true
                fi
            done

            if [ "$found_residual" = true ]; then
                sleep 2
                echo "   残留进程已清理"
            fi

            # ========== 阶段 5: 端口强制释放（如果服务需要端口） ==========
            echo "🔍 检查端口..."
            local port_released=false
            local required_ports=(8000 8001)

            for port in "${required_ports[@]}"; do
                if ss -tulnp 2>/dev/null | grep -q ":$port "; then
                    echo "🚨 端口 $port 仍被占用，强制释放..."

                    local pids=$(ss -tulnp 2>/dev/null | grep ":$port " | grep -oP 'pid=\K[0-9]+' | sort -u)
                    if [ -n "$pids" ]; then
                        for pid in $pids; do
                            echo "   终止占用端口 $port 的进程 PID: $pid"
                            kill -9 $pid 2>/dev/null
                        done
                        port_released=true
                    fi
                fi
            done

            if [ "$port_released" = true ]; then
                sleep 2
                echo "✅ 端口已强制释放"
            else
                echo "✅ 所有端口正常释放"
            fi

            # ========== 阶段 6: 临时文件清理 ==========
            echo "🧹 清理临时文件..."
            local launch_params_count=$(find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 2>/dev/null | wc -l)
            if [ "$launch_params_count" -gt 0 ]; then
                find /tmp -maxdepth 1 -name 'launch_params_*' -type d -mtime -1 -exec rm -rf {} + 2>/dev/null
                echo "   清理了 $launch_params_count 个 launch_params 临时目录"
            fi

            echo "✅ XXX Service 已停止"
            ;;

        restart)
            manage_xxx_service stop
            sleep 2
            manage_xxx_service start
            ;;

        status)
            if [ -f "$PID_FILE" ]; then
                local all_running=true
                local pids=""

                while read pid; do
                    if kill -0 $pid 2>/dev/null; then
                        pids="$pids $pid"
                    else
                        all_running=false
                    fi
                done < "$PID_FILE"

                if [ "$all_running" = true ]; then
                    echo "✅ XXX Service 运行中 (PIDs:$pids)"

                    # 显示 ROS2 节点信息（如果适用）
                    if ros2 node list 2>/dev/null | grep -q "/agvc/xxx_node"; then
                        echo "   ROS2 节点: /agvc/xxx_node 已注册"
                    fi

                    # 显示端口信息（如果适用）
                    local required_ports=(8000 8001)
                    for port in "${required_ports[@]}"; do
                        if ss -tulnp 2>/dev/null | grep -q ":$port "; then
                            echo "   端口 $port: 已监听"
                        fi
                    done

                    return 0
                else
                    echo "⚠️ XXX Service 部分进程未运行"
                    rm -f "$PID_FILE"
                    return 1
                fi
            else
                echo "🚫 XXX Service 未运行"
                return 1
            fi
            ;;

        logs)
            if [ -f "$LOG_FILE" ]; then
                echo "📄 实时日志 (Ctrl+C 退出):"
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                tail -f "$LOG_FILE"
            else
                echo "❌ 日志文件不存在: $LOG_FILE"
                return 1
            fi
            ;;

        *)
            echo "用法: manage_xxx_service {start|stop|restart|status|logs}"
            echo ""
            echo "指令说明："
            echo "  start   - 启动服务"
            echo "  stop    - 停止服务"
            echo "  restart - 重启服务"
            echo "  status  - 查看服务状态"
            echo "  logs    - 查看实时日志"
            return 1
            ;;
    esac
}
```

## 🔍 验证方法决策树

### 决策流程图

```
选择验证方法：
├─ 是否为 ROS2 节点？
│  ├─ 是 → 使用 verify_ros2_node_startup()
│  │       适用：管理单一 ROS2 节点（plc_service, ecs_core）
│  │       优势：深度验证（进程存在 + ROS2 网络注册）
│  │
│  └─ 否 → 是否需要检查端口？
│     ├─ 是 → 组合验证：
│     │       1. verify_process_startup()  # 进程启动
│     │       2. wait_for_port_with_retry() # 端口开启
│     │       适用：Web 服务（8000, 8001, 8002）
│     │
│     └─ 否 → 使用 verify_process_startup()
│               适用：非 ROS2 进程（Zenoh Router, Launch 文件）
```

### 验证函数说明

#### 1. verify_ros2_node_startup()

**用途**：验证 ROS2 节点是否成功启动并注册到网络

```bash
# 使用示例
if verify_ros2_node_startup "/agvc/plc_service" 10; then
    echo "✅ 节点验证成功"
else
    echo "❌ 节点验证失败"
    return 1
fi
```

**参数**：
- `$1`: 完整节点名称（例如：`/agvc/plc_service`）
- `$2`: 超时时间（秒），默认 10 秒
- `$3`: 是否显示进度，默认 true

**适用场景**：
- `ros2 run` 启动的单一节点
- `ros2 launch` 启动的命名节点
- 需要确认 ROS2 网络注册的服务

**优势**：
- 深度验证（不仅进程存在，还验证 ROS2 通信正常）
- 智能等待（最多等待 timeout 秒）
- 进度显示（避免用户焦虑）

#### 2. verify_process_startup()

**用途**：验证进程是否成功启动（进程模式匹配）

```bash
# 使用示例
if verify_process_startup "xxx_launch" 10; then
    echo "✅ 进程验证成功"
else
    echo "❌ 进程验证失败"
    return 1
fi
```

**参数**：
- `$1`: pgrep 搜索模式
- `$2`: 超时时间（秒），默认 10 秒
- `$3`: 是否显示进度，默认 true

**适用场景**：
- `ros2 launch` 启动的多进程服务
- 非 ROS2 原生进程（如 Zenoh Router）
- 不需要验证 ROS2 网络注册的服务

**优势**：
- 快速检测（查询 /proc 文件系统，< 100ms）
- 灵活匹配（支持正则表达式模式）
- 返回 PIDs 列表

#### 3. wait_for_port_with_retry()

**用途**：等待端口开启（用于 Web 服务验证）

```bash
# 使用示例
if wait_for_port_with_retry 8000 30; then
    echo "✅ 端口 8000 已开启"
else
    echo "❌ 端口 8000 等待超时"
    return 1
fi
```

**参数**：
- `$1`: 端口号
- `$2`: 最大等待时间（秒），默认 30 秒

**适用场景**：
- Web 服务启动验证
- 需要确认端口监听的服务
- 网络服务健康检查

**优势**：
- 独立于进程检查（端口开启 = 服务就绪）
- 进度显示（每 2 秒更新）
- 使用 `ss` 命令（比 `lsof` 更通用）

### 组合验证示例

```bash
# 案例 1: PLC 服务（核心 ROS2 节点）
if verify_ros2_node_startup "/agvc/plc_service" 10; then
    echo "✅ PLC 服务启动成功"
fi

# 案例 2: Web API Launch（多进程服务 + 端口）
if verify_process_startup "web_api_launch" 5; then
    if wait_for_port_with_retry 8000 30 && \
       wait_for_port_with_retry 8001 30 && \
       wait_for_port_with_retry 8002 30; then
        echo "✅ Web API 启动成功"
    fi
fi

# 案例 3: Zenoh Router（原生进程 + 可选端口）
if verify_process_startup "rmw_zenohd" 10; then
    # 端口验证失败不报错（某些配置可能不绑定端口）
    if wait_for_port_with_retry 7447 10; then
        echo "✅ Zenoh Router 端口已开启"
    else
        echo "⚠️ Zenoh Router 端口未开启（可能是配置原因）"
    fi
    echo "✅ Zenoh Router 启动成功"
fi
```

## 🎨 简化模板（ROS2 单一节点）

适用于使用 `ros2 run` 启动的简单节点：

```bash
manage_xxx_node() {
    local LOG_FILE="/tmp/xxx_node.log"
    local PID_FILE="/tmp/xxx_node.pid"

    case "$1" in
        start)
            # 检查是否已运行
            if [ -f "$PID_FILE" ] && kill -0 $(cat "$PID_FILE") 2>/dev/null; then
                echo "✅ XXX Node 已经在运行中"
                return 0
            fi

            # 检查依赖
            if [ ! -f "/app/config/xxx_config.yaml" ]; then
                echo "❌ 配置文件不存在"
                return 1
            fi

            # 启动节点
            echo "🚀 启动 XXX Node..."
            nohup ros2 run xxx_package xxx_node \
                --ros-args -r __ns:=/agvc \
                --params-file /app/config/xxx_config.yaml \
                > "$LOG_FILE" 2>&1 &

            echo $! > "$PID_FILE"

            # 验证启动
            if verify_ros2_node_startup "/agvc/xxx_node" 10; then
                echo "✅ XXX Node 启动成功 (PID: $(cat $PID_FILE))"
            else
                echo "❌ XXX Node 启动失败"
                return 1
            fi
            ;;

        stop)
            if [ -f "$PID_FILE" ]; then
                local PID=$(cat "$PID_FILE")
                if kill -0 $PID 2>/dev/null; then
                    echo "⏳ 停止 XXX Node..."
                    kill $PID 2>/dev/null
                    sleep 3

                    # 强制终止（如果还在运行）
                    if kill -0 $PID 2>/dev/null; then
                        kill -9 $PID 2>/dev/null
                    fi
                fi
                rm -f "$PID_FILE"
            fi

            # 备用清理
            pkill -9 -f "xxx_node" 2>/dev/null
            echo "✅ XXX Node 已停止"
            ;;

        restart)
            manage_xxx_node stop
            sleep 2
            manage_xxx_node start
            ;;

        status)
            if [ -f "$PID_FILE" ] && kill -0 $(cat "$PID_FILE") 2>/dev/null; then
                echo "✅ XXX Node 运行中 (PID: $(cat $PID_FILE))"
                return 0
            else
                echo "🚫 XXX Node 未运行"
                return 1
            fi
            ;;

        *)
            echo "用法: manage_xxx_node {start|stop|restart|status}"
            return 1
            ;;
    esac
}
```

## ⚠️ 常见陷阱和解决方案

### 1. PID 文件管理

**陷阱**：只记录父进程，子进程残留
```bash
# ❌ 错误
echo $! > "$PID_FILE"
```

**解决**：记录所有相关进程
```bash
# ✅ 正确
echo $PARENT_PID > "$PID_FILE"
sleep 5
pgrep -P $PARENT_PID >> "$PID_FILE"  # 添加子进程
pgrep -f "service_pattern" >> "$PID_FILE"  # 添加服务进程
```

### 2. 端口冲突处理

**陷阱**：不检查端口，直接启动
```bash
# ❌ 错误
nohup service_command > log 2>&1 &
```

**解决**：启动前自动清理端口
```bash
# ✅ 正确
if ss -tulnp | grep -q ":8000 "; then
    # 提取 PID 并终止
    local pids=$(ss -tulnp | grep ":8000 " | grep -oP 'pid=\K[0-9]+' | sort -u)
    for pid in $pids; do
        kill -9 $pid
    done
fi
```

### 3. 停止流程

**陷阱**：只 kill 一次，不检查是否退出
```bash
# ❌ 错误
kill $(cat "$PID_FILE")
rm -f "$PID_FILE"
```

**解决**：两阶段停止 + 残留清理
```bash
# ✅ 正确
# 阶段 1: 优雅停止 (SIGTERM)
for pid in $PIDS; do
    kill $pid 2>/dev/null
done
sleep 3

# 阶段 2: 强制终止 (SIGKILL)
for pid in $PIDS; do
    if kill -0 $pid 2>/dev/null; then
        kill -9 $pid 2>/dev/null
    fi
done
```

### 4. 验证方法选择

**陷阱**：使用 `sleep` 等待固定时间
```bash
# ❌ 错误
nohup service_command > log 2>&1 &
sleep 10  # 固定等待
echo "服务已启动"
```

**解决**：使用动态验证
```bash
# ✅ 正确
nohup service_command > log 2>&1 &
if verify_ros2_node_startup "/agvc/xxx_node" 15; then
    echo "✅ 服务已启动"
else
    echo "❌ 服务启动失败"
    return 1
fi
```

### 5. 临时文件清理

**陷阱**：不清理 launch_params 临时目录
```bash
# ❌ 错误：停止后不清理
kill $PID
```

**解决**：每次停止时清理
```bash
# ✅ 正确
kill $PID
find /tmp -name 'launch_params_*' -mtime -1 -exec rm -rf {} +
```

## 📊 核心检查清单

创建新的管理函数时，确保包含以下所有项目：

### 启动流程（start）
- [ ] 检查是否已运行（幂等性）
- [ ] 检查工作空间建置
- [ ] 检查依赖文件存在
- [ ] 端口冲突检查和清理（如果需要）
- [ ] 启动服务并记录 PID
- [ ] 记录子进程和服务进程
- [ ] 验证启动（ROS2 节点/进程/端口）
- [ ] 失败时提供诊断建议

### 停止流程（stop）
- [ ] 阶段 1: 优雅停止 (SIGTERM)
- [ ] 阶段 2: 强制终止 (SIGKILL)
- [ ] 阶段 3: 备用清理（无 PID 文件时）
- [ ] 阶段 4: 残留进程清理
- [ ] 阶段 5: 端口强制释放（如果需要）
- [ ] 阶段 6: 临时文件清理

### 状态检查（status）
- [ ] 检查 PID 文件存在
- [ ] 检查所有进程存活
- [ ] 显示 ROS2 节点状态（如果适用）
- [ ] 显示端口状态（如果适用）
- [ ] 返回正确的退出码（0=运行中, 1=未运行）

### 通用要求
- [ ] 支持 logs 子命令（tail -f 日志）
- [ ] 详细的错误信息和诊断建议
- [ ] 统一的输出格式（✅ ❌ ⚠️ 💡 等）
- [ ] 正确的返回值（成功=0, 失败=1）

## 🔗 参考

- **最佳实践示例**: `manage_web_api_launch`（app/setup_modules/node-management.bash:603）
- **验证函数**: common.bash（app/setup_modules/common.bash）
- **开发指南**: [ROS2 开发规范](./ros2-development.md)
