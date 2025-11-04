#!/bin/bash
# =============================================================================
# Remote AGV Management Module
# 远程 AGV 节点管理模块
#
# 功能: 通过 SSH 远程管理 AGV 容器的 launch 节点
# 依赖: sshpass, ssh, yq (可选，用于读取 YAML 配置)
# =============================================================================

# 从 node_registry.yaml 读取 AGV 配置
get_agv_config() {
    local agv_name="$1"
    local config_file="/app/config/node_registry.yaml"

    if [ ! -f "$config_file" ]; then
        echo "null|null|null|null|null"
        return 1
    fi

    # 尝试使用 yq 解析 YAML（如果可用）
    # 注意: 容器使用 Python yq (kislyuk/yq)，语法为 yq -r '.path'
    if command -v yq >/dev/null 2>&1; then
        local agv_ip=$(yq -r ".remote_agvs.$agv_name.ip" "$config_file" 2>/dev/null)
        local agv_port=$(yq -r ".remote_agvs.$agv_name.port" "$config_file" 2>/dev/null)
        local agv_user=$(yq -r ".remote_agvs.$agv_name.user" "$config_file" 2>/dev/null)
        local agv_password=$(yq -r ".remote_agvs.$agv_name.password" "$config_file" 2>/dev/null)
        local agv_type=$(yq -r ".remote_agvs.$agv_name.type" "$config_file" 2>/dev/null)

        echo "$agv_ip|$agv_port|$agv_user|$agv_password|$agv_type"
    else
        # 降级方案：使用硬编码配置（与旧版本兼容）
        # 注意: 2222 是远端 AGV 宿主机的 SSH 端口（用于从外部连接）
        case "$agv_name" in
            cargo02)
                echo "192.168.10.11|2222|ct|36274806|cargo_agv"
                ;;
            loader02)
                echo "192.168.10.12|2222|ct|36274806|loader_agv"
                ;;
            unloader02)
                echo "192.168.10.13|2222|ct|36274806|unloader_agv"
                ;;
            *)
                echo "null|null|null|null|null"
                return 1
                ;;
        esac
    fi
}

# 管理远程 AGV Launch
manage_remote_agv_launch() {
    local agv_name="${1:-}"
    local action="${2:-status}"

    if [ -z "$agv_name" ]; then
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo "  🚗 远程 AGV Launch 管理工具"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo "用法: manage_remote_agv_launch <agv_name> {start|stop|restart|status|logs}"
        echo ""
        echo "可用的 AGV:"
        echo "  - cargo02     (192.168.10.11:2200)  Cargo Mover AGV"
        echo "  - loader02    (192.168.10.12:2200)  Loader AGV"
        echo "  - unloader02  (192.168.10.13:2200)  Unloader AGV"
        echo ""
        echo "示例:"
        echo "  manage_remote_agv_launch cargo02 status   # 检查状态"
        echo "  manage_remote_agv_launch cargo02 start    # 启动 launch"
        echo "  manage_remote_agv_launch cargo02 logs     # 查看日志"
        echo ""
        echo "快捷命令:"
        echo "  list_agvs                           # 列出所有 AGV"
        echo "  check_agvs                          # 批量检查状态"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        return 1
    fi

    # 从配置文件读取 AGV 信息
    local config=$(get_agv_config "$agv_name")
    if [ "$config" == "null|null|null|null|null" ]; then
        echo "❌ 未知的 AGV: $agv_name"
        echo "💡 可用的 AGV: cargo02, loader02, unloader02"
        echo "💡 请检查 /app/config/node_registry.yaml 配置"
        return 1
    fi

    IFS='|' read -r agv_ip agv_port agv_user agv_password agv_type <<< "$config"

    # 验证配置完整性
    if [ "$agv_ip" == "null" ] || [ -z "$agv_ip" ]; then
        echo "❌ AGV 配置不完整: $agv_name"
        return 1
    fi

    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  🚗 管理远程 AGV: $agv_name"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "📍 目标: $agv_user@$agv_ip:$agv_port"
    echo "🏷️  类型: $agv_type"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    # 构建 SSH 命令
    local ssh_cmd="sshpass -p $agv_password ssh -p $agv_port -o StrictHostKeyChecking=no -o ConnectTimeout=5 $agv_user@$agv_ip"

    case "$action" in
        status)
            echo "📊 检查 AGV 节点状态..."
            echo ""

            # 检查 SSH 连接
            if ! $ssh_cmd "echo '✅ SSH 连接成功'" 2>/dev/null; then
                echo "❌ SSH 连接失败"
                echo "💡 请检查:"
                echo "   - AGV 容器是否运行"
                echo "   - SSH 服务是否启动 (Port $agv_port)"
                echo "   - 网络连接是否正常"
                echo "   - 凭证是否正确"
                return 1
            fi

            echo "🔍 查询 ROS 2 节点..."
            local nodes=$($ssh_cmd "source /app/setup.bash >/dev/null 2>&1 && ros2 node list 2>/dev/null" | grep -E '(plc_service|joy_linux_node|agv_core_node)')

            if [ -n "$nodes" ]; then
                echo "✅ AGV 节点运行中:"
                echo "$nodes" | while read -r node; do
                    echo "   • $node"
                done
            else
                echo "⚠️  未检测到 AGV 节点"
                echo "💡 AGV Launch 可能未启动，使用 'start' 命令启动"
            fi
            echo ""
            ;;

        start)
            echo "🚀 启动 AGV Launch..."
            echo ""

            # 获取 AGV 类型对应的 package 名称
            local package_name=""
            case "$agv_type" in
                cargo_agv) package_name="cargo_mover_agv" ;;
                loader_agv) package_name="loader_agv" ;;
                unloader_agv) package_name="unloader_agv" ;;
                *)
                    echo "❌ 未知的 AGV 类型: $agv_type"
                    return 1
                    ;;
            esac

            echo "📦 Package: $package_name"
            echo "🎯 Action: ros2 launch $package_name launch.py"
            echo ""

            # 发送启动命令
            $ssh_cmd "source /app/setup.bash >/dev/null 2>&1 && nohup ros2 launch $package_name launch.py > /tmp/agv_launch.log 2>&1 &"

            if [ $? -eq 0 ]; then
                echo "✅ AGV Launch 启动命令已发送"
                sleep 2
                echo ""
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                echo "  验证节点状态..."
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                manage_remote_agv_launch "$agv_name" status
            else
                echo "❌ AGV Launch 启动失败"
                return 1
            fi
            ;;

        stop)
            echo "🛑 停止 AGV 节点..."
            echo ""

            # 获取 package 名称
            local package_name=""
            case "$agv_type" in
                cargo_agv) package_name="cargo_mover_agv" ;;
                loader_agv) package_name="loader_agv" ;;
                unloader_agv) package_name="unloader_agv" ;;
            esac

            # 发送停止命令
            $ssh_cmd "pkill -f 'ros2 launch $package_name'"

            if [ $? -eq 0 ]; then
                echo "✅ AGV 节点停止命令已发送"
                sleep 1
                echo ""
            else
                echo "⚠️  可能没有运行的 AGV 节点"
            fi
            ;;

        restart)
            echo "🔄 重启 AGV Launch..."
            echo ""
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  步骤 1/2: 停止现有节点"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            manage_remote_agv_launch "$agv_name" stop
            sleep 2
            echo ""
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  步骤 2/2: 启动新节点"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            manage_remote_agv_launch "$agv_name" start
            ;;

        logs)
            echo "📜 查看 AGV Launch 日志..."
            echo ""
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            $ssh_cmd "tail -n 30 /tmp/agv_launch.log 2>/dev/null" || {
                echo "⚠️  日志文件不存在或无法访问"
                echo "💡 可能 AGV Launch 从未启动过"
            }
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            ;;

        *)
            echo "❌ 未知的操作: $action"
            echo "用法: manage_remote_agv_launch $agv_name {start|stop|restart|status|logs}"
            return 1
            ;;
    esac
}

# 列出所有可用的远程 AGV
list_remote_agvs() {
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  🚗 可用的远程 AGV"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    local config_file="/app/config/node_registry.yaml"

    if command -v yq >/dev/null 2>&1 && [ -f "$config_file" ]; then
        local agv_names=$(yq -r '.remote_agvs | keys[]' "$config_file" 2>/dev/null)

        if [ -n "$agv_names" ]; then
            echo "$agv_names" | while read -r agv_name; do
                local config=$(get_agv_config "$agv_name")
                IFS='|' read -r agv_ip agv_port agv_user agv_password agv_type <<< "$config"

                echo "  • $agv_name"
                echo "    ├─ 类型: $agv_type"
                echo "    ├─ IP: $agv_ip"
                echo "    └─ Port: $agv_port"
                echo ""
            done
        else
            echo "  (从配置文件未读取到 AGV)"
        fi
    else
        # 降级方案：显示硬编码的 AGV
        echo "  • cargo02"
        echo "    ├─ 类型: cargo_agv"
        echo "    ├─ IP: 192.168.10.11"
        echo "    └─ Port: 2200"
        echo ""
        echo "  • loader02"
        echo "    ├─ 类型: loader_agv"
        echo "    ├─ IP: 192.168.10.12"
        echo "    └─ Port: 2200"
        echo ""
        echo "  • unloader02"
        echo "    ├─ 类型: unloader_agv"
        echo "    ├─ IP: 192.168.10.13"
        echo "    └─ Port: 2200"
        echo ""
    fi

    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "💡 使用方法: manage_remote_agv_launch <agv_name> {start|stop|status|logs}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
}

# 批量检查所有远程 AGV 状态
check_all_remote_agvs() {
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  📊 批量检查所有远程 AGV 状态"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""

    local config_file="/app/config/node_registry.yaml"
    local agv_names=""

    if command -v yq >/dev/null 2>&1 && [ -f "$config_file" ]; then
        agv_names=$(yq -r '.remote_agvs | keys[]' "$config_file" 2>/dev/null)
    else
        # 降级方案
        agv_names="cargo02 loader02 unloader02"
    fi

    for agv_name in $agv_names; do
        manage_remote_agv_launch "$agv_name" status
        echo ""
    done

    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  检查完成"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
}

# ===== 别名定义 =====
alias list_agvs='list_remote_agvs'
alias check_agvs='check_all_remote_agvs'

# ===== 模块初始化完成 =====
log_debug "✅ Remote AGV Management 模组已载入"
