# RosAGV Bash 命令使用手冊

## 📖 目錄

- [1. 檔案描述符與重定向](#1-檔案描述符與重定向)
- [2. 管道與命令組合](#2-管道與命令組合)
- [3. RosAGV 核心命令集](#3-rosagv-核心命令集)
- [4. 實用腳本模式](#4-實用腳本模式)
- [5. 進階技巧與最佳實踐](#5-進階技巧與最佳實踐)
- [6. 快速參考卡](#6-快速參考卡)
- [7. 常見問題與解決方案](#7-常見問題與解決方案)

---

## 1. 檔案描述符與重定向

### 1.1 檔案描述符基礎

每個 Bash 進程都有三個標準檔案描述符：

```bash
0 - stdin  (標準輸入)   ← 預設從鍵盤讀取
1 - stdout (標準輸出)   ← 預設輸出到終端
2 - stderr (標準錯誤)   ← 預設輸出到終端
```

### 1.2 重定向符號詳解

| 符號 | 功能 | 範例 | 說明 |
|------|------|------|------|
| `>` | 覆蓋寫入 | `echo "text" > file.txt` | stdout 重定向到檔案 |
| `>>` | 附加寫入 | `echo "text" >> file.txt` | stdout 附加到檔案 |
| `2>` | 錯誤重定向 | `command 2> error.log` | stderr 重定向到檔案 |
| `2>&1` | 錯誤到標準輸出 | `command 2>&1` | stderr 重定向到 stdout |
| `&>` | 全部重定向 | `command &> all.log` | stdout+stderr 到檔案 |
| `/dev/null` | 黑洞裝置 | `command > /dev/null` | 丟棄輸出 |

### 1.3 管道中的檔案描述符

**重要概念：每個進程都有獨立的檔案描述符**

```bash
command1 | command2 | command3
```

每個 command 都有自己的 0, 1, 2：

```
command1:
  └─ stdout(1) ───┐
                  │ (pipe)
command2:         │
  ├─ stdin(0) ────┘
  ├─ stdout(1) ───┐
  └─ stderr(2) ──→ 終端
                  │ (pipe)
command3:         │
  ├─ stdin(0) ────┘
  ├─ stdout(1) ──→ 終端
  └─ stderr(2) ──→ 終端
```

### 1.4 RosAGV 實際範例

#### 配置檢查（來自 validate-docker-config.sh）
```bash
# 靜默檢查配置，只關心成功/失敗
if docker compose -f docker-compose.yml config &> /dev/null; then
    echo "✅ 配置正確"
else
    echo "❌ 配置錯誤"
    docker compose -f docker-compose.yml config  # 顯示錯誤詳情
fi
```

#### 網路連接測試（來自 check_network.sh）
```bash
# ping 測試，完全靜默
if ping -c 3 "$TARGET_IP" > /dev/null 2>&1; then
    echo "✅ 網路連接正常"
else
    echo "❌ 網路連接失敗"
fi
```

#### 健康檢查組合
```bash
# 檢查服務，只記錄錯誤
timeout 5 curl -s http://192.168.100.100:8000/health > /dev/null 2>> error.log
if [ $? -eq 0 ]; then
    echo "✅ Web API 正常"
else
    echo "❌ Web API 異常，檢查 error.log"
fi
```

---

## 2. 管道與命令組合

### 2.1 管道基礎

管道 (`|`) 將前一個命令的 stdout 連接到下一個命令的 stdin：

```bash
command1 | command2 | command3
```

**注意：stderr 不會透過管道傳遞！**

### 2.2 條件執行

| 操作符 | 功能 | 範例 | 說明 |
|--------|------|------|------|
| `&&` | 成功則執行 | `test && action` | 前命令成功才執行後命令 |
| `\|\|` | 失敗則執行 | `test \|\| fallback` | 前命令失敗才執行後命令 |
| `;` | 順序執行 | `cmd1; cmd2` | 依序執行，不管成功失敗 |

### 2.3 RosAGV 實際管道應用

#### 服務狀態檢查管道
```bash
# 檢查容器並篩選特定狀態
docker ps | grep "agvc_server" | awk '{print $1}' | xargs docker inspect
```

#### 日誌分析管道
```bash
# 分析錯誤日誌，統計錯誤類型
docker logs agvc_server 2>&1 | grep "ERROR" | awk '{print $3}' | sort | uniq -c
```

#### 網路診斷管道
```bash
# 檢查端口占用
ss -tulpn | grep ":8000" | awk '{print $7}' | cut -d',' -f2
```

#### 健康檢查組合
```bash
# 全面健康檢查
check_docker() {
    docker info >/dev/null 2>&1 && echo "Docker: ✅" || echo "Docker: ❌"
}

check_network() {
    timeout 3 bash -c "echo > /dev/tcp/192.168.100.100/7447" 2>/dev/null && \
    echo "Zenoh: ✅" || echo "Zenoh: ❌"
}

check_api() {
    curl -s -o /dev/null -w "%{http_code}" http://192.168.100.100:8000/health | \
    grep -q "200" && echo "API: ✅" || echo "API: ❌"
}

# 執行所有檢查
check_docker && check_network && check_api
```

---

## 3. RosAGV 核心命令集

### 3.1 容器管理

#### Docker Compose 操作
```bash
# 啟動服務（分離模式）
docker compose -f docker-compose.agvc.yml up -d

# 檢查容器狀態
docker compose -f docker-compose.agvc.yml ps

# 查看服務日誌
docker compose -f docker-compose.agvc.yml logs -f agvc_server

# 進入容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 重啟特定服務
docker compose -f docker-compose.agvc.yml restart agvc_server

# 停止所有服務
docker compose -f docker-compose.agvc.yml down
```

#### Docker 操作
```bash
# 列出運行中的容器
docker ps

# 查看容器詳細資訊
docker inspect container_name

# 容器內執行命令
docker exec -it container_name command

# 檢查 Docker 狀態
docker info >/dev/null 2>&1 && echo "正常" || echo "異常"
```

### 3.2 系統監控

#### 進程管理
```bash
# 查看所有進程
ps aux

# 查找特定進程
ps aux | grep "agvc"

# 進程樹顯示
pstree -p

# 殺掉進程
pkill -f "process_name"
```

#### 網路監控
```bash
# 查看網路連接
ss -tulpn

# 檢查特定端口
ss -tulpn | grep ":8000"

# 測試 TCP 連接
timeout 3 bash -c "echo > /dev/tcp/192.168.100.100/7447"

# 網路介面狀態
ip addr show
```

### 3.3 HTTP 與 API 測試

#### curl 常用模式
```bash
# 基本 GET 請求
curl -s http://192.168.100.100:8000/health

# 只取得 HTTP 狀態碼
curl -s -o /dev/null -w "%{http_code}" http://192.168.100.100:8000/health

# 測試響應時間
curl -s -o /dev/null -w "%{time_total}" http://192.168.100.100:8000/health

# POST 請求
curl -X POST -H "Content-Type: application/json" \
     -d '{"key":"value"}' http://192.168.100.100:8000/api

# 忽略證書錯誤
curl -k -s https://example.com

# 設定超時
timeout 5 curl -s http://192.168.100.100:8000/health
```

### 3.4 文本處理

#### grep 模式匹配
```bash
# 基本搜尋
grep "ERROR" logfile.txt

# 忽略大小寫
grep -i "error" logfile.txt

# 顯示行號
grep -n "pattern" file.txt

# 只顯示匹配的檔案名
grep -l "pattern" *.txt

# 靜默模式（只返回狀態碼）
grep -q "pattern" file.txt && echo "找到" || echo "沒找到"

# 反向匹配
grep -v "exclude_pattern" file.txt
```

#### awk 文本處理
**命名來源：** AWK 由三位創造者姓氏首字母組成：**A**ho, **W**einberger, **K**ernighan（1977年貝爾實驗室開發）

```bash
# 印出特定欄位
ps aux | awk '{print $2, $11}'

# 條件篩選
ss -tulpn | awk '$4 ~ /:8000$/ {print $7}'

# 統計計算
docker ps | awk 'NR>1 {count++} END {print "容器數量:", count}'

# 格式化輸出
df -h | awk '{printf "%-20s %s\n", $1, $5}'
```

#### sed 文本編輯
**命名來源：** SED 是 **S**tream **ED**itor 的縮寫，專為流式文本編輯設計（1973-1974年 Lee E. McMahon 開發）

```bash
# 替換文本
sed 's/old/new/g' file.txt

# 刪除空行
sed '/^$/d' file.txt

# 印出特定行
sed -n '10,20p' file.txt

# 就地編輯
sed -i 's/old/new/g' file.txt
```

### 3.5 檔案操作

#### find 檔案搜尋
```bash
# 按名稱搜尋
find /path -name "*.log"

# 按時間搜尋（24小時內修改）
find /path -mtime -1

# 按大小搜尋
find /path -size +100M

# 搜尋並執行操作
find /path -name "*.tmp" -delete

# 搜尋特定類型並統計
find /app -name "*.py" | wc -l
```

#### 檔案查看
```bash
# 查看檔案開頭
head -20 file.txt

# 查看檔案結尾
tail -20 file.txt

# 即時監控檔案
tail -f logfile.txt

# 查看特定行數範圍
sed -n '10,20p' file.txt

# 分頁查看
less file.txt
```

---

## 4. 實用腳本模式

### 4.1 服務健康檢查模式

#### 基本健康檢查框架
```bash
#!/bin/bash
# 健康檢查腳本模板

check_service() {
    local service_name=$1
    local check_command=$2
    local success_message=$3
    local failure_message=$4

    echo "檢查 $service_name..."
    if eval "$check_command" >/dev/null 2>&1; then
        echo "✅ $success_message"
        return 0
    else
        echo "❌ $failure_message"
        return 1
    fi
}

# 使用範例
check_service "Docker" \
    "docker info" \
    "Docker 服務正常" \
    "Docker 服務異常"

check_service "Web API" \
    "timeout 5 curl -s http://192.168.100.100:8000/health" \
    "Web API 回應正常" \
    "Web API 無回應"
```

#### RosAGV 完整健康檢查
```bash
#!/bin/bash
# RosAGV 系統健康檢查

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HEALTH_LOG="/tmp/rosagv_health.log"

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_result() {
    local status=$1
    local message=$2
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $status: $message" >> "$HEALTH_LOG"
}

check_docker() {
    if docker info >/dev/null 2>&1; then
        echo -e "${GREEN}✅ Docker 服務正常${NC}"
        log_result "SUCCESS" "Docker service is running"
        return 0
    else
        echo -e "${RED}❌ Docker 服務異常${NC}"
        log_result "ERROR" "Docker service not available"
        return 1
    fi
}

check_containers() {
    local containers=("agvc_server" "postgres" "nginx")
    local all_ok=true

    for container in "${containers[@]}"; do
        if docker ps -q -f name="$container" >/dev/null 2>&1 && \
           docker ps -f name="$container" --format "{{.Status}}" | grep -q "Up"; then
            echo -e "${GREEN}✅ $container 容器運行中${NC}"
            log_result "SUCCESS" "$container container is running"
        else
            echo -e "${RED}❌ $container 容器未運行${NC}"
            log_result "ERROR" "$container container not running"
            all_ok=false
        fi
    done

    $all_ok
}

check_network() {
    local endpoints=("192.168.100.100:7447" "192.168.100.100:8000")
    local all_ok=true

    for endpoint in "${endpoints[@]}"; do
        local host=$(echo $endpoint | cut -d: -f1)
        local port=$(echo $endpoint | cut -d: -f2)

        if timeout 3 bash -c "echo > /dev/tcp/$host/$port" 2>/dev/null; then
            echo -e "${GREEN}✅ $endpoint 網路可達${NC}"
            log_result "SUCCESS" "$endpoint network reachable"
        else
            echo -e "${RED}❌ $endpoint 網路不通${NC}"
            log_result "ERROR" "$endpoint network unreachable"
            all_ok=false
        fi
    done

    $all_ok
}

check_api_health() {
    local api_url="http://192.168.100.100:8000/health"
    local http_code

    http_code=$(timeout 5 curl -s -o /dev/null -w "%{http_code}" "$api_url" 2>/dev/null)

    if [ "$http_code" = "200" ]; then
        echo -e "${GREEN}✅ Web API 健康檢查通過${NC}"
        log_result "SUCCESS" "Web API health check passed"
        return 0
    else
        echo -e "${RED}❌ Web API 健康檢查失敗 (HTTP: $http_code)${NC}"
        log_result "ERROR" "Web API health check failed (HTTP: $http_code)"
        return 1
    fi
}

# 主檢查流程
main() {
    echo "🏥 RosAGV 系統健康檢查開始"
    echo "時間: $(date)"
    echo "=========================================="

    local overall_status=0

    check_docker || overall_status=1
    check_containers || overall_status=1
    check_network || overall_status=1
    check_api_health || overall_status=1

    echo "=========================================="
    if [ $overall_status -eq 0 ]; then
        echo -e "${GREEN}🎉 系統整體狀態正常${NC}"
        log_result "SUCCESS" "Overall system health check passed"
    else
        echo -e "${RED}⚠️  系統存在問題，請檢查上述錯誤${NC}"
        log_result "WARNING" "System health check detected issues"
    fi

    echo "詳細日誌: $HEALTH_LOG"
    return $overall_status
}

main "$@"
```

### 4.2 配置驗證模式

#### Docker Compose 配置驗證
```bash
#!/bin/bash
# 配置檔案驗證腳本

validate_compose_config() {
    local compose_file=$1
    local service_name=$2

    echo "驗證 $compose_file..."

    # 語法檢查
    if docker compose -f "$compose_file" config >/dev/null 2>&1; then
        echo "✅ $compose_file 語法正確"
    else
        echo "❌ $compose_file 語法錯誤："
        docker compose -f "$compose_file" config
        return 1
    fi

    # 檢查必要的服務
    if docker compose -f "$compose_file" config | grep -q "$service_name:"; then
        echo "✅ 服務 '$service_name' 存在於配置中"
    else
        echo "❌ 服務 '$service_name' 不存在於配置中"
        return 1
    fi

    return 0
}

# RosAGV 配置驗證
validate_rosagv_configs() {
    local configs=(
        "docker-compose.yml:rosagv"
        "docker-compose.agvc.yml:agvc_server"
    )

    for config in "${configs[@]}"; do
        local file=$(echo $config | cut -d: -f1)
        local service=$(echo $config | cut -d: -f2)

        validate_compose_config "$file" "$service" || return 1
    done

    echo "🎉 所有配置檔案驗證通過"
    return 0
}

validate_rosagv_configs
```

### 4.3 日誌處理模式

#### 智能日誌分析
```bash
#!/bin/bash
# 日誌分析腳本

analyze_logs() {
    local container_name=$1
    local hours=${2:-1}  # 預設分析最近1小時

    echo "📊 分析 $container_name 最近 $hours 小時的日誌"
    echo "=============================="

    # 取得日誌
    local logs=$(docker logs --since="${hours}h" "$container_name" 2>&1)

    # 錯誤統計
    echo "🚨 錯誤統計："
    echo "$logs" | grep -i "error" | awk '{print $3}' | sort | uniq -c | sort -nr
    echo

    # 警告統計
    echo "⚠️  警告統計："
    echo "$logs" | grep -i "warning\|warn" | awk '{print $3}' | sort | uniq -c | sort -nr
    echo

    # 請求統計（針對 Web API）
    echo "📈 HTTP 請求統計："
    echo "$logs" | grep -E "GET|POST|PUT|DELETE" | awk '{print $6, $7}' | sort | uniq -c | sort -nr | head -10
    echo

    # 時間分布分析
    echo "⏰ 活動時間分布："
    echo "$logs" | awk '{print substr($1, 1, 13)}' | sort | uniq -c | tail -24
}

# 批量日誌分析
batch_analyze() {
    local containers=("agvc_server" "nginx" "postgres")

    for container in "${containers[@]}"; do
        if docker ps -q -f name="$container" >/dev/null 2>&1; then
            analyze_logs "$container" 2
            echo "==============================="
            echo
        fi
    done
}

batch_analyze
```

### 4.4 自動重啟模式

#### 智能服務重啟
```bash
#!/bin/bash
# 智能服務重啟腳本

restart_service_if_needed() {
    local service_name=$1
    local health_check_cmd=$2
    local restart_cmd=$3
    local max_retries=${4:-3}

    echo "🔍 檢查 $service_name 狀態..."

    for ((i=1; i<=max_retries; i++)); do
        if eval "$health_check_cmd" >/dev/null 2>&1; then
            echo "✅ $service_name 運行正常"
            return 0
        fi

        echo "❌ $service_name 異常，嘗試重啟 (第 $i/$max_retries 次)"
        eval "$restart_cmd"

        # 等待服務啟動
        sleep 10
    done

    echo "🚨 $service_name 重啟失敗，需要人工介入"
    return 1
}

# RosAGV 服務監控與自動重啟
monitor_rosagv_services() {
    # Web API 監控
    restart_service_if_needed \
        "Web API" \
        "timeout 5 curl -s http://192.168.100.100:8000/health" \
        "docker compose -f docker-compose.agvc.yml restart agvc_server" \
        3

    # PostgreSQL 監控
    restart_service_if_needed \
        "PostgreSQL" \
        "docker exec postgres pg_isready -U agvc" \
        "docker compose -f docker-compose.agvc.yml restart postgres" \
        2

    # Nginx 監控
    restart_service_if_needed \
        "Nginx" \
        "timeout 3 curl -s -o /dev/null http://localhost:80" \
        "docker compose -f docker-compose.agvc.yml restart nginx" \
        2
}

monitor_rosagv_services
```

---

## 5. 進階技巧與最佳實踐

### 5.1 錯誤處理與可靠性

#### Bash 錯誤處理最佳實踐
```bash
#!/bin/bash
# 錯誤處理最佳實踐

# 嚴格模式
set -euo pipefail  # 遇到錯誤立即退出，未定義變數報錯，管道錯誤報錯

# 錯誤陷阱
trap 'echo "錯誤發生在第 $LINENO 行"' ERR

# 清理陷阱
cleanup() {
    echo "執行清理工作..."
    # 清理臨時檔案
    rm -f /tmp/rosagv_temp_*
    # 關閉網路連接
    # 恢復系統狀態
}
trap cleanup EXIT

# 檢查必要條件
check_prerequisites() {
    local missing_tools=()

    for tool in docker curl jq; do
        if ! command -v "$tool" >/dev/null 2>&1; then
            missing_tools+=("$tool")
        fi
    done

    if [ ${#missing_tools[@]} -ne 0 ]; then
        echo "❌ 缺少必要工具: ${missing_tools[*]}"
        exit 1
    fi
}

# 重試機制
retry() {
    local max_attempts=$1
    local delay=$2
    shift 2
    local command=("$@")

    for ((i=1; i<=max_attempts; i++)); do
        if "${command[@]}"; then
            return 0
        fi

        if [ $i -lt $max_attempts ]; then
            echo "嘗試 $i/$max_attempts 失敗，${delay}秒後重試..."
            sleep "$delay"
        fi
    done

    echo "所有重試均失敗"
    return 1
}

# 使用範例
check_prerequisites
retry 3 5 curl -s http://192.168.100.100:8000/health
```

#### 超時控制
```bash
# 各種超時控制方式

# 1. timeout 命令（推薦）
timeout 10 long_running_command

# 2. 網路連接超時
timeout 3 bash -c "echo > /dev/tcp/192.168.100.100/7447"

# 3. curl 超時
curl --connect-timeout 5 --max-time 10 http://example.com

# 4. 自定義超時函數
with_timeout() {
    local timeout=$1
    shift

    timeout "$timeout" "$@"
    local exit_code=$?

    case $exit_code in
        124) echo "命令超時 (${timeout}秒)"; return 1 ;;
        0) return 0 ;;
        *) echo "命令失敗 (退出碼: $exit_code)"; return $exit_code ;;
    esac
}

# 使用範例
with_timeout 30 docker compose up -d
```

### 5.2 效能優化

#### 避免不必要的進程調用
```bash
# ❌ 效能差：多次調用外部命令
check_container_bad() {
    local container=$1
    if docker ps | grep "$container" >/dev/null; then
        if docker ps --format "{{.Status}}" -f name="$container" | grep "Up" >/dev/null; then
            return 0
        fi
    fi
    return 1
}

# ✅ 效能好：一次調用獲得所需資訊
check_container_good() {
    local container=$1
    local status=$(docker ps --format "{{.Names}}\t{{.Status}}" | grep "^$container\t")
    [[ $status == *"Up"* ]]
}

# 批量處理優化
# ❌ 效能差：逐一處理
for container in agvc_server postgres nginx; do
    docker inspect "$container" >/dev/null 2>&1 && echo "$container: OK"
done

# ✅ 效能好：批量處理
containers=(agvc_server postgres nginx)
docker inspect "${containers[@]}" 2>/dev/null | jq -r '.[].Name + ": OK"' 2>/dev/null || {
    # 個別檢查失敗的容器
    for container in "${containers[@]}"; do
        docker inspect "$container" >/dev/null 2>&1 && echo "$container: OK" || echo "$container: FAIL"
    done
}
```

#### 快取與記憶化
```bash
# 結果快取機制
CACHE_DIR="/tmp/rosagv_cache"
CACHE_TTL=300  # 5分鐘

get_cached_result() {
    local cache_key=$1
    local cache_file="$CACHE_DIR/$cache_key"

    # 檢查快取是否存在且未過期
    if [[ -f "$cache_file" && $(($(date +%s) - $(stat -c %Y "$cache_file"))) -lt $CACHE_TTL ]]; then
        cat "$cache_file"
        return 0
    fi

    return 1
}

set_cached_result() {
    local cache_key=$1
    local result=$2
    local cache_file="$CACHE_DIR/$cache_key"

    mkdir -p "$CACHE_DIR"
    echo "$result" > "$cache_file"
}

# 使用範例：快取容器狀態
get_container_status() {
    local container=$1
    local cache_key="container_status_$container"

    # 嘗試從快取獲取
    if result=$(get_cached_result "$cache_key"); then
        echo "$result"
        return 0
    fi

    # 獲取實際狀態並快取
    local status=$(docker ps --format "{{.Status}}" -f name="$container" 2>/dev/null || echo "NOT_FOUND")
    set_cached_result "$cache_key" "$status"
    echo "$status"
}
```

### 5.3 安全考量

#### 輸入驗證
```bash
# 安全的輸入驗證
validate_container_name() {
    local name=$1

    # 只允許字母數字和底線
    if [[ ! $name =~ ^[a-zA-Z0-9_]+$ ]]; then
        echo "❌ 無效的容器名稱: $name"
        return 1
    fi

    # 長度限制
    if [[ ${#name} -gt 50 ]]; then
        echo "❌ 容器名稱過長: $name"
        return 1
    fi

    return 0
}

validate_ip_address() {
    local ip=$1

    if [[ $ip =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
        # 進一步檢查每個八位元組
        IFS='.' read -ra ADDR <<< "$ip"
        for octet in "${ADDR[@]}"; do
            if [[ $octet -lt 0 || $octet -gt 255 ]]; then
                echo "❌ 無效的 IP 位址: $ip"
                return 1
            fi
        done
        return 0
    else
        echo "❌ IP 位址格式錯誤: $ip"
        return 1
    fi
}

# 安全的檔案操作
safe_file_operation() {
    local file_path=$1

    # 檢查路徑遍歷攻擊
    if [[ $file_path == *".."* ]]; then
        echo "❌ 不安全的檔案路徑: $file_path"
        return 1
    fi

    # 限制在允許的目錄內
    local allowed_dirs=("/tmp" "/var/log/rosagv" "/app/logs")
    local allowed=false

    for dir in "${allowed_dirs[@]}"; do
        if [[ $file_path == $dir/* ]]; then
            allowed=true
            break
        fi
    done

    if [[ $allowed == false ]]; then
        echo "❌ 檔案路徑不在允許範圍內: $file_path"
        return 1
    fi

    return 0
}
```

#### 權限最小化
```bash
# 以最小權限執行
run_with_minimal_privileges() {
    local user=${1:-nobody}
    local group=${2:-nogroup}
    shift 2

    if [[ $EUID -eq 0 ]]; then
        # 降級權限執行
        su -s /bin/bash -c "$*" "$user"
    else
        # 已經是非 root 用戶
        "$@"
    fi
}

# 檢查檔案權限
check_file_permissions() {
    local file=$1
    local expected_perms=$2

    local actual_perms=$(stat -c "%a" "$file" 2>/dev/null)

    if [[ $actual_perms != $expected_perms ]]; then
        echo "⚠️  檔案權限不正確: $file (實際: $actual_perms, 期望: $expected_perms)"
        return 1
    fi

    return 0
}
```

### 5.4 調試技巧

#### 調試模式
```bash
#!/bin/bash
# 調試技巧

# 調試開關
DEBUG=${DEBUG:-false}
VERBOSE=${VERBOSE:-false}

debug_log() {
    if [[ $DEBUG == true ]]; then
        echo "[DEBUG] $*" >&2
    fi
}

verbose_log() {
    if [[ $VERBOSE == true ]]; then
        echo "[VERBOSE] $*" >&2
    fi
}

# 函數執行追踪
trace_function() {
    local func_name=$1
    shift

    debug_log "進入函數: $func_name, 參數: $*"
    local start_time=$(date +%s.%N)

    # 執行函數
    "$func_name" "$@"
    local exit_code=$?

    local end_time=$(date +%s.%N)
    local duration=$(echo "$end_time - $start_time" | bc -l)

    debug_log "離開函數: $func_name, 退出碼: $exit_code, 執行時間: ${duration}s"
    return $exit_code
}

# 變數監控
monitor_variable() {
    local var_name=$1
    local var_value=${!var_name}
    debug_log "變數 $var_name = '$var_value'"
}

# 使用範例
DEBUG=true ./script.sh  # 啟用調試模式
```

#### 效能分析
```bash
# 腳本執行時間分析
profile_script() {
    local script=$1
    shift

    echo "開始執行效能分析: $script"
    time -p bash "$script" "$@"
}

# 函數執行時間測量
time_function() {
    local func_name=$1
    shift

    local start=$(date +%s.%N)
    "$func_name" "$@"
    local exit_code=$?
    local end=$(date +%s.%N)

    local duration=$(echo "$end - $start" | bc -l)
    echo "函數 $func_name 執行時間: ${duration}s"

    return $exit_code
}

# 記憶體使用監控
monitor_memory() {
    local pid=${1:-$$}

    while kill -0 "$pid" 2>/dev/null; do
        local mem=$(ps -p "$pid" -o rss= 2>/dev/null)
        if [[ -n $mem ]]; then
            echo "[$(date '+%H:%M:%S')] PID $pid 記憶體使用: $((mem/1024)) MB"
        fi
        sleep 1
    done
}
```

---

## 6. 快速參考卡

### 6.1 重定向符號

```bash
# 標準重定向
>         # 覆蓋寫入 stdout
>>        # 附加寫入 stdout
2>        # 覆蓋寫入 stderr
2>>       # 附加寫入 stderr
&>        # stdout + stderr 到檔案
2>&1      # stderr 重定向到 stdout

# 常用組合
> file 2>&1      # 等同於 &> file
> /dev/null 2>&1 # 完全靜默
2> /dev/null     # 只隱藏錯誤
```

### 6.2 管道與條件

```bash
# 管道
|         # stdout 管道
|&        # stdout + stderr 管道

# 條件執行
&&        # 前命令成功才執行
||        # 前命令失敗才執行
;         # 順序執行

# 組合範例
cmd1 && cmd2 || cmd3    # cmd1成功執行cmd2，否則執行cmd3
cmd1; cmd2; cmd3        # 依序執行所有命令
```

### 6.3 RosAGV 常用命令

```bash
# 容器管理
docker compose -f docker-compose.agvc.yml up -d
docker compose -f docker-compose.agvc.yml ps
docker compose -f docker-compose.agvc.yml logs -f service_name
docker compose -f docker-compose.agvc.yml exec service_name bash

# 健康檢查
curl -s -o /dev/null -w "%{http_code}" http://192.168.100.100:8000/health
timeout 3 bash -c "echo > /dev/tcp/192.168.100.100/7447"
docker exec postgres pg_isready -U agvc

# 系統監控
ss -tulpn | grep ":8000"
ps aux | grep agvc
docker ps --format "table {{.Names}}\t{{.Status}}"
```

### 6.4 文本處理

```bash
# grep 常用參數
-i        # 忽略大小寫
-v        # 反向匹配
-q        # 靜默模式
-n        # 顯示行號
-l        # 只顯示檔案名

# awk 常用模式 (Aho, Weinberger, Kernighan)
awk '{print $2}'           # 印出第二欄
awk 'NR>1 {print}'         # 跳過第一行
awk '/pattern/ {print}'    # 匹配模式
awk '{count++} END {print count}'  # 計數

# sed 常用操作 (Stream EDitor)
sed 's/old/new/g'          # 全域替換
sed -n '10,20p'            # 印出第10-20行
sed '/pattern/d'           # 刪除匹配行
```

---

## 7. 常見問題與解決方案

### 7.1 管道相關問題

#### Q: 為什麼管道中的錯誤訊息沒有被處理？
```bash
# 問題：stderr 不會透過管道傳遞，只有 stdout 會傳遞
command1 | command2    # ❌ 錯誤方式：錯誤訊息不會傳遞給 command2

# 解決：明確將 stderr 重定向到 stdout
command1 2>&1 | command2    # ✅ 正確方式：錯誤訊息也會傳遞給 command2

# 或者分開處理
command1 > output.log 2> error.log
```

#### Q: 如何在管道中獲得中間命令的退出狀態？
```bash
# 問題：只能獲得最後一個命令的退出狀態
command1 | command2 | command3
echo $?  # 只能獲得 command3 的退出狀態

# 解決：使用 PIPESTATUS 陣列
command1 | command2 | command3
echo "退出狀態: ${PIPESTATUS[0]} ${PIPESTATUS[1]} ${PIPESTATUS[2]}"

# 或者使用 set -o pipefail
set -o pipefail
command1 | command2 | command3  # 任一命令失敗，整個管道失敗
```

### 7.2 檔案描述符問題

#### Q: 如何確保檔案描述符不洩露？
```bash
# 問題：開啟的檔案描述符沒有關閉
exec 3< input_file
# ... 使用檔案描述符 3
# 忘記關閉

# 解決：使用陷阱確保清理
cleanup() {
    exec 3<&-  # 關閉檔案描述符 3
}
trap cleanup EXIT

exec 3< input_file
# ... 使用檔案描述符 3
```

#### Q: 如何在子程序中保持檔案描述符？
```bash
# 問題：子程序中檔案描述符可能被關閉
(command) 3< input_file  # 子程序可能無法訪問 fd 3

# 解決：在子程序中明確開啟
{
    exec 3< input_file
    command
    exec 3<&-
}
```

### 7.3 重定向問題

#### Q: 如何同時將輸出保存到檔案和顯示在終端？
```bash
# 解決：使用 tee
command | tee output.log          # stdout 同時到螢幕和檔案
command 2>&1 | tee output.log     # stdout+stderr 同時到螢幕和檔案
command |& tee output.log         # 簡化寫法（Bash 4.0+）
```

#### Q: 如何避免重定向覆蓋重要檔案？
```bash
# 問題：意外覆蓋檔案
echo "new content" > important_file.txt

# 解決：使用 noclobber 選項
set -o noclobber
echo "new content" > important_file.txt  # 如果檔案存在會失敗

# 強制覆蓋
echo "new content" >| important_file.txt

# 或者總是使用附加模式
echo "new content" >> important_file.txt
```

### 7.4 容器相關問題

#### Q: 容器內外命令執行的差異？
```bash
# 問題：忘記在容器內執行 ROS2 命令
ros2 topic list  # 在宿主機執行會失敗

# 解決：明確在容器內執行
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "
  source /app/setup.bash &&
  ros2 topic list
"

# 或者進入容器後執行
docker compose -f docker-compose.agvc.yml exec agvc_server bash
# 在容器內：
source /app/setup.bash
ros2 topic list
```

#### Q: 如何處理容器重啟時的數據持久化？
```bash
# 問題：容器重啟後數據丟失
docker compose restart agvc_server

# 解決：使用卷掛載
# 在 docker-compose.yml 中：
# volumes:
#   - ./app/data:/app/data
#   - ./app/logs:/app/logs

# 檢查卷掛載
docker compose -f docker-compose.agvc.yml config | grep -A 5 volumes
```

### 7.5 網路連接問題

#### Q: 如何診斷網路連接問題？
```bash
# 分層診斷網路問題

# 1. 檢查容器是否運行
docker ps -f name=agvc_server

# 2. 檢查端口監聽
docker compose -f docker-compose.agvc.yml exec agvc_server ss -tulpn | grep ":8000"

# 3. 檢查容器網路
docker network ls
docker network inspect rosagv_agvc_network

# 4. 測試內部連接
docker compose -f docker-compose.agvc.yml exec agvc_server curl -s http://localhost:8000/health

# 5. 測試外部連接
curl -s http://192.168.100.100:8000/health

# 6. 檢查防火牆
sudo iptables -L | grep 8000
```

#### Q: 如何處理端口衝突？
```bash
# 診斷端口衝突
ss -tulpn | grep ":8000"

# 找出佔用端口的程序
sudo lsof -i :8000

# 如果是其他 Docker 容器
docker ps --format "table {{.Names}}\t{{.Ports}}" | grep 8000

# 優雅停止衝突的服務
sudo systemctl stop conflicting_service

# 或者修改配置使用不同端口
```

---

## 🏛️ 工具歷史背景

### Unix 文本處理工具的傳承
- **sed (1973-1974)**: Stream Editor，由 Lee E. McMahon 開發，基於 `ed` 編輯器
- **awk (1977)**: 由 Aho、Weinberger、Kernighan 三人合作開發，名字取自三人姓氏首字母
- **grep (1974)**: Global Regular Expression Print，Ken Thompson 開發

這些工具都來自 AT&T 貝爾實驗室，體現了 Unix 哲學：「做好一件事，並且做到極致」。

---

## 📚 延伸學習資源

### RosAGV 相關文檔
- [ROS2 整合指南](./ros2-integration.md)

### Bash 進階學習
- [Bash 手冊](https://www.gnu.org/software/bash/manual/)
- [ShellCheck](https://www.shellcheck.net/) - Bash 腳本靜態分析工具
- [Bash 最佳實踐](https://mywiki.wooledge.org/BashFAQ)

---

**📝 最後更新：2025-09-25**
**🔄 版本：1.0**
**👨‍💻 維護者：RosAGV 開發團隊**