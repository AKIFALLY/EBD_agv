#!/bin/bash

# RosAGV Docker 配置驗證腳本
# 用於驗證文檔中的所有 Docker 指令都可實際執行

set -e

echo "=== RosAGV Docker 配置驗證 ==="
echo "驗證時間: $(date)"
echo "執行目錄: $(pwd)"
echo ""

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 檢查函數
check_success() {
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ $1${NC}"
    else
        echo -e "${RED}❌ $1${NC}"
        return 1
    fi
}

check_warning() {
    echo -e "${YELLOW}⚠️ $1${NC}"
}

# 1. 檢查必要檔案
echo "1. 檢查必要檔案..."
required_files=(
    "docker-compose.yml"
    "docker-compose.agvc.yml"
    "Dockerfile"
    "Dockerfile.agvc"
    "app/startup.agv.bash"
    "app/startup.agvc.bash"
)

for file in "${required_files[@]}"; do
    if [ -f "$file" ]; then
        check_success "$file 存在"
    else
        check_warning "$file 缺失"
    fi
done

echo ""

# 2. 檢查 Docker 環境
echo "2. 檢查 Docker 環境..."

# 檢查 Docker 是否安裝
if command -v docker &> /dev/null; then
    check_success "Docker 已安裝"
    docker --version
else
    echo -e "${RED}❌ Docker 未安裝${NC}"
    exit 1
fi

# 檢查 Docker Compose 是否安裝
if docker compose version &> /dev/null; then
    check_success "Docker Compose 已安裝"
    docker compose version
else
    echo -e "${RED}❌ Docker Compose 未安裝${NC}"
    exit 1
fi

# 檢查 Docker 服務狀態
if systemctl is-active --quiet docker; then
    check_success "Docker 服務運行中"
else
    check_warning "Docker 服務未運行"
fi

echo ""

# 3. 驗證 Docker Compose 檔案語法
echo "3. 驗證 Docker Compose 檔案語法..."

# 驗證 AGV 配置檔案
if docker compose -f docker-compose.yml config &> /dev/null; then
    check_success "docker-compose.yml 語法正確"
else
    echo -e "${RED}❌ docker-compose.yml 語法錯誤${NC}"
    docker compose -f docker-compose.yml config
fi

# 驗證 AGVC 配置檔案
if docker compose -f docker-compose.agvc.yml config &> /dev/null; then
    check_success "docker-compose.agvc.yml 語法正確"
else
    echo -e "${RED}❌ docker-compose.agvc.yml 語法錯誤${NC}"
    docker compose -f docker-compose.agvc.yml config
fi

echo ""

# 4. 檢查端口可用性
echo "4. 檢查端口可用性..."
ports_to_check=(80 2200 5432 5050 7447 8000 8001 8002 5173)

for port in "${ports_to_check[@]}"; do
    if netstat -tulpn 2>/dev/null | grep ":$port " > /dev/null; then
        check_warning "端口 $port 已被使用"
        # 顯示佔用端口的程序
        netstat -tulpn 2>/dev/null | grep ":$port " | head -1
    else
        check_success "端口 $port 可用"
    fi
done

echo ""

# 5. 檢查 Docker 映像
echo "5. 檢查 Docker 映像..."

# 檢查是否存在本地映像
images_to_check=(
    "yazelin/agv:latest"
    "yazelin/agvc:latest"
    "postgres:latest"
    "dpage/pgadmin4:latest"
    "nginx:latest"
)

for image in "${images_to_check[@]}"; do
    if docker images | grep -q "${image%:*}"; then
        check_success "$image 本地映像存在"
    else
        check_warning "$image 本地映像不存在，將在啟動時拉取"
    fi
done

echo ""

# 6. 檢查網路配置
echo "6. 檢查網路配置..."

# 檢查是否存在自訂網路
if docker network ls | grep -q "bridge_network"; then
    check_success "bridge_network 網路存在"
else
    check_warning "bridge_network 網路不存在，將在啟動時建立"
fi

# 檢查 IP 範圍衝突
if ip route | grep -q "192.168.100.0/24"; then
    check_warning "192.168.100.0/24 網段可能與現有路由衝突"
    ip route | grep "192.168.100.0/24"
else
    check_success "192.168.100.0/24 網段可用"
fi

echo ""

# 7. 檢查資料卷
echo "7. 檢查 Docker 資料卷..."

volumes_to_check=(
    "postgres_data"
    "pgadmin_data"
)

for volume in "${volumes_to_check[@]}"; do
    if docker volume ls | grep -q "$volume"; then
        check_success "$volume 資料卷存在"
    else
        check_warning "$volume 資料卷不存在，將在啟動時建立"
    fi
done

echo ""

# 8. 檢查目錄掛載
echo "8. 檢查目錄掛載..."

mount_dirs=(
    "~/RosAGV/app"
    "~/RosAGV/nginx"
)

for dir in "${mount_dirs[@]}"; do
    expanded_dir="${dir/#\~/$HOME}"
    if [ -d "$expanded_dir" ]; then
        check_success "$dir 目錄存在"
    else
        check_warning "$dir 目錄不存在"
    fi
done

echo ""

# 9. 檢查特殊裝置
echo "9. 檢查特殊裝置..."

# 檢查輸入裝置目錄
if [ -d "/dev/input" ]; then
    check_success "/dev/input 目錄存在"
    input_devices=$(ls /dev/input/ 2>/dev/null | wc -l)
    echo "   發現 $input_devices 個輸入裝置"
else
    check_warning "/dev/input 目錄不存在"
fi

# 檢查 X11 支援
if [ -d "/tmp/.X11-unix" ]; then
    check_success "/tmp/.X11-unix 目錄存在"
else
    check_warning "/tmp/.X11-unix 目錄不存在，GUI 應用程式可能無法顯示"
fi

echo ""

# 10. 測試基本 Docker 指令
echo "10. 測試基本 Docker 指令..."

# 測試 Docker Compose 指令 (不實際啟動)
if docker compose -f docker-compose.yml ps &> /dev/null; then
    check_success "AGV Docker Compose 指令可執行"
else
    check_warning "AGV Docker Compose 指令執行失敗"
fi

if docker compose -f docker-compose.agvc.yml ps &> /dev/null; then
    check_success "AGVC Docker Compose 指令可執行"
else
    check_warning "AGVC Docker Compose 指令執行失敗"
fi

echo ""

# 總結
echo "=== 驗證總結 ==="
echo "驗證完成時間: $(date)"

# 檢查是否有容器正在運行
running_containers=$(docker ps --format "table {{.Names}}\t{{.Status}}" | grep -E "(rosagv|agvc_server|postgres|pgadmin|nginx)" || true)

if [ -n "$running_containers" ]; then
    echo ""
    echo "當前運行的 RosAGV 相關容器:"
    echo "$running_containers"
else
    echo ""
    echo "目前沒有 RosAGV 相關容器在運行"
fi

echo ""
echo "建議執行以下指令來啟動系統:"
echo "1. 啟動 AGVC 管理系統: docker compose -f docker-compose.agvc.yml up -d"
echo "2. 啟動 AGV 車載系統: docker compose -f docker-compose.yml up -d"
echo "3. 檢查系統狀態: ./scripts/health-check.sh"

echo ""
echo "=== 驗證完成 ==="
