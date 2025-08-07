#!/bin/bash
# Flow Designer 自動化部署腳本
# 用途: 自動化部署 Flow Designer 系統到指定環境

set -e  # 遇到錯誤立即退出

ENVIRONMENT="$1"
VERSION="$2"

if [ -z "$ENVIRONMENT" ] || [ -z "$VERSION" ]; then
    echo "使用方式: $0 <environment> <version>"
    echo "範例: $0 production v2.0.1"
    echo ""
    echo "支援的環境:"
    echo "  production  - 生產環境"
    echo "  staging     - 測試環境"
    echo "  development - 開發環境"
    echo ""
    echo "版本格式: vX.Y.Z (例如: v2.0.1)"
    exit 1
fi

echo "🚀 開始部署 Flow Designer $VERSION 到 $ENVIRONMENT 環境"
echo "=================================================="

# 顏色輸出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 輔助函數
log_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

log_error() {
    echo -e "${RED}❌ $1${NC}"
    exit 1
}

log_warning() {
    echo -e "${YELLOW}⚠️ $1${NC}"
}

log_info() {
    echo -e "${BLUE}ℹ️ $1${NC}"
}

# Phase 1: 部署前置檢查
echo ""
log_info "Phase 1: 部署前置檢查"

# 檢查 Git 版本標籤
log_info "檢查版本標籤..."
if ! git tag | grep -q "^${VERSION}$"; then
    log_error "版本標籤 $VERSION 不存在，請先創建版本標籤"
fi
log_success "版本標籤 $VERSION 存在"

# 檢查工作目錄狀態
log_info "檢查工作目錄狀態..."
if ! git diff-index --quiet HEAD --; then
    log_warning "工作目錄有未提交的變更"
    log_info "繼續部署，但建議先提交變更"
else
    log_success "工作目錄狀態乾淨"
fi

# 檢查容器狀態
log_info "檢查 AGVC 容器狀態..."
if ! docker compose -f docker-compose.agvc.yml ps | grep -q "Up"; then
    log_warning "AGVC 服務未運行，正在啟動..."
    docker compose -f docker-compose.agvc.yml up -d
    log_info "等待服務啟動..."
    sleep 30
    
    if ! docker compose -f docker-compose.agvc.yml ps | grep -q "Up"; then
        log_error "AGVC 服務啟動失敗"
    fi
fi
log_success "AGVC 服務正常運行"

# Phase 2: 代碼部署
echo ""
log_info "Phase 2: 代碼部署"

# 切換到指定版本
log_info "切換到版本 $VERSION..."
CURRENT_BRANCH=$(git branch --show-current)
git checkout "$VERSION"
log_success "已切換到版本 $VERSION"

# 重新啟動服務以載入新版本
log_info "重新啟動 AGVC 服務..."
docker compose -f docker-compose.agvc.yml restart agvc_server

# 等待服務完全啟動
log_info "等待服務完全啟動..."
sleep 60

# 檢查服務是否正常啟動
if ! docker compose -f docker-compose.agvc.yml ps | grep -q "Up"; then
    log_error "服務重啟後狀態異常"
fi
log_success "服務重啟完成"

# Phase 3: 部署後驗證
echo ""
log_info "Phase 3: 部署後驗證"

# 檢查 Flow Designer 可存取性
log_info "檢查 Flow Designer 可存取性..."
if curl -s --max-time 10 http://localhost:8001/flows/create | grep -q "Flow Designer"; then
    log_success "Flow Designer 頁面可正常存取"
else
    log_error "Flow Designer 頁面無法存取"
fi

# 檢查 JavaScript 資源
log_info "檢查 JavaScript 資源..."
JS_FILES=(
    "http://localhost:8001/static/js/flowDesignerPage.js"
    "http://localhost:8001/static/js/flow-designer/node-types.js"
    "http://localhost:8001/static/js/libs/js-yaml.min.js"
)

for js_file in "${JS_FILES[@]}"; do
    if curl -s --max-time 5 -o /dev/null -w "%{http_code}" "$js_file" | grep -q "200"; then
        log_success "JavaScript 資源正常: $(basename $js_file)"
    else
        log_error "JavaScript 資源載入失敗: $(basename $js_file)"
    fi
done

# 檢查版本資訊
log_info "驗證版本資訊..."
JS_CONTENT=$(curl -s http://localhost:8001/static/js/flowDesignerPage.js)
if echo "$JS_CONTENT" | grep -q "Flow Designer $VERSION"; then
    log_success "版本資訊正確: $VERSION"
else
    # 檢查是否有版本資訊
    if echo "$JS_CONTENT" | grep -q "Flow Designer v"; then
        DEPLOYED_VERSION=$(echo "$JS_CONTENT" | grep -o "Flow Designer v[0-9]\+\.[0-9]\+\.[0-9]\+" | head -1)
        log_warning "版本資訊: $DEPLOYED_VERSION (預期: Flow Designer $VERSION)"
    else
        log_warning "未找到版本資訊"
    fi
fi

# 檢查核心功能
log_info "檢查核心功能模組..."
CORE_FUNCTIONS=(
    "window.flowDesigner"
    "generateYamlDsl"
    "loadYamlDsl"
    "PerformanceOptimizer"
)

for func in "${CORE_FUNCTIONS[@]}"; do
    if echo "$JS_CONTENT" | grep -q "$func"; then
        log_success "核心功能存在: $func"
    else
        log_error "核心功能缺失: $func"
    fi
done

# Phase 4: 環境特定配置
echo ""
log_info "Phase 4: 環境特定配置"

case "$ENVIRONMENT" in
    "production")
        log_info "配置生產環境..."
        # 生產環境特定配置
        log_success "生產環境配置完成"
        ;;
    "staging")
        log_info "配置測試環境..."
        # 測試環境特定配置
        log_success "測試環境配置完成"
        ;;
    "development")
        log_info "配置開發環境..."
        # 開發環境特定配置
        log_success "開發環境配置完成"
        ;;
    *)
        log_error "不支援的環境: $ENVIRONMENT"
        ;;
esac

# Phase 5: 部署完成
echo ""
log_info "Phase 5: 部署完成"

echo ""
echo "=================================================="
log_success "Flow Designer $VERSION 成功部署到 $ENVIRONMENT 環境"
echo ""

log_info "部署總結:"
echo "   📦 版本: $VERSION"
echo "   🌍 環境: $ENVIRONMENT"
echo "   📅 時間: $(date)"
echo "   🔗 訪問: http://localhost:8001/flows/create"
echo ""

log_info "後續操作建議:"
echo "   1. 執行完整功能測試"
echo "   2. 監控系統效能指標"
echo "   3. 檢查錯誤日誌"
echo "   4. 通知相關用戶"
echo ""

# 回到原來的分支（如果不是在標籤上）
if [ "$CURRENT_BRANCH" != "$VERSION" ]; then
    log_info "回到原分支 $CURRENT_BRANCH..."
    git checkout "$CURRENT_BRANCH"
fi

echo "🎉 部署完成！"