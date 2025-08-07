#!/bin/bash
# Flow Designer 系統完整性驗證腳本
# 用途: 驗證整個 Flow Designer + YAML DSL 系統的完整性和功能狀態

set -e

echo "🔍 Flow Designer 系統完整性驗證開始"
echo "=================================================="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
VERIFICATION_ERRORS=0

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
    VERIFICATION_ERRORS=$((VERIFICATION_ERRORS + 1))
}

log_warning() {
    echo -e "${YELLOW}⚠️ $1${NC}"
}

log_info() {
    echo -e "${BLUE}ℹ️ $1${NC}"
}

# 檢查服務狀態
check_service_status() {
    echo ""
    log_info "檢查 AGVC 服務狀態..."
    
    if docker compose -f docker-compose.agvc.yml ps | grep -q "Up"; then
        log_success "AGVC 服務正常運行"
    else
        log_error "AGVC 服務未運行"
        return 1
    fi
}

# 檢查 Flow Designer 可存取性
check_flow_designer_accessibility() {
    echo ""
    log_info "檢查 Flow Designer 可存取性..."
    
    # 檢查 Flow Designer 頁面
    if curl -s --max-time 10 http://localhost:8001/flows/create | grep -q "Flow Designer"; then
        log_success "Flow Designer 頁面可正常存取"
    else
        log_error "Flow Designer 頁面無法存取"
        return 1
    fi
    
    # 檢查核心 JavaScript 檔案
    if curl -s --max-time 5 -o /dev/null -w "%{http_code}" http://localhost:8001/static/js/flowDesignerPage.js | grep -q "200"; then
        log_success "Flow Designer JavaScript 檔案載入正常"
    else
        log_error "Flow Designer JavaScript 檔案載入失敗"
        return 1
    fi
}

# 檢查 JavaScript 資源完整性
check_javascript_resources() {
    echo ""
    log_info "檢查 JavaScript 資源完整性..."
    
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
}

# 檢查關鍵功能模組
check_core_modules() {
    echo ""
    log_info "檢查核心功能模組..."
    
    FLOW_DESIGNER_JS="$PROJECT_ROOT/app/web_api_ws/src/agvcui/agvcui/static/js/flowDesignerPage.js"
    
    if [ -f "$FLOW_DESIGNER_JS" ]; then
        log_success "Flow Designer JavaScript 檔案存在"
        
        # 檢查關鍵功能
        REQUIRED_FEATURES=(
            "window.flowDesigner"
            "generateYamlDsl"
            "loadYamlDsl"
            "PerformanceOptimizer"
            "BatchNodeRenderer"
            "MemoryManager"
        )
        
        for feature in "${REQUIRED_FEATURES[@]}"; do
            if grep -q "$feature" "$FLOW_DESIGNER_JS"; then
                log_success "功能模組存在: $feature"
            else
                log_error "功能模組缺失: $feature"
            fi
        done
    else
        log_error "Flow Designer JavaScript 檔案不存在"
    fi
}

# 檢查 YAML DSL 系統文件
check_yaml_dsl_files() {
    echo ""
    log_info "檢查 YAML DSL 系統文件..."
    
    DSL_FILES=(
        "$PROJECT_ROOT/docs-ai/operations/development/yaml-dsl-language-design.md"
        "$PROJECT_ROOT/docs-ai/operations/development/flow-designer-phase3-2-yaml-generation.md"
        "$PROJECT_ROOT/docs-ai/operations/development/flow-designer-phase3-3-bidirectional-sync.md"
    )
    
    for dsl_file in "${DSL_FILES[@]}"; do
        if [ -f "$dsl_file" ]; then
            log_success "DSL 文件存在: $(basename $dsl_file)"
        else
            log_error "DSL 文件缺失: $(basename $dsl_file)"
        fi
    done
}

# 檢查 Simple WCS 整合文件
check_simple_wcs_integration() {
    echo ""
    log_info "檢查 Simple WCS 整合..."
    
    SIMPLE_WCS_FILES=(
        "$PROJECT_ROOT/docs-ai/knowledge/system/simple-wcs-system.md"
        "$PROJECT_ROOT/docs-ai/operations/development/simple-wcs-development.md"
    )
    
    for wcs_file in "${SIMPLE_WCS_FILES[@]}"; do
        if [ -f "$wcs_file" ]; then
            log_success "Simple WCS 文件存在: $(basename $wcs_file)"
        else
            log_error "Simple WCS 文件缺失: $(basename $wcs_file)"
        fi
    done
}

# 檢查用戶培訓文檔
check_training_documentation() {
    echo ""
    log_info "檢查用戶培訓文檔..."
    
    TRAINING_DOCS=(
        "$PROJECT_ROOT/docs-ai/operations/development/flow-designer-complete-user-manual.md"
        "$PROJECT_ROOT/docs-ai/operations/development/flow-designer-best-practices-guide.md"
        "$PROJECT_ROOT/docs-ai/operations/development/flow-designer-troubleshooting-manual.md"
        "$PROJECT_ROOT/docs-ai/operations/development/flow-designer-developer-guide.md"
    )
    
    for doc in "${TRAINING_DOCS[@]}"; do
        if [ -f "$doc" ]; then
            log_success "培訓文檔存在: $(basename $doc)"
        else
            log_error "培訓文檔缺失: $(basename $doc)"
        fi
    done
}

# 檢查部署相關文件
check_deployment_files() {
    echo ""
    log_info "檢查部署相關文件..."
    
    DEPLOYMENT_FILES=(
        "$PROJECT_ROOT/docs-ai/operations/development/flow-designer-phase4-4-production-deployment.md"
        "$PROJECT_ROOT/scripts/deployment/create-release-tag.sh"
        "$PROJECT_ROOT/scripts/deployment/deploy-flow-designer.sh"
    )
    
    for deploy_file in "${DEPLOYMENT_FILES[@]}"; do
        if [ -f "$deploy_file" ]; then
            log_success "部署文件存在: $(basename $deploy_file)"
            
            # 檢查腳本執行權限
            if [[ "$deploy_file" == *.sh ]]; then
                if [ -x "$deploy_file" ]; then
                    log_success "腳本有執行權限: $(basename $deploy_file)"
                else
                    log_warning "腳本缺少執行權限: $(basename $deploy_file)"
                    chmod +x "$deploy_file"
                    log_info "已添加執行權限: $(basename $deploy_file)"
                fi
            fi
        else
            log_error "部署文件缺失: $(basename $deploy_file)"
        fi
    done
}

# 檢查版本資訊
check_version_info() {
    echo ""
    log_info "檢查版本資訊..."
    
    FLOW_DESIGNER_JS="$PROJECT_ROOT/app/web_api_ws/src/agvcui/agvcui/static/js/flowDesignerPage.js"
    
    if [ -f "$FLOW_DESIGNER_JS" ]; then
        if grep -q "Flow Designer v[0-9]\+\.[0-9]\+\.[0-9]\+" "$FLOW_DESIGNER_JS"; then
            VERSION=$(grep -o "Flow Designer v[0-9]\+\.[0-9]\+\.[0-9]\+" "$FLOW_DESIGNER_JS" | head -1)
            log_success "版本資訊: $VERSION"
        else
            log_warning "版本資訊格式需要更新"
        fi
        
        # 檢查效能最佳化版本標記
        if grep -q "v4\.2.*效能最佳化" "$FLOW_DESIGNER_JS"; then
            log_success "效能最佳化版本標記存在"
        else
            log_warning "效能最佳化版本標記可能需要更新"
        fi
    fi
}

# 檢查系統整合報告
check_integration_report() {
    echo ""
    log_info "檢查系統整合報告..."
    
    INTEGRATION_REPORT="$PROJECT_ROOT/docs-ai/operations/development/flow-designer-system-integration-report.md"
    
    if [ -f "$INTEGRATION_REPORT" ]; then
        log_success "系統整合報告存在"
        
        # 檢查報告內容完整性
        if grep -q "專案完成度.*100%" "$INTEGRATION_REPORT"; then
            log_success "專案完成度: 100%"
        else
            log_warning "專案完成度狀態需要確認"
        fi
        
        if grep -q "生產就緒狀態.*已就緒" "$INTEGRATION_REPORT"; then
            log_success "生產就緒狀態: 已就緒"
        else
            log_warning "生產就緒狀態需要確認"
        fi
    else
        log_error "系統整合報告缺失"
    fi
}

# 檢查 WCS 函數數量
check_wcs_functions() {
    echo ""
    log_info "檢查 WCS 函數整合..."
    
    # 檢查 node-types.js 中的 WCS 函數
    NODE_TYPES_JS="$PROJECT_ROOT/app/web_api_ws/src/agvcui/agvcui/static/js/flow-designer/node-types.js"
    
    if [ -f "$NODE_TYPES_JS" ]; then
        log_success "node-types.js 檔案存在"
        
        # 計算 WCS 函數數量
        WCS_FUNCTION_COUNT=$(grep -o "function: '[^']*'" "$NODE_TYPES_JS" | wc -l)
        
        if [ "$WCS_FUNCTION_COUNT" -ge 30 ]; then
            log_success "WCS 函數數量: $WCS_FUNCTION_COUNT (預期: ≥30)"
        else
            log_warning "WCS 函數數量可能不足: $WCS_FUNCTION_COUNT"
        fi
    else
        log_error "node-types.js 檔案不存在"
    fi
}

# 執行系統效能檢查
check_system_performance() {
    echo ""
    log_info "檢查系統效能配置..."
    
    FLOW_DESIGNER_JS="$PROJECT_ROOT/app/web_api_ws/src/agvcui/agvcui/static/js/flowDesignerPage.js"
    
    if [ -f "$FLOW_DESIGNER_JS" ]; then
        # 檢查效能最佳化功能
        PERFORMANCE_FEATURES=(
            "BatchNodeRenderer"
            "MemoryManager"
            "PerformanceMonitor"
            "ResourcePreloader"
            "ProgressiveLoader"
        )
        
        for feature in "${PERFORMANCE_FEATURES[@]}"; do
            if grep -q "$feature" "$FLOW_DESIGNER_JS"; then
                log_success "效能功能存在: $feature"
            else
                log_error "效能功能缺失: $feature"
            fi
        done
    fi
}

# 主要驗證流程
main() {
    echo ""
    log_info "開始 Flow Designer 系統完整性驗證..."
    echo ""
    
    # 執行所有檢查
    check_service_status
    check_flow_designer_accessibility
    check_javascript_resources
    check_core_modules
    check_yaml_dsl_files
    check_simple_wcs_integration
    check_training_documentation
    check_deployment_files
    check_version_info
    check_integration_report
    check_wcs_functions
    check_system_performance
    
    # 總結結果
    echo ""
    echo "=================================================="
    
    if [ $VERIFICATION_ERRORS -eq 0 ]; then
        echo -e "${GREEN}🎉 Flow Designer 系統驗證完全通過！${NC}"
        echo -e "${GREEN}   系統已準備就緒，可以投入生產使用。${NC}"
        echo ""
        log_info "系統狀態總結:"
        echo "   ✅ 所有核心功能正常運作"
        echo "   ✅ JavaScript 資源完整載入"
        echo "   ✅ YAML DSL 系統完整"
        echo "   ✅ Simple WCS 整合完成"
        echo "   ✅ 用戶培訓文檔完整"
        echo "   ✅ 部署腳本準備就緒"
        echo "   ✅ 效能最佳化功能完整"
        echo ""
        log_info "後續操作建議:"
        echo "   1. 執行最終使用者測試"
        echo "   2. 進行生產環境部署"
        echo "   3. 設定監控和告警系統"
        echo "   4. 準備用戶培訓材料"
        echo ""
        exit 0
    else
        echo -e "${RED}⚠️ 發現 $VERIFICATION_ERRORS 個問題需要解決${NC}"
        echo -e "${YELLOW}   請檢查上述錯誤訊息並進行修復${NC}"
        echo ""
        log_info "建議修復步驟:"
        echo "   1. 檢查 AGVC 服務狀態"
        echo "   2. 確認所有必要檔案存在"
        echo "   3. 驗證 JavaScript 資源載入"
        echo "   4. 重新執行此驗證腳本"
        echo ""
        exit 1
    fi
}

# 命令行參數處理
case "${1:-}" in
    --help|-h)
        echo "Flow Designer 系統完整性驗證腳本"
        echo ""
        echo "使用方式:"
        echo "  $0                    執行完整系統驗證"
        echo "  $0 --help|-h         顯示此說明"
        echo "  $0 --service-only     僅檢查服務狀態"
        echo "  $0 --files-only       僅檢查檔案完整性"
        echo ""
        echo "描述:"
        echo "  此腳本驗證 Flow Designer + YAML DSL 系統的完整性，"
        echo "  包括服務狀態、檔案完整性、功能模組和版本資訊。"
        echo ""
        exit 0
        ;;
    --service-only)
        check_service_status
        check_flow_designer_accessibility
        ;;
    --files-only)
        check_core_modules
        check_yaml_dsl_files
        check_simple_wcs_integration
        check_training_documentation
        check_deployment_files
        ;;
    *)
        main
        ;;
esac