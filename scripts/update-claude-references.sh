#!/bin/bash

# 批量更新 CLAUDE.md 文件中的 @docs-ai/ 引用
# 將舊的 legacy 引用路徑替換為新的三層架構路徑

echo "🔄 開始批量更新 CLAUDE.md 文件中的 @docs-ai/ 引用..."
echo "=================================================="

# 定義引用映射規則
declare -A reference_map=(
    # Universal 系列映射
    ["@docs-ai/universal/system/ros2-architecture.md"]="@docs-ai/context/system/technology-stack.md"
    ["@docs-ai/universal/architecture/dual-environment.md"]="@docs-ai/context/system/dual-environment.md"
    ["@docs-ai/universal/workflow/development-guide.md"]="@docs-ai/operations/development/ros2-development.md"
    
    # Projects 系列映射
    ["@docs-ai/projects/agv/core/state-machine-context.md"]="@docs-ai/context/workspaces/agv-workspaces.md"
    ["@docs-ai/projects/agv/packages/agv_base-implementation.md"]="@docs-ai/context/workspaces/agv-workspaces.md"
    ["@docs-ai/projects/agv/packages/loader_agv-implementation.md"]="@docs-ai/knowledge/agv-domain/vehicle-types.md"
    ["@docs-ai/projects/agv/packages/agv_interfaces-definition.md"]="@docs-ai/operations/development/ros2-development.md"
    ["@docs-ai/projects/agv/packages/cargo_unloader_agv-development.md"]="@docs-ai/knowledge/agv-domain/vehicle-types.md"
    ["@docs-ai/projects/agv/vehicles/cargo-mover-context.md"]="@docs-ai/knowledge/agv-domain/vehicle-types.md"
    ["@docs-ai/projects/agv/vehicles/loader-agv-context.md"]="@docs-ai/knowledge/agv-domain/vehicle-types.md"
    ["@docs-ai/projects/agv/vehicles/unloader-agv-context.md"]="@docs-ai/knowledge/agv-domain/vehicle-types.md"
    ["@docs-ai/projects/agv/sensors/joystick-integration.md"]="@docs-ai/operations/development/ros2-development.md"
    ["@docs-ai/projects/agv/sensors/hokuyo-integration.md"]="@docs-ai/operations/development/ros2-development.md"
    ["@docs-ai/projects/agvc/web/fastapi-development.md"]="@docs-ai/operations/development/web-development.md"
    ["@docs-ai/projects/agvc/database/postgresql-crud.md"]="@docs-ai/operations/development/database-operations.md"
    ["@docs-ai/projects/agvc/database/postgresql-development.md"]="@docs-ai/operations/development/database-operations.md"
    ["@docs-ai/projects/agvc/database/sqlmodel-orm.md"]="@docs-ai/operations/development/database-operations.md"
    ["@docs-ai/projects/agvc/control/ecs-wcs-rcs.md"]="@docs-ai/context/workspaces/agvc-workspaces.md"
    ["@docs-ai/projects/agvc/control/wcs-decision-engine.md"]="@docs-ai/context/workspaces/agvc-workspaces.md"
    ["@docs-ai/projects/shared/communication/plc-zenoh.md"]="@docs-ai/knowledge/protocols/zenoh-rmw.md"
    ["@docs-ai/projects/shared/algorithms/path-planning.md"]="@docs-ai/knowledge/agv-domain/navigation-systems.md"
    ["@docs-ai/projects/shared/interfaces/ros2-agv-interfaces.md"]="@docs-ai/knowledge/protocols/ros2-interfaces.md"
    
    # Tools 系列映射
    ["@docs-ai/tools/rosagv-unified-tools.md"]="@docs-ai/operations/maintenance/system-diagnostics.md"
    ["@docs-ai/tools/rosagv-tools-overview.md"]="@docs-ai/operations/maintenance/system-diagnostics.md"
    ["@docs-ai/tools/docker/container-management.md"]="@docs-ai/operations/development/docker-development.md"
    ["@docs-ai/tools/diagnostics/system-health-check.md"]="@docs-ai/operations/maintenance/system-diagnostics.md"
    ["@docs-ai/tools/logging/log-analysis.md"]="@docs-ai/operations/maintenance/log-analysis.md"
    ["@docs-ai/tools/network/connectivity-testing.md"]="@docs-ai/operations/maintenance/system-diagnostics.md"
    ["@docs-ai/tools/development/build-and-test.md"]="@docs-ai/operations/development/ros2-development.md"
    
    # Context Level 系列映射
    ["@docs-ai/context/level1/system-overview.md"]="@docs-ai/context/system/rosagv-overview.md"
    ["@docs-ai/context/level2/workspace-agv_ws.md"]="@docs-ai/context/workspaces/agv-workspaces.md"
    ["@docs-ai/context/level2/workspace-tafl_wcs_ws-design.md"]="@docs-ai/context/workspaces/agvc-workspaces.md"
    ["@docs-ai/context/level3/package-agv_base.md"]="@docs-ai/context/workspaces/agv-workspaces.md"
)

# 查找所有 CLAUDE.md 文件
echo "🔍 查找所有 CLAUDE.md 文件..."
CLAUDE_FILES=$(find . -name "CLAUDE.md" -type f)

if [ -z "$CLAUDE_FILES" ]; then
    echo "❌ 沒有找到任何 CLAUDE.md 文件"
    exit 1
fi

echo "✅ 找到以下 CLAUDE.md 文件："
echo "$CLAUDE_FILES"
echo ""

# 統計變數
TOTAL_FILES=0
UPDATED_FILES=0
TOTAL_REPLACEMENTS=0

# 處理每個文件
for file in $CLAUDE_FILES; do
    echo "📝 處理文件: $file"
    TOTAL_FILES=$((TOTAL_FILES + 1))
    FILE_UPDATED=false
    FILE_REPLACEMENTS=0
    
    # 對每個映射規則進行替換
    for old_ref in "${!reference_map[@]}"; do
        new_ref="${reference_map[$old_ref]}"
        
        # 檢查文件中是否包含舊引用
        if rg -q "$old_ref" "$file" 2>/dev/null; then
            echo "  🔄 替換: $old_ref → $new_ref"
            
            # 使用 sed 進行替換
            sed -i "s|$old_ref|$new_ref|g" "$file"
            
            FILE_UPDATED=true
            FILE_REPLACEMENTS=$((FILE_REPLACEMENTS + 1))
            TOTAL_REPLACEMENTS=$((TOTAL_REPLACEMENTS + 1))
        fi
    done
    
    if [ "$FILE_UPDATED" = true ]; then
        echo "  ✅ 文件已更新，共 $FILE_REPLACEMENTS 個替換"
        UPDATED_FILES=$((UPDATED_FILES + 1))
    else
        echo "  ℹ️  文件無需更新"
    fi
    echo ""
done

echo "📊 更新統計："
echo "  總文件數: $TOTAL_FILES"
echo "  已更新文件數: $UPDATED_FILES"
echo "  總替換次數: $TOTAL_REPLACEMENTS"
echo ""

if [ $TOTAL_REPLACEMENTS -gt 0 ]; then
    echo "🎉 批量更新完成！"
    echo "建議執行以下命令驗證引用有效性："
    echo "  ./scripts/validate-prompts-references.sh"
else
    echo "ℹ️  沒有需要更新的引用"
fi

echo "=================================================="
