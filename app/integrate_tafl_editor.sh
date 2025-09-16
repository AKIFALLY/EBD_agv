#!/bin/bash
# Script to integrate direct TAFL execution into agvcui

echo "=========================================="
echo "🚀 TAFL Editor Direct Integration Script"
echo "=========================================="

# Check if we're in container
if [ ! -f /app/setup.bash ]; then
    echo "❌ This script must be run inside the AGVC container"
    echo "   Run: docker compose -f docker-compose.agvc.yml exec agvc_server bash"
    exit 1
fi

echo "✅ Running in AGVC container"

# Backup original TAFL editor
echo "📦 Backing up original TAFL editor..."
if [ -f /app/web_api_ws/src/agvcui/agvcui/routers/tafl_editor.py ]; then
    cp /app/web_api_ws/src/agvcui/agvcui/routers/tafl_editor.py \
       /app/web_api_ws/src/agvcui/agvcui/routers/tafl_editor.py.backup
    echo "   Backup saved to tafl_editor.py.backup"
fi

# Copy new direct integration module
echo "📝 Installing direct integration module..."
cp /app/tafl_editor_direct_integration.py \
   /app/web_api_ws/src/agvcui/agvcui/routers/tafl_editor_direct.py

# Create a simple test to verify imports
echo "🧪 Testing module imports..."
python3 -c "
import sys
sys.path.insert(0, '/app/tafl_wcs_ws/src/tafl_wcs')
sys.path.insert(0, '/app/tafl_ws/src/tafl')
try:
    from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
    from tafl_wcs.tafl_db_bridge import TAFLDatabaseBridge
    print('✅ Enhanced modules can be imported')
except ImportError as e:
    print(f'⚠️ Enhanced modules not available: {e}')
    print('   You may need to build tafl_wcs first:')
    print('   cd /app/tafl_wcs_ws && colcon build --packages-select tafl_wcs')
"

echo ""
echo "=========================================="
echo "📋 Next Steps:"
echo "=========================================="
echo ""
echo "1. Update agvc_ui_server.py to use the new router:"
echo "   Edit: /app/web_api_ws/src/agvcui/agvcui/agvc_ui_server.py"
echo ""
echo "   Replace:"
echo "   from .routers import tafl_editor"
echo ""
echo "   With:"
echo "   from .routers import tafl_editor_direct"
echo ""
echo "   And update router registration:"
echo "   app.include_router(tafl_editor_direct.router)"
echo ""
echo "2. Restart AGVCUI service:"
echo "   cd /app/web_api_ws"
echo "   python3 src/agvcui/agvcui/agvc_ui_server.py"
echo ""
echo "3. Test the integration:"
echo "   curl http://localhost:8001/tafl/status"
echo ""
echo "=========================================="
echo "✨ Integration script completed!"
echo "=========================================="