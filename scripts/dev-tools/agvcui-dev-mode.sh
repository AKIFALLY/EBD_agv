#!/bin/bash
# AGVCUI Development Mode with Auto-reload
# This script starts AGVCUI with auto-reload enabled for development

echo "🔄 Starting AGVCUI in development mode with auto-reload..."
echo "📝 Changes to Python files will automatically restart the server"
echo "⚠️  Note: JavaScript/CSS changes still require browser refresh"

# Navigate to the workspace
cd /app/web_api_ws/src/agvcui

# Start with uvicorn auto-reload
uvicorn agvcui.agvc_ui_server:app \
    --host 0.0.0.0 \
    --port 8001 \
    --reload \
    --reload-dir agvcui \
    --reload-include "*.py" \
    --reload-include "*.html" \
    --reload-include "*.js" \
    --reload-include "*.css" \
    --log-level info

echo "✅ AGVCUI development server stopped"