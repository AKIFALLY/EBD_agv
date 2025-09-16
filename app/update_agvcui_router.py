#!/usr/bin/env python3
"""
Script to update agvcui to use the direct integration router
"""

import os
import shutil
from datetime import datetime

def update_agvcui_server():
    """Update agvc_ui_server.py to use direct integration"""
    
    server_file = "/app/web_api_ws/src/agvcui/agvcui/agvc_ui_server.py"
    backup_file = f"{server_file}.backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    
    print("=" * 60)
    print("🔧 Updating AGVCUI to use Direct Integration")
    print("=" * 60)
    
    # Check if file exists
    if not os.path.exists(server_file):
        print(f"❌ File not found: {server_file}")
        return False
    
    # Create backup
    print(f"📦 Creating backup: {backup_file}")
    shutil.copy(server_file, backup_file)
    
    # Read the file
    with open(server_file, 'r') as f:
        content = f.read()
    
    # Check if already updated
    if 'tafl_editor_direct' in content:
        print("✅ Already using direct integration router")
        return True
    
    # Update import statement
    print("📝 Updating import statement...")
    old_import = "from agvcui.routers import linear_flow_designer, nodes, tafl_editor"
    new_import = "from agvcui.routers import linear_flow_designer, nodes, tafl_editor_direct"
    
    if old_import in content:
        content = content.replace(old_import, new_import)
        print("   ✅ Import updated")
    else:
        print("   ⚠️ Import line not found exactly, trying alternative...")
        # Try to replace just the tafl_editor import
        content = content.replace("tafl_editor", "tafl_editor_direct")
        print("   ✅ Replaced all tafl_editor references")
    
    # Update router registration
    print("📝 Updating router registration...")
    old_router = "self.app.include_router(tafl_editor.get_router(self.templates))"
    new_router = "self.app.include_router(tafl_editor_direct.router)"
    
    if old_router in content:
        content = content.replace(old_router, new_router)
        print("   ✅ Router registration updated")
    else:
        # Try alternative patterns
        if "tafl_editor_direct.get_router" in content:
            content = content.replace(
                "tafl_editor_direct.get_router(self.templates)",
                "tafl_editor_direct.router"
            )
            print("   ✅ Router registration updated (alternative)")
    
    # Write the updated content
    print("💾 Saving updated file...")
    with open(server_file, 'w') as f:
        f.write(content)
    
    print("✅ File updated successfully")
    
    # Verify the changes
    print("\n📋 Verification:")
    with open(server_file, 'r') as f:
        lines = f.readlines()
        for i, line in enumerate(lines):
            if 'tafl_editor_direct' in line:
                print(f"   Line {i+1}: {line.strip()}")
    
    return True

def main():
    """Main function"""
    
    # Check if in container
    if not os.path.exists("/app/setup.bash"):
        print("❌ This script must be run inside the AGVC container")
        return 1
    
    # Update the server file
    if update_agvcui_server():
        print("\n" + "=" * 60)
        print("✅ Update Complete!")
        print("=" * 60)
        print("\n📋 Next Steps:")
        print("1. Restart AGVCUI service:")
        print("   manage_web_api_launch restart")
        print("   OR")
        print("   cd /app/web_api_ws")
        print("   python3 src/agvcui/agvcui/agvc_ui_server.py")
        print("\n2. Test the integration:")
        print("   curl http://localhost:8001/tafl/status")
        print("\n3. Open TAFL Editor:")
        print("   http://localhost:8001/tafl/editor")
        return 0
    else:
        print("\n❌ Update failed")
        return 1

if __name__ == "__main__":
    exit(main())