#!/bin/bash
# 安裝 Git Hook 以自動同步 Flow Functions

ROSAGV_DIR="/home/ct/RosAGV"
HOOK_FILE="$ROSAGV_DIR/.git/hooks/pre-commit"

# 顏色定義
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}安裝 Flow Functions 自動同步 Git Hook${NC}"
echo ""

# 建立 Git Hook
cat > "$HOOK_FILE" << 'EOF'
#!/bin/bash
# Git pre-commit hook for Flow Functions auto-sync

# 檢查是否有修改 flow_executor.py
if git diff --cached --name-only | grep -q "flow_executor.py"; then
    echo "檢測到 flow_executor.py 變更，自動同步 Flow Functions..."
    
    # 執行自動同步
    if [ -f "/home/ct/RosAGV/scripts/flow-tools/auto-sync-functions.sh" ]; then
        /home/ct/RosAGV/scripts/flow-tools/auto-sync-functions.sh
        
        # 將更新的檔案加入 commit
        git add /home/ct/RosAGV/app/config/wcs/flow_functions.yaml
        git add /home/ct/RosAGV/app/web_api_ws/src/agvcui/agvcui/routers/linear_flow_designer.py
        
        echo "✅ Flow Functions 已自動同步並加入 commit"
    fi
fi

exit 0
EOF

# 設定執行權限
chmod +x "$HOOK_FILE"

echo -e "${GREEN}✅ Git Hook 已安裝到: $HOOK_FILE${NC}"
echo ""
echo -e "${YELLOW}說明:${NC}"
echo "  當您提交包含 flow_executor.py 的變更時，"
echo "  Git 會自動執行同步腳本，更新所有相關檔案。"
echo ""
echo -e "${YELLOW}移除 Hook:${NC}"
echo "  rm $HOOK_FILE"