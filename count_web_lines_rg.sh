#!/bin/bash

# 全局變數用於累計總計
TOTAL_HTML_FILES=0
TOTAL_HTML_LINES=0
TOTAL_JS_FILES=0
TOTAL_JS_LINES=0
TOTAL_CSS_FILES=0
TOTAL_CSS_LINES=0

# 定義一個計算特定目錄和檔案類型總行數的函式
# 參數 $1: 檔案副檔名 (例如 "html", "js", "css")
# 參數 $2: 搜尋路徑
# 參數 $3+: 排除的 grep 模式 (可選)
count_lines_by_extension() {
    local extension="$1"
    local search_path="$2"
    shift 2
    local exclude_patterns=("$@")

    local total_lines=0
    local processed_files_count=0

    echo "--- 正在計算 ${search_path} 中所有 .${extension} 檔案的總行數 ---"

    # 建立基本 rg 命令
    local rg_cmd="rg -g '*.${extension}' -l . '${search_path}' 2>/dev/null"

    # 添加 grep 過濾排除模式
    for pattern in "${exclude_patterns[@]}"; do
        rg_cmd+=" | grep -v '${pattern}'"
    done

    # 計算總共有多少個檔案
    local total_files=$(eval "$rg_cmd" | wc -l | awk '{print $1}')

    if [ "$total_files" -eq 0 ]; then
        echo "  未找到符合條件的檔案"
        echo "---------------------------------------------------"
        return
    fi

    echo "  偵測到總共有 $total_files 個 .${extension} 檔案需要處理。"

    # 遍歷檔案並計算行數
    while IFS= read -r file; do
        processed_files_count=$((processed_files_count + 1))

        if [[ -f "$file" && -r "$file" ]]; then
            line_count=$(wc -l < "$file" | awk '{print $1}')
            total_lines=$((total_lines + line_count))
        else
            echo "  警告: 無法讀取或找到檔案: $file"
        fi
    done < <(eval "$rg_cmd")

    echo "---------------------------------------------------"
    echo "  ${search_path} 中所有 .${extension} 檔案的總行數為: $total_lines"
    echo ""

    # 更新全局統計
    case "$extension" in
        html)
            TOTAL_HTML_FILES=$((TOTAL_HTML_FILES + total_files))
            TOTAL_HTML_LINES=$((TOTAL_HTML_LINES + total_lines))
            ;;
        js)
            TOTAL_JS_FILES=$((TOTAL_JS_FILES + total_files))
            TOTAL_JS_LINES=$((TOTAL_JS_LINES + total_lines))
            ;;
        css)
            TOTAL_CSS_FILES=$((TOTAL_CSS_FILES + total_files))
            TOTAL_CSS_LINES=$((TOTAL_CSS_LINES + total_lines))
            ;;
    esac
}

# 統計 Web API 工作空間
echo "============================================================"
echo "=== 統計 Web API 工作空間 (app/web_api_ws/src/) ==="
echo "============================================================"
echo ""

WEB_API_PATH="/home/ct/EBD_agv/app/web_api_ws/src"

# HTML 檔案（排除 build 和 install 目錄）
count_lines_by_extension "html" "$WEB_API_PATH" \
    "build/" \
    "install/"

# 自定義 JS 檔案（排除第三方庫和編譯輸出）
count_lines_by_extension "js" "$WEB_API_PATH" \
    "build/" \
    "install/" \
    "/lib/"

# 自定義 CSS 檔案（排除第三方庫和編譯輸出）
count_lines_by_extension "css" "$WEB_API_PATH" \
    "build/" \
    "install/" \
    "bulma_1_0_4" \
    "materialdesignicons.css" \
    "materialdesignicons.min.css" \
    "leaflet.css" \
    "Control.FullScreen.css" \
    "codemirror" \
    "font-awesome"

echo ""
echo "============================================================"
echo "=== 統計業務流程文檔系統 (design/business-process-docs/) ==="
echo "============================================================"
echo ""

DOCS_PATH="/home/ct/EBD_agv/design/business-process-docs"

# HTML 檔案
count_lines_by_extension "html" "$DOCS_PATH"

# 自定義 JS 檔案（排除 vendors 和 JSON）
count_lines_by_extension "js" "$DOCS_PATH" \
    "vendors" \
    ".json"

# 自定義 CSS 檔案（排除 vendors）
count_lines_by_extension "css" "$DOCS_PATH" \
    "vendors"

echo ""
echo "============================================================"
echo "=== 總計統計 (自定義代碼，已排除第三方庫) ==="
echo "============================================================"
echo ""
echo "HTML 檔案:"
echo "  檔案數量: $TOTAL_HTML_FILES"
echo "  總行數:   $TOTAL_HTML_LINES"
echo ""
echo "自定義 JavaScript 檔案:"
echo "  檔案數量: $TOTAL_JS_FILES"
echo "  總行數:   $TOTAL_JS_LINES"
echo ""
echo "自定義 CSS 檔案:"
echo "  檔案數量: $TOTAL_CSS_FILES"
echo "  總行數:   $TOTAL_CSS_LINES"
echo ""
echo "---------------------------------------------------"
GRAND_TOTAL=$((TOTAL_HTML_LINES + TOTAL_JS_LINES + TOTAL_CSS_LINES))
TOTAL_FILES=$((TOTAL_HTML_FILES + TOTAL_JS_FILES + TOTAL_CSS_FILES))
echo "總計: $TOTAL_FILES 個檔案，共 $GRAND_TOTAL 行代碼"
echo "============================================================"
