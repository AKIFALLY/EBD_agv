#!/bin/bash

# CDN 資源本地化腳本
# 
# 重要發現：Tailwind CSS CDN 行為說明
# - https://cdn.tailwindcss.com 返回的是 Tailwind CSS Play JavaScript 運行時，不是靜態 CSS
# - 這個 JavaScript 會動態掃描 HTML 並生成 CSS，完全模擬開發環境
# - 因此需要作為 <script> 載入，而不是 <link rel="stylesheet">
# - 檔案大小約 404KB，是完整的 Tailwind CSS 編譯器和運行時

# Define the base directory for your local paths
BASE_DIR="/home/ct/RosAGV/design/business-process-docs"

# Create directories if they don't exist
mkdir -p "$BASE_DIR/css/vendors"
mkdir -p "$BASE_DIR/js/vendors"
mkdir -p "$BASE_DIR/js/vendors/components"

echo "Downloading CDN files..."

# 1. Tailwind CSS Play (JavaScript Runtime)
echo "Downloading Tailwind CSS Play (JavaScript Runtime)..."
curl -o "$BASE_DIR/js/vendors/tailwind-play.js" "https://cdn.tailwindcss.com"
if [ $? -eq 0 ]; then
  echo "Tailwind CSS Play (JavaScript Runtime) downloaded successfully."
  echo "Note: This is a JavaScript file that dynamically generates CSS, not a static CSS file."
else
  echo "Failed to download Tailwind CSS Play. Trying static CSS alternative..."
  curl -o "$BASE_DIR/css/vendors/tailwind-static.css" "https://cdn.jsdelivr.net/npm/tailwindcss@3.4.1/dist/tailwind.min.css"
  if [ $? -eq 0 ]; then
    echo "Static Tailwind CSS downloaded successfully."
    echo "Note: You'll need to update index.html to use <link> instead of <script> for static CSS."
  else
    echo "Failed to download Tailwind CSS from all sources."
  fi
fi

# 2. Prism.js Core
echo "Downloading Prism.js Core..."
curl -o "$BASE_DIR/js/vendors/prism-core.js" "https://cdnjs.cloudflare.com/ajax/libs/prism/1.29.0/prism.min.js"
if [ $? -eq 0 ]; then
  echo "Prism.js Core downloaded successfully."
else
  echo "Failed to download Prism.js Core. Trying jsDelivr alternative..."
  curl -o "$BASE_DIR/js/vendors/prism-core.js" "https://cdn.jsdelivr.net/npm/prismjs@1.29.0/prism.min.js"
  if [ $? -eq 0 ]; then
    echo "Prism.js Core (jsDelivr) downloaded successfully."
  else
    echo "Failed to download Prism.js Core from all sources."
  fi
fi

# 3. Prism.js Autoloader
echo "Downloading Prism.js Autoloader..."
curl -o "$BASE_DIR/js/vendors/prism-autoloader.js" "https://cdnjs.cloudflare.com/ajax/libs/prism/1.29.0/plugins/autoloader/prism-autoloader.min.js"
if [ $? -eq 0 ]; then
  echo "Prism.js Autoloader downloaded successfully."
else
  echo "Failed to download Prism.js Autoloader. Trying jsDelivr alternative..."
  curl -o "$BASE_DIR/js/vendors/prism-autoloader.js" "https://cdn.jsdelivr.net/npm/prismjs@1.29.0/plugins/autoloader/prism-autoloader.min.js"
  if [ $? -eq 0 ]; then
    echo "Prism.js Autoloader (jsDelivr) downloaded successfully."
  else
    echo "Failed to download Prism.js Autoloader from all sources."
  fi
fi

# 4. Prism.js Theme CSS
echo "Downloading Prism.js Theme CSS..."
curl -o "$BASE_DIR/css/vendors/prism-theme.css" "https://cdnjs.cloudflare.com/ajax/libs/prism/1.29.0/themes/prism.min.css"
if [ $? -eq 0 ]; then
  echo "Prism.js Theme CSS downloaded successfully."
else
  echo "Failed to download Prism.js Theme CSS. Trying jsDelivr alternative..."
  curl -o "$BASE_DIR/css/vendors/prism-theme.css" "https://cdn.jsdelivr.net/npm/prismjs@1.29.0/themes/prism.min.css"
  if [ $? -eq 0 ]; then
    echo "Prism.js Theme CSS (jsDelivr) downloaded successfully."
  else
    echo "Failed to download Prism.js Theme CSS from all sources."
  fi
fi

# 5. Marked.js
echo "Downloading Marked.js..."
curl -o "$BASE_DIR/js/vendors/marked.js" "https://cdn.jsdelivr.net/npm/marked@9.1.6/marked.min.js"
if [ $? -eq 0 ]; then
  echo "Marked.js downloaded successfully."
else
  echo "Failed to download Marked.js."
fi

# 6. Prism.js Language Components
echo "Downloading Prism.js Language Components..."

# Common languages for RosAGV project (包含 RosAGV 文檔中可能用到的所有語言)
LANGUAGES=("yaml" "bash" "python" "javascript" "json" "json5" "css" "markup" "docker" "sql" "ini" "toml" "xml" "diff")

for lang in "${LANGUAGES[@]}"; do
  echo "Downloading Prism.js $lang component..."
  curl -o "$BASE_DIR/js/vendors/components/prism-$lang.min.js" "https://cdnjs.cloudflare.com/ajax/libs/prism/1.29.0/components/prism-$lang.min.js"
  if [ $? -eq 0 ]; then
    echo "Prism.js $lang component downloaded successfully."
  else
    echo "Failed to download Prism.js $lang component."
  fi
done

echo "All downloads attempted."