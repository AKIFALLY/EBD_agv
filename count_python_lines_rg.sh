#!/bin/bash

# 定義一個計算特定檔案類型總行數的函式
# 參數 $1: 檔案副檔名 (例如 "py", "sh", "md")
count_lines_by_extension() {
    local extension="$1"
    local total_lines=0
    local processed_files_count=0

    # rg 的 -t 選項通常支援常見的副檔名，如 py, sh, md。
    # 如果遇到不支援的，可能需要改用 -g "*.${extension}"
    local rg_type_flag="-t ${extension}"

    echo "--- 正在計算所有 .${extension} 檔案的總行數 ---"

    # 第一步：計算總共有多少個指定副檔名的檔案 (用於顯示進度)
    local total_files=$(rg ${rg_type_flag} -l . | wc -l | awk '{print $1}')

    echo "偵測到總共有 $total_files 個 .${extension} 檔案需要處理。"
    #echo "---------------------------------------------------"

    # 第二步：遍歷檔案並計算行數，同時顯示進度 (使用進程替換避免子 shell 問題)
    while IFS= read -r file; do
        processed_files_count=$((processed_files_count + 1))

        # 檢查檔案是否存在且可讀，避免錯誤
        if [[ -f "$file" && -r "$file" ]]; then
            line_count=$(wc -l < "$file" | awk '{print $1}')
            total_lines=$((total_lines + line_count))
            #echo "正在處理第 $processed_files_count 個檔案 (共 $total_files 個): $file (行數: $line_count)"
        else
            echo "警告: 無法讀取或找到檔案: $file"
        fi
    done < <(rg ${rg_type_flag} -l .)

    echo "---------------------------------------------------"
    echo "目前目錄下所有 .${extension} 檔案的總行數為: $total_lines"
    echo "" # 增加一個空行以便於區分不同類型的報告
}

# --- 呼叫函式來計算不同檔案類型的總行數 ---

# 計算 .py 檔案總行數
count_lines_by_extension "py"

# 計算 .sh 檔案總行數
# 注意：rg 預設可能只識別 .sh，如果同時有 .bash 且你想一起算，可能需要調整
# 或者單獨調用兩次： count_lines_by_extension "sh" 和 count_lines_by_extension "bash"
# 為了簡潔，這裡先用 "sh" 代表腳本檔
count_lines_by_extension "sh"

# 計算 .md 檔案總行數
count_lines_by_extension "md"