#!/bin/bash

# --- 要檢查的套件列表 ---
PACKAGES=(
  aggregate
  ca-certificates
  curl
  dnsutils
  fzf
  gh
  git
  gnupg2
  iproute2
  ipset
  iptables
  yq
  jq
  less
  man-db
  procps
  unzip
  ripgrep
  zsh
  zstd
)

# --- 顏色定義 ---
GREEN='[0;32m'
RED='[0;31m'
YELLOW='[1;33m'
NC='[0m' # No Color

# --- 腳本開始 ---
echo -e "${YELLOW}===== 正在檢查套件安裝狀態 =====${NC}"
echo ""

# 用於存放未安裝的套件
missing_packages=()

# 迴圈檢查每一個套件
for pkg in "${PACKAGES[@]}"; do
  # 使用 dpkg -s 檢查套件狀態，並將標準輸出和錯誤輸出都導向 /dev/null
  # 我們只關心指令的返回碼 (exit code)
  # 成功 (返回 0) 表示已安裝，失敗 (返回非 0) 表示未安裝
  if dpkg -s "$pkg" > /dev/null 2>&1; then
    # 使用 printf 進行格式化對齊輸出
    printf "%-20s [ ${GREEN}已安裝${NC} ]
" "$pkg"
  else
    printf "%-20s [ ${RED}未安裝${NC} ]
" "$pkg"
    missing_packages+=("$pkg")
  fi
done

echo ""

# 檢查是否有未安裝的套件
if [ ${#missing_packages[@]} -ne 0 ]; then
  echo -e "${YELLOW}偵測到未安裝的套件，正在嘗試安裝...${NC}"
  echo "未安裝的套件: ${missing_packages[*]}"
  
  # 更新套件列表並安裝
  # 使用 sudo 執行，可能需要使用者輸入密碼
  echo "正在更新套件列表..."
  sudo apt-get update
  echo "正在安裝缺少的套件..."
  sudo apt-get install -y "${missing_packages[@]}"
  
  echo ""
  echo -e "${YELLOW}===== 重新檢查安裝狀態 =====${NC}"
  # 再次檢查安裝結果
  all_successful=true
  for pkg in "${missing_packages[@]}"; do
    if dpkg -s "$pkg" > /dev/null 2>&1; then
      printf "%-20s [ ${GREEN}安裝成功${NC} ]
" "$pkg"
    else
      printf "%-20s [ ${RED}安裝失敗${NC} ]
" "$pkg"
      all_successful=false
    fi
  done

  if [ "$all_successful" = true ]; then
    echo -e "
${GREEN}所有套件均已成功安裝！${NC}"
  else
    echo -e "
${RED}部分套件安裝失敗，請檢查上面的錯誤訊息。${NC}"
  fi
else
  echo -e "${GREEN}所有必要的套件都已經安裝。${NC}"
fi


echo ""
echo -e "${YELLOW}===== 檢查完畢 =====${NC}"
