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
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# --- 腳本開始 ---
echo -e "${YELLOW}===== 正在檢查套件安裝狀態 =====${NC}"
echo ""

# 迴圈檢查每一個套件
for pkg in "${PACKAGES[@]}"; do
  # 使用 dpkg -s 檢查套件狀態，並將標準輸出和錯誤輸出都導向 /dev/null
  # 我們只關心指令的返回碼 (exit code)
  # 成功 (返回 0) 表示已安裝，失敗 (返回非 0) 表示未安裝
  if dpkg -s "$pkg" > /dev/null 2>&1; then
    # 使用 printf 進行格式化對齊輸出
    printf "%-20s [ ${GREEN}已安裝${NC} ]\n" "$pkg"
  else
    printf "%-20s [ ${RED}未安裝${NC} ]\n" "$pkg"
  fi
done

echo ""
echo -e "${YELLOW}===== 檢查完畢 =====${NC}"