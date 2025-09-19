# agv_base - AGV 基礎核心框架

## 📚 Context Loading
../../../../CLAUDE.md  # 引用根目錄系統文档
../../CLAUDE.md  # 引用上層 agv_ws 工作空間文档

# AGV Base 層專屬知識（套件特有）
@docs-ai/knowledge/agv-domain/magic-value-analysis.md  # 魔術值分析

## 📋 套件概述
agv_base 是 AGV 工作空間中的 **基礎核心框架套件**，實現 3層狀態機架構中的 Base 層。提供 AGV 狀態機的抽象基類、事件處理機制、硬體控制介面等通用邏輯，為所有 AGV 車型提供統一的基礎架構。

**🎯 定位**: 3層狀態機的 Base 層實作，所有 AGV 車型的共同基礎

## 🔧 Base 層核心架構

### 核心框架檔案
```
agv_base/
├─ agv_node_base.py      # AGV 節點基類 (50ms 主循環，所有車型共用)
├─ base_context.py       # Base 層狀態機上下文管理
├─ context_abc.py        # Context 抽象基類定義
├─ event.py              # 統一事件處理機制
├─ robot.py              # 機器人硬體抽象層 (PGNO 系統)
└─ agv_states/           # Base 層狀態定義目錄
```

### Base 層特有功能
- **🔄 50ms 主循環**: 所有 AGV 車型的統一執行週期
- **📡 事件驅動**: 統一的事件處理和狀態轉換機制
- **🤖 Robot 抽象**: 機械臂的統一控制介面 (PGNO 系統)
- **🏗️ Context 管理**: 狀態機上下文的基礎實作和生命週期管理

## 🚀 Base 層專用開發

### 基礎框架測試
```bash
# 【推薦方式】透過上層工作空間工具
# 參考: ../CLAUDE.md 開發環境設定

# 【直接測試】Base 層框架
cd /app/agv_ws/src/agv_base
python3 -m pytest test/ -v

# Base 層套件建置
colcon build --packages-select agv_base
```

### Base 層核心驗證
```bash
# 檢查 Base 層核心類別載入
python3 -c "
from agv_base.agv_node_base import AGVNodeBase
from agv_base.base_context import BaseContext
from agv_base.robot import Robot
print('Base 層核心框架載入成功')
"

# 驗證狀態機基礎
python3 -c "
from agv_base.agv_states import *
print('Base 層狀態定義載入成功')
"
```

## 🔗 Base 層整合點
- **三個 AGV 車型**: 繼承 BaseContext 和 AGVNodeBase
- **plc_proxy_ws**: 透過 Robot 類別進行 PLC 通訊
- **agv_interfaces**: 定義 AGV 狀態和事件訊息格式

## 🚨 Base 層專項故障排除

**⚠️ 通用故障排除請參考**: ../CLAUDE.md 故障排除章節

### Base 層特有問題
- **50ms 主循環問題**: 檢查 AGVNodeBase 執行週期
- **事件處理異常**: 驗證 event.py 事件機制  
- **Robot 抽象問題**: 檢查 PGNO 系統整合
- **Context 管理錯誤**: 檢查狀態機上下文生命週期