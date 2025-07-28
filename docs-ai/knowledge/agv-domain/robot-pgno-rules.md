# Robot PGNO 參數規則

## 🎯 適用場景
- AGV 機械臂控制的 PGNO 參數配置
- 確保取料和放料動作的參數順序正確性
- 為 Robot 控制開發提供技術規範

## 📋 PGNO 參數順序規則

### 基本原則
Robot PGNO 參數必須嚴格遵循特定的順序規則，以確保機械臂動作的正確執行。

### ACTION_FROM (取料動作)
**規則**: 源位置參數在前
```python
# 取料動作 - 源位置在前
TAKE_XXX_PGNO = context.robot.ACTION_FROM + \
    context.robot.SOURCE_POSITION + context.robot.NONE_POSITION
```

**說明**:
- `ACTION_FROM`: 取料動作指令
- `SOURCE_POSITION`: 取料來源位置參數
- `NONE_POSITION`: 空位置參數

### ACTION_TO (放料動作)  
**規則**: NONE_POSITION在前，目標位置在後
```python  
# 放料動作 - NONE_POSITION在前，目標位置在後
PUT_XXX_PGNO = context.robot.ACTION_TO + \
    context.robot.NONE_POSITION + context.robot.TARGET_POSITION
```

**說明**:
- `ACTION_TO`: 放料動作指令
- `NONE_POSITION`: 空位置參數
- `TARGET_POSITION`: 放料目標位置參數

## 🔧 實際應用範例

### Cargo Mover AGV 範例
```python
# 從架台取料
TAKE_RACK_PGNO = context.robot.ACTION_FROM + \
    context.robot.rack_position + context.robot.NONE_POSITION

# 放料到傳送箱  
PUT_TRANSFER_PGNO = context.robot.ACTION_TO + \
    context.robot.NONE_POSITION + context.robot.transfer_position
```

### Loader AGV 範例
```python
# 從傳送箱取料
TAKE_TRANSFER_PGNO = context.robot.ACTION_FROM + \
    context.robot.transfer_position + context.robot.NONE_POSITION

# 放料到清潔機
PUT_CLEANER_PGNO = context.robot.ACTION_TO + \
    context.robot.NONE_POSITION + context.robot.cleaner_position
```

## ⚠️ 重要注意事項

### 參數順序錯誤的後果
- 機械臂動作異常
- PLC 通訊錯誤  
- 任務執行失敗
- 安全風險

### 驗證方法
```python
def validate_pgno_format(pgno_string):
    """驗證 PGNO 參數格式是否正確"""
    parts = pgno_string.split('+')
    
    if parts[0] == "ACTION_FROM":
        # 取料動作: SOURCE_POSITION + NONE_POSITION
        return len(parts) == 3 and parts[2] == "NONE_POSITION"
    elif parts[0] == "ACTION_TO":
        # 放料動作: NONE_POSITION + TARGET_POSITION  
        return len(parts) == 3 and parts[1] == "NONE_POSITION"
    
    return False
```

## 📋 開發檢查清單

### PGNO 參數開發前檢查
- [ ] 確認動作類型 (取料 vs 放料)
- [ ] 理解源位置和目標位置定義
- [ ] 檢查參數順序規則
- [ ] 驗證 PGNO 字串格式

### PGNO 參數開發後檢查
- [ ] 執行參數格式驗證
- [ ] 測試機械臂動作正確性
- [ ] 檢查 PLC 通訊回應
- [ ] 確認任務執行結果

## 🔗 交叉引用
- AGV 車型特性: @docs-ai/knowledge/agv-domain/vehicle-types.md
- Robot 控制邏輯: @docs-ai/context/workspaces/agv-workspaces.md
- PLC 通訊協定: @docs-ai/knowledge/protocols/plc-communication.md
- 開發規範: @docs-ai/operations/development/core-principles.md