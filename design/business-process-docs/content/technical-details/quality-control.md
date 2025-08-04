# 品質管制系統

## 概述

RosAGV 系統整合了完整的品質管制機制，透過視覺檢測、製程適配驗證和異常處理流程，確保眼鏡生產的品質一致性和可追溯性。

## 核心組件

### 視覺檢測系統

#### SensorPart 相機系統
- **技術規格**: 3D 視覺定位 + OCR 識別
- **部署位置**: Cargo AGV 機械臂
- **主要功能**:
  - Rack 整體 3D 掃描定位
  - 單一 Carrier 精確 OCR 識別
  - 產品編號/條碼讀取
  - 產品類型自動分類

#### 檢測流程
```
3D 視覺檢測工作流程
1. Rack 整體掃描 → 識別所有 Carrier 位置座標
2. 建立定位基準點 → 更新機械臂座標系
3. 逐一 Carrier 處理：
   ├── 移動定位到 Carrier 前方
   ├── OCR 掃描讀取產品資訊
   ├── 識別產品類型和製程需求
   └── 執行製程適配性檢查
```

### 製程適配驗證

#### 智能品質管制邏輯
- **製程匹配檢查**: 
  - 查詢產品的 `process_settings_id`
  - 比對當前房間的製程能力
  - 判斷製程適配性 (泡藥1次 vs 2次)
  
#### 分支處理機制
```python
# 製程適配檢查邏輯
if 製程匹配:
    # 正常流程
    機械臂抓取 Carrier
    8bit 通訊開啟傳送箱門
    將 Carrier 放入入口傳送箱
    更新系統狀態為正常
else:
    # 異常處理
    標記 Carrier 為 NG 狀態
    機械臂回到 Home 位置
    記錄異常日誌
    繼續處理下一個 Carrier
```

## 品質管制機制

### 多層次檢查體系

#### 第一層：產品識別驗證
- **OCR 準確性檢查**: 確保產品編號正確讀取
- **產品類型驗證**: S/L 尺寸產品正確分類
- **條碼完整性**: 防止破損或不清晰的條碼進入製程

#### 第二層：製程適配性檢查
- **房間能力驗證**: 
  - Room1/Room2 只接受泡藥1次產品
  - 泡藥2次產品自動標記為 NG
- **設備資源檢查**: 確認目標設備的可用性
- **負載均衡**: 避免特定設備過載

#### 第三層：異常狀態處理
- **NG 產品隔離**: NG 產品保留在 Rack 上，不影響其他產品處理
- **完整性記錄**: 記錄每個 Carrier 的處理狀態和異常原因
- **可追溯性**: 建立從原料到成品的完整追蹤鏈

### NG 處理系統

#### NG 判定標準
```sql
-- NG Rack 判定邏輯
SELECT r.id as rack_id, COUNT(c.id) as ng_count
FROM rack r
LEFT JOIN carrier c ON c.rack_id = r.id
WHERE c.status = 'NG'
GROUP BY r.id
HAVING COUNT(c.id) > 0;
-- 有任何 NG Carrier 的 Rack 整體標記為 NG
```

#### NG 處理工作流程
```
NG Rack 完整處理流程
1. 系統檢測 → 自動識別 NG Rack
2. WCS 調度 → 產生 NG Rack 派車任務
3. KUKA AGV → 將 NG Rack 搬運到 NG 處理區 (71-72)
4. 人工檢查 → 作業員檢查 NG 原因
   ├── 產品修正或報廢處理
   ├── 清理 Rack 上的所有產品
   └── 分析 NG 根本原因
5. 狀態重置 → AGVCUI 網頁操作
   ├── 登入 agvc.ui 管理介面
   ├── 將 Rack 重新設為空 Rack 狀態
   └── 更新資料庫狀態記錄
6. 物理回收 → 人工搬運空 Rack 到回放格
7. 系統識別 → 空 Rack 重新進入循環使用
```

## 品質數據管理

### 資料庫追蹤

#### 品質相關資料表
```sql
-- Carrier 狀態追蹤
carrier 表:
├── id: Carrier 識別碼
├── rack_id: 所屬 Rack
├── status: 狀態 (NORMAL/NG/PROCESSING)
├── product_id: 產品識別
├── quality_check_result: 品質檢查結果
├── ng_reason: NG 原因碼
└── created_at, updated_at: 時間戳

-- 品質檢查記錄
quality_log 表 (系統生成):
├── carrier_id: 被檢查的 Carrier
├── check_type: 檢查類型 (OCR/PROCESS_MATCH/VISUAL)
├── check_result: 檢查結果 (PASS/FAIL)
├── error_details: 錯誤詳情 JSON
├── inspector: 檢查員 (系統/人工)
└── timestamp: 檢查時間
```

#### 品質報表生成
```sql
-- 品質統計報表
SELECT 
    DATE(created_at) as date,
    COUNT(*) as total_carriers,
    SUM(CASE WHEN status = 'NORMAL' THEN 1 ELSE 0 END) as pass_count,
    SUM(CASE WHEN status = 'NG' THEN 1 ELSE 0 END) as ng_count,
    ROUND(SUM(CASE WHEN status = 'NORMAL' THEN 1 ELSE 0 END) * 100.0 / COUNT(*), 2) as pass_rate
FROM carrier 
WHERE created_at >= DATE_SUB(NOW(), INTERVAL 30 DAY)
GROUP BY DATE(created_at)
ORDER BY date DESC;
```

### 即時監控

#### Socket.IO 即時通知
```javascript
// 品質異常即時通知
socket.on('quality_alert', (data) => {
    const alert = {
        type: 'NG_DETECTED',
        rack_id: data.rack_id,
        carrier_id: data.carrier_id,
        ng_reason: data.ng_reason,
        timestamp: data.timestamp,
        location: data.current_location
    };
    
    // 顯示警示訊息
    showQualityAlert(alert);
    
    // 更新統計面板
    updateQualityStats();
});
```

## 系統整合

### 與製程系統整合

#### AGV 狀態機整合
```python
# 品質檢查集成到 AGV 狀態機
class CargoAgvQualityState(BaseContext):
    def __init__(self):
        self.sensor_part = SensorPartInterface()
        self.quality_checker = QualityChecker()
        
    def execute_quality_check(self, carrier):
        # OCR 識別
        ocr_result = self.sensor_part.scan_carrier(carrier)
        
        # 製程適配檢查
        process_match = self.quality_checker.check_process_compatibility(
            ocr_result.product_id, 
            self.current_room.process_settings
        )
        
        if process_match:
            return QualityResult.PASS
        else:
            self.log_ng_reason(carrier, "PROCESS_MISMATCH")
            return QualityResult.NG
```

#### WCS 決策引擎整合
```python
# WCS 中的品質相關任務優先級
QUALITY_TASK_PRIORITIES = {
    'NG_RACK_HANDLING': 90,      # NG Rack 回收 (高優先級)
    'QUALITY_CHECK': 70,         # 品質檢查任務
    'NORMAL_TRANSPORT': 60       # 正常搬運任務
}

def create_ng_handling_task(rack_id):
    """創建 NG Rack 處理任務"""
    return Task(
        work_id='220001',  # KUKA 移動貨架任務
        priority=90,
        source_location=f'room_{room_id}_entrance',
        target_location='ng_processing_area_71',
        rack_id=rack_id,
        task_type='NG_HANDLING'
    )
```

### Web API 整合

#### 品質數據 API
```python
# FastAPI 品質監控端點
@app.get("/quality/stats")
async def get_quality_stats(
    date_from: date = None,
    date_to: date = None,
    session: AsyncSession = Depends(get_session)
):
    """獲取品質統計數據"""
    return await get_quality_statistics(session, date_from, date_to)

@app.get("/quality/ng-racks")
async def get_ng_racks(session: AsyncSession = Depends(get_session)):
    """獲取當前 NG Rack 列表"""
    return await get_current_ng_racks(session)

@app.post("/quality/reset-rack/{rack_id}")
async def reset_rack_status(
    rack_id: int,
    session: AsyncSession = Depends(get_session)
):
    """重置 Rack 狀態 (AGVCUI 操作)"""
    return await reset_rack_to_empty(session, rack_id)
```

## 效能監控

### 關鍵品質指標 (KQI)

#### 實時品質指標
- **通過率 (Pass Rate)**: 正常通過品質檢查的比例
- **NG 檢出率**: 及時發現品質問題的比例
- **誤檢率**: 錯誤將正常產品標記為 NG 的比例
- **處理週期時間**: 從檢測到 NG 處理完成的時間

#### 系統效能指標
- **OCR 識別準確率**: SensorPart 系統的識別精度
- **視覺定位精度**: 3D 掃描的定位準確性
- **處理吞吐量**: 單位時間內處理的 Carrier 數量
- **系統可用度**: 品質檢查系統的運行時間比例

### 預警機制

#### 品質趨勢監控
```python
# 品質趨勢分析
class QualityTrendAnalyzer:
    def __init__(self):
        self.alert_thresholds = {
            'ng_rate_high': 0.05,      # NG 率超過 5%
            'ng_rate_spike': 0.02,     # NG 率突然增加 2%
            'ocr_accuracy_low': 0.95   # OCR 準確率低於 95%
        }
    
    def analyze_quality_trend(self, recent_data):
        alerts = []
        
        current_ng_rate = self.calculate_ng_rate(recent_data)
        if current_ng_rate > self.alert_thresholds['ng_rate_high']:
            alerts.append({
                'type': 'HIGH_NG_RATE',
                'value': current_ng_rate,
                'threshold': self.alert_thresholds['ng_rate_high']
            })
        
        return alerts
```

## 最佳實踐

### 運營建議

#### 日常品質管理
1. **定期校準**: SensorPart 相機系統定期校準確保準確性
2. **數據分析**: 每日檢視品質統計報表，識別趨勢和異常
3. **根因分析**: 對 NG 產品進行根本原因分析，改善製程
4. **持續改進**: 基於品質數據調整檢查參數和標準

#### 異常處理流程
1. **快速響應**: NG 檢出後立即進入處理流程
2. **隔離控制**: 確保 NG 產品不進入後續製程
3. **狀態追蹤**: 全程記錄 NG 處理的每個步驟
4. **系統恢復**: 處理完成後及時恢復正常運行

### 系統維護

#### 預防性維護
```bash
# 品質系統健康檢查
r quality-check                    # 檢查品質系統狀態
r sensor-calibration              # 執行感測器校準
r quality-data-cleanup            # 清理過期品質數據
```

#### 故障排除
```bash
# 常見品質系統問題診斷
r quality-diag                    # 品質系統診斷
r ocr-accuracy-test               # OCR 準確性測試
r vision-system-status            # 視覺系統狀態檢查
```

## 相關文檔

- [系統架構](../system-architecture/dual-environment.md) - RosAGV 雙環境架構
- [技術棧詳細](../system-architecture/technology-stack.md) - 系統技術組件
- [車型特性](../agv-vehicles/vehicle-types.md) - AGV 車型和功能
- [PLC 整合](plc-integration.md) - PLC 通訊和控制
- [眼鏡生產流程](../business-processes/eyewear-production.md) - 完整業務流程