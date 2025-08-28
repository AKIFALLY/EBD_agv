# TAFL WCS 測試目錄

## 測試檔案組織

### 單元測試
- `test_copyright.py` - 版權聲明檢查
- `test_flake8.py` - 代碼風格檢查
- `test_pep257.py` - 文檔字串檢查

### 功能測試
- `test_simple_db.py` - 簡單資料庫連接測試
- `test_check_all_data.py` - 完整資料查詢測試
- `test_final_data.py` - 最終資料驗證測試
- `test_final_check.py` - 最終檢查測試

### 整合測試
- `test_tafl_system.py` - TAFL 系統整合測試

## 執行測試

### 執行所有測試
```bash
cd /app/tafl_wcs_ws
colcon test --packages-select tafl_wcs
```

### 執行特定測試
```bash
cd /app/tafl_wcs_ws
python3 -m pytest src/tafl_wcs/test/test_simple_db.py -v
```

### 執行資料庫測試
```bash
cd /app/tafl_wcs_ws/src/tafl_wcs
python3 test/test_check_all_data.py
```

## 測試環境要求
- ROS 2 Jazzy 環境
- PostgreSQL 資料庫連接
- Python 3.12+
- pytest 測試框架