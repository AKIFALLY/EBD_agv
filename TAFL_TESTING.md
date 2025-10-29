# TAFL 測試快速參考

## 📖 完整文檔

TAFL 測試的完整文檔位於：

```
docs-ai/operations/development/testing/tafl-flow-testing-guide.md
```

## 🚀 快速命令

### 執行所有測試
```bash
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c \
  "source /app/setup.bash && agvc_source && \
   cd /app/tafl_wcs_ws/src/tafl_wcs/test && python3 run_all_tests.py"
```

### 執行特定測試
```bash
# Loader 測試（6個）
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c \
  "source /app/setup.bash && agvc_source && \
   python3 /app/tafl_wcs_ws/src/tafl_wcs/test/test_loader_flows.py"

# Unloader 測試（4個）
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c \
  "source /app/setup.bash && agvc_source && \
   python3 /app/tafl_wcs_ws/src/tafl_wcs/test/test_unloader_flows.py"
```

## 📂 測試位置

```
tafl_wcs_ws/src/tafl_wcs/test/          # 業務流程測試
tafl_ws/src/tafl/test/                  # TAFL 語言核心測試
```

## 📊 測試覆蓋

- **Loader 流程**: 6/6 (100%) ✅
- **Unloader 流程**: 4/4 (100%) ✅
- **業務流程**: 8/8 (100%) ✅
- **總計**: 18 個測試場景 🎉

詳細內容請參考 `docs-ai` 文檔。
