# 關於資料庫高 Rollback 率的說明

## TL;DR
**97%+ 的 rollback 率是完全正常的！** 這不是問題，請不要驚慌。

## 為什麼會有這麼高的 Rollback 率？

### 原因
1. **系統主要執行讀取操作**：我們的系統讀寫比約為 154:1
2. **SQLAlchemy/SQLModel 的設計**：每個 `with session:` 區塊結束時，如果沒有 commit，會自動 rollback
3. **大量的只讀查詢**：系統有 100+ 個使用 `with session:` 的只讀查詢

### 這是問題嗎？
**不是！** 因為：
- PostgreSQL 中，只讀事務的 rollback 幾乎沒有成本
- 只是清理事務狀態，沒有實際資料回滾
- Cache hit ratio 99.90% 證明性能優秀

## 程式碼範例

### 現有程式碼（完全正確）
```python
# 這段程式碼是正確的，不需要修改！
with connection_pool.get_session() as session:
    data = session.exec(select(User)).all()
    # 結束時自動 rollback - 這是正常的
```

### 如果真的想避免 rollback（通常不需要）
```python
# 選項 1：明確 commit（會增加 commit 數）
with connection_pool.get_session() as session:
    data = session.exec(select(User)).all()
    session.commit()  # 避免 rollback，但增加 commit

# 選項 2：使用 autocommit（僅適合簡單查詢）
with connection_pool.get_autocommit_session() as session:
    data = session.exec(select(User)).all()
    # 沒有事務，因此沒有 rollback
```

## 監控重點

不要只看 rollback 率，應該關注：
1. **Cache hit ratio**：應該 > 95%
2. **查詢延遲**：應該 < 100ms
3. **連接數**：應該 < max_connections
4. **實際錯誤**：檢查應用程式日誌

## 結論

高 rollback 率是 SQLAlchemy + PostgreSQL + 讀取密集型應用的正常現象。
這證明系統正在正確地管理事務，而不是出了問題。

如果未來有人看到 97% 的 rollback 率，請參考這份文件，不要驚慌！

---
最後更新：2025-08-13