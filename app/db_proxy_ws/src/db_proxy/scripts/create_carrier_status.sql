-- 創建 carrier_status 表
CREATE TABLE IF NOT EXISTS carrier_status (
    id INTEGER PRIMARY KEY,
    name VARCHAR(50) NOT NULL,
    description TEXT,
    color VARCHAR(20) DEFAULT 'is-light'
);

-- 插入預設載具狀態資料
INSERT OR IGNORE INTO carrier_status (id, name, description, color) VALUES
(1, '空閒', '載具空閒，可以使用', 'is-success'),
(2, '使用中', '載具正在使用中', 'is-warning'),
(3, '故障', '載具發生故障', 'is-danger'),
(4, '待處理', '載具等待處理', 'is-info'),
(5, '處理中', '載具正在處理製程', 'is-primary'),
(6, 'NG', '載具處理結果不良', 'is-dark'),
(7, '維護中', '載具正在維護', 'is-light'),
(8, '已完成', '載具處理完成', 'is-link');

-- 檢查插入結果
SELECT * FROM carrier_status;
