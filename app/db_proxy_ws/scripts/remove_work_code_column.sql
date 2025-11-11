-- 移除 work_code 欄位
-- 執行日期: 2025-11-06
-- 原因: work_code 欄位使用率很低，僅在 TAFL 查詢中使用，決定移除以簡化資料模型

-- 删除 work 表的 work_code 列
ALTER TABLE work DROP COLUMN IF EXISTS work_code;

-- 驗證欄位已移除
SELECT column_name, data_type
FROM information_schema.columns
WHERE table_name = 'work'
ORDER BY ordinal_position;
