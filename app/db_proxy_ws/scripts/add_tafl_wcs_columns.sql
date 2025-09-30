-- Add missing columns for tafl_wcs integration

-- Add columns to work table
ALTER TABLE work ADD COLUMN IF NOT EXISTS work_code VARCHAR(100);

-- Add columns to task table
ALTER TABLE task ADD COLUMN IF NOT EXISTS task_id VARCHAR(100);
ALTER TABLE task ADD COLUMN IF NOT EXISTS type VARCHAR(100);
ALTER TABLE task ADD COLUMN IF NOT EXISTS location_id INTEGER;
ALTER TABLE task ADD COLUMN IF NOT EXISTS rack_id INTEGER;

-- Update existing records with default values
UPDATE work SET work_code = 'WORK_' || id WHERE work_code IS NULL;
UPDATE task SET task_id = 'TASK_' || id WHERE task_id IS NULL;
UPDATE task SET type = 'default' WHERE type IS NULL;