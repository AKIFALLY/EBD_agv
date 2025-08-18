-- Update flow_log table schema for flow_wcs integration

-- Add missing columns to flow_log table (if they don't exist)
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS flow_name VARCHAR(200);
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS work_id INTEGER REFERENCES work(id);
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS section VARCHAR(100);
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS step_id VARCHAR(100);
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS function VARCHAR(100);
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS params JSON;
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS result JSON;
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS error_message TEXT;
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS duration FLOAT;

-- Legacy columns (for backward compatibility)
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS step_index INTEGER;
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS step_type VARCHAR(100);
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS message TEXT;

-- Note: metadata column already exists in the original schema
-- The flow_metadata field maps to the 'metadata' column in database