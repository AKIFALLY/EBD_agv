from typing import Type, Optional, Dict
import json
from datetime import datetime, timezone
from dateutil.parser import isoparse

ISO8601_FORMAT = "%Y-%m-%dT%H:%M:%S.%fZ"  # UTC ISO 格式


def datetime_to_str(dt: Optional[datetime]) -> str:
    """將 datetime 轉為 ISO8601 格式字串，空值回傳空字串"""
    if dt is None:
        return ""
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt.astimezone(timezone.utc).strftime(ISO8601_FORMAT)


def str_to_datetime(s: str) -> Optional[datetime]:
    """將 ISO8601 字串轉為 datetime，空字串或格式錯誤回傳 None"""
    if not s or s.strip() == "":
        return None
    try:
        return datetime.strptime(s, ISO8601_FORMAT).replace(tzinfo=timezone.utc)
    except ValueError:
        try:
            return isoparse(s).astimezone(timezone.utc)
        except Exception:
            return None


def model_to_msg(model_obj, msg_class, field_map: Optional[Dict[str, str]] = None):
    """將 ORM model 轉為 ROS2 message"""
    msg = msg_class()
    for msg_field, field_type in msg.get_fields_and_field_types().items():
        model_field = field_map[msg_field] if field_map and msg_field in field_map else msg_field

        if not hasattr(model_obj, model_field):
            continue

        value = getattr(model_obj, model_field)

        # 處理空值
        if value is None or (isinstance(value, str) and value.strip() == ""):
            if field_type == "string":
                value = ""
            else:
                continue  # 跳過非字串的 None 值

        # dict → JSON 字串
        if field_type == "string" and isinstance(value, dict):
            value = json.dumps(value)

        # datetime → ISO8601 字串（Z 結尾）
        if field_type == "string" and isinstance(value, datetime):
            value = datetime_to_str(value)

        setattr(msg, msg_field, value)
    return msg


def msg_to_model(msg_obj, model_class: Type, field_map: Optional[Dict[str, str]] = None):
    """將 ROS2 message 轉回 ORM model"""
    model_data = {}
    for msg_field, field_type in msg_obj.get_fields_and_field_types().items():
        model_field = field_map[msg_field] if field_map and msg_field in field_map else msg_field
        value = getattr(msg_obj, msg_field)
        print(f"msg_field:{msg_field} field_type:{field_type}")

        if field_type == "string":
            # 空字串轉為 None
            if value == "":
                value = None
                print(f"msg_field:{msg_field} None")
            else:
                # 嘗試轉換為 datetime
                dt = str_to_datetime(value)
                if dt is not None:
                    value = dt
                else:
                    # 嘗試轉換為 dict（JSON）
                    if value.strip().startswith("{"):
                        try:
                            value = json.loads(value)
                        except json.JSONDecodeError:
                            pass  # 保留原始字串

        model_data[model_field] = value
    return model_class(**model_data)
