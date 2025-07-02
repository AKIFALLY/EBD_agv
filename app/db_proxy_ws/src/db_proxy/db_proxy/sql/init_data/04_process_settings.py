"""
04. 製程設置初始化資料
無相依性
"""

from db_proxy.models import ProcessSettings
from sqlmodel import select


def initialize_process_settings(session):
    """初始化製程設置資料"""
    print("⚙️ 初始化製程設置資料...")
    
    default_processes = [
        {"soaking_times": 1, "description": "泡藥泡一次"},
        {"soaking_times": 2, "description": "泡藥泡兩次"}
    ]
    
    for proc in default_processes:
        exists = session.exec(select(ProcessSettings).where(
            ProcessSettings.soaking_times == proc["soaking_times"])).first()
        if not exists:
            session.add(ProcessSettings(**proc))
    
    session.commit()
    print("✅ 製程設置資料初始化完成")
