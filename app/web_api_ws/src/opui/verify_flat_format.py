#!/usr/bin/env python3
"""
驗證 OPUI Socket.IO API 扁平化格式改造
檢查前後端程式碼是否正確實作扁平化格式
"""

import re
import os

def check_frontend_api():
    """檢查前端 API 是否正確實作扁平化格式"""
    api_file = "opui/frontend/static/js/api.js"
    
    if not os.path.exists(api_file):
        print("❌ 找不到前端 API 檔案")
        return False
    
    with open(api_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    checks = []
    
    # 檢查 _convertToUnifiedFormat 方法是否使用扁平化格式
    if 'format: \'flat\'' in content:
        checks.append("✅ _convertToUnifiedFormat 使用扁平化格式標識")
    else:
        checks.append("❌ _convertToUnifiedFormat 未使用扁平化格式標識")
    
    # 檢查 login 方法是否直接使用扁平化格式
    if 'const loginData = {' in content and 'clientId: userState.clientId' in content:
        checks.append("✅ login 方法使用扁平化格式")
    else:
        checks.append("❌ login 方法未使用扁平化格式")
    
    # 檢查是否移除了分離式架構相關程式碼
    if 'user_data = data.get("user"' not in content:
        checks.append("✅ 已移除分離式架構相關程式碼")
    else:
        checks.append("❌ 仍存在分離式架構相關程式碼")
    
    # 檢查版本號是否更新為 3.0
    if 'version: \'3.0\'' in content:
        checks.append("✅ 版本號已更新為 3.0")
    else:
        checks.append("❌ 版本號未更新")
    
    print("📱 前端 API 檢查結果:")
    for check in checks:
        print(f"  {check}")
    
    return all("✅" in check for check in checks)

def check_backend_socket():
    """檢查後端 Socket 處理器是否正確實作扁平化格式"""
    socket_file = "opui/core/op_ui_socket.py"
    
    if not os.path.exists(socket_file):
        print("❌ 找不到後端 Socket 檔案")
        return False
    
    with open(socket_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    checks = []
    
    # 檢查 client_update 方法是否使用扁平化格式
    if 'data.get("clientId")' in content and 'data.get("machineId")' in content:
        checks.append("✅ client_update 使用扁平化格式提取")
    else:
        checks.append("❌ client_update 未使用扁平化格式提取")
    
    # 檢查是否移除了分離式架構支援
    if 'user_data = data.get("user"' not in content:
        checks.append("✅ 已移除分離式架構支援")
    else:
        checks.append("❌ 仍存在分離式架構支援")
    
    # 檢查 login 方法是否使用扁平化格式
    login_pattern = r'async def login.*?data\.get\("clientId"\)'
    if re.search(login_pattern, content, re.DOTALL):
        checks.append("✅ login 方法使用扁平化格式")
    else:
        checks.append("❌ login 方法未使用扁平化格式")
    
    # 檢查註釋是否已更新
    if '統一使用扁平化格式' in content:
        checks.append("✅ 註釋已更新為扁平化格式")
    else:
        checks.append("❌ 註釋未更新")
    
    print("🖥️ 後端 Socket 檢查結果:")
    for check in checks:
        print(f"  {check}")
    
    return all("✅" in check for check in checks)

def check_documentation():
    """檢查文檔是否已更新"""
    doc_file = "docs/SOCKET_API_FORMAT_GUIDE.md"
    
    if not os.path.exists(doc_file):
        print("❌ 找不到格式指南文檔")
        return False
    
    with open(doc_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    checks = []
    
    # 檢查標題是否更新
    if '扁平化格式統一指南' in content:
        checks.append("✅ 文檔標題已更新")
    else:
        checks.append("❌ 文檔標題未更新")
    
    # 檢查是否包含扁平化格式說明
    if '統一扁平化格式' in content:
        checks.append("✅ 包含扁平化格式說明")
    else:
        checks.append("❌ 缺少扁平化格式說明")
    
    print("📚 文檔檢查結果:")
    for check in checks:
        print(f"  {check}")
    
    return all("✅" in check for check in checks)

def main():
    """主要驗證函數"""
    print("🔍 開始驗證 OPUI Socket.IO API 扁平化格式改造...")
    print("=" * 60)
    
    frontend_ok = check_frontend_api()
    print()
    
    backend_ok = check_backend_socket()
    print()
    
    doc_ok = check_documentation()
    print()
    
    print("=" * 60)
    if frontend_ok and backend_ok and doc_ok:
        print("🎉 所有檢查通過！扁平化格式改造成功完成。")
        print("\n📋 改造摘要:")
        print("  ✅ 前端 API 已統一使用扁平化格式")
        print("  ✅ 後端 Socket 處理器已統一使用扁平化格式")
        print("  ✅ 文檔已更新為扁平化格式指南")
        print("  ✅ 移除了分離式架構相關程式碼")
        print("  ✅ 版本號已更新為 3.0")
        return True
    else:
        print("❌ 部分檢查未通過，請檢查上述問題。")
        return False

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
