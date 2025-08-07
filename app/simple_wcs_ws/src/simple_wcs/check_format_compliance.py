#!/usr/bin/env python3
"""
檢查 flows 目錄中的檔案是否符合 FLOW_FORMAT_STANDARD.yaml 規範
"""

import yaml
from pathlib import Path
from typing import Dict, List, Tuple

def load_yaml(file_path: Path) -> Dict:
    """載入 YAML 檔案"""
    with open(file_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

def check_required_fields(data: Dict) -> Tuple[bool, List[str]]:
    """檢查必要欄位"""
    required_fields = ['name', 'nodes']  # 根據標準，這是最基本的必要欄位
    missing = []
    
    for field in required_fields:
        if field not in data:
            missing.append(field)
    
    return len(missing) == 0, missing

def check_optional_fields(data: Dict) -> List[str]:
    """檢查可選欄位"""
    optional_fields = [
        'description', 'version', 'author', 'enabled', 
        'priority', 'work_id', 'connections', 'flow_designer_data'
    ]
    present = []
    
    for field in optional_fields:
        if field in data:
            present.append(field)
    
    return present

def check_node_structure(nodes: List[Dict]) -> Tuple[bool, List[str]]:
    """檢查節點結構"""
    issues = []
    
    for i, node in enumerate(nodes):
        # 必要欄位
        if 'id' not in node:
            issues.append(f"節點 {i} 缺少 'id' 欄位")
        if 'type' not in node:
            issues.append(f"節點 {i} 缺少 'type' 欄位")
        if 'function' not in node:
            issues.append(f"節點 {i} 缺少 'function' 欄位")
        
        # 檢查類型
        if 'type' in node and node['type'] not in ['condition', 'action', 'logic']:
            issues.append(f"節點 {node.get('id', i)} 的類型 '{node['type']}' 不符合規範")
    
    return len(issues) == 0, issues

def check_connections_format(connections: List[Dict]) -> Tuple[bool, List[str]]:
    """檢查連接格式"""
    issues = []
    
    for i, conn in enumerate(connections):
        # 檢查標準格式 (from/to) 或舊格式 (source/target)
        has_standard = 'from' in conn and 'to' in conn
        has_old = 'source' in conn and 'target' in conn
        
        if not (has_standard or has_old):
            issues.append(f"連接 {i} 缺少必要的連接資訊")
    
    return len(issues) == 0, issues

def check_flow_designer_data(fd_data: Dict) -> Tuple[bool, List[str]]:
    """檢查 Flow Designer 資料格式"""
    issues = []
    
    if 'nodes' not in fd_data:
        issues.append("flow_designer_data 缺少 'nodes' 欄位")
    if 'connections' not in fd_data:
        issues.append("flow_designer_data 缺少 'connections' 欄位")
    
    # 檢查節點格式
    if 'nodes' in fd_data:
        for i, node in enumerate(fd_data['nodes']):
            if 'id' not in node:
                issues.append(f"視覺化節點 {i} 缺少 'id' 欄位")
            if 'position' not in node:
                issues.append(f"視覺化節點 {i} 缺少 'position' 欄位")
            elif not ('x' in node['position'] and 'y' in node['position']):
                issues.append(f"視覺化節點 {i} 的 position 缺少 x 或 y")
    
    return len(issues) == 0, issues

def analyze_file(file_path: Path):
    """分析單個檔案"""
    print(f"\n{'='*60}")
    print(f"📄 檔案: {file_path.name}")
    print(f"{'='*60}")
    
    try:
        data = load_yaml(file_path)
        
        # 1. 檢查必要欄位
        has_required, missing = check_required_fields(data)
        if has_required:
            print("✅ 必要欄位: 全部存在")
        else:
            print(f"❌ 缺少必要欄位: {', '.join(missing)}")
        
        # 2. 檢查可選欄位
        optional = check_optional_fields(data)
        print(f"📋 可選欄位: {', '.join(optional) if optional else '無'}")
        
        # 3. 檢查節點結構
        if 'nodes' in data:
            nodes_ok, node_issues = check_node_structure(data['nodes'])
            if nodes_ok:
                print(f"✅ 節點結構: 正確 (共 {len(data['nodes'])} 個節點)")
            else:
                print(f"❌ 節點結構問題:")
                for issue in node_issues[:5]:  # 只顯示前5個問題
                    print(f"   - {issue}")
                if len(node_issues) > 5:
                    print(f"   ... 還有 {len(node_issues)-5} 個問題")
        
        # 4. 檢查連接格式
        if 'connections' in data:
            conns_ok, conn_issues = check_connections_format(data['connections'])
            if conns_ok:
                print(f"✅ 連接格式: 正確 (共 {len(data['connections'])} 條連接)")
            else:
                print(f"⚠️ 連接格式問題:")
                for issue in conn_issues[:3]:
                    print(f"   - {issue}")
        else:
            print("ℹ️ 連接定義: 無")
        
        # 5. 檢查 Flow Designer 資料
        if 'flow_designer_data' in data:
            fd_ok, fd_issues = check_flow_designer_data(data['flow_designer_data'])
            if fd_ok:
                print("✅ Flow Designer 資料: 格式正確")
            else:
                print(f"⚠️ Flow Designer 資料問題:")
                for issue in fd_issues[:3]:
                    print(f"   - {issue}")
        else:
            print("ℹ️ Flow Designer 資料: 無 (可選)")
        
        # 6. 特殊欄位檢查
        if 'flow_config' in data:
            print("⚠️ 包含非標準欄位 'flow_config' (可能是擴展配置)")
        
        if 'steps' in data:
            print("ℹ️ 包含 'steps' 欄位 (執行步驟定義)")
        
        # 總體評估
        print("\n📊 總體評估:")
        if has_required and nodes_ok:
            if 'connections' in data and conns_ok:
                print("   ✅ 完全符合標準格式")
            else:
                print("   ⚠️ 基本符合標準，但連接定義可能有問題")
        else:
            print("   ❌ 不符合標準格式")
            
    except Exception as e:
        print(f"❌ 無法解析檔案: {e}")

def main():
    """主函數"""
    print("\n" + "="*70)
    print("🔍 Flow 檔案格式符合性檢查")
    print("標準參考: FLOW_FORMAT_STANDARD.yaml")
    print("="*70)
    
    flows_dir = Path("/home/ct/RosAGV/app/config/wcs/flows")
    yaml_files = list(flows_dir.glob("*.yaml"))
    
    print(f"\n找到 {len(yaml_files)} 個 YAML 檔案")
    
    for file_path in yaml_files:
        analyze_file(file_path)
    
    print("\n" + "="*70)
    print("📝 總結:")
    print("="*70)
    print("""
    根據 FLOW_FORMAT_STANDARD.yaml 標準：
    
    ✅ 符合標準的特徵:
    - 包含 'name' 欄位（必要）
    - 包含 'nodes' 欄位（必要）
    - 節點有 id, type, function 欄位
    - type 為 condition/action/logic 之一
    - 可選包含 connections 定義
    - 可選包含 flow_designer_data
    
    ⚠️ 格式差異:
    - test_agv_dispatch.yaml 使用了 'flow_config' 擴展配置
    - rack_rotation 檔案的連接格式可能使用舊格式
    - 有些節點使用 'name' 而非 'description'
    
    🔧 建議:
    1. 統一使用 'from/to' 格式定義連接
    2. 考慮將 'flow_config' 整合到頂層欄位
    3. 確保所有節點都有完整的 inputs/outputs 定義
    """)

if __name__ == "__main__":
    main()