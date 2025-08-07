"""
從 WCS Functions 產生節點定義
自動產生 Flow Designer 可用的節點定義
"""

import sys
import json
import yaml
from pathlib import Path

# 添加 simple_wcs 到路徑
sys.path.append('/app/simple_wcs_ws/src/simple_wcs')

from simple_wcs.wcs_functions import WCSFunctions, FUNCTION_REGISTRY
from simple_wcs.dsl_function_registry import WCS_FUNCTION_MAPPINGS


def get_node_icon(func_name, node_type):
    """根據函數名稱和類型返回合適的圖標"""
    # 特定函數的圖標 - 使用專業的文字符號
    specific_icons = {
        # 條件檢查類
        'check_rack_status': '◉',
        'check_agv_available': '◈',
        'check_battery_level': '⚡',
        'check_location_status': '◎',
        'check_pending_tasks': '☰',
        'check_a_side_full': 'Ⓐ',
        'check_b_side_full': 'Ⓑ',
        
        # 動作執行類
        'dispatch_agv': '▶',
        'create_task': '⊕',
        'update_rack_status': '↻',
        'rotate_rack': '⟲',
        'move_agv': '→',
        'wait_for_completion': '⏳',
        'send_notification': '◈',
        
        # 邏輯運算類
        'and_gate': '∧',
        'or_gate': '∨',
        'not_gate': '¬',
        
        # 其他特定功能
        'get_room_outlet_locations': '⊡',
        'get_room_inlet_locations': '⊞',
        'can_dispatch': '◆',
    }
    
    if func_name in specific_icons:
        return specific_icons[func_name]
    
    # 根據類型返回默認圖標 - 專業符號
    type_icons = {
        'condition': '◇',  # 菱形表示判斷
        'action': '▷',     # 三角形表示執行
        'logic': '⊙',      # 圓圈表示邏輯
        'script': '{ }'    # 大括號表示腳本
    }
    return type_icons.get(node_type, '■')

def get_node_color(node_type):
    """根據節點類型返回顏色"""
    type_colors = {
        'condition': '#3498db',  # 藍色
        'action': '#27ae60',     # 綠色
        'logic': '#e74c3c',      # 紅色
        'script': '#f39c12'      # 橙色
    }
    return type_colors.get(node_type, '#95a5a6')

def generate_node_definitions():
    """從 WCS Functions 產生節點定義"""
    
    # 創建 WCS Functions 實例以獲取函數資訊
    wcs = WCSFunctions()
    
    nodes = {}
    
    # 從 FUNCTION_REGISTRY 產生節點
    for func_name, method_name in FUNCTION_REGISTRY.items():
        if hasattr(wcs, method_name):
            method = getattr(wcs, method_name)
            
            # 檢查函數類型
            node_type = 'action'  # 預設
            if 'check_' in func_name:
                node_type = 'condition'
            elif func_name in ['and_gate', 'or_gate', 'not_gate']:
                node_type = 'logic'
            
            # 分析函數參數
            import inspect
            sig = inspect.signature(method)
            params = {}
            for param_name, param in sig.parameters.items():
                if param_name not in ['self', 'kwargs']:
                    param_type = 'string'  # 預設類型
                    if param.annotation != inspect.Parameter.empty:
                        if param.annotation == bool:
                            param_type = 'boolean'
                        elif param.annotation == int:
                            param_type = 'integer'
                        elif param.annotation == float:
                            param_type = 'float'
                    params[param_name] = param_type
            
            # 產生節點定義
            node_def = {
                'name': func_name.replace('_', ' ').title(),
                'type': node_type,
                'function': func_name,
                'category': 'WCS Functions',
                'description': method.__doc__.strip() if method.__doc__ else f'{func_name} function',
                'inputs': {},
                'outputs': {},
                'parameters': params,
                'icon': get_node_icon(func_name, node_type),
                'color': get_node_color(node_type)
            }
            
            # 根據節點類型設定輸入輸出
            if node_type == 'condition':
                node_def['outputs'] = {'result': 'boolean'}
            elif node_type == 'logic':
                if func_name == 'not_gate':
                    node_def['inputs'] = {'input': 'boolean'}
                else:
                    node_def['inputs'] = {'input1': 'boolean', 'input2': 'boolean'}
                node_def['outputs'] = {'result': 'boolean'}
            elif node_type == 'action':
                node_def['outputs'] = {'result': 'any'}
            
            nodes[func_name] = node_def
    
    # 添加腳本節點範例
    script_nodes = {
        'custom_calculation': {
            'name': 'Custom Calculation',
            'type': 'script',
            'function': 'custom_calculation',
            'category': 'WCS Functions',
            'description': '執行自定義數學計算（例如：複雜的成本計算、效率評估等）',
            'inputs': {
                'value1': {'type': 'float', 'description': '第一個數值'},
                'value2': {'type': 'float', 'description': '第二個數值'},
                'operation': {'type': 'string', 'description': '運算類型'}
            },
            'outputs': {
                'result': {'type': 'float', 'description': '計算結果'}
            },
            'parameters': {
                'formula': 'string'
            },
            'icon': 'Σ',
            'color': '#f39c12'
        },
        'data_transform': {
            'name': 'Data Transform',
            'type': 'script',
            'function': 'data_transform',
            'category': 'WCS Functions',
            'description': '轉換數據格式或結構（例如：JSON 轉換、資料映射等）',
            'inputs': {
                'input_data': {'type': 'any', 'description': '輸入資料'}
            },
            'outputs': {
                'output_data': {'type': 'any', 'description': '轉換後資料'}
            },
            'parameters': {
                'transform_type': 'string',
                'mapping': 'string'
            },
            'icon': '⟳',
            'color': '#f39c12'
        },
        'api_request': {
            'name': 'API Request',
            'type': 'script',
            'function': 'api_request',
            'category': 'WCS Functions',
            'description': '調用外部 API 或內部服務（例如：ERP 整合、MES 通訊等）',
            'inputs': {
                'trigger': {'type': 'any', 'description': '觸發信號'}
            },
            'outputs': {
                'response': {'type': 'object', 'description': 'API 回應'},
                'error': {'type': 'boolean', 'description': '錯誤狀態'}
            },
            'parameters': {
                'url': 'string',
                'method': 'string',
                'headers': 'string',
                'body': 'string'
            },
            'icon': '☁',
            'color': '#f39c12'
        },
        'log_message': {
            'name': 'Log Message',
            'type': 'script',
            'function': 'log_message',
            'category': 'WCS Functions',
            'description': '記錄日誌訊息到系統（用於除錯和追蹤）',
            'inputs': {
                'message': {'type': 'string', 'description': '日誌訊息'},
                'data': {'type': 'any', 'description': '附加資料'}
            },
            'outputs': {
                'logged': {'type': 'boolean', 'description': '記錄成功'}
            },
            'parameters': {
                'log_level': 'string',
                'log_file': 'string'
            },
            'icon': '▤',
            'color': '#f39c12'
        },
        'conditional_script': {
            'name': 'Conditional Script',
            'type': 'script',
            'function': 'conditional_script',
            'category': 'WCS Functions',
            'description': '根據條件執行不同的腳本邏輯（複雜的分支處理）',
            'inputs': {
                'condition': {'type': 'any', 'description': '條件值'},
                'data': {'type': 'any', 'description': '處理資料'}
            },
            'outputs': {
                'result': {'type': 'any', 'description': '執行結果'},
                'branch_taken': {'type': 'string', 'description': '執行的分支'}
            },
            'parameters': {
                'if_script': 'string',
                'else_script': 'string'
            },
            'icon': '⊨',
            'color': '#f39c12'
        }
    }
    
    # 合併腳本節點到節點定義中
    nodes.update(script_nodes)
    
    return nodes


def save_node_definitions(nodes, output_file='wcs_node_definitions.yaml'):
    """儲存節點定義到 YAML 檔案"""
    output_path = Path(f'/app/config/wcs/flows/{output_file}')
    
    # 建立文件結構
    doc = {
        'node_definitions': {
            'version': '1.0',
            'source': 'wcs_functions',
            'nodes': nodes
        }
    }
    
    with open(output_path, 'w', encoding='utf-8') as f:
        yaml.dump(doc, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
    
    print(f"節點定義已儲存到: {output_path}")
    return output_path


def update_flow_designer_api():
    """更新 Flow Designer API 以包含 WCS 節點"""
    nodes = generate_node_definitions()
    
    # 轉換為 Flow Designer 格式
    flow_designer_nodes = {}
    
    for key, node in nodes.items():
        # 使用函數名作為 key，保留字典格式的 inputs/outputs
        flow_designer_nodes[key] = {
            'name': node['name'],
            'type': node['type'],
            'function': node['function'],
            'category': node['category'],
            'description': node['description'],
            'inputs': node['inputs'],  # 保留字典格式
            'outputs': node['outputs'],  # 保留字典格式
            'parameters': node['parameters'],
            'icon': node.get('icon', '■'),  # 保留圖標
            'color': node.get('color', '#95a5a6')  # 保留顏色
        }
    
    return flow_designer_nodes


if __name__ == "__main__":
    # 產生節點定義
    nodes = generate_node_definitions()
    
    print(f"產生了 {len(nodes)} 個節點定義")
    
    # 顯示一些範例
    print("\n範例節點定義:")
    for i, (key, node) in enumerate(nodes.items()):
        if i < 3:
            print(f"\n{key}:")
            print(f"  名稱: {node['name']}")
            print(f"  類型: {node['type']}")
            print(f"  描述: {node['description'][:50]}...")
            print(f"  參數: {node['parameters']}")
    
    # 儲存到檔案
    save_node_definitions(nodes)
    
    # 產生 Flow Designer 格式
    fd_nodes = update_flow_designer_api()
    print(f"\n轉換為 Flow Designer 格式: {len(fd_nodes)} 個節點")