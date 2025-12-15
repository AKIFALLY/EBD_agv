#!/usr/bin/env python3
"""
生成 CLAUDE.md 架構層統計資訊
統計三層架構中 CLAUDE.md 檔案的分佈和引用關係

作者：AI Agent
創建日期：2025-09-30
"""

import json
import os
import re
from pathlib import Path
from collections import defaultdict
from datetime import datetime

# 基礎路徑配置
ROSAGV_ROOT = Path("/home/ct/EBD_agv")
AI_AGENTS_PATH = ROSAGV_ROOT / "ai-agents"
DOCS_AI_PATH = ROSAGV_ROOT / "docs-ai"
OUTPUT_PATH = ROSAGV_ROOT / "design/business-process-docs/js/claude-architecture.json"

def scan_ai_agents():
    """掃描 ai-agents 目錄下的規則文檔"""
    agents = []
    if AI_AGENTS_PATH.exists():
        for md_file in AI_AGENTS_PATH.glob("*.md"):
            agents.append({
                "file": md_file.name,
                "path": str(md_file.relative_to(ROSAGV_ROOT)),
                "size": md_file.stat().st_size,
                "title": extract_title(md_file)
            })
    return agents

def scan_root_claude():
    """掃描根目錄的 CLAUDE.md"""
    root_claude = ROSAGV_ROOT / "CLAUDE.md"
    if not root_claude.exists():
        return None

    # 提取 docs-ai 引用
    docs_ai_refs = extract_docs_ai_references(root_claude)

    return {
        "file": "CLAUDE.md",
        "path": "CLAUDE.md",
        "size": root_claude.stat().st_size,
        "lines": count_lines(root_claude),
        "docs_ai_references": len(docs_ai_refs),
        "referenced_docs": docs_ai_refs
    }

def scan_workspace_claude():
    """掃描工作空間層級的 CLAUDE.md (Layer 2)"""
    workspaces = []

    # 查找所有 *_ws/CLAUDE.md
    for claude_file in ROSAGV_ROOT.glob("app/**/CLAUDE.md"):
        relative_path = claude_file.relative_to(ROSAGV_ROOT)
        path_str = str(relative_path)

        # 只要工作空間層級（不包含 src/）
        if "_ws/CLAUDE.md" in path_str and "/src/" not in path_str:
            docs_ai_refs = extract_docs_ai_references(claude_file)

            # 提取工作空間名稱
            workspace_name = claude_file.parent.name

            workspaces.append({
                "workspace": workspace_name,
                "file": claude_file.name,
                "path": str(relative_path),
                "size": claude_file.stat().st_size,
                "lines": count_lines(claude_file),
                "docs_ai_references": len(docs_ai_refs),
                "referenced_docs": docs_ai_refs[:5]  # 只保留前5個
            })

    return sorted(workspaces, key=lambda x: x['workspace'])

def scan_specialized_claude():
    """掃描專業層級的 CLAUDE.md (Layer 3)"""
    specialized = []

    # 查找所有 src/*/CLAUDE.md
    for claude_file in ROSAGV_ROOT.glob("app/**/src/*/CLAUDE.md"):
        relative_path = claude_file.relative_to(ROSAGV_ROOT)
        path_str = str(relative_path)

        docs_ai_refs = extract_docs_ai_references(claude_file)

        # 提取工作空間和模組名稱
        parts = Path(path_str).parts
        workspace_idx = None
        for i, part in enumerate(parts):
            if part.endswith("_ws"):
                workspace_idx = i
                break

        workspace_name = parts[workspace_idx] if workspace_idx else "unknown"
        module_name = claude_file.parent.name

        specialized.append({
            "workspace": workspace_name,
            "module": module_name,
            "file": claude_file.name,
            "path": str(relative_path),
            "size": claude_file.stat().st_size,
            "lines": count_lines(claude_file),
            "docs_ai_references": len(docs_ai_refs),
            "referenced_docs": docs_ai_refs[:3]  # 只保留前3個
        })

    return sorted(specialized, key=lambda x: (x['workspace'], x['module']))

def extract_title(md_file):
    """從 Markdown 檔案提取標題"""
    try:
        with open(md_file, 'r', encoding='utf-8') as f:
            for line in f:
                if line.startswith("# "):
                    return line[2:].strip()
    except:
        pass
    return md_file.stem

def count_lines(file_path):
    """計算檔案行數"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            return len(f.readlines())
    except:
        return 0

def extract_docs_ai_references(claude_file):
    """從 CLAUDE.md 提取 docs-ai 引用（包含強引用和弱引用）"""
    refs = []
    try:
        with open(claude_file, 'r', encoding='utf-8') as f:
            content = f.read()

            # 強引用：@docs-ai/xxx
            strong_pattern = r'@docs-ai/([^\s\]]+\.md)'
            strong_matches = re.findall(strong_pattern, content)
            refs.extend(strong_matches)

            # 弱引用：docs-ai/xxx（不包含 @ 符號）
            # 使用負向後顧斷言確保前面不是 @
            weak_pattern = r'(?<!@)docs-ai/([^\s\]#]+\.md)'
            weak_matches = re.findall(weak_pattern, content)
            refs.extend(weak_matches)
    except:
        pass

    return list(set(refs))  # 去重

def calculate_cross_references(docs_ai_index_path):
    """計算 CLAUDE ↔ docs-ai 的交叉引用關係"""
    cross_refs = {
        "claude_to_docs": {},
        "docs_to_claude": {}
    }

    # 如果 docs-ai-index.json 存在，讀取並分析
    if docs_ai_index_path.exists():
        try:
            with open(docs_ai_index_path, 'r', encoding='utf-8') as f:
                docs_index = json.load(f)

            # 從 docs-ai 索引提取被哪些 CLAUDE 引用
            for doc_path, doc_info in docs_index.get('documents', {}).items():
                claude_refs = []
                for ref in doc_info.get('strong_referenced_by', []):
                    if 'CLAUDE.md' in ref:
                        claude_refs.append(ref)

                if claude_refs:
                    cross_refs["docs_to_claude"][doc_path] = claude_refs
        except:
            pass

    return cross_refs

def generate_stats():
    """生成完整的架構統計"""
    print("🔍 掃描 AI Agent 規則文檔...")
    ai_agents = scan_ai_agents()

    print("📄 掃描根目錄 CLAUDE.md...")
    root_claude = scan_root_claude()

    print("🔧 掃描工作空間層 CLAUDE.md...")
    workspace_claude = scan_workspace_claude()

    print("🔬 掃描專業層 CLAUDE.md...")
    specialized_claude = scan_specialized_claude()

    print("🔗 計算交叉引用...")
    docs_ai_index_path = ROSAGV_ROOT / "design/business-process-docs/js/docs-ai-index.json"
    cross_refs = calculate_cross_references(docs_ai_index_path)

    # 構建統計資料
    stats = {
        "version": "1.0",
        "generated_at": datetime.now().isoformat(),
        "generated_by": "generate-claude-architecture-stats.py v1.0",

        "summary": {
            "total_claude_files": 1 + len(workspace_claude) + len(specialized_claude),
            "layer1_components": {
                "ai_agents": len(ai_agents),
                "root_claude": 1 if root_claude else 0,
                "docs_ai_refs": root_claude.get('docs_ai_references', 0) if root_claude else 0,
                "total": len(ai_agents) + (1 if root_claude else 0)
            },
            "layer2_workspaces": len(workspace_claude),
            "layer3_modules": len(specialized_claude)
        },

        "architecture_layers": {
            "layer1": {
                "name": "🌐 通用層",
                "description": "AI Agent 核心規則與開發指導",
                "components": {
                    "ai_agents": {
                        "count": len(ai_agents),
                        "files": ai_agents
                    },
                    "root_claude": root_claude,
                    "total_components": len(ai_agents) + (1 if root_claude else 0)
                }
            },

            "layer2": {
                "name": "🔧 工作空間層",
                "description": "領域特定知識",
                "count": len(workspace_claude),
                "workspaces": workspace_claude,
                "workspace_names": [ws['workspace'] for ws in workspace_claude]
            },

            "layer3": {
                "name": "🔬 專業層",
                "description": "模組實作細節",
                "count": len(specialized_claude),
                "modules": specialized_claude,
                "grouped_by_workspace": group_by_workspace(specialized_claude)
            }
        },

        "cross_references": cross_refs,

        "file_lists": {
            "all_claude_files": [
                "CLAUDE.md"
            ] + [ws['path'] for ws in workspace_claude] + [sp['path'] for sp in specialized_claude]
        }
    }

    return stats

def group_by_workspace(specialized_modules):
    """將專業層模組按工作空間分組"""
    grouped = defaultdict(list)
    for module in specialized_modules:
        grouped[module['workspace']].append(module['module'])
    return dict(grouped)

def main():
    """主函數"""
    print("=" * 60)
    print("🏗️ RosAGV CLAUDE 架構統計生成器 v1.0")
    print("   統計三層架構中的 CLAUDE.md 分佈")
    print("=" * 60)

    # 生成統計
    stats = generate_stats()

    # 確保輸出目錄存在
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)

    # 寫入 JSON 文件
    with open(OUTPUT_PATH, 'w', encoding='utf-8') as f:
        json.dump(stats, f, ensure_ascii=False, indent=2)

    print(f"\n✅ 統計生成成功！")
    print(f"📁 輸出檔案: {OUTPUT_PATH}")

    # 顯示統計摘要
    summary = stats['summary']
    print(f"\n📊 統計摘要:")
    print(f"  🌐 Layer 1 (通用層):")
    print(f"     - AI Agent 規則: {summary['layer1_components']['ai_agents']} 個")
    print(f"     - 根 CLAUDE.md: {summary['layer1_components']['root_claude']} 個")
    print(f"     - 引用 docs-ai: {summary['layer1_components']['docs_ai_refs']} 個")
    print(f"     - 總計: {summary['layer1_components']['total']} 個組件")

    print(f"\n  🔧 Layer 2 (工作空間層):")
    print(f"     - 工作空間 CLAUDE.md: {summary['layer2_workspaces']} 個")

    print(f"\n  🔬 Layer 3 (專業層):")
    print(f"     - 專業模組 CLAUDE.md: {summary['layer3_modules']} 個")

    print(f"\n  📄 CLAUDE.md 總計: {summary['total_claude_files']} 個")

    # 顯示工作空間列表
    print(f"\n📦 工作空間列表 (Layer 2):")
    for workspace in stats['architecture_layers']['layer2']['workspace_names'][:10]:
        print(f"     - {workspace}")
    if len(stats['architecture_layers']['layer2']['workspace_names']) > 10:
        print(f"     ... 還有 {len(stats['architecture_layers']['layer2']['workspace_names']) - 10} 個")

if __name__ == "__main__":
    main()