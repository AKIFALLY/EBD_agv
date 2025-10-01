#!/usr/bin/env python3
"""
生成 docs-ai 文檔索引和引用統計（強引用與弱引用版本）
- 強引用：@docs-ai/ 開頭的引用（權重 1.0）
- 弱引用：docs-ai/ 開頭但無 @ 的引用（權重 0.3）
- 內部引用：文檔之間的互相引用關係

作者：AI Agent
更新日期：2025-09-19
"""

import json
import os
import re
from pathlib import Path
from collections import defaultdict
from datetime import datetime

# 基礎路徑配置
ROSAGV_ROOT = Path("/home/ct/RosAGV")
DOCS_AI_PATH = ROSAGV_ROOT / "docs-ai"
OUTPUT_PATH = ROSAGV_ROOT / "design/business-process-docs/js/docs-ai-index.json"

# 引用權重配置
STRONG_WEIGHT = 1.0  # 強引用權重
WEAK_WEIGHT = 0.3    # 弱引用權重

# 文檔分類定義
CATEGORIES = {
    "core": {
        "name": "🔥 核心原則",
        "description": "AI Agent 必讀文檔",
        "patterns": [
            "operations/development/core/",
            "operations/tools/unified-tools.md"
        ]
    },
    "architecture": {
        "name": "📚 系統架構",
        "description": "系統設計和架構",
        "patterns": [
            "context/system/",
            "context/workspaces/"
        ]
    },
    "testing": {
        "name": "🧪 測試標準",
        "description": "測試相關文檔",
        "patterns": [
            "operations/development/testing/"
        ]
    },
    "operations": {
        "name": "📖 操作指南",
        "description": "開發和運維指導",
        "patterns": [
            "operations/guides/",
            "operations/deployment/",
            "operations/development/"
        ]
    },
    "knowledge": {
        "name": "🏷️ 領域知識",
        "description": "業務和技術知識",
        "patterns": [
            "knowledge/"
        ]
    }
}

# 三層架構定義
LAYER_DEFINITIONS = {
    "layer1": {
        "name": "🌐 通用層",
        "description": "系統架構、核心原則、通用工具",
        "target": "根目錄 CLAUDE.md",
        "patterns": [
            # 系統架構（必須理解）
            "context/system/dual-environment.md",
            "context/system/rosagv-overview.md",
            "context/system/technology-stack.md",
            "context/system/language-configuration.md",
            # 核心開發原則（必須遵守）
            "operations/development/core/core-principles.md",
            "operations/development/core/linus-torvalds-ai-agent-principles.md",
            "operations/development/core/documentation-standards.md",
            # 通用工具與操作（日常使用）
            "operations/tools/unified-tools.md",
            "operations/development/docker-development.md",
            "operations/guides/system-diagnostics.md",
            "operations/guides/troubleshooting.md"
        ]
    },
    "layer2": {
        "name": "🔧 工作空間層",
        "description": "領域知識、開發流程、通用協議",
        "target": "*_ws/CLAUDE.md",
        "patterns": [
            # 工作空間架構
            "context/workspaces/",
            # 通用協議和介面
            "knowledge/protocols/ros2-interfaces.md",
            "knowledge/protocols/zenoh-rmw.md",
            # 開發流程
            "operations/development/ros2/ros2-development.md",
            "operations/development/testing/testing-standards.md",
            "operations/development/database-operations.md",
            # 領域知識（根據工作空間選擇性引用）
            "knowledge/agv-domain/",
            "knowledge/system/",
            "knowledge/protocols/",
            "knowledge/business/"
        ]
    },
    "layer3": {
        "name": "🔬 專業層",
        "description": "特定實作、專業細節、模組特定",
        "target": "src/*/CLAUDE.md",
        "patterns": [
            # 高度專業化文檔
            "knowledge/agv-domain/robot-pgno-rules.md",
            "knowledge/agv-domain/magic-value-analysis.md",
            "knowledge/agv-domain/write-path-state-analysis.md",
            "knowledge/system/tafl/",
            "operations/development/testing/ros2-pytest-testing.md"
        ]
    }
}

def determine_document_layer(doc_path):
    """根據文檔路徑判斷其應屬於哪個層級"""
    # 精確匹配優先
    for layer_key, layer_info in LAYER_DEFINITIONS.items():
        for pattern in layer_info["patterns"]:
            if pattern.endswith(".md"):
                # 精確檔案匹配
                if doc_path == pattern:
                    return layer_key
            elif pattern.endswith("/"):
                # 目錄匹配
                if doc_path.startswith(pattern):
                    return layer_key

    # 基於內容和路徑的通用規則
    # Layer 1: 通用系統級知識
    if any(p in doc_path for p in [
        "context/system/",
        "operations/development/core/",
        "operations/tools/unified-tools.md",
        "operations/guides/system-diagnostics.md",
        "operations/guides/troubleshooting.md"
    ]):
        return "layer1"

    # Layer 3: 高度專業化
    if any(p in doc_path for p in [
        "robot-pgno-rules",
        "magic-value-analysis",
        "write-path-state-analysis",
        "/tafl/tafl-",
        "ros2-pytest-testing"
    ]):
        return "layer3"

    # Layer 2: 工作空間級（默認）
    return "layer2"

def scan_docs_ai_files():
    """掃描所有 docs-ai 目錄下的 .md 文件"""
    docs = {}

    for md_file in DOCS_AI_PATH.rglob("*.md"):
        relative_path = md_file.relative_to(DOCS_AI_PATH)
        path_str = str(relative_path).replace('\\', '/')

        # 讀取文件標題和描述
        title = path_str
        description = ""

        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                lines = f.readlines()
                for line in lines[:10]:  # 只檢查前10行
                    if line.startswith("# "):
                        title = line[2:].strip()
                    elif line.startswith("## 🎯 適用場景"):
                        # 讀取下一行作為描述
                        idx = lines.index(line)
                        if idx + 1 < len(lines):
                            description = lines[idx + 1].strip().lstrip("- ")
                        break
        except:
            pass

        # 判斷分類
        category = "other"
        for cat_key, cat_info in CATEGORIES.items():
            for pattern in cat_info["patterns"]:
                if pattern in path_str:
                    category = cat_key
                    break
            if category != "other":
                break

        # 判斷層級
        layer = determine_document_layer(path_str)

        docs[path_str] = {
            "path": path_str,
            "title": title,
            "description": description[:100] if description else "",
            "category": category,
            "layer": layer,  # 新增：文檔層級

            # 引用統計（強引用與弱引用）
            "strong_references": 0,      # 強引用次數
            "weak_references": 0,         # 弱引用次數
            "total_weighted_refs": 0,     # 加權總分

            # 向後兼容欄位
            "references": 0,              # 總引用次數（強+弱）

            # 引用來源
            "strong_referenced_by": [],  # 強引用來源
            "weak_referenced_by": [],    # 弱引用來源
            "referenced_by": [],         # 所有引用來源（向後兼容）

            # 層級引用來源（新增）
            "layer_references": {
                "layer1": [],  # 被哪些根層級 CLAUDE.md 引用
                "layer2": [],  # 被哪些工作空間層級 CLAUDE.md 引用
                "layer3": []   # 被哪些專業層級 CLAUDE.md 引用
            },

            # 向外引用
            "references_to": [],          # 該文檔引用的其他文檔

            "size": md_file.stat().st_size,
            "modified": md_file.stat().st_mtime
        }

    return docs

def determine_claude_file_layer(claude_file_path):
    """根據 CLAUDE.md 檔案路徑判斷其層級"""
    path_str = str(claude_file_path)

    # Layer 1: 根目錄 CLAUDE.md
    if path_str == "CLAUDE.md":
        return "layer1"

    # Layer 2: 工作空間層級 (*_ws/CLAUDE.md)
    if "_ws/CLAUDE.md" in path_str and "/src/" not in path_str:
        return "layer2"

    # Layer 3: 專業模組層級 (src/*/CLAUDE.md)
    if "/src/" in path_str and "CLAUDE.md" in path_str:
        return "layer3"

    # 其他（如 README.md, .github/*.md）
    return "other"

def scan_claude_references():
    """掃描所有 CLAUDE.md 文件中的 @docs-ai 引用（區分強引用和弱引用）"""
    strong_references = defaultdict(list)
    weak_references = defaultdict(list)
    layer_references = defaultdict(lambda: defaultdict(list))  # 新增：層級引用追蹤

    # 收集所有需要掃描的檔案
    scan_files = []

    # 掃描根目錄的 CLAUDE.md 和 README.md
    scan_files.extend(list(ROSAGV_ROOT.glob("CLAUDE.md")))
    scan_files.extend(list(ROSAGV_ROOT.glob("README.md")))
    # 掃描所有工作空間的 CLAUDE.md
    scan_files.extend(ROSAGV_ROOT.glob("app/**/CLAUDE.md"))
    # 掃描 .github 中的指導文件
    scan_files.extend(ROSAGV_ROOT.glob(".github/*.md"))

    for scan_file in scan_files:
        relative_file = scan_file.relative_to(ROSAGV_ROOT)
        file_layer = determine_claude_file_layer(relative_file)

        try:
            with open(scan_file, 'r', encoding='utf-8') as f:
                content = f.read()

                # 強引用：@docs-ai/xxx
                strong_pattern = r'@docs-ai/([^\s\]]+\.md)'
                strong_matches = re.findall(strong_pattern, content)

                for match in strong_matches:
                    strong_references[match].append(str(relative_file))
                    # 記錄層級引用
                    if file_layer in ["layer1", "layer2", "layer3"]:
                        layer_references[match][file_layer].append(str(relative_file))

                # 弱引用：docs-ai/xxx （不包含 @ 符號）
                # 使用負向後顧斷言確保前面不是 @
                weak_pattern = r'(?<!@)docs-ai/([^\s\]]+\.md)'
                weak_matches = re.findall(weak_pattern, content)

                for match in weak_matches:
                    weak_references[match].append(str(relative_file))

        except Exception as e:
            print(f"警告：無法處理 {scan_file}: {e}")
            continue

    return strong_references, weak_references, layer_references

def scan_internal_references(docs):
    """掃描 docs-ai 文檔之間的互相引用（建立文檔網絡）"""
    internal_refs = defaultdict(lambda: {"strong": [], "weak": []})

    for doc_path, doc_info in docs.items():
        doc_file = DOCS_AI_PATH / doc_path

        try:
            with open(doc_file, 'r', encoding='utf-8') as f:
                content = f.read()

                # 掃描該文檔引用的其他文檔（強引用）
                strong_pattern = r'@docs-ai/([^\s\]]+\.md)'
                strong_matches = re.findall(strong_pattern, content)

                for match in strong_matches:
                    if match in docs and match != doc_path:
                        # 記錄：doc_path 引用了 match（強引用）
                        doc_info["references_to"].append({"doc": match, "type": "strong"})
                        internal_refs[match]["strong"].append(doc_path)

                # 掃描該文檔引用的其他文檔（弱引用）
                weak_pattern = r'(?<!@)docs-ai/([^\s\]]+\.md)'
                weak_matches = re.findall(weak_pattern, content)

                for match in weak_matches:
                    if match in docs and match != doc_path:
                        # 記錄：doc_path 引用了 match（弱引用）
                        doc_info["references_to"].append({"doc": match, "type": "weak"})
                        internal_refs[match]["weak"].append(doc_path)

        except Exception as e:
            print(f"警告：無法掃描內部引用 {doc_file}: {e}")
            continue

    return internal_refs

def calculate_weighted_score(strong_refs, weak_refs):
    """計算加權引用分數"""
    return strong_refs * STRONG_WEIGHT + weak_refs * WEAK_WEIGHT

def categorize_importance_v2(strong_refs, weak_refs):
    """基於加權分數的重要性分類"""
    score = calculate_weighted_score(strong_refs, weak_refs)

    if score >= 10:
        return "critical"
    elif score >= 5:
        return "important"
    elif score >= 2:
        return "common"
    elif score > 0:
        return "referenced"
    else:
        return "unreferenced"

def generate_index():
    """生成完整的索引文件"""
    print("🔍 掃描 docs-ai 文件...")
    docs = scan_docs_ai_files()

    print("📊 掃描 CLAUDE.md 引用...")
    strong_references, weak_references, layer_references = scan_claude_references()

    print("🕸️ 掃描文檔內部引用...")
    internal_refs = scan_internal_references(docs)

    # 更新引用計數（來自 CLAUDE.md 和其他外部檔案）
    for doc_path, ref_list in strong_references.items():
        if doc_path in docs:
            docs[doc_path]["strong_references"] = len(ref_list)
            docs[doc_path]["strong_referenced_by"] = ref_list

    for doc_path, ref_list in weak_references.items():
        if doc_path in docs:
            docs[doc_path]["weak_references"] = len(ref_list)
            docs[doc_path]["weak_referenced_by"] = ref_list

    # 更新層級引用信息
    for doc_path, layer_refs in layer_references.items():
        if doc_path in docs:
            docs[doc_path]["layer_references"] = dict(layer_refs)

    # 更新內部引用計數（來自其他 docs-ai 文檔）
    for doc_path, refs in internal_refs.items():
        if doc_path in docs:
            # 加入內部強引用
            docs[doc_path]["strong_references"] += len(refs["strong"])
            docs[doc_path]["strong_referenced_by"].extend(
                [f"docs-ai/{ref}" for ref in refs["strong"]]
            )

            # 加入內部弱引用
            docs[doc_path]["weak_references"] += len(refs["weak"])
            docs[doc_path]["weak_referenced_by"].extend(
                [f"docs-ai/{ref}" for ref in refs["weak"]]
            )

    # 計算加權分數和重要性，並設置向後兼容欄位
    for doc_path, doc_info in docs.items():
        # 計算加權總分
        doc_info["total_weighted_refs"] = calculate_weighted_score(
            doc_info["strong_references"],
            doc_info["weak_references"]
        )

        # 設置重要性等級
        doc_info["importance"] = categorize_importance_v2(
            doc_info["strong_references"],
            doc_info["weak_references"]
        )

        # 向後兼容：總引用次數
        doc_info["references"] = doc_info["strong_references"] + doc_info["weak_references"]

        # 向後兼容：所有引用來源
        doc_info["referenced_by"] = list(set(
            doc_info["strong_referenced_by"] + doc_info["weak_referenced_by"]
        ))

    # 統計信息（增強版）
    stats = {
        "total_docs": len(docs),

        # 引用統計
        "total_strong_refs": sum(d["strong_references"] for d in docs.values()),
        "total_weak_refs": sum(d["weak_references"] for d in docs.values()),
        "total_references": sum(d["references"] for d in docs.values()),

        # 平均引用
        "avg_strong_refs": round(
            sum(d["strong_references"] for d in docs.values()) / len(docs) if docs else 0,
            2
        ),
        "avg_weak_refs": round(
            sum(d["weak_references"] for d in docs.values()) / len(docs) if docs else 0,
            2
        ),

        # 文檔分類統計
        "critical_docs": len([d for d in docs.values() if d["importance"] == "critical"]),
        "important_docs": len([d for d in docs.values() if d["importance"] == "important"]),
        "common_docs": len([d for d in docs.values() if d["importance"] == "common"]),
        "weakly_referenced_docs": len([
            d for d in docs.values()
            if d["weak_references"] > 0 and d["strong_references"] == 0
        ]),
        "unreferenced_docs": len([d for d in docs.values() if d["importance"] == "unreferenced"]),

        # 三層架構統計（新增）
        "layer_distribution": {
            "layer1": {
                "name": LAYER_DEFINITIONS["layer1"]["name"],
                "description": LAYER_DEFINITIONS["layer1"]["description"],
                "count": len([d for d in docs.values() if d.get("layer") == "layer1"]),
                "docs": [d["path"] for d in docs.values() if d.get("layer") == "layer1"][:5]
            },
            "layer2": {
                "name": LAYER_DEFINITIONS["layer2"]["name"],
                "description": LAYER_DEFINITIONS["layer2"]["description"],
                "count": len([d for d in docs.values() if d.get("layer") == "layer2"]),
                "docs": [d["path"] for d in docs.values() if d.get("layer") == "layer2"][:5]
            },
            "layer3": {
                "name": LAYER_DEFINITIONS["layer3"]["name"],
                "description": LAYER_DEFINITIONS["layer3"]["description"],
                "count": len([d for d in docs.values() if d.get("layer") == "layer3"]),
                "docs": [d["path"] for d in docs.values() if d.get("layer") == "layer3"][:5]
            }
        },

        # 內部引用網絡統計
        "internal_network_edges": sum(len(d["references_to"]) for d in docs.values()),

        # 分類統計
        "categories": {
            cat_key: {
                "name": cat_info["name"],
                "description": cat_info["description"],
                "count": len([d for d in docs.values() if d["category"] == cat_key])
            }
            for cat_key, cat_info in CATEGORIES.items()
        },

        "generated_at": datetime.now().isoformat(),
        "generated_by": "generate-docs-ai-index.py v3.0"  # 升級版本號
    }

    # 構建最終輸出
    output = {
        "version": "3.0",  # 版本標記（升級以支援三層架構）
        "stats": stats,
        "categories": CATEGORIES,
        "layer_definitions": LAYER_DEFINITIONS,  # 新增：三層架構定義
        "documents": docs,

        # 排行榜（基於加權分數）
        "top_referenced": sorted(
            [d for d in docs.values() if d["total_weighted_refs"] > 0],
            key=lambda x: x["total_weighted_refs"],
            reverse=True
        )[:10],

        # 只有強引用的文檔
        "strong_only": sorted(
            [d for d in docs.values() if d["strong_references"] > 0 and d["weak_references"] == 0],
            key=lambda x: x["strong_references"],
            reverse=True
        )[:5],

        # 只有弱引用的文檔
        "weak_only": sorted(
            [d for d in docs.values() if d["weak_references"] > 0 and d["strong_references"] == 0],
            key=lambda x: x["weak_references"],
            reverse=True
        )[:5],

        # 三層架構專用排行榜（新增）
        "layer_top_docs": {
            "layer1": sorted(
                [d for d in docs.values() if d.get("layer") == "layer1"],
                key=lambda x: x["total_weighted_refs"],
                reverse=True
            )[:5],
            "layer2": sorted(
                [d for d in docs.values() if d.get("layer") == "layer2"],
                key=lambda x: x["total_weighted_refs"],
                reverse=True
            )[:5],
            "layer3": sorted(
                [d for d in docs.values() if d.get("layer") == "layer3"],
                key=lambda x: x["total_weighted_refs"],
                reverse=True
            )[:5]
        }
    }

    return output

def main():
    """主函數"""
    print("=" * 60)
    print("📚 RosAGV docs-ai 索引生成器 v3.0")
    print("   支援強引用與弱引用區分")
    print("   支援三層文檔架構分析")
    print("=" * 60)

    # 生成索引
    index = generate_index()

    # 確保輸出目錄存在
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)

    # 寫入 JSON 文件
    with open(OUTPUT_PATH, 'w', encoding='utf-8') as f:
        json.dump(index, f, ensure_ascii=False, indent=2)

    print(f"\n✅ 索引生成成功！")
    print(f"📁 輸出檔案: {OUTPUT_PATH}")
    print(f"\n📊 統計信息:")
    print(f"  - 總文檔數: {index['stats']['total_docs']}")
    print(f"  - 強引用總數: {index['stats']['total_strong_refs']}")
    print(f"  - 弱引用總數: {index['stats']['total_weak_refs']}")
    print(f"  - 總引用數: {index['stats']['total_references']}")
    print(f"  - 平均強引用: {index['stats']['avg_strong_refs']}")
    print(f"  - 平均弱引用: {index['stats']['avg_weak_refs']}")
    print(f"  - 關鍵文檔: {index['stats']['critical_docs']}")
    print(f"  - 重要文檔: {index['stats']['important_docs']}")
    print(f"  - 只有弱引用文檔: {index['stats']['weakly_referenced_docs']}")
    print(f"  - 未引用文檔: {index['stats']['unreferenced_docs']}")
    print(f"  - 內部引用邊數: {index['stats']['internal_network_edges']}")

    print(f"\n📚 三層架構分佈:")
    for layer_key, layer_info in index['stats']['layer_distribution'].items():
        print(f"  {layer_info['name']}: {layer_info['count']} 個文檔")
        if layer_info.get('docs'):
            for doc_path in layer_info['docs'][:3]:
                print(f"    - {doc_path}")

    print(f"\n🔥 最常被引用的文檔（基於加權分數）:")
    for i, doc in enumerate(index["top_referenced"][:5], 1):
        layer_name = LAYER_DEFINITIONS.get(doc.get('layer', 'layer2'), {}).get('name', '未分層')
        print(f"  {i}. {doc['title']} [{layer_name}]")
        print(f"     路徑: {doc['path']}")
        print(f"     強引用: {doc['strong_references']} | 弱引用: {doc['weak_references']}")
        print(f"     加權分數: {doc['total_weighted_refs']:.1f}")

    # 顯示只有弱引用的文檔（這些可能需要提升為強引用）
    if index.get("weak_only"):
        print(f"\n⚠️ 只有弱引用的文檔（可能需要提升為強引用）:")
        for doc in index["weak_only"][:3]:
            print(f"  - {doc['title']} ({doc['weak_references']} 次弱引用)")

if __name__ == "__main__":
    main()