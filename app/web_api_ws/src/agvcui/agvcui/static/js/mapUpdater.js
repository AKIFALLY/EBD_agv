
import {
    NodeObject,
    LineObject
} from '../objects/index.js';
// mapUpdater.js

export function MapChangehandler(newState, mapContext) {
    const {
        map,
        nodeObjects,
        edgeObjects,
        kukaNodeObjects,
        kukaEdgeObjects
    } = mapContext;

    const nodes = newState.nodes || [];
    const edges = newState.edges || [];
    const kukaNodes = newState.kukaNodes || [];
    const kukaEdges = newState.kukaEdges || [];

    // 從 store 讀取兩個獨立的佈林值
    const shouldShowCT = newState.showCtNodes !== undefined ? newState.showCtNodes : true;
    const shouldShowKUKA = newState.showKukaNodes !== undefined ? newState.showKukaNodes : true;

    // 清除 CT 節點和邊線（如果不應該顯示）
    if (!shouldShowCT) {
        clearNodes(nodeObjects);
        clearEdges(edgeObjects);
    } else {
        processNodes(nodes, nodeObjects, (node) => createNode(map, node, nodeObjects, "ct-node"), updateNode);
        processEdges(edges, edgeObjects, nodeObjects, (edge) => createEdge(map, edge, nodeObjects, edgeObjects, "ct-edge"), updateEdge);
    }

    // 清除 KUKA 節點和邊線（如果不應該顯示）
    if (!shouldShowKUKA) {
        clearNodes(kukaNodeObjects);
        clearEdges(kukaEdgeObjects);
    } else {
        processNodes(kukaNodes, kukaNodeObjects, (node) => createNode(map, node, kukaNodeObjects, "kuka-node"), updateNode);
        processEdges(kukaEdges, kukaEdgeObjects, kukaNodeObjects, (edge) => createEdge(map, edge, kukaNodeObjects, kukaEdgeObjects, "kuka-edge"), updateEdge);
    }
}

// === 共用邏輯 ===

function processNodes(nodes, objectMap, createFn, updateFn) {
    // Step 1: 收集新節點的 ID 集合
    const newNodeIds = new Set(nodes.map(n => n.id));

    // Step 2: 刪除不再存在的節點
    const nodesToDelete = [];
    objectMap.forEach((nodeObject, nodeId) => {
        if (!newNodeIds.has(nodeId)) {
            nodesToDelete.push(nodeId);
        }
    });

    nodesToDelete.forEach(nodeId => {
        const nodeObject = objectMap.get(nodeId);
        if (nodeObject && nodeObject.remove) {
            nodeObject.remove();
        }
        objectMap.delete(nodeId);
        console.log(`Removed deleted node: ${nodeId}`);
    });

    // Step 3: 更新或創建節點
    nodes.forEach(node => {
        if (objectMap.has(node.id)) {
            updateFn(node, objectMap);
        } else {
            createFn(node);
        }
    });
}

function processEdges(edges, edgeMap, nodeMap, createFn, updateFn) {
    // Step 1: 收集新邊線的名稱集合
    const newEdgeNames = new Set(edges.map(e => e.name));

    // Step 2: 刪除不再存在的邊線
    const edgesToDelete = [];
    edgeMap.forEach((lineObject, edgeName) => {
        if (!newEdgeNames.has(edgeName)) {
            edgesToDelete.push(edgeName);
        }
    });

    edgesToDelete.forEach(edgeName => {
        const lineObject = edgeMap.get(edgeName);
        if (lineObject && lineObject.remove) {
            lineObject.remove();
        }
        edgeMap.delete(edgeName);
        console.log(`Removed deleted edge: ${edgeName}`);
    });

    // Step 3: 更新或創建邊線
    edges.forEach(edge => {
        if (edgeMap.has(edge.name)) {
            updateFn(edge, nodeMap, edgeMap);
        } else {
            createFn(edge);
        }
    });
}

// === 清除函數 ===

function clearNodes(objectMap) {
    // 移除所有節點的 DOM 元素
    objectMap.forEach((nodeObject) => {
        if (nodeObject && nodeObject.remove) {
            nodeObject.remove();
        }
    });
    // 清空 Map
    objectMap.clear();
}

function clearEdges(edgeMap) {
    // 移除所有邊線的 DOM 元素（包括箭頭）
    edgeMap.forEach((lineObject) => {
        if (lineObject && lineObject.remove) {
            lineObject.remove();
        }
    });
    // 清空 Map
    edgeMap.clear();
}

// === Node 建立/更新 ===

function createNode(map, node, objectMap, className) {
    // x, y 已經是像素座標 (px)
    const latlng = new L.LatLng(node.y, node.x);

    let cssClassName = className;

    // ct-node (路徑節點) 使用 node.type (字串枚舉)
    if (className === 'ct-node') {
        // 使用 node.type (字串類型，如 'REST_AREA', 'CHARGING_AREA', 'TRANSPORT_POINT', 'NONE')
        const nodeType = node.type;
        // 依節點類型加入 CSS class
        if (nodeType && nodeType !== 'NONE') {
            cssClassName = `${className} node-type-${nodeType.toLowerCase()}`;
        }
    }
    // kuka-node (KUKA 節點) 使用 node_type_id (數字)
    else if (className === 'kuka-node') {
        // 使用 node_type_id (數字，如 2, 4, 6, 10)
        const nodeTypeId = node.node_type_id;
        if (nodeTypeId) {
            cssClassName = `${className} node-type-${nodeTypeId}`;
        }
    }

    const nodeObject = new NodeObject(map, latlng, node.id, cssClassName);
    objectMap.set(node.id, nodeObject);

    // 立即設置互動功能
    if (window.mapObjectManager) {
        window.mapObjectManager.setupNodeInteraction(nodeObject);
    }
}

function updateNode(node, objectMap) {
    const existing = objectMap.get(node.id);
    if (existing) {
        // x, y 已經是像素座標 (px)
        existing.updateLatLng(new L.LatLng(node.y, node.x));
    }
}

// === Edge 建立/更新 ===

function createEdge(map, edge, nodeMap, edgeMap, className = '') {
    const edgeName = edge.name;
    const from = nodeMap.get(edge.from_id)?.latlng;
    const to = nodeMap.get(edge.to_id)?.latlng;

    if (!from || !to) {
        console.warn(`無法建立 edge ${edgeName}，from(${edge.from_id}) 或 to(${edge.to_id}) 無座標`);
        return;
    }

    const line = new LineObject(map, from, to, {
        id: edgeName,
        ...(className && { className })
    });

    edgeMap.set(edgeName, line);
}

function updateEdge(edge, nodeMap, edgeMap) {
    const edgeName = edge.name;
    const from = nodeMap.get(edge.from_id)?.latlng;
    const to = nodeMap.get(edge.to_id)?.latlng;
    const line = edgeMap.get(edgeName);

    if (!line) {
        console.warn(`找不到 edge: ${edgeName}`);
        return;
    }

    line.updatePoints(from, to);
}
