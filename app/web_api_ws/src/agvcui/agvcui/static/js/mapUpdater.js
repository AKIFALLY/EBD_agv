
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

    processNodes(nodes, nodeObjects, (node) => createNode(map, node, nodeObjects, "ct-node"), updateNode);
    processEdges(edges, edgeObjects, nodeObjects, (edge) => createEdge(map, edge, nodeObjects, edgeObjects), updateEdge);

    processNodes(kukaNodes, kukaNodeObjects, (node) => createNode(map, node, kukaNodeObjects, "kuka-node"), updateNode);
    processEdges(kukaEdges, kukaEdgeObjects, kukaNodeObjects, (edge) => createEdge(map, edge, kukaNodeObjects, kukaEdgeObjects, "kuka-edge"), updateEdge);
}

// === 共用邏輯 ===

function processNodes(nodes, objectMap, createFn, updateFn) {
    nodes.forEach(node => {
        if (objectMap.has(node.id)) {
            updateFn(node, objectMap);
        } else {
            createFn(node);
        }
    });
}

function processEdges(edges, edgeMap, nodeMap, createFn, updateFn) {
    edges.forEach(edge => {
        if (edgeMap.has(edge.name)) {
            updateFn(edge, nodeMap, edgeMap);
        } else {
            createFn(edge);
        }
    });
}

// === Node 建立/更新 ===

function createNode(map, node, objectMap, className) {
    const latlng = new L.LatLng(node.y, node.x);
    const nodeType = node.node_type_id;
    let cssClassName = className
    //依節點類型加入節點名稱
    if (nodeType !== null) {
        cssClassName = `${className} node-type-${nodeType}`;
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
