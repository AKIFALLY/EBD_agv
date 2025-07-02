import { mapStore, signalsStore, roomsStore, machinesStore, racksStore, carriersStore, tasksStore } from '../store/index.js';
import { notify } from './notify.js';

// AGV 動畫配置（簡化版 + 目標點平滑）
const AGV_ANIMATION_CONFIG = {
    mode: 'smooth',             // 'instant' 或 'smooth'
    lerpSpeed: 4.0,             // 插值速度
    useTargetSmoothing: true,   // 啟用目標點平滑
    targetSmoothSpeed: 4.0      // 目標點平滑速度
};

// 動畫模式說明：
// instant: 立即移動，無動畫，最佳響應性
// smooth:  平滑動畫，包含目標平滑，消除頓挫感 (推薦)



// 速度係數說明：
// 8.0-12.0:  標準速度，平衡流暢度和響應性 (推薦)
// 15.0-20.0: 快速響應，接近即時但保持平滑
// 5.0-8.0:   較慢，適合觀察細節

// 緩衝區大小說明 (僅 multi-point 模式)：
// 3: 最小緩衝，較快響應但平滑度一般
// 5: 推薦設定，平衡響應性和平滑度
// 7: 最大緩衝，最平滑但響應較慢
import {
    RotatingMovingObject,
    TransferBoxObject,
    CleanerPortsObject,
    SoakingPortsObject,
    DryerPortsObject,
    OvenPortsObject,
    EqpInfoObject,
    RackInfoObject,
    DockedRackInfoObject,
    NodeObject,
    LineObject
} from '../objects/index.js';
import { MapChangehandler } from './mapUpdater.js';
import { mapInteraction } from './mapInteraction.js';
import { mapObjectManager } from './mapObjectManager.js';
import { mapTaskManager } from './mapTaskManager.js';
import { mapDataSync } from './mapDataSync.js';
import { mapPerformanceMonitor } from './mapPerformanceMonitor.js';

export const mapPage = (() => {

    const eqpObjects = [];//所有地圖上的設備物件
    const agvObjects = new Map();//所有地圖上的 agv 物件
    const rackObjects = new Map();//所有地圖上的 rack 物件
    const dockedRackObjects = new Map(); //所有停靠區的racks物件
    const nodeObjects = new Map();//所有地圖上的 node 物件
    const edgeObjects = new Map();//所有地圖上的 edge 物件
    const kukaNodeObjects = new Map();//所有地圖上的 KUKA node 物件
    const kukaEdgeObjects = new Map();//所有地圖上的 KUKA node 物件
    const eqpInfoCountObjects = [];//所有地圖上的 eqpInfoCount 物件
    // nodePositions 預設你已經在別處定義了，是 Map，裡面存 node id => { latlng: L.LatLng, ... }


    function handleMapChange(newState) {
        const agvs = newState.agvs || [];//所有地圖上的 agv 物件
        console.debug('agvs:', agvs);
        agvs.forEach(agv => {
            // 如果有 id，就更新該物件
            if (agv.id && agvObjects.has(agv.id)) {
                //console.log("Updating existing AGV:", agv);
                const agvObject = agvObjects.get(agv.id);
                const latLng = L.latLng(agv.y, agv.x);
                //console.log("Updating existing AGV:", latLng, agv.heading);
                agvObject.setTargetPosition(latLng, agv.heading);
            } else {
                console.log(agv);
                console.log(`No object found for name: ${agv.name} , add one`);
                // 沒有 id，就新增該物件
                const agvName = agv.name || "agv001";
                const latLng = L.latLng(agv.y, agv.x);
                const className = 'agv-' + agvName.toLowerCase().match(/[a-z]+/g)?.join('') || '';
                const newAgvObject = new RotatingMovingObject(map, latLng, agvName, className);
                newAgvObject.id = agv.id;
                newAgvObject.agvId = agv.id; // 添加資料庫 ID
                newAgvObject.setData({ agvId: agv.id, name: agv.name }); // 設置資料

                // 設定動畫模式
                newAgvObject.setAnimationMode(
                    AGV_ANIMATION_CONFIG.mode,
                    AGV_ANIMATION_CONFIG.lerpSpeed
                );

                // 設定目標點平滑
                newAgvObject.setTargetSmoothing(
                    AGV_ANIMATION_CONFIG.useTargetSmoothing,
                    AGV_ANIMATION_CONFIG.targetSmoothSpeed
                );

                newAgvObject.setTargetPosition(latLng, agv.heading);
                agvObjects.set(agv.id, newAgvObject);

                // 立即設置互動功能
                if (window.mapObjectManager) {
                    console.log('Setting up interaction for AGV:', agv.id, 'name:', agv.name, 'className:', className);
                    mapObjectManager.setupAgvInteraction(newAgvObject);
                    console.log('AGV interaction setup completed for:', agv.id);

                    // 驗證點擊處理器是否正確設置
                    setTimeout(() => {
                        console.log('AGV', agv.id, 'click handlers count:', newAgvObject.clickHandlers?.length || 0);
                    }, 100);
                } else {
                    console.warn('mapObjectManager not available for AGV:', agv.id);
                }
            }
        });

        const mapContext = {
            map, // Leaflet 地圖物件
            nodeObjects,
            edgeObjects,
            kukaNodeObjects,
            kukaEdgeObjects,
        };
        // 假設你收到新狀態資料時
        MapChangehandler(newState, mapContext);

    }


    function handleRoomsChange(newState) {
        if (!newState?.rooms) return;
        const rooms = newState.rooms || [];
        //console.debug('rooms:', rooms);
    }
    function handleMachinesChange(newState) {
        if (!newState?.machines) return;
        const machines = newState.machines || [];
        console.debug('machines:', machines);
    }
    function handleSignalsChange(newState) {
        if (!newState?.signals) return;

        const signals = newState.signals || [];
        //製作訊號字典(用以加速查詢)
        const signalMap = buildSignalMap(signals);
        console.log('signalMap:', signalMap);

        eqpObjects.forEach(eqp => eqp.updateSignals(signalMap));
    }

    function updateCarriedRack(rack) {
        const agv = agvObjects.get(rack.agv_id);
        if (!agv) {
            console.warn(`Cannot find agv_id for rack ${rack.id} with agv_id ${rack.agv_id}`);
            return;
        }
        const latLng = agv.latlng;
        updateRackObject(rack, latLng, agv);
    }



    function updateRackObject(rack, latLng, agv = null) {
        let rackObject = rackObjects.get(rack.id);
        if (rackObject) {
            // Rack exists, update it
            rackObject.update(rack);
            rackObject.updateLatLng(latLng);

            const wasOnAgv = !!rackObject.attachedAgvId;
            const isOnAgv = !!agv;

            if (wasOnAgv && !isOnAgv) {
                // Moved from AGV to Node
                const oldAgv = agvObjects.get(rackObject.attachedAgvId);
                if (oldAgv) oldAgv.detachObject();
                rackObject.attachedAgvId = null;
            } else if (!wasOnAgv && isOnAgv) {
                // Moved from Node to AGV
                agv.attachObject(rackObject);
                rackObject.attachedAgvId = agv.id;
            } else if (wasOnAgv && isOnAgv && rackObject.attachedAgvId !== agv.id) {
                // Moved from one AGV to another
                const oldAgv = agvObjects.get(rackObject.attachedAgvId);
                if (oldAgv) oldAgv.detachObject();
                agv.attachObject(rackObject);
                rackObject.attachedAgvId = agv.id;
            }
        } else {
            // New rack
            rackObject = new RackInfoObject(map, latLng, rack.id, rack.name);
            rackObject.update(rack);
            rackObjects.set(rack.id, rackObject);

            // 立即設置互動功能
            if (window.mapObjectManager) {
                mapObjectManager.setupRackInteraction(rackObject);
                console.log('Rack interaction setup for:', rack.id);
            } else {
                console.warn('mapObjectManager not available for Rack:', rack.id);
            }

            if (agv) {
                agv.attachObject(rackObject);
                rackObject.attachedAgvId = agv.id;
            }
        }
    }

    function cleanupRemovedRacks(receivedRackIds) {
        rackObjects.forEach((rackObject, rackId) => {
            if (!receivedRackIds.has(rackId)) {
                if (rackObject.attachedAgvId) {
                    const agv = agvObjects.get(rackObject.attachedAgvId);
                    if (agv) agv.detachObject();
                }
                rackObject.remove();
                rackObjects.delete(rackId);
            }
        });
    }

    function updateDockedRackInfo(dockedRacks, kukaNodesMap) {
        const racksByLocation = new Map();
        dockedRacks.forEach(rack => {
            if (!racksByLocation.has(rack.location_id)) {
                racksByLocation.set(rack.location_id, []);
            }
            racksByLocation.get(rack.location_id).push(rack);
        });

        const handledNodeIds = new Set();

        racksByLocation.forEach((racks, locationId) => {
            const node = kukaNodesMap.get(locationId);
            if (!node) {
                console.warn(`Cannot find node_id ${locationId} for docked racks`);
                return;
            }
            handledNodeIds.add(locationId);

            let dockedRackObject = dockedRackObjects.get(locationId);
            if (!dockedRackObject) {
                const latLng = L.latLng(node.y, node.x);
                dockedRackObject = new DockedRackInfoObject(map, latLng, node.id, node.name);
                dockedRackObjects.set(locationId, dockedRackObject);
            }
            dockedRackObject.update(racks);

            // 如果這些 rack 之前是獨立的 RackObject，要移除
            racks.forEach(rack => {
                if (rackObjects.has(rack.id)) {
                    const rackObject = rackObjects.get(rack.id);
                    if (rackObject.attachedAgvId) {
                        const agv = agvObjects.get(rackObject.attachedAgvId);
                        if (agv) agv.detachObject();
                    }
                    rackObject.remove();
                    rackObjects.delete(rack.id);
                }
            });
        });

        // 清理不再有 docked racks 的節點
        dockedRackObjects.forEach((_, locationId) => {
            if (!handledNodeIds.has(locationId)) {
                const dockedRackObject = dockedRackObjects.get(locationId);
                dockedRackObject.remove();
                dockedRackObjects.delete(locationId);
            }
        });
    }


    function handleRacksChange(newState) {
        if (!newState?.racks) return;

        const allRacks = newState.racks || [];
        console.debug('allRacks:', allRacks);
        const { kukaNodes } = mapStore.getState();
        const kukaNodesMap = new Map(kukaNodes.map(node => [node.id, node]));
        const receivedRackIds = new Set(allRacks.map(r => r.id));

        const carriedRacks = allRacks.filter(r => r.is_carry === 1);
        const dockedRacks = allRacks.filter(r => r.is_docked === 1);
        const stationaryRacks = allRacks.filter(r => r.is_carry !== 1 && r.is_docked !== 1);

        // 1. 處理在 AGV 上的 Racks
        carriedRacks.forEach(rack => {
            updateCarriedRack(rack);
        });

        // 2. 處理停靠的 Racks
        updateDockedRackInfo(dockedRacks, kukaNodesMap);

        // 3. 處理靜置的 Racks
        stationaryRacks.forEach(rack => {
            const node = kukaNodesMap.get(rack.location_id);
            if (node) {
                const latLng = L.latLng(node.y, node.x);
                updateRackObject(rack, latLng);
            } else {
                console.warn(`Cannot find node_id for stationary rack ${rack.id} with location_id ${rack.location_id}`);
            }
        });


        // 4. 清理伺服器上已不存在的 Rack
        cleanupRemovedRacks(receivedRackIds);
    }

    function handleCarriersChange(newState) {
        if (!newState?.carriers) return;
        const carriers = newState.carriers || [];

        // 根據 eqpInfoCountObjects 的 id 來更新對應的 counter
        eqpInfoCountObjects.forEach(infoObj => {
            const id = infoObj.id.split('-')[1]
            const count = carriers.filter(c => c.port_id && String(c.port_id).startsWith(id)).length;
            infoObj.updateEqpCount(count);
        });
    }

    function handleTasksChange(newState) {
        if (!newState?.tasks) return;
        const tasks = newState.tasks || [];
        console.debug('tasks:', tasks);

        // 更新任務管理器資料
        if (window.mapTaskManager) {
            window.mapTaskManager.loadTaskData();
        }
    }

    function buildSignalMap(signals) {
        const map = new Map();
        const doorKeywords = ["InnerTop", "InnerBottom", "OuterTop", "OuterBottom"];
        signals.forEach(s => {
            const name = s.name;
            if (name.endsWith("Presence") || name.endsWith("Load") || name.endsWith("Unload")) {
                let index = name.split('_')[2].replace('Port0', '');
                let key = `${s.eqp_id}_${index}_${name.split('_').pop()}`;
                map.set(key, s); // ✅ 正確寫法
            } else {
                const match = doorKeywords.find(k => name.includes(k));
                if (match) {
                    const key = `${s.eqp_id}_${match}`;
                    map.set(key, s); // ✅ 正確寫法
                }
            }
        });
        return map;
    }

    let map = {};
    // mapSetup 裡新增點擊事件：
    function mapSetup() {
        const mapState = mapStore.getState();
        const mapFile = mapState.mapFile;

        map = L.map("map", {
            crs: L.CRS.Simple,
            zoomSnap: 0.25,
            zoomDelta: 0.25,
            minZoom: -2,
            maxZoom: 2,
            attributionControl: false,
            fullscreenControl: true,
            fullscreenControlOptions: { position: "topleft" },


            boxZoom: false, // ❌ 關閉 Shift + 拖曳縮放
        });

        map.fitBounds(mapState.mapInitBounds);
        //gridLayer.addTo(map);  // 將 gridLayer 加到地圖

        //map.on("load", () => drawGrid(map));
        //map.on("moveend", () => drawGrid(map));
        const gridOverlay = document.querySelector(".grid-overlay");

        map.on("zoom", () => {
            const zoom = map.getZoom();
            const scale = map.getZoomScale(zoom, 0);  // or Math.pow(2, zoom)
            gridOverlay.style.backgroundSize = `${40 * scale}px ${40 * scale}px`;

            console.log(zoom);
            const newWeight = 3 + zoom;         // 你可以改成更合適的計算
            const newArrowSize = 3 * (3 + zoom);  // 箭頭大小也跟著縮放
            edgeObjects.forEach(edge => {
                edge.updateWeight(newWeight);
                edge.updateArrowSize(newArrowSize);
            });

        });
        //右鍵選單事件
        //map.on('contextmenu', function (e) {
        //    const menu = document.getElementById('map-context-menu');
        //    menu.style.left = `${e.originalEvent.pageX}px`;
        //    menu.style.top = `${e.originalEvent.pageY}px`;
        //    menu.style.display = 'block';
        //
        //    // 暫存點選位置
        //    window._contextLatLng = e.latlng;
        //});

        //// 點其他地方關閉選單
        //map.on('click', () => {
        //    document.getElementById('map-context-menu').style.display = 'none';
        //});
        //
        //// 範例操作
        //function handleMenu(action) {
        //    const latlng = window._contextLatLng;
        //    if (!latlng) return;
        //
        //    if (action === 'add-marker') {
        //        L.marker(latlng).addTo(map);
        //    } else if (action === 'zoom-in') {
        //        map.setZoom(map.getZoom() + 1);
        //    } else if (action === 'center') {
        //        map.panTo(latlng);
        //    }
        //
        //    document.getElementById('map-context-menu').style.display = 'none';
        //}

        map.on("move", () => {
            const { x, y } = map.getPixelBounds().min;
            gridOverlay.style.backgroundPosition = `${-x}px ${-y}px`;
        });

        //地圖檔載入
        L.imageOverlay(mapFile, mapState.mapFileBounds).addTo(map);


        // 初始化 網格線
        const { x, y } = map.getPixelBounds().min;
        gridOverlay.style.backgroundPosition = `${-x}px ${-y}px`;
        const zoom = map.getZoom();
        const scale = map.getZoomScale(zoom, 0);  // or Math.pow(2, zoom)
        gridOverlay.style.backgroundSize = `${40 * scale}px ${40 * scale}px`;
        const rotatingObjects = [];

        //// 建立 rotating object
        //const obj = new RotatingObject(map, [752, 1450], "mdi mdi-navigation", "blue");
        //rotatingObjects.push(obj);
        //
        //// 建立 transferbox object
        //const room2_transferbox_in = new TransferBoxObject(map, [800, 1620], "room2_in");
        //const room2_transferbox_out = new TransferBoxObject(map, [800, 1340], "room2_out");
        //const room2_cleaner = new CleanerPortsObject(map, [550, 1620], "room2_cleaner");
        //const room2_dryer = new DryerPortsObject(map, [550, 1520], "room2_dryer");

        //const kuka_agv001 = new RotatingMovingObject(map, L.latLng(1640, 2880), "agv_kuka001", "kuka-agv");
        //kuka_agv001.setTargetPosition(L.latLng(1640, 2860));
        //
        //const kuka_agv002 = new RotatingMovingObject(map, L.latLng(1740, 2880), "agv_kuka001", "kuka-agv");
        //kuka_agv002.setTargetPosition(L.latLng(1740, 2860));

        const agv = new RotatingMovingObject(map, L.latLng(1680, 2640), "agv_cargo01", "agv-cargo");

        // 設定動畫模式
        agv.setAnimationMode(
            AGV_ANIMATION_CONFIG.mode,
            AGV_ANIMATION_CONFIG.lerpSpeed
        );

        // 設定目標點平滑
        agv.setTargetSmoothing(
            AGV_ANIMATION_CONFIG.useTargetSmoothing,
            AGV_ANIMATION_CONFIG.targetSmoothSpeed
        );

        agv.setTargetPosition(L.latLng(1680, 2660));

        //const agv_cargo02 = new RotatingMovingObject(map, L.latLng(1680, 3270), "agv_cargo02", "agv-cargo");
        //agv_cargo02.setTargetPosition(L.latLng(1680, 3260));
        //const agv_loader02 = new RotatingMovingObject(map, L.latLng(1320, 3260), "agv_loader02", "agv-loader");
        //agv_loader02.setTargetPosition(L.latLng(1120, 3260));
        //const agv_unloader02 = new RotatingMovingObject(map, L.latLng(1320, 2660), "agv_unloader02", "agv-unloader");
        //agv_unloader02.setTargetPosition(L.latLng(1120, 2660));

        const room2TransferboxIn = new TransferBoxObject(map, L.latLng(1560, 3260), "201");
        const room2TransferboxOut = new TransferBoxObject(map, L.latLng(1560, 2660), "202");
        const room2Cleaner = new CleanerPortsObject(map, L.latLng(1020, 3260), "203");
        const room2Soaking = new SoakingPortsObject(map, L.latLng(1540, 2960), "204");
        const room2Dryer = new DryerPortsObject(map, L.latLng(1240, 2960), "205");
        const room2Oven = new OvenPortsObject(map, L.latLng(1020, 2660), "206");

        eqpObjects.push(room2TransferboxIn);
        eqpObjects.push(room2TransferboxOut);
        eqpObjects.push(room2Cleaner);
        eqpObjects.push(room2Soaking);
        eqpObjects.push(room2Dryer);
        eqpObjects.push(room2Oven);

        const room2TransferboxInInfo = new EqpInfoObject(map, L.latLng(1560, 3380), "201", "TransferboxIn");
        const room2TransferboxOutInfo = new EqpInfoObject(map, L.latLng(1560, 2540), "202", "TransferboxOut");
        const room2SoakingInfo = new EqpInfoObject(map, L.latLng(1580, 2960), "204", "Soaking");
        const room2DryerInfo = new EqpInfoObject(map, L.latLng(1120, 2960), "205", "Dryer");
        const room2CleanerInfo = new EqpInfoObject(map, L.latLng(920, 3260), "203", "Cleaner", true);//with counter
        const room2OvenInfo = new EqpInfoObject(map, L.latLng(920, 2660), "206", "Oven", true);//with counter

        eqpInfoCountObjects.push(room2CleanerInfo);
        eqpInfoCountObjects.push(room2OvenInfo);


        // 點擊地圖時，移動並旋轉物件
        map.on("click", e => {

            const clickLatLng = e.latlng;
            console.log("clickLatLng", clickLatLng)
            agv.setTargetPosition(clickLatLng);

            // 點地圖顯藏物件
            const panel = document.getElementById('info-panel');
            if (!panel) return;
            //if (!panel.classList.contains("hidden")) {
            //    panel.classList.add('hidden');
            //    return;
            //}

        });

        // 動畫循環
        //let lastTime = null;
        //function animate(time) {
        //    if (!lastTime) lastTime = time;
        //    const delta = (time - lastTime) / 1000;
        //    lastTime = time;
        //
        //    requestAnimationFrame(animate);
        //}
        //requestAnimationFrame(animate);


    }

    function updateAgvs(map, agvData) {
        // 將 agvData 畫到地圖上
        for (let id in agvData) {
            const agv = agvData[id];
            // 這裡可繼續建立 Leaflet Marker，略
        }
    }

    function updateEqps(map, eqpData) {
        for (let id in eqpData) {
            const eqp = eqpData[id];
            if (!eqp.show) continue;
            // 建立 eqp 的 Marker，略
        }
    }
    function setup() {
        // 開始效能測量
        mapPerformanceMonitor.init();
        mapPerformanceMonitor.startMeasure('map-setup');

        mapSetup();

        // 初始化地圖互動功能
        mapInteraction.init(map);
        mapObjectManager.init();

        // 確保管理器在全域可用
        window.mapObjectManager = mapObjectManager;
        window.mapTaskManager = mapTaskManager;

        // 初始化資料同步
        mapDataSync.init();

        // 為現有物件設置互動功能
        setupObjectInteractions();

        // 結束效能測量
        mapPerformanceMonitor.endMeasure('map-setup');

        console.log('Map page setup completed');

        // 測試點擊事件設置
        setTimeout(() => {
            console.log('Testing click handlers...');
            console.log('AGV objects count:', agvObjects.size);
            console.log('Rack objects count:', rackObjects.size);
            console.log('Node objects count:', nodeObjects.size);

            // 測試第一個 AGV 的點擊處理器
            if (agvObjects.size > 0) {
                const firstAgv = agvObjects.values().next().value;
                console.log('First AGV click handlers:', firstAgv.clickHandlers?.length || 0);
            }
        }, 2000);

        //console.log(parseFloat(getComputedStyle(document.documentElement).fontSize))

        //bindNumButtonEvents();
        //bindProductBtnEvents();
        //bindCallEmptyBtnEvents();
        //bindDispatchFullBtnEvents();
        //bindRoomBtnEvents();
        //bindRackSelectedEvents(); // 只需 setup 時綁一次
        // 可根據 store 狀態初始化 UI
        //roomsStore.on('change', handleRoomsChange);
        //
        ////第一次打開時更新 測試用
        mapStore.on('change', handleMapChange);
        roomsStore.on('change', handleRoomsChange);
        machinesStore.on('change', handleMachinesChange);
        signalsStore.on('change', handleSignalsChange);
        racksStore.on('change', handleRacksChange);
        carriersStore.on('change', handleCarriersChange);
        tasksStore.on('change', handleTasksChange);




        let isShiftDown = false;
        let isDragging = false;
        let startLatLng = null;
        let tempMarker = null;

        document.addEventListener('keydown', (e) => {
            console.log(e.key);
            if (e.key === 'Shift') {
                isShiftDown = true;
                //map.dragging.enable(); // 啟用拖曳
                map.dragging.disable(); // 禁用拖曳
            }
        });

        document.addEventListener('keyup', (e) => {
            console.log(e.key);
            if (e.key === 'Shift') {
                isShiftDown = false;
                map.dragging.enable(); // 啟用拖曳
                //map.dragging.disable(); // 禁用拖曳
            }
        });

        map.on('mousedown', (e) => {
            if (!isShiftDown) return;

            isDragging = true;
            startLatLng = e.latlng;
            console.log(e.latlng);
            // Optional: show a visual feedback
            tempMarker = L.circle(startLatLng, {
                radius: 5,
                color: 'red'
            }).addTo(map);
        });

        map.on('mousemove', (e) => {
            if (isDragging && tempMarker) {
                const newLatLng = e.latlng;
                tempMarker.setLatLng(newLatLng);
            }
        });

        map.on('mouseup', (e) => {
            if (!isDragging) return;

            isDragging = false;

            // Finalize or remove temporary marker
            if (tempMarker) {
                map.removeLayer(tempMarker);
                tempMarker = null;
            }

            console.log('Dragged from:', startLatLng, 'to:', e.latlng);
        });




        // ✅ 加入 Polygon 畫圖功能
        //let polygonPoints = [];
        //let polygonLayer = null;
        //
        //map.on('click', (e) => {
        //
        //    const latlng = e.latlng;
        //    polygonPoints.push(latlng);
        //
        //    L.circleMarker(latlng, {
        //        radius: 4,
        //        color: 'red'
        //    }).addTo(map);
        //
        //    if (polygonPoints.length >= 3) {
        //        if (polygonLayer) map.removeLayer(polygonLayer);
        //        polygonLayer = L.polygon(polygonPoints, {
        //            color: 'blue',
        //            fillOpacity: 0.4
        //        }).addTo(map);
        //    }
        //});
        //
        //window.resetPolygon = () => {
        //    polygonPoints = [];
        //    if (polygonLayer) {
        //        map.removeLayer(polygonLayer);
        //        polygonLayer = null;
        //    }
        //    map.eachLayer(layer => {
        //        if (layer instanceof L.CircleMarker) {
        //            map.removeLayer(layer);
        //        }
        //    });
        //};
        //
        //window.savePolygon = () => {
        //    const latlngs = polygonPoints.map(p => [p.lat, p.lng]);
        //    console.log("📝 Polygon saved:", JSON.stringify(latlngs));
        //    alert("Polygon saved. Check console.");
        //};
        //document.getElementById("resetPolygonBtn").addEventListener("click", () => {
        //    resetPolygon();
        //});
        //
        //document.getElementById("savePolygonBtn").addEventListener("click", () => {
        //    savePolygon();
        //});

    }

    // 為現有物件設置互動功能
    function setupObjectInteractions() {
        // 為設備物件設置互動
        eqpObjects.forEach(eqpObject => {
            mapObjectManager.setupEquipmentInteraction(eqpObject);
        });

        // 為 AGV 物件設置互動
        agvObjects.forEach(agvObject => {
            mapObjectManager.setupAgvInteraction(agvObject);
        });

        // 為貨架物件設置互動
        rackObjects.forEach(rackObject => {
            mapObjectManager.setupRackInteraction(rackObject);
        });

        // 為節點物件設置互動
        nodeObjects.forEach(nodeObject => {
            mapObjectManager.setupNodeInteraction(nodeObject);
        });

        kukaNodeObjects.forEach(nodeObject => {
            mapObjectManager.setupNodeInteraction(nodeObject);
        });
    }

    return {
        setup,
        setupObjectInteractions,
    };
})();
