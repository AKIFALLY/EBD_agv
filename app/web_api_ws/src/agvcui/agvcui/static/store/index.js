// ✅ indexStore.js
import { createStore } from './miniStore.js';

const mapStore = createStore('mapState', {
    mapFile: '/static/alan-demo-map.drawio.svg',
    mapFileBounds: [[0, 0], [3010, 5720]],//原圖(1505,2860)縮放比是 5cm = 1px , 圖放大2倍(3010,5720) (2.5cm = 1px) 
    mapInitBounds: [[1005, 2660], [1805, 3260]],//Room2 [[1005, 2660], [1805, 3260]]
    nodes: [],//CtNode
    edges: [],//CtEdge
    kukaNodes: [],//KukaNode節點
    kukaEdges: [],//KukaEdge
    agvs: []
});

const machinesStore = createStore('machinesState', {
    machines: [
        { id: 1, enable: 1 },
        { id: 2, enable: 1 },
        { id: 3, enable: 0 },
        { id: 4, enable: 0 }
    ]
});

const roomsStore = createStore('roomsState', {
    rooms: [
        { id: 1, enable: 1 },
        { id: 2, enable: 1 },
        { id: 3, enable: 0 },
        { id: 4, enable: 0 },
        { id: 5, enable: 0 }
    ]
});

const signalsStore = createStore('signalsState', {
    signals: [
        { id: 1, value: "True", type_of_value: "bool" },
        { id: 2, value: "False", type_of_value: "bool" },
        { id: 3, value: "False", type_of_value: "bool" },
        { id: 4, value: "False", type_of_value: "bool" },
        { id: 5, value: "1", type_of_value: "int" },
        { id: 6, value: "-1", type_of_value: "int" },
        { id: 7, value: "1", type_of_value: "int" },
        { id: 8, value: "-1", type_of_value: "int" }
    ]
});
const racksStore = createStore('racksState', {
    racks: []
});
const carriersStore = createStore('carriersState', {
    carriers: [
        { id: 1, room_id: 2, rack_id: 123, port_id: null, rack_index: 17, status_id: null },
        { id: 2, room_id: 2, rack_id: 123, port_id: null, rack_index: 18, status_id: null },
        { id: 3, room_id: 2, rack_id: 123, port_id: null, rack_index: 19, status_id: null },
        { id: 4, room_id: 2, rack_id: 123, port_id: null, rack_index: 20, status_id: null },
        { id: 5, room_id: 2, rack_id: 123, port_id: null, rack_index: 21, status_id: null },
        { id: 6, room_id: 2, rack_id: 123, port_id: null, rack_index: 22, status_id: null },
        { id: 7, room_id: 2, rack_id: 123, port_id: null, rack_index: 1, status_id: null },
        { id: 8, room_id: 2, rack_id: 123, port_id: null, rack_index: 2, status_id: null }
    ]
});

const tasksStore = createStore('tasksState', {
    tasks: []
});

const agvsStore = createStore('agvsState', {
    agvs: []
});

const userStore = createStore('userState', {
    id: null,
    username: null,
    role: null,
    full_name: null,
    is_active: false,
    isLoggedIn: false,
    isConnected: false
});

// ✅ 統一 export 所有 store
export {
    mapStore,
    machinesStore,
    roomsStore,
    signalsStore,
    racksStore,
    carriersStore,
    tasksStore,
    agvsStore,
    userStore,
};

// 🌐 全域暴露 stores 供其他模組使用
window.mapStore = mapStore;
window.machinesStore = machinesStore;
window.roomsStore = roomsStore;
window.signalsStore = signalsStore;
window.racksStore = racksStore;
window.carriersStore = carriersStore;
window.tasksStore = tasksStore;
window.agvsStore = agvsStore;
window.userStore = userStore;
