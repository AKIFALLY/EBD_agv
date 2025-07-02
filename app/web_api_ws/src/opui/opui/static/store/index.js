// ✅ indexStore.js
import { createStore } from './miniStore.js';

const parkingStore = createStore('parkingState', {
    parkingSpace: {
        left: [
            { id: 1, name: '001' },
            { id: 2, name: '002' }
        ],
        right: [
            { id: 3, name: '003' },
            { id: 4, name: '004' }
        ]
    }
});

const machinesStore = createStore('machinesState', {
    machines: [
        { id: 1, enable: 1 },
        { id: 2, enable: 1 },
        { id: 3, enable: 0 },
        { id: 4, enable: 0 }
    ]
});

const productsStore = createStore('productsState', {
    products: [
        { id: 1, name: 'NODATA', size: 'S' }
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

const clientStore = createStore('clientState', {
    op: {
        left: {
            productSelected: 0,
            product: [
                { name: '', size: 'S', id: null, count: 32, room: 2, rackId: null },
                { name: '', size: 'S', id: null, count: 32, room: 2, rackId: null }
            ]
        },
        right: {
            productSelected: 0,
            product: [
                { name: '', size: 'S', id: null, count: 32, room: 2, rackId: null },
                { name: '', size: 'S', id: null, count: 32, room: 2, rackId: null }
            ]
        }
    },
    machineId: 1,
    clientId: null,
});

const taskStore = createStore('taskState', {
    activeTasks: {
        left: null,   // { taskId: 123, type: 'call_empty', status: 'pending', createdAt: '2025-01-01T00:00:00Z' }
        right: null   // { taskId: 124, type: 'dispatch_full', status: 'pending', createdAt: '2025-01-01T00:00:00Z' }
    }
});

// ✅ 統一 export 所有 store
export {
    clientStore,
    parkingStore,
    machinesStore,
    productsStore,
    roomsStore,
    taskStore
};
