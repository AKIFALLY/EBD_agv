// ✅ indexStore.js
import { createStore } from './miniStore.js';

const agvStore = createStore('agvState', { agvId: "agv001" });

// ✅ 統一 export 所有 store
export {
    agvStore
};
