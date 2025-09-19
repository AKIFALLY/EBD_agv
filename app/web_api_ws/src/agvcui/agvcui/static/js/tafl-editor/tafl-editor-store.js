/**
 * TAFL Editor Store - 基於 miniStore 的狀態管理
 * 管理 TAFL Editor 的所有狀態和事件
 */

import { createStore } from '../../store/miniStore.js';

class TAFLFlowStore {
    constructor() {
        // Add unique identifier to track store instance
        this.storeId = `store_${Date.now()}_${Math.random()}`;
        
        // TAFL v1.1 預設流程結構
        const defaultFlow = {
            metadata: {
                id: null,
                name: 'Untitled Flow',
                version: '1.1',
                description: '',
                enabled: true  // Default to enabled for new flows
            },
            settings: {
                execution_interval: 5
            },
            preload: {},
            rules: {},
            variables: {},
            flow: []
        };
        
        // 創建底層 miniStore
        this.baseStore = createStore('taflEditor', {
            currentFlow: defaultFlow,
            selectedCardId: null,
            isDirty: false,
            lastSaved: null,
            verbDefinitions: {},
            cardCounter: 0
        });
        
        // Fix cardCounter and reset flow after loading from localStorage
        const state = this.baseStore.getState();
        let updatedState = { ...state };
        
        // Fix cardCounter if invalid
        if (typeof state.cardCounter !== 'number' || isNaN(state.cardCounter)) {
            updatedState.cardCounter = 0;
        }
        
        // Validate and clean up flow.flow array
        if (state.currentFlow && state.currentFlow.flow) {
            // Filter out invalid cards (cards without any verb)
            const validFlow = state.currentFlow.flow.filter(card => {
                if (!card || typeof card !== 'object') return false;
                // Check if card has at least one valid verb key
                const hasVerb = Object.keys(card).some(key => 
                    key !== 'id' && key !== 'comment' && key !== '_id'
                );
                return hasVerb;
            });
            
            if (validFlow.length !== state.currentFlow.flow.length) {
                updatedState.currentFlow = {
                    ...state.currentFlow,
                    flow: validFlow
                };
                this.baseStore.setState(updatedState);
            }
        }
        
        // DON'T auto-load old flow on initialization - comment out to preserve loaded flows
        // If you want to always start with an empty flow, uncomment these lines:
        // updatedState.currentFlow = defaultFlow;
        // updatedState.selectedCardId = null;
        // updatedState.isDirty = false;
        // this.baseStore.setState(updatedState);
        
        // 額外的事件監聽器 (用於細粒度事件)
        this.listeners = new Map();
        
        // 歷史記錄管理 - Linus 式簡潔設計
        this.history = [];
        this.historyIndex = -1;
        this.maxHistorySize = 50;  // 避免記憶體爆炸
        this.isApplyingHistory = false;  // 防止循環
        
        // 訂閱基礎 store 變更
        this.baseStore.on('change', (state) => {
            this.emit('state:changed', state);
        });
    }
    
    // ========== 狀態獲取 ==========
    
    getState() {
        return this.baseStore.getState();
    }
    
    getFlow() {
        const state = this.baseStore.getState();
        return state.currentFlow;
    }
    
    getSelectedCardId() {
        return this.baseStore.getState().selectedCardId;
    }
    
    isDirty() {
        return this.baseStore.getState().isDirty;
    }
    
    getVerbDefinitions() {
        return this.baseStore.getState().verbDefinitions;
    }
    
    // ========== Flow 操作 ==========
    
    updateFlow(updates) {
        const state = this.baseStore.getState();
        const newFlow = {
            ...state.currentFlow,
            ...updates
        };
        this.baseStore.setState({
            ...state,
            currentFlow: newFlow,
            isDirty: true
        });
        this.emit('flow:changed', newFlow);
        this.emit('dirty:changed', true);
    }
    
    updateMetadata(metadata) {
        const state = this.baseStore.getState();
        const newFlow = {
            ...state.currentFlow,
            metadata: {
                ...state.currentFlow.metadata,
                ...metadata
            }
        };
        this.baseStore.setState({
            ...state,
            currentFlow: newFlow,
            isDirty: true
        });
        this.emit('metadata:changed', newFlow.metadata);
        this.emit('dirty:changed', true);
    }
    
    updateVariables(variables) {
        const state = this.baseStore.getState();
        const newFlow = {
            ...state.currentFlow,
            variables: variables
        };
        this.baseStore.setState({
            ...state,
            currentFlow: newFlow,
            isDirty: true
        });
        this.emit('variables:changed', variables);
        this.emit('dirty:changed', true);
    }
    
    loadFlow(flowData) {
        
        this.baseStore.setState({
            ...this.baseStore.getState(),
            currentFlow: flowData,
            isDirty: false,
            lastSaved: new Date().toISOString()
        });
        
        const stateAfter = this.baseStore.getState();
        
        // Verify cards can be found by ID
        if (flowData.flow && flowData.flow.length > 0) {
            const firstCardId = flowData.flow[0].id;
            const foundCard = this.findCardById(firstCardId);
        }
        
        this.emit('flow:loaded', flowData);
    }
    
    clearFlow() {
        
        // Use the miniStore's clear() to properly clear localStorage
        // This resets to initial state and removes localStorage entry
        this.baseStore.clear();
        
        const newState = this.baseStore.getState();
        
        // Emit the cleared event
        this.emit('flow:cleared');
    }
    
    // ========== Card 操作 ==========
    
    generateId() {
        const state = this.baseStore.getState();
        // Ensure cardCounter is a valid number
        let currentCounter = state.cardCounter;
        if (typeof currentCounter !== 'number' || isNaN(currentCounter)) {
            currentCounter = 0;
        }
        const newCounter = currentCounter + 1;
        
        // Update the counter immediately to ensure uniqueness
        this.baseStore.setState({
            ...state,
            cardCounter: newCounter
        });
        
        // Return an object with both id and newCounter as expected by addCard
        // Use both counter and timestamp for uniqueness
        const id = `card_${newCounter}_${Date.now()}_${Math.random().toString(36).substr(2, 5)}`;
        return { id, newCounter };
    }
    
    addCard(verb, position = null, params = {}) {
        
        const state = this.baseStore.getState();
        const flow = state.currentFlow.flow;
        
        const { id: cardId, newCounter } = this.generateId();
        
        const newCard = {
            id: cardId,
            [verb]: params || {}
        };
        
        let newFlow;
        if (position && position.index !== undefined) {
            // 在特定位置插入
            newFlow = [...flow];
            newFlow.splice(position.index, 0, newCard);
        } else {
            // 添加到末尾
            newFlow = [...flow, newCard];
        }
        
        
        const updatedCurrentFlow = {
            ...state.currentFlow,
            flow: newFlow
        };
        
        
        const newState = {
            ...state,
            currentFlow: updatedCurrentFlow,
            isDirty: true,
            cardCounter: newCounter  // Update the counter atomically with the flow
        };
        
        
        // Debug: Check if setState actually updates localStorage
        const lsBefore = localStorage.getItem('taflEditor');
        
        this.baseStore.setState(newState);
        
        // 保存快照 - 必須在狀態改變後調用
        this.saveSnapshot(`Add ${verb} card`);
        
        const lsAfter = localStorage.getItem('taflEditor');
        
        // Verify the state was actually updated
        const verifyState = this.baseStore.getState();
        
        this.emit('card:added', {
            card: newCard,
            position: position || { index: newFlow.length - 1 }
        });
        
        return cardId;
    }
    
    updateCard(cardId, updates) {
        // 保存快照
        this.saveSnapshot(`Update card ${cardId}`);
        
        const state = this.baseStore.getState();
        const card = this.findCardById(cardId);
        if (!card) return;
        
        // 深度更新卡片
        const newFlow = this.updateCardInFlow(state.currentFlow.flow, cardId, updates);
        
        const updatedCurrentFlow = {
            ...state.currentFlow,
            flow: newFlow
        };
        
        this.baseStore.setState({
            ...state,
            currentFlow: updatedCurrentFlow,
            isDirty: true
        });
        
        // 保存快照 - 必須在狀態改變後調用
        this.saveSnapshot(`Update card ${cardId}`);
        
        this.emit('card:updated', {
            cardId,
            cardData: { ...card, ...updates }
        });
    }
    
    deleteCard(cardId) {
        
        const state = this.baseStore.getState();
        const newFlow = this.removeCardFromFlow(state.currentFlow.flow, cardId);
        
        const updatedCurrentFlow = {
            ...state.currentFlow,
            flow: newFlow
        };
        
        let newSelectedId = state.selectedCardId;
        if (state.selectedCardId === cardId) {
            newSelectedId = null;
        }
        
        this.baseStore.setState({
            ...state,
            currentFlow: updatedCurrentFlow,
            selectedCardId: newSelectedId,
            isDirty: true
        });
        
        // 保存快照 - 必須在狀態改變後調用
        this.saveSnapshot(`Delete card ${cardId}`);
        
        this.emit('card:deleted', cardId);
    }
    
    moveCard(cardId, newPosition) {
        const state = this.baseStore.getState();
        const card = this.findCardById(cardId);
        if (!card) {
            return;
        }
        
        // Deep clone the card to preserve all data (especially SET verb data)
        const cardToMove = JSON.parse(JSON.stringify(card));
        
        // Debug logging for SET cards
        const verb = Object.keys(cardToMove).find(k => !['id', 'comment'].includes(k));
        if (verb === 'set') {
        }
        
        // 先移除卡片
        let newFlow = this.removeCardFromFlow(state.currentFlow.flow, cardId);
        
        // 再插入到新位置
        if (newPosition.nested) {
            // 插入到嵌套位置
            newFlow = this.insertCardInNested(newFlow, cardToMove, newPosition);
        } else {
            // 插入到頂層位置
            newFlow.splice(newPosition.index, 0, cardToMove);
        }
        
        // Verify SET card data after move
        if (verb === 'set') {
            const movedCard = this.findCardByIdInFlow(newFlow, cardId);
            if (movedCard) {
            }
        }
        
        const updatedCurrentFlow = {
            ...state.currentFlow,
            flow: newFlow
        };
        
        this.baseStore.setState({
            ...state,
            currentFlow: updatedCurrentFlow,
            isDirty: true
        });
        
        // 保存快照 - 必須在狀態改變後調用
        this.saveSnapshot(`Move card ${cardId}`);
        
        this.emit('card:moved', {
            cardId,
            position: newPosition
        });
    }
    
    selectCard(cardId) {
        const state = this.baseStore.getState();
        
        const isSameCard = state.selectedCardId === cardId;
        if (isSameCard) {
        }
        
        // Only update selectedCardId, don't spread the entire state (which might be stale)
        this.baseStore.setState({
            selectedCardId: cardId
        });
        
        // 獲取卡片資料
        const cardData = this.findCardById(cardId);
        
        if (!cardData) {
            // 列出所有可用的卡片 ID
            const allCardIds = [];
            const collectIds = (cards) => {
                if (!Array.isArray(cards)) return;
                cards.forEach(card => {
                    if (card.id) allCardIds.push(card.id);
                    // Check nested structures
                    const verb = Object.keys(card).find(k => !['id', 'comment'].includes(k));
                    if (verb && card[verb]) {
                        const params = card[verb];
                        if (params.then) collectIds(params.then);
                        if (params.else) collectIds(params.else);
                        if (params.do) collectIds(params.do);
                        if (params.cases) {
                            params.cases.forEach(c => {
                                if (c.do) collectIds(c.do);
                            });
                        }
                        if (params.default) collectIds(params.default);
                    }
                });
            };
            collectIds(state.currentFlow.flow);
        }
        
        this.emit('card:selected', cardId);
        this.emit('selection:changed', { 
            cardId, 
            cardData 
        });
    }
    
    deselectCard() {
        const state = this.baseStore.getState();
        
        // 如果沒有選擇，不做任何事
        if (!state.selectedCardId) return;
        
        // 清除選擇
        // Only update selectedCardId, don't spread the entire state
        this.baseStore.setState({
            selectedCardId: null
        });
        
        // 發出事件
        this.emit('card:deselected');
        this.emit('selection:changed', { 
            cardId: null, 
            cardData: null 
        });
    }
    
    // ========== 屬性更新方法 ==========
    
    /**
     * 更新卡片的單一屬性
     * @param {string} cardId - 卡片ID
     * @param {string} propertyPath - 屬性路徑 (支援巢狀，如 "switch.cases.0.when")
     * @param {any} value - 新值
     */
    updateCardProperty(cardId, propertyPath, value) {
        const card = this.findCardById(cardId);
        if (!card) {
            return;
        }
        
        // 深拷貝卡片
        const updatedCard = JSON.parse(JSON.stringify(card));
        
        // 處理巢狀路徑
        const pathParts = propertyPath.split('.');
        let target = updatedCard;
        
        // 導航到目標屬性的父物件
        for (let i = 0; i < pathParts.length - 1; i++) {
            const part = pathParts[i];
            
            // 處理陣列索引
            if (!isNaN(parseInt(part))) {
                target = target[parseInt(part)];
            } else {
                if (!target[part]) {
                    target[part] = {};
                }
                target = target[part];
            }
        }
        
        // 設定最終值
        const lastKey = pathParts[pathParts.length - 1];
        if (!isNaN(parseInt(lastKey))) {
            target[parseInt(lastKey)] = value;
        } else {
            target[lastKey] = value;
        }
        
        // 更新到 Store
        this.updateCard(cardId, updatedCard);
        
        // 發出細粒度事件
        this.emit('property:updated', {
            cardId,
            propertyPath,
            value,
            cardData: this.findCardById(cardId)
        });
    }
    
    /**
     * 批量更新屬性
     * @param {string} cardId - 卡片ID
     * @param {Object} properties - 要更新的屬性集合
     */
    updateCardProperties(cardId, properties) {
        const card = this.findCardById(cardId);
        if (!card) {
            return;
        }
        
        // 批量更新
        Object.entries(properties).forEach(([path, value]) => {
            this.updateCardProperty(cardId, path, value);
        });
        
        // 發出批量更新事件
        this.emit('properties:updated', {
            cardId,
            properties,
            cardData: this.findCardById(cardId)
        });
    }
    
    // ========== Switch-Case 特殊操作 ==========
    
    /**
     * 添加 Switch Case
     * @param {string} cardId - Switch 卡片ID
     * @returns {number} 新 Case 的索引
     */
    addSwitchCase(cardId) {
        const card = this.findCardById(cardId);
        if (!card?.switch) {
            return -1;
        }
        
        // 新建 Case (注意：TAFL v1.1.1 使用 'do' 而非 'then')
        const newCase = {
            when: '',
            do: []  // TAFL 使用 'do' 而非 'then'
        };
        
        // 添加到 cases 陣列，但要確保 default 保持在最後
        const existingCases = card.switch.cases || [];
        const defaultIndex = existingCases.findIndex(c => c.when === 'default');
        
        let cases;
        let newIndex;
        
        if (defaultIndex !== -1) {
            // 有 default，插入在它之前
            cases = [
                ...existingCases.slice(0, defaultIndex),
                newCase,
                ...existingCases.slice(defaultIndex)
            ];
            newIndex = defaultIndex; // 新 case 的索引就是原本 default 的位置
        } else {
            // 沒有 default，加在最後
            cases = [...existingCases, newCase];
            newIndex = cases.length - 1;
        }
        
        // 更新卡片 - 只更新 switch 屬性
        this.updateCard(cardId, {
            switch: {
                ...card.switch,
                cases
            }
        });
        
        // 發出事件
        this.emit('switch:case-added', {
            cardId,
            caseIndex: newIndex,
            caseData: newCase
        });
        
        return newIndex;
    }
    
    /**
     * 移除 Switch Case
     * @param {string} cardId - Switch 卡片ID
     * @param {number} caseIndex - 要移除的 Case 索引
     */
    removeSwitchCase(cardId, caseIndex) {
        const card = this.findCardById(cardId);
        if (!card?.switch?.cases) {
            return;
        }
        
        // 移除指定的 Case
        const cases = card.switch.cases.filter((_, index) => index !== caseIndex);
        
        // 更新卡片 - 只更新 switch 屬性
        this.updateCard(cardId, {
            switch: {
                ...card.switch,
                cases
            }
        });
        
        // 發出事件
        this.emit('switch:case-removed', {
            cardId,
            caseIndex
        });
    }
    
    /**
     * 添加 Switch Default (TAFL v1.1.1 規範: default 是特殊的 case，when: "default")
     * @param {string} cardId - Switch 卡片ID
     */
    addSwitchDefault(cardId) {
        const card = this.findCardById(cardId);
        if (!card?.switch) {
            return;
        }
        
        // 檢查是否已有 default case
        const hasDefault = card.switch.cases?.some(c => c.when === 'default');
        if (hasDefault) {
            return;
        }
        
        // 根據 TAFL v1.1.1，default 是特殊的 case，放在最後
        const defaultCase = {
            when: 'default',
            do: []  // TAFL 使用 'do' 而非 'then'
        };
        
        const cases = [...(card.switch.cases || []), defaultCase];
        
        // 更新卡片，移除舊的 default 字段（如果有）
        const { default: _, ...switchWithoutDefault } = card.switch;
        
        // 只更新 switch 屬性，不要傳遞整個卡片
        this.updateCard(cardId, {
            switch: {
                ...switchWithoutDefault,
                cases
            }
        });
        
        // 發出事件
        this.emit('switch:default-added', { cardId });
    }
    
    /**
     * 移除 Switch Default (TAFL v1.1.1 規範: 移除 when: "default" 的 case)
     * @param {string} cardId - Switch 卡片ID
     */
    removeSwitchDefault(cardId) {
        const card = this.findCardById(cardId);
        if (!card?.switch?.cases) {
            return;
        }
        
        // 移除 when: "default" 的 case
        const cases = card.switch.cases.filter(c => c.when !== 'default');
        
        // 更新卡片，同時移除舊的 default 字段（如果有）
        const { default: _, ...switchWithoutDefault } = card.switch;
        
        // 只更新 switch 屬性，不要傳遞整個卡片
        this.updateCard(cardId, {
            switch: {
                ...switchWithoutDefault,
                cases
            }
        });
        
        // 發出事件
        this.emit('switch:default-removed', { cardId });
    }
    
    /**
     * 移動 Switch Case 順序
     * @param {string} cardId - Switch 卡片ID
     * @param {number} fromIndex - 原索引
     * @param {number} toIndex - 目標索引
     */
    moveSwitchCase(cardId, fromIndex, toIndex) {
        const card = this.findCardById(cardId);
        if (!card?.switch?.cases) {
            return;
        }
        
        const cases = [...card.switch.cases];
        
        // 移動元素
        const [movedCase] = cases.splice(fromIndex, 1);
        cases.splice(toIndex, 0, movedCase);
        
        // 更新卡片 - 只更新 switch 屬性
        this.updateCard(cardId, {
            switch: {
                ...card.switch,
                cases
            }
        });
        
        // 發出事件
        this.emit('switch:case-moved', {
            cardId,
            fromIndex,
            toIndex
        });
    }
    
    // ========== 查詢方法 ==========
    
    findCardById(cardId) {
        
        // Debug the state
        const currentFlow = this.getFlow();
        
        if (!currentFlow || !currentFlow.flow) {
            return null;
        }
        
        const flow = currentFlow.flow;
        const result = this.findCardInArray(flow, cardId);
        
        return result;
    }
    
    findCardByIdInFlow(flow, cardId) {
        
        if (!flow || !Array.isArray(flow)) {
            return null;
        }
        
        const result = this.findCardInArray(flow, cardId);
        
        return result;
    }
    
    findCardInArray(cards, cardId) {
        for (const card of cards) {
            if (card.id === cardId) {
                return card;
            }
            
            // 只有當對象有 id 屬性時才是真正的卡片，才進行嵌套搜索
            if (!card.id) {
                continue;  // 跳過非卡片對象（如 switch 的 case 項）
            }
            
            // 搜尋嵌套結構
            const verb = this.getCardVerb(card);
            if (verb && card[verb]) {
                const params = card[verb];
                
                // 檢查各種嵌套結構 - 使用正確的屬性名稱
                // 注意: 'cases' 需要特殊處理，不能直接傳遞
                for (const key of ['then', 'else', 'do', 'default']) {
                    if (params[key] && Array.isArray(params[key])) {
                        const found = this.findCardInArray(params[key], cardId);
                        if (found) return found;
                    }
                }
                
                // 檢查 switch cases 的特殊結構
                if (params.cases && Array.isArray(params.cases)) {
                    for (const caseItem of params.cases) {
                        if (caseItem.do && Array.isArray(caseItem.do)) {
                            const found = this.findCardInArray(caseItem.do, cardId);
                            if (found) return found;
                        }
                    }
                }
            }
        }
        return null;
    }
    
    getCardVerb(cardData) {
        if (!cardData) return null;
        
        const nonVerbKeys = ['id', 'comment', 'skip_if', 'store_as', 'as', 'when', 'do'];
        
        // 硬編碼的 TAFL 動詞列表作為備用
        // 確保即使 verbDefinitions 未載入也能識別
        const knownVerbs = [
            'query', 'check', 'create', 'update', 
            'if',     // 明確包含 if
            'for', 'switch', 'set', 'stop', 'notify'
        ];
        
        const verbDefs = this.getVerbDefinitions();
        
        // 優先使用 verbDefinitions
        if (verbDefs) {
            const foundVerb = Object.keys(cardData).find(key => 
                !nonVerbKeys.includes(key) && verbDefs[key]
            );
            if (foundVerb) return foundVerb;
        }
        
        // 備用方案：使用硬編碼的已知動詞列表
        const foundVerb = Object.keys(cardData).find(key => 
            knownVerbs.includes(key)
        );
        if (foundVerb) return foundVerb;
        
        // 最終備用：任何非排除鍵（保持向後相容）
        for (const key of Object.keys(cardData)) {
            if (!nonVerbKeys.includes(key)) {
                return key;
            }
        }
        
        return null;
    }
    
    // ========== 輔助方法 ==========
    
    updateCardInFlow(cards, cardId, updates) {
        return cards.map(card => {
            if (card.id === cardId) {
                return { ...card, ...updates };
            }
            
            // 只有當對象有 id 屬性時才是真正的卡片，才處理嵌套結構
            if (!card.id) {
                return card;  // 不是卡片，直接返回
            }
            
            // 處理嵌套結構
            const verb = this.getCardVerb(card);
            if (verb && card[verb]) {
                const params = card[verb];
                // Handle string params (like 'set' verb) correctly
                const updatedParams = typeof params === 'string' ? params : { ...params };
                
                // Only process nested structures if params is an object
                if (typeof params === 'object') {
                    // 注意：不要直接處理 'cases'，它需要特殊處理
                    for (const key of ['then', 'else', 'do', 'default']) {
                        if (params[key] && Array.isArray(params[key])) {
                            updatedParams[key] = this.updateCardInFlow(params[key], cardId, updates);
                        }
                    }
                    
                    // 處理 switch cases 的特殊結構
                    if (params.cases && Array.isArray(params.cases)) {
                        updatedParams.cases = params.cases.map(caseItem => {
                            if (caseItem.do && Array.isArray(caseItem.do)) {
                                return {
                                    ...caseItem,
                                    do: this.updateCardInFlow(caseItem.do, cardId, updates)
                                };
                            }
                            return caseItem;
                        });
                    }
                }
                
                return {
                    ...card,
                    [verb]: updatedParams
                };
            }
            
            return card;
        });
    }
    
    removeCardFromFlow(cards, cardId) {
        const result = [];
        
        for (const card of cards) {
            if (card.id === cardId) {
                continue; // 跳過要刪除的卡片
            }
            
            // 處理嵌套結構
            const verb = this.getCardVerb(card);
            if (verb && card[verb]) {
                const params = card[verb];
                // Handle string params (like 'set' verb) correctly
                const updatedParams = typeof params === 'string' ? params : { ...params };
                
                // Only process nested structures if params is an object
                if (typeof params === 'object') {
                    // 使用與 insertCardInNested 相同的鍵名列表（移除 default，因為它現在是 case 的一部分）
                    for (const key of ['then', 'else', 'do', 'body', 'steps', 'catch', 'finally', 'cases']) {
                        if (params[key] && Array.isArray(params[key])) {
                            updatedParams[key] = this.removeCardFromFlow(params[key], cardId);
                        }
                    }
                    
                    // 處理 switch cases 的特殊結構
                    if (params.cases && Array.isArray(params.cases)) {
                        updatedParams.cases = params.cases.map(caseItem => {
                            if (caseItem.do && Array.isArray(caseItem.do)) {
                                return {
                                    ...caseItem,
                                    do: this.removeCardFromFlow(caseItem.do, cardId)
                                };
                            }
                            return caseItem;
                        });
                    }
                }
                
                result.push({
                    ...card,
                    [verb]: updatedParams
                });
            } else {
                result.push(card);
            }
        }
        
        return result;
    }
    
    insertCardInNested(cards, newCard, position) {
        return cards.map(card => {
            // 支援兩種屬性名稱格式（向後相容）
            const parentId = position.nested.parentCardId || position.nested.parentId;
            const branchName = position.nested.branchType || position.nested.branch;
            
            if (card.id === parentId) {
                const verb = this.getCardVerb(card);
                if (verb && card[verb]) {
                    const params = card[verb];
                    // Handle string params (like 'set' verb) correctly
                    const updatedParams = typeof params === 'string' ? params : { ...params };
                    
                    // Only update nested structures if params is an object
                    if (typeof params === 'object') {
                        // Special handling for switch cases (branchName like "case-0", "case-1", or "default")
                        if (verb === 'switch' && (branchName.startsWith('case-') || branchName === 'default')) {
                            
                            let caseIndex;
                            if (branchName === 'default') {
                                // Find the default case (when === 'default')
                                caseIndex = updatedParams.cases?.findIndex(c => c.when === 'default');
                            } else {
                                caseIndex = parseInt(branchName.split('-')[1]);
                            }
                            
                            if (updatedParams.cases && updatedParams.cases[caseIndex]) {
                                // Deep clone the cases array
                                updatedParams.cases = updatedParams.cases.map((c, i) => {
                                    if (i === caseIndex) {
                                        const updatedCase = { ...c };
                                        if (!updatedCase.do) {
                                            updatedCase.do = [];
                                        }
                                        // Clone the do array and insert at the specified index
                                        updatedCase.do = [...updatedCase.do];
                                        const insertIndex = typeof position.index === 'number' ? position.index : 0;
                                        updatedCase.do.splice(insertIndex, 0, newCard);
                                        return updatedCase;
                                    }
                                    return c;
                                });
                            }
                        } else {
                            // Normal nested structure handling (then, else, do, etc.)
                            const branchKey = branchName;
                            if (!updatedParams[branchKey]) {
                                updatedParams[branchKey] = [];
                            }
                            
                            updatedParams[branchKey] = [...updatedParams[branchKey]];
                            updatedParams[branchKey].splice(position.index || 0, 0, newCard);
                        }
                    }
                    
                    return {
                        ...card,
                        [verb]: updatedParams
                    };
                }
            }
            
            // 遞迴處理嵌套結構
            const verb = this.getCardVerb(card);
            if (verb && card[verb]) {
                const params = card[verb];
                // Handle string params (like 'set' verb) correctly
                const updatedParams = typeof params === 'string' ? params : { ...params };
                
                // Only process nested structures if params is an object
                if (typeof params === 'object') {
                    // 使用正確的 TAFL 鍵名: then, else, do, catch, finally 等
                    for (const key of ['then', 'else', 'do', 'body', 'steps', 'catch', 'finally']) {
                        if (params[key] && Array.isArray(params[key])) {
                            updatedParams[key] = this.insertCardInNested(params[key], newCard, position);
                        }
                    }
                    
                    // Handle switch cases recursively
                    if (verb === 'switch' && params.cases && Array.isArray(params.cases)) {
                        updatedParams.cases = params.cases.map(caseItem => {
                            if (caseItem.do && Array.isArray(caseItem.do)) {
                                return {
                                    ...caseItem,
                                    do: this.insertCardInNested(caseItem.do, newCard, position)
                                };
                            }
                            return caseItem;
                        });
                    }
                }
                
                return {
                    ...card,
                    [verb]: updatedParams
                };
            }
            
            return card;
        });
    }
    
    // ========== 其他操作 ==========
    
    setVerbDefinitions(verbs) {
        const state = this.baseStore.getState();
        this.baseStore.setState({
            ...state,
            verbDefinitions: verbs
        });
        this.emit('verbs:loaded', verbs);
    }
    
    setDirty(isDirty) {
        const state = this.baseStore.getState();
        this.baseStore.setState({
            ...state,
            isDirty
        });
        this.emit('dirty:changed', isDirty);
    }
    
    setSaved() {
        const state = this.baseStore.getState();
        this.baseStore.setState({
            ...state,
            isDirty: false,
            lastSaved: new Date().toISOString()
        });
        this.emit('flow:saved');
    }
    
    // ========== 事件管理 ==========
    
    on(event, callback) {
        // Use local listeners
        if (!this.listeners.has(event)) {
            this.listeners.set(event, new Set());
        }
        this.listeners.get(event).add(callback);
        return callback;
    }
    
    off(event, callback) {
        const listeners = this.listeners.get(event);
        if (listeners) {
            listeners.delete(callback);
            if (listeners.size === 0) {
                this.listeners.delete(event);
            }
        }
    }
    
    emit(event, data) {
        // Emit to local listeners
        const listeners = this.listeners.get(event);
        if (listeners) {
            listeners.forEach(callback => {
                try {
                    callback(data);
                } catch (error) {
                    console.error(`Error in event listener for ${event}:`, error);
                }
            });
        }
    }
    
    // ========== 批次更新 ==========
    
    batchUpdate(updateFn) {
        // 暫時禁用事件
        const originalEmit = this.emit;
        const pendingEvents = [];
        
        this.emit = (event, data) => {
            pendingEvents.push({ event, data });
        };
        
        try {
            // 執行批次更新
            updateFn();
            
            // 恢復事件發送
            this.emit = originalEmit;
            
            // 發送批次事件
            this.emit('batch:updated', pendingEvents);
            
            // 發送個別事件
            pendingEvents.forEach(({ event, data }) => {
                this.emit(event, data);
            });
        } catch (error) {
            this.emit = originalEmit;
            throw error;
        }
    }
    
    // ========== 歷史管理 (Undo/Redo) ==========
    
    /**
     * 保存狀態快照 - 簡單直接，不搞花哨
     */
    saveSnapshot(description = '') {
        // 防止歷史操作觸發新快照
        if (this.isApplyingHistory) return;
        
        const state = this.getState();
        const snapshot = {
            timestamp: Date.now(),
            description,
            // 深拷貝整個 flow - 簡單粗暴但有效
            flow: JSON.parse(JSON.stringify(state.currentFlow))
        };
        
        // 如果有撤銷過，清理之後的歷史
        if (this.historyIndex < this.history.length - 1) {
            this.history = this.history.slice(0, this.historyIndex + 1);
        }
        
        // 添加新快照
        this.history.push(snapshot);
        this.historyIndex++;
        
        // 限制歷史大小
        if (this.history.length > this.maxHistorySize) {
            this.history.shift();
            this.historyIndex--;
        }
        
        this.emit('history:changed', {
            canUndo: this.canUndo(),
            canRedo: this.canRedo()
        });
    }
    
    /**
     * 撤銷 - 回到上一個狀態
     */
    undo() {
        if (!this.canUndo()) return false;
        
        this.historyIndex--;
        this.isApplyingHistory = true;
        
        const snapshot = this.history[this.historyIndex];
        this.baseStore.setState({
            ...this.getState(),
            currentFlow: JSON.parse(JSON.stringify(snapshot.flow)),
            selectedCardId: null  // 清除選擇
        });
        
        this.isApplyingHistory = false;
        this.emit('history:undo', snapshot);
        this.emit('history:changed', {
            canUndo: this.canUndo(),
            canRedo: this.canRedo()
        });
        return true;
    }
    
    /**
     * 重做 - 前進到下一個狀態
     */
    redo() {
        if (!this.canRedo()) return false;
        
        this.historyIndex++;
        this.isApplyingHistory = true;
        
        const snapshot = this.history[this.historyIndex];
        this.baseStore.setState({
            ...this.getState(),
            currentFlow: JSON.parse(JSON.stringify(snapshot.flow)),
            selectedCardId: null  // 清除選擇
        });
        
        this.isApplyingHistory = false;
        this.emit('history:redo', snapshot);
        this.emit('history:changed', {
            canUndo: this.canUndo(),
            canRedo: this.canRedo()
        });
        return true;
    }
    
    /**
     * 檢查是否可以撤銷
     */
    canUndo() {
        return this.historyIndex > 0;
    }
    
    /**
     * 檢查是否可以重做
     */
    canRedo() {
        return this.historyIndex < this.history.length - 1;
    }
    
    /**
     * 清除歷史記錄
     */
    clearHistory() {
        this.history = [];
        this.historyIndex = -1;
        this.emit('history:changed', {
            canUndo: false,
            canRedo: false
        });
    }
}

// 創建並導出單例
export const taflFlowStore = new TAFLFlowStore();