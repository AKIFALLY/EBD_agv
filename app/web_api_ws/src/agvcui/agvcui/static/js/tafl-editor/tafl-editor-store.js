/**
 * TAFL Editor Store - 基於 miniStore 的狀態管理
 * 管理 TAFL Editor 的所有狀態和事件
 */

import { createStore } from '../../store/miniStore.js';

class TAFLFlowStore {
    constructor() {
        // TAFL v1.1 預設流程結構
        const defaultFlow = {
            metadata: {
                id: null,
                name: 'Untitled Flow',
                version: '1.1',
                description: ''
            },
            settings: {
                timeout: 3600,
                max_retries: 3,
                retry_on_failure: false
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
        let needsUpdate = false;
        let updatedState = { ...state };
        
        // Fix cardCounter if invalid
        if (typeof state.cardCounter !== 'number' || isNaN(state.cardCounter)) {
            updatedState.cardCounter = 0;
            needsUpdate = true;
        }
        
        // Always reset to empty flow on initialization (don't auto-load old flow)
        updatedState.currentFlow = defaultFlow;
        updatedState.selectedCardId = null;
        updatedState.isDirty = false;
        needsUpdate = true;
        
        if (needsUpdate) {
            this.baseStore.setState(updatedState);
        }
        
        // 額外的事件監聽器 (用於細粒度事件)
        this.listeners = new Map();
        
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
        return this.baseStore.getState().currentFlow;
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
    }
    
    loadFlow(flowData) {
        this.baseStore.setState({
            ...this.baseStore.getState(),
            currentFlow: flowData,
            isDirty: false,
            lastSaved: new Date().toISOString()
        });
        this.emit('flow:loaded', flowData);
    }
    
    clearFlow() {
        const defaultFlow = {
            metadata: {
                id: null,
                name: 'Untitled Flow',
                version: '1.1',
                description: ''
            },
            settings: {
                timeout: 3600,
                max_retries: 3,
                retry_on_failure: false
            },
            preload: {},
            rules: {},
            variables: {},
            flow: []
        };
        
        const state = this.baseStore.getState();
        this.baseStore.setState({
            ...state,
            currentFlow: defaultFlow,
            selectedCardId: null,
            isDirty: false
            // 不重置 cardCounter，保持全域唯一性
        });
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
        this.baseStore.setState({
            ...state,
            cardCounter: newCounter
        });
        return `card_${newCounter}_${Date.now()}`;
    }
    
    addCard(verb, position = null, params = {}) {
        const state = this.baseStore.getState();
        const flow = state.currentFlow.flow;
        const cardId = this.generateId();
        
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
        
        this.baseStore.setState({
            ...state,
            currentFlow: updatedCurrentFlow,
            isDirty: true
        });
        
        this.emit('card:added', {
            card: newCard,
            position: position || { index: newFlow.length - 1 }
        });
        
        return cardId;
    }
    
    updateCard(cardId, updates) {
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
        
        this.emit('card:updated', {
            cardId,
            card: { ...card, ...updates }
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
        
        this.emit('card:deleted', cardId);
    }
    
    moveCard(cardId, newPosition) {
        const state = this.baseStore.getState();
        const card = this.findCardById(cardId);
        if (!card) return;
        
        // 先移除卡片
        let newFlow = this.removeCardFromFlow(state.currentFlow.flow, cardId);
        
        // 再插入到新位置
        if (newPosition.nested) {
            // 插入到嵌套位置
            newFlow = this.insertCardInNested(newFlow, card, newPosition);
        } else {
            // 插入到頂層位置
            newFlow.splice(newPosition.index, 0, card);
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
        
        this.emit('card:moved', {
            cardId,
            position: newPosition
        });
    }
    
    selectCard(cardId) {
        const state = this.baseStore.getState();
        if (state.selectedCardId === cardId) return;
        
        this.baseStore.setState({
            ...state,
            selectedCardId: cardId
        });
        
        this.emit('card:selected', cardId);
    }
    
    // ========== 查詢方法 ==========
    
    findCardById(cardId) {
        const flow = this.getFlow().flow;
        return this.findCardInArray(flow, cardId);
    }
    
    findCardInArray(cards, cardId) {
        for (const card of cards) {
            if (card.id === cardId) {
                return card;
            }
            
            // 搜尋嵌套結構
            const verb = this.getCardVerb(card);
            if (verb && card[verb]) {
                const params = card[verb];
                
                // 檢查各種嵌套結構 - 使用正確的屬性名稱
                for (const key of ['then', 'else', 'do', 'cases', 'default']) {
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
        
        const nonVerbKeys = ['id', 'comment', 'skip_if', 'store_as', 'as'];
        const verbDefs = this.getVerbDefinitions();
        
        // 找出動詞鍵
        for (const key of Object.keys(cardData)) {
            if (!nonVerbKeys.includes(key) && verbDefs && verbDefs[key]) {
                return key;
            }
        }
        
        // 備用：找出任何非排除鍵
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
            
            // 處理嵌套結構
            const verb = this.getCardVerb(card);
            if (verb && card[verb]) {
                const params = card[verb];
                const updatedParams = { ...params };
                
                for (const key of ['then', 'else', 'do', 'cases', 'default']) {
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
                const updatedParams = { ...params };
                
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
                    const updatedParams = { ...params };
                    const branchKey = branchName;
                    
                    if (!updatedParams[branchKey]) {
                        updatedParams[branchKey] = [];
                    }
                    
                    updatedParams[branchKey] = [...updatedParams[branchKey]];
                    updatedParams[branchKey].splice(position.index || 0, 0, newCard);
                    
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
                const updatedParams = { ...params };
                
                // 使用正確的 TAFL 鍵名: then, else, do, catch, finally 等
                for (const key of ['then', 'else', 'do', 'body', 'steps', 'catch', 'finally']) {
                    if (params[key] && Array.isArray(params[key])) {
                        updatedParams[key] = this.insertCardInNested(params[key], newCard, position);
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
        if (!this.listeners.has(event)) {
            this.listeners.set(event, new Set());
        }
        this.listeners.get(event).add(callback);
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
        const listeners = this.listeners.get(event);
        if (listeners) {
            listeners.forEach(callback => {
                try {
                    callback(data);
                } catch (error) {
                    console.error(`Error in event handler for ${event}:`, error);
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
}

// 創建並導出單例
export const taflFlowStore = new TAFLFlowStore();