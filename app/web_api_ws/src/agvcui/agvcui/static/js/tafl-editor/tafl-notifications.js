/**
 * TAFL Editor 通知系統模組
 * 提供獨立的通知顯示功能，支援堆疊和毛玻璃效果
 */

class TAFLNotifications {
    constructor() {
        this.notifications = new Map(); // 存儲當前顯示的通知
        this.notificationZIndex = 10000; // 基礎 z-index
        this.notificationSpacing = 10; // 通知之間的間距
        this.notificationHeight = 60; // 預估的通知高度
        this.init();
    }

    init() {
        // 確保有通知容器
        if (!document.getElementById('tafl-notifications-container')) {
            const container = document.createElement('div');
            container.id = 'tafl-notifications-container';
            container.style.cssText = `
                position: fixed;
                top: 20px;
                right: 20px;
                z-index: ${this.notificationZIndex};
                pointer-events: none;
            `;
            document.body.appendChild(container);
        }

        // 注入通知樣式
        if (!document.getElementById('tafl-notifications-styles')) {
            const style = document.createElement('style');
            style.id = 'tafl-notifications-styles';
            style.textContent = `
                .tafl-notification {
                    position: fixed;
                    right: 20px;
                    min-width: 300px;
                    max-width: 500px;
                    padding: 1rem;
                    margin-bottom: 0.5rem;
                    border-radius: 8px;
                    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
                    transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
                    pointer-events: auto;
                    animation: slideInRight 0.3s ease-out;
                    
                    /* 毛玻璃效果 */
                    background: rgba(255, 255, 255, 0.85);
                    backdrop-filter: blur(10px);
                    -webkit-backdrop-filter: blur(10px);
                    border: 1px solid rgba(255, 255, 255, 0.3);
                }
                
                .tafl-notification.is-dark {
                    background: rgba(32, 32, 32, 0.85);
                    border: 1px solid rgba(255, 255, 255, 0.1);
                    color: #ffffff;
                }
                
                .tafl-notification.is-removing {
                    animation: slideOutRight 0.3s ease-in;
                    opacity: 0;
                    transform: translateX(100%);
                }
                
                .tafl-notification.is-success {
                    border-left: 4px solid #48c774;
                }
                
                .tafl-notification.is-warning {
                    border-left: 4px solid #ffdd57;
                }
                
                .tafl-notification.is-error,
                .tafl-notification.is-danger {
                    border-left: 4px solid #f14668;
                }
                
                .tafl-notification.is-info {
                    border-left: 4px solid #3298dc;
                }
                
                .tafl-notification-content {
                    display: flex;
                    align-items: center;
                    justify-content: space-between;
                }
                
                .tafl-notification-message {
                    flex: 1;
                    margin-right: 1rem;
                    font-size: 0.95rem;
                    line-height: 1.5;
                }
                
                .tafl-notification-close {
                    cursor: pointer;
                    opacity: 0.7;
                    transition: opacity 0.2s;
                    font-size: 1.2rem;
                    line-height: 1;
                    padding: 0.25rem;
                    background: transparent;
                    border: none;
                    color: inherit;
                }
                
                .tafl-notification-close:hover {
                    opacity: 1;
                }
                
                @keyframes slideInRight {
                    from {
                        transform: translateX(100%);
                        opacity: 0;
                    }
                    to {
                        transform: translateX(0);
                        opacity: 1;
                    }
                }
                
                @keyframes slideOutRight {
                    from {
                        transform: translateX(0);
                        opacity: 1;
                    }
                    to {
                        transform: translateX(100%);
                        opacity: 0;
                    }
                }
                
                /* 響應式調整 */
                @media (max-width: 768px) {
                    .tafl-notification {
                        min-width: 250px;
                        right: 10px;
                        left: 10px;
                        max-width: calc(100% - 20px);
                    }
                }
            `;
            document.head.appendChild(style);
        }
    }

    /**
     * 顯示通知
     * @param {string} message - 通知訊息
     * @param {string} type - 通知類型 (success, warning, error, danger, info)
     * @param {number} duration - 顯示時長（毫秒），0 表示不自動消失
     * @returns {string} - 通知 ID
     */
    show(message, type = 'info', duration = 3000) {
        const id = `notification-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
        
        // 創建通知元素
        const notification = document.createElement('div');
        notification.id = id;
        notification.className = `tafl-notification is-${type}`;
        
        // 檢查是否為深色模式
        if (document.body.classList.contains('dark-theme')) {
            notification.classList.add('is-dark');
        }
        
        // 設置內容
        notification.innerHTML = `
            <div class="tafl-notification-content">
                <div class="tafl-notification-message">${this.escapeHtml(message)}</div>
                <button class="tafl-notification-close" aria-label="關閉">×</button>
            </div>
        `;
        
        // 計算位置（堆疊）
        const topPosition = this.calculateTopPosition();
        notification.style.top = `${topPosition}px`;
        notification.style.zIndex = this.notificationZIndex + this.notifications.size;
        
        // 添加到容器
        const container = document.getElementById('tafl-notifications-container');
        container.appendChild(notification);
        
        // 存儲通知信息
        this.notifications.set(id, {
            element: notification,
            height: notification.offsetHeight,
            top: topPosition
        });
        
        // 綁定關閉按鈕
        const closeBtn = notification.querySelector('.tafl-notification-close');
        closeBtn.addEventListener('click', () => this.remove(id));
        
        // 自動關閉
        if (duration > 0) {
            setTimeout(() => this.remove(id), duration);
        }
        
        return id;
    }

    /**
     * 計算新通知的頂部位置
     */
    calculateTopPosition() {
        let topPosition = 20; // 初始頂部間距
        
        // 遍歷現有通知，計算累積高度
        for (const [, notificationData] of this.notifications) {
            topPosition += notificationData.height + this.notificationSpacing;
        }
        
        return topPosition;
    }

    /**
     * 移除通知
     * @param {string} id - 通知 ID
     */
    remove(id) {
        const notificationData = this.notifications.get(id);
        if (!notificationData) return;
        
        const notification = notificationData.element;
        
        // 添加移除動畫
        notification.classList.add('is-removing');
        
        // 動畫結束後移除元素
        setTimeout(() => {
            if (notification.parentNode) {
                notification.parentNode.removeChild(notification);
            }
            this.notifications.delete(id);
            
            // 重新排列剩餘通知
            this.restackNotifications();
        }, 300);
    }

    /**
     * 重新排列通知位置
     */
    restackNotifications() {
        let topPosition = 20;
        
        for (const [, notificationData] of this.notifications) {
            const notification = notificationData.element;
            notification.style.top = `${topPosition}px`;
            notificationData.top = topPosition;
            topPosition += notificationData.height + this.notificationSpacing;
        }
    }

    /**
     * 清除所有通知
     */
    clear() {
        for (const [id] of this.notifications) {
            this.remove(id);
        }
    }

    /**
     * 顯示成功通知
     */
    success(message, duration = 3000) {
        return this.show(message, 'success', duration);
    }

    /**
     * 顯示警告通知
     */
    warning(message, duration = 3000) {
        return this.show(message, 'warning', duration);
    }

    /**
     * 顯示錯誤通知
     */
    error(message, duration = 5000) {
        return this.show(message, 'error', duration);
    }

    /**
     * 顯示信息通知
     */
    info(message, duration = 3000) {
        return this.show(message, 'info', duration);
    }

    /**
     * HTML 轉義
     */
    escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }
}

// 創建全局實例
const taflNotifications = new TAFLNotifications();

// 導出模組
export { taflNotifications as default, TAFLNotifications };