// notify.js
export const notify = (() => {
    function showMessage({ notifyId, contentId, msg, type, duration }) {
        const notify = document.getElementById(notifyId);
        const content = document.getElementById(contentId);
        if (!notify || !content) return;

        // 重設 class 並加入 type
        notify.className = `notification is-light ${type}`;

        // 使用 textContent 保護安全
        content.textContent = msg;

        // 顯示通知區塊
        notify.style.display = "block";

        // 清除之前 timeout
        if (notify._timeoutId) clearTimeout(notify._timeoutId);

        notify._timeoutId = setTimeout(() => {
            notify.style.display = "none";
            notify._timeoutId = null;
        }, duration);
    }

    function showNotifyMessage(msg, type = "is-success", duration = 3000) {
        showMessage({
            notifyId: "notify-message",
            contentId: "notify-message-content",
            msg,
            type,
            duration,
        });
    }

    function showErrorMessage(msg, type = "is-danger", duration = 3000) {
        showMessage({
            notifyId: "error-message",
            contentId: "error-message-content",
            msg,
            type,
            duration,
        });
    }

    function bindCloseButtons() {
        // 綁定 base.html 兩個通知的 close 按鈕
        const notifyClose = document.querySelector('#notify-message .delete');
        const errorClose = document.querySelector('#error-message .delete');
        if (notifyClose) notifyClose.onclick = () => {
            const notify = document.getElementById('notify-message');
            if (notify) notify.style.display = 'none';
        };
        if (errorClose) errorClose.onclick = () => {
            const error = document.getElementById('error-message');
            if (error) error.style.display = 'none';
        };
    }

    function setup() {
        bindCloseButtons();
    }

    return {
        setup,
        showNotifyMessage,
        showErrorMessage,
    };
})();
