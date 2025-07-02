/**
 * Device Form 頁面功能
 * 處理設備表單的動態端口和信號管理
 */

function addPort() {
    const container = document.getElementById('portsContainer');
    const portHtml = `
        <div class="port-item box" style="margin-bottom: 1rem;">
            <div class="columns">
                <div class="column is-half">
                    <div class="field">
                        <label class="label is-small">端口名稱</label>
                        <div class="control">
                            <input class="input is-small port-name" type="text" placeholder="端口名稱">
                        </div>
                    </div>
                </div>
                <div class="column is-half">
                    <div class="field">
                        <label class="label is-small">端口描述</label>
                        <div class="control">
                            <input class="input is-small port-description" type="text" placeholder="端口描述">
                        </div>
                    </div>
                </div>
            </div>
            
            <div class="signals-container">
                <label class="label is-small">信號列表</label>
                <button type="button" class="button is-small is-info add-signal">
                    <i class="mdi mdi-plus"></i> 添加信號
                </button>
            </div>
            
            <button type="button" class="button is-small is-danger remove-port" style="margin-top: 0.5rem;">
                <i class="mdi mdi-delete"></i> 刪除端口
            </button>
        </div>
    `;
    container.insertAdjacentHTML('beforeend', portHtml);
    attachEventListeners();
}

function addSignal(signalsContainer) {
    const signalHtml = `
        <div class="signal-item" style="margin-bottom: 0.5rem;">
            <div class="columns is-mobile">
                <div class="column is-3">
                    <input class="input is-small signal-name" type="text" placeholder="信號名稱">
                </div>
                <div class="column is-2">
                    <input class="input is-small signal-value" type="text" placeholder="值">
                </div>
                <div class="column is-2">
                    <div class="select is-small is-fullwidth">
                        <select class="signal-type">
                            <option value="bool">布林</option>
                            <option value="int">整數</option>
                            <option value="float">浮點數</option>
                            <option value="string">字符串</option>
                        </select>
                    </div>
                </div>
                <div class="column is-3">
                    <input class="input is-small signal-dm" type="text" placeholder="DM地址">
                </div>
                <div class="column is-2">
                    <button type="button" class="button is-small is-danger remove-signal">
                        <i class="mdi mdi-delete"></i>
                    </button>
                </div>
            </div>
        </div>
    `;
    const addButton = signalsContainer.querySelector('.add-signal');
    addButton.insertAdjacentHTML('beforebegin', signalHtml);
    attachEventListeners();
}

function attachEventListeners() {
    // 添加信號按鈕
    document.querySelectorAll('.add-signal').forEach(btn => {
        btn.replaceWith(btn.cloneNode(true)); // 移除舊的事件監聽器
    });
    document.querySelectorAll('.add-signal').forEach(btn => {
        btn.addEventListener('click', function() {
            addSignal(this.closest('.signals-container'));
        });
    });

    // 刪除信號按鈕
    document.querySelectorAll('.remove-signal').forEach(btn => {
        btn.replaceWith(btn.cloneNode(true));
    });
    document.querySelectorAll('.remove-signal').forEach(btn => {
        btn.addEventListener('click', function() {
            this.closest('.signal-item').remove();
        });
    });

    // 刪除端口按鈕
    document.querySelectorAll('.remove-port').forEach(btn => {
        btn.replaceWith(btn.cloneNode(true));
    });
    document.querySelectorAll('.remove-port').forEach(btn => {
        btn.addEventListener('click', function() {
            this.closest('.port-item').remove();
        });
    });
}

function submitDeviceForm() {
    // 收集設備基本信息
    const locationValue = document.getElementById('deviceLocationId').value;
    const deviceData = {
        name: document.getElementById('deviceName').value,
        description: document.getElementById('deviceDescription').value,
        location_id: locationValue && locationValue !== "" ? parseInt(locationValue) : null,
        ports: []
    };

    // 收集端口和信號信息
    document.querySelectorAll('.port-item').forEach(portItem => {
        const portData = {
            name: portItem.querySelector('.port-name').value,
            description: portItem.querySelector('.port-description').value,
            signals: []
        };

        portItem.querySelectorAll('.signal-item').forEach(signalItem => {
            const signalData = {
                name: signalItem.querySelector('.signal-name').value,
                value: signalItem.querySelector('.signal-value').value,
                type_of_value: signalItem.querySelector('.signal-type').value,
                dm_address: signalItem.querySelector('.signal-dm').value || null
            };

            if (signalData.name) {
                portData.signals.push(signalData);
            }
        });

        if (portData.name) {
            deviceData.ports.push(portData);
        }
    });

    // 將數據轉換為JSON並提交
    document.getElementById('deviceJsonInput').value = JSON.stringify(deviceData);
    document.getElementById('deviceForm').submit();
}

function setupFormEvents() {
    // 添加端口按鈕事件
    const addPortBtn = document.getElementById('addPortBtn');
    if (addPortBtn) {
        addPortBtn.addEventListener('click', function() {
            addPort();
        });
    }

    // 表單提交事件
    const deviceForm = document.getElementById('deviceForm');
    if (deviceForm) {
        deviceForm.addEventListener('submit', function(e) {
            e.preventDefault();
            submitDeviceForm();
        });
    }

    // 為現有的按鈕添加事件監聽器
    attachEventListeners();
}

export const deviceFormPage = (() => {
    function setup() {
        // 設置表單事件
        setupFormEvents();

        // 將函數掛載到 window 供調試使用（可選）
        if (typeof window !== 'undefined') {
            window.addPort = addPort;
            window.addSignal = addSignal;
            window.submitDeviceForm = submitDeviceForm;
        }
    }

    return {
        setup,
    };
})();
