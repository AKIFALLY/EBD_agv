export class BaseObject {
    constructor(map, latlng, iconHtml, iconSize = [40, 40], iconAnchor = [20, 20], zIndex = 1) {
        this.map = map;
        this.latlng = latlng;
        this.data = {}; // 儲存物件相關資料
        this.clickHandlers = []; // 點擊事件處理器

        const icon = L.divIcon({
            className: '',
            html: iconHtml,
            iconSize,
            iconAnchor
        });
        this.marker = L.marker(latlng, { icon, zIndexOffset: zIndex }).addTo(map);
        this.el = this.marker.getElement()?.firstElementChild;
        this.eqp_id = '';

        // 設置點擊事件
        this.setupClickEvents();

        requestAnimationFrame(this.animate.bind(this));
    }

    // 設置點擊事件
    setupClickEvents() {
        if (this.marker) {
            this.marker.on('click', (e) => {
                e.originalEvent.stopPropagation();
                this.handleClick(e);
            });
        }
    }

    // 處理點擊事件
    handleClick(e) {
        this.clickHandlers.forEach(handler => {
            try {
                handler(this, e);
            } catch (error) {
                console.error('Click handler error:', error);
            }
        });
    }

    // 添加點擊事件處理器
    addClickHandler(handler) {
        if (typeof handler === 'function') {
            this.clickHandlers.push(handler);
        }
    }

    // 移除點擊事件處理器
    removeClickHandler(handler) {
        const index = this.clickHandlers.indexOf(handler);
        if (index > -1) {
            this.clickHandlers.splice(index, 1);
        }
    }

    // 設置物件資料
    setData(data) {
        this.data = { ...this.data, ...data };
    }

    // 獲取物件資料
    getData(key = null) {
        return key ? this.data[key] : this.data;
    }
    updateScale(delta) {
        if (!this.el) return;
        const scale = this.map.getZoomScale(this.map.getZoom(), 0);
        this.el.style.transform = `scale(${scale})`;
    }
    remove() {
        if (this.marker) {
            this.marker.remove();
        }
    }
    updateLatLng(latlng) {
        this.latlng = latlng;
        this.marker.setLatLng(latlng);
    }
    updateTransform(delta) {
        this.updateScale(delta);
    }
    animate(time) {
        if (!this.lastTime) this.lastTime = time;
        const delta = (time - this.lastTime) / 1000;
        this.lastTime = time;
        this.updateTransform(delta);
        requestAnimationFrame(this.animate.bind(this));
    }
    updateSignals(signalMap) {
        if (!this.el || this.eqp_id === '') return;

        this.updateEqpSignals(signalMap)
    }
    updateEqpSignals(signalMap) {
        const start = performance.now();
        signalMap.forEach((value, key) => {
            //console.log(key, value);
        });
        //console.log("🔍 this.eqp_id = ", this.eqp_id, typeof this.eqp_id);
        //for (const key of signalMap.keys()) {
        //    console.log("🧩 key = [" + key + "]");
        //}
        const prefix = `${this.eqp_id}_`;
        // 篩選出所有屬於這個設備的 signal key
        //const matchingKeys = Object.keys(signalMap).filter(key => key.startsWith(`${this.eqp_id}_`));
        const matchingKeys = Array.from(signalMap.keys()).filter(key => key.startsWith(`${this.eqp_id}_`));
        if (matchingKeys.length === 0) return;
        //console.log("matchingKeys.length !== 0")
        //在席訊號更新
        this.updateSignalDisplay(signalMap, 'Presence', 'mdi-safety-goggles', {
            trueClasses: ['presence'],
            falseClasses: ['no-presence']
        });
        //Load請求訊號更新
        this.updateSignalDisplay(signalMap, 'Load', 'mdi-transfer-down', {
            trueClasses: ['load-request'],
            falseClasses: []
        });
        //Unload請求訊號更新
        this.updateSignalDisplay(signalMap, 'Unload', 'mdi-transfer-up', {
            trueClasses: ['unload-request'],
            falseClasses: []
        });

        const deltaTime = performance.now() - start;
        //console.debug(deltaTime.toFixed(3), "ms");
    }

    updateSignalDisplay(signalMap, signalSuffix, iconClassName, classOptions = {}) {
        for (let index = 1; index <= 8; index++) {
            const key = `${this.eqp_id}_${index}_${signalSuffix}`;
            //console.debug(key);
            const signal = signalMap.get(key);

            if (signal) {
                const cell = this.el.querySelector(`.cell[data-port='${index}']`);
                if (cell) {
                    const isActive = signal.value === "True" || signal.value === '1';
                    // Show/hide icon
                    const icon = cell.querySelector(`.${iconClassName}`);
                    if (icon) {
                        icon.classList.toggle('hidden', !isActive);
                    }
                    // Handle trueClasses
                    if (Array.isArray(classOptions.trueClasses)) {
                        classOptions.trueClasses.forEach(cls => {
                            cell.classList.toggle(cls, isActive);
                        });
                    }
                    // Handle falseClasses
                    if (Array.isArray(classOptions.falseClasses)) {
                        classOptions.falseClasses.forEach(cls => {
                            cell.classList.toggle(cls, !isActive);
                        });
                    }
                }
            }
        }
    }
}
