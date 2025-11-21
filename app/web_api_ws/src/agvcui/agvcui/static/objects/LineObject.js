export class LineObject {
    constructor(map, pointA, pointB, options = {}) {
        this.map = map;
        this.id = options.id || '';
        this.edgeId = options.id || '';

        const [start, end] = offsetLineEnds(map, pointA, pointB);

        this.defaultColor = options.color || '#AAA';
        this.hoverColor = options.hoverColor || 'red';

        this.defaultWeight = options.weight || 3;
        this.hoverWeight = options.hoverWeight || 5;

        this.opacity = options.opacity !== undefined ? options.opacity : 0.5; // 0~1 透明度

        // 處理 CSS 類別名稱
        this.className = options.className || '';

        this.polyline = L.polyline([start, end], {
            color: options.color || '#AAA',
            weight: options.weight || 3,
            opacity: this.opacity, // 0~1 透明度
            dashArray: options.dashArray || null,
            className: this.className // 添加 CSS 類別支援
        }).addTo(map);

        // 創建一個透明的、更寬的覆蓋層來捕獲點擊事件
        this.clickLayer = L.polyline([start, end], {
            color: 'transparent',
            weight: Math.max(15, (options.weight || 3) * 3), // 至少15px寬，或原線條的3倍
            opacity: 0,
            interactive: true,
            className: this.className // 添加 CSS 類別支援
        }).addTo(map);

        this.arrowColor = options.arrowColor || options.color || '#AAA';
        this.arrowSize = options.arrowSize || 10;

        // 為 arrow decorator 也添加 CSS 類別（通過設定到容器）
        this.arrow = L.polylineDecorator(this.polyline, {
            patterns: [
                {
                    offset: '100%',
                    repeat: 0,
                    symbol: L.Symbol.arrowHead({
                        pixelSize: this.arrowSize,
                        polygon: true,
                        pathOptions: {
                            color: this.arrowColor,
                            fillColor: this.arrowColor,
                            opacity: this.opacity,
                            fill: true
                        }
                    })
                }
            ]
        }).addTo(map);

        // 加入 hover 事件到透明覆蓋層
        this.clickLayer.on('mouseover', () => {
            this.polyline.setStyle({ color: this.hoverColor, weight: this.hoverWeight });
            this.arrow.setPatterns([
                {
                    offset: '100%',
                    repeat: 0,
                    symbol: L.Symbol.arrowHead({
                        pixelSize: this.arrowSize,
                        polygon: true,
                        pathOptions: {
                            color: this.hoverColor,
                            fillColor: this.hoverColor,
                            opacity: this.opacity,
                            fillOpacity: 1,  // 滑過時變不透明
                            fill: true
                        }
                    })
                }
            ]);
        });

        this.clickLayer.on('mouseout', () => {
            this.polyline.setStyle({ color: this.defaultColor, weight: this.defaultWeight });
            this.arrow.setPatterns([
                {
                    offset: '100%',
                    repeat: 0,
                    symbol: L.Symbol.arrowHead({
                        pixelSize: this.arrowSize,
                        polygon: true,
                        pathOptions: {
                            color: this.arrowColor,
                            fillColor: this.arrowColor,
                            opacity: this.opacity,
                            fill: true
                        }
                    })
                }
            ]);
        });

        // ✅ 點擊顯示彈出視窗 - 使用透明覆蓋層
        this.clickLayer.on('click', (e) => {
            // console.log('Edge clicked:', this.id);
            // console.log('Available globals:', {
            //     mapObjectManager: !!window.mapObjectManager,
            //     mapInteraction: !!window.mapInteraction,
            //     handleEdgeClick: !!(window.mapObjectManager && window.mapObjectManager.handleEdgeClick)
            // });
            e.originalEvent.stopPropagation();
            this.showPopup(e);
        });
    }

    showPopup(e) {
        // console.log('showPopup called for edge:', this.id);

        // 直接創建和顯示彈出視窗，不依賴複雜的模組系統
        this.createAndShowPopup(e.latlng);
    }

    createAndShowPopup(latlng) {
        // 移除現有的彈出視窗
        const existingPopup = document.getElementById('edge-popup');
        if (existingPopup) {
            existingPopup.remove();
        }

        // 創建彈出視窗 HTML
        const popupHtml = `
            <div id="edge-popup" class="map-popup" style="display: block; position: absolute; z-index: 1001;">
                <div class="map-popup-header">
                    <h5 class="title is-6">邊線: ${this.id}</h5>
                    <button class="map-popup-close" onclick="document.getElementById('edge-popup').remove()">
                        <i class="mdi mdi-close"></i>
                    </button>
                </div>
                <div class="map-popup-content">
                    <table class="table is-narrow is-fullwidth popup-table">
                        <tbody>
                            <tr>
                                <td class="popup-label">邊線 ID</td>
                                <td><span class="tag is-info">${this.id}</span></td>
                            </tr>
                            <tr>
                                <td class="popup-label">邊線名稱</td>
                                <td>${this.edgeId}</td>
                            </tr>
                        </tbody>
                    </table>
                </div>
            </div>
        `;

        // 添加到頁面
        document.body.insertAdjacentHTML('beforeend', popupHtml);

        // 計算位置
        const popup = document.getElementById('edge-popup');
        const point = this.map.latLngToContainerPoint(latlng);
        popup.style.left = `${point.x + 10}px`;
        popup.style.top = `${point.y - 10}px`;

        // 點擊地圖其他地方關閉彈出視窗
        const closeHandler = (e) => {
            if (!popup.contains(e.target)) {
                popup.remove();
                this.map.off('click', closeHandler);
            }
        };

        // 延遲添加事件監聽器，避免立即觸發
        setTimeout(() => {
            this.map.on('click', closeHandler);
        }, 100);
    }

    updatePoints(pointA, pointB) {
        // console.log(`[updatePoints] 更新前 - 地图缩放级别: ${this.map.getZoom()}`);
        // console.log(`[updatePoints] pointA:`, pointA, `pointB:`, pointB);

        // 如果需要手動更新，同時更新可見線條和透明覆蓋層
        const [start, end] = offsetLineEnds(this.map, pointA, pointB, 10);

        // console.log(`[updatePoints] 计算后端点 - start:`, start, `end:`, end);
        // console.log(`[updatePoints] 当前箭头大小: ${this.arrowSize}`);

        this.polyline.setLatLngs([start, end]);
        this.clickLayer.setLatLngs([start, end]);

        // 强制箭头 decorator 基于新的线段位置重新绘制
        this.arrow.setPatterns([
            {
                offset: '100%',
                repeat: 0,
                symbol: L.Symbol.arrowHead({
                    pixelSize: this.arrowSize,
                    polygon: true,
                    pathOptions: {
                        color: this.arrowColor,
                        fillColor: this.arrowColor,
                        opacity: this.opacity,
                        fill: true
                    }
                })
            }
        ]);

        // console.log(`[updatePoints] ✅ 箭头已重绘`);
    }

    updateWeight(newWeight) {
        this.polyline.setStyle({ weight: newWeight });
    }

    updateArrowSize(newSize) {
        this.arrowSize = newSize;
        this.arrow.setPatterns([
            {
                offset: '100%',
                repeat: 0,
                symbol: L.Symbol.arrowHead({
                    pixelSize: this.arrowSize,
                    polygon: true,
                    pathOptions: {
                        color: this.arrowColor,
                        fillColor: this.arrowColor,
                        opacity: this.opacity,
                        fill: true
                    }
                })
            }
        ]);
    }

    setColor(color) {
        this.polyline.setStyle({ color });
        this.arrowColor = color;
        this.updateArrowSize(this.arrowSize);
    }

    remove() {
        this.map.removeLayer(this.polyline);
        this.map.removeLayer(this.arrow);
        this.map.removeLayer(this.clickLayer);
    }
}

//在終點 地圖0縮放時會偏移10px 箭頭會畫在這個位置
function offsetLineEnds(map, pointA, pointB, offsetPx = 10) {
    const zoom = map.getZoom();

    // 根据缩放级别调整偏移，保持地图坐标距离不变
    const scale = Math.pow(2, zoom);
    const adjustedOffsetPx = offsetPx * scale;

    // console.log(`[offsetLineEnds] zoom=${zoom}, scale=${scale.toFixed(3)}, offsetPx=${offsetPx}, adjustedOffsetPx=${adjustedOffsetPx.toFixed(2)}`);

    const pA = map.latLngToContainerPoint(pointA);
    const pB = map.latLngToContainerPoint(pointB);

    // console.log(`[offsetLineEnds] 屏幕坐标 pA:`, pA, `pB:`, pB);

    const dx = pB.x - pA.x;
    const dy = pB.y - pA.y;
    const dist = Math.sqrt(dx * dx + dy * dy);

    // console.log(`[offsetLineEnds] 屏幕距离: ${dist.toFixed(2)}px`);

    const ux = dx / dist;
    const uy = dy / dist;

    // 对于太短的线段，按比例缩小偏移量
    let finalOffsetPx = adjustedOffsetPx;
    if (dist <= 2 * adjustedOffsetPx) {
        // 使用线段长度的25%作为偏移，保证始终有空间
        finalOffsetPx = dist * 0.25;
        // console.log(`[offsetLineEnds] 线段较短，调整偏移: ${adjustedOffsetPx.toFixed(2)}px → ${finalOffsetPx.toFixed(2)}px`);
    }

    //起點偏移20%
    const newPA = L.point(pA.x + ux * 0.2 * dist, pA.y + uy * 0.2 * dist);
    //終點偏移finalOffsetPx（调整后的像素）
    const newPB = L.point(pB.x - ux * finalOffsetPx, pB.y - uy * finalOffsetPx);

    const startLatLng = map.containerPointToLatLng(newPA);
    const endLatLng = map.containerPointToLatLng(newPB);

    // console.log(`[offsetLineEnds] 地图坐标 start:`, startLatLng, `end:`, endLatLng);

    return [startLatLng, endLatLng];
}
