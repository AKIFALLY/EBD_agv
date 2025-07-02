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
        this.polyline = L.polyline([start, end], {
            color: options.color || '#AAA',
            weight: options.weight || 3,
            opacity: this.opacity, // 0~1 透明度
            dashArray: options.dashArray || null
        }).addTo(map);

        this.arrowColor = options.arrowColor || options.color || '#AAA';
        this.arrowSize = options.arrowSize || 10;

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

        // 加入 hover 事件
        this.polyline.on('mouseover', () => {
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

        this.polyline.on('mouseout', () => {
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
        // ✅ 點擊顯示 info
        this.polyline.on('click', () => {
            console.log('click');
            // console.log(this.id);
            this.showInfo();
        });
    }

    showInfo() {
        const panel = document.getElementById('info-panel');
        if (!panel) return;

        panel.innerHTML = `
            <h4>Edge Info</h4>
            <p><strong>ID:</strong> ${this.id}</p>
            <p><strong>Edge ID:</strong> ${this.edgeId}</p>
        `;
        // 顯示
        panel.classList.remove('hidden');
    }

    updatePoints(pointA, pointB) {
        //不再更新,map會自動更新polyline(如果要手動更新,需要同時更新offsetPx)
        //const [start, end] = offsetLineEnds(this.map, pointA, pointB, 10);
        //this.polyline.setLatLngs([start, end]);
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
    }
}

//在終點 地圖0縮放時會偏移10px 箭頭會畫在這個位置
function offsetLineEnds(map, pointA, pointB, offsetPx = 10) {
    const pA = map.latLngToContainerPoint(pointA);
    const pB = map.latLngToContainerPoint(pointB);

    const dx = pB.x - pA.x;
    const dy = pB.y - pA.y;
    const dist = Math.sqrt(dx * dx + dy * dy);

    if (dist <= 2 * offsetPx) {
        // 線太短，無法偏移這麼多，直接返回原點或中點
        return [pointA, pointB];
    }

    const ux = dx / dist;
    const uy = dy / dist;

    //起點偏移20%
    const newPA = L.point(pA.x + ux * 0.2 * dist, pA.y + uy * 0.2 * dist);
    //終點偏移10px
    const newPB = L.point(pB.x - ux * offsetPx, pB.y - uy * offsetPx);

    return [
        map.containerPointToLatLng(newPA),
        map.containerPointToLatLng(newPB)
    ];
}
