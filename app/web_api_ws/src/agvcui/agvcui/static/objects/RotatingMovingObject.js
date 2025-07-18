import { BaseObject } from './BaseObject.js';
export class RotatingMovingObject extends BaseObject {
    constructor(map, latlng, id, classname) {
        const html = `
            <div id="ct-agv-${id}" class="agv-size rotating-icon ${classname}">
            </div>
        `;
        if (classname === 'agv-kuka') {
            super(map, latlng, html, [52, 72], [26, 36], 1000);
        } else {
            super(map, latlng, html, [80, 96], [40, 48], 1000);
        }
        this.name = id;
        this.id = `ct-agv-${id}`;
        this.angle = 0;
        this.targetAngle = 0;
        this.targetLatLng = latlng;
        this.lastTime = null;
        this.attachedObject = null;

        // 簡化：動畫模式控制
        this.animationMode = 'instant'; // 'instant' 或 'smooth'
        this.lerpSpeed = 8.0; // 插值速度
        this.minDistance = 0.000001; // 最小距離閾值

        // 角度精度控制優化
        this.angleThreshold = 0.1; // 角度精度閾值（度）- 從1度降低到0.1度
        this.minAngleThreshold = 0.01; // 最小角度閾值，避免無限循環

        // 目標點平滑功能
        this.useTargetSmoothing = true; // 是否啟用目標點平滑
        this.targetSmoothSpeed = 6.0; // 目標點平滑速度
        this.smoothTargetLatLng = latlng; // 平滑的目標位置
        this.smoothTargetAngle = 0; // 平滑的目標角度
    }

    attachObject(obj) {
        this.attachedObject = obj;
    }

    detachObject() {
        const detached = this.attachedObject;
        this.attachedObject = null;
        return detached;
    }

    // 簡化：直接設定真實位置和角度，讓動畫系統處理平滑移動
    setRealPosition(latlng, angle) {
        this.targetLatLng = latlng;
        this.targetAngle = angle;
    }

    // 保留舊方法以維持相容性
    setTargetPosition(latlng, heading = null) {
        const targetAngle = (heading == null) ? this._calculateAngleTo(latlng) : heading;
        this.setRealPosition(latlng, targetAngle);
    }

    // 簡化：設定動畫模式和速度（移除 PID 參數）
    setAnimationMode(mode, speed = 8.0) {
        this.animationMode = mode; // 'instant' 或 'smooth'
        this.lerpSpeed = speed;
    }

    // 調整插值速度
    setLerpSpeed(speed) {
        this.lerpSpeed = speed;
    }

    // 設定目標點平滑功能
    setTargetSmoothing(enabled, smoothSpeed = 6.0) {
        this.useTargetSmoothing = enabled;
        this.targetSmoothSpeed = smoothSpeed;

        // 如果停用平滑，立即同步目標點
        if (!enabled) {
            this.smoothTargetLatLng = L.latLng(this.targetLatLng.lat, this.targetLatLng.lng);
            this.smoothTargetAngle = this.targetAngle;
        }
    }
    _calculateAngleTo(targetLatLng) {
        const from = this.map.latLngToLayerPoint(this.latlng);
        const to = this.map.latLngToLayerPoint(targetLatLng);
        return Math.atan2(to.y - from.y, to.x - from.x) * (180 / Math.PI) + 90;
    }

    // 優化的角度差異計算，處理角度環繞問題
    _calculateAngleDifference(targetAngle, currentAngle) {
        let diff = targetAngle - currentAngle;

        // 處理角度環繞，選擇最短路徑
        if (diff > 180) {
            diff -= 360;
        } else if (diff < -180) {
            diff += 360;
        }

        return diff;
    }

    // 標準化角度到 0-360 範圍
    _normalizeAngle(angle) {
        while (angle < 0) angle += 360;
        while (angle >= 360) angle -= 360;
        return angle;
    }




    updateTransform(delta) {
        // 首先更新平滑目標點
        if (this.useTargetSmoothing && this.animationMode === 'smooth') {
            this.updateSmoothTarget(delta);
        } else {
            // 不使用目標平滑時，直接同步
            this.smoothTargetLatLng = L.latLng(this.targetLatLng.lat, this.targetLatLng.lng);
            this.smoothTargetAngle = this.targetAngle;
        }

        if (this.animationMode === 'instant') {
            // 即時模式：直接跳到目標位置
            this.angle = this.targetAngle;
            this.latlng = L.latLng(this.targetLatLng.lat, this.targetLatLng.lng);
        } else {
            // 平滑模式：使用線性插值（朝向平滑目標點移動）
            this.updateWithLinearInterpolation(delta);
        }

        // 更新 marker 位置
        this.updateLatLng(this.latlng);

        // 更新附加物件位置
        if (this.attachedObject) {
            this.attachedObject.updateLatLng(this.latlng);
        }

        // 更新 CSS transform
        if (!this.el) return;
        const scale = Math.pow(2, this.map.getZoom());
        this.el.style.transform = `rotate(${this.angle}deg) scale(${scale})`;
    }

    // 更新平滑目標點
    updateSmoothTarget(delta) {
        const currentSmoothLat = this.smoothTargetLatLng.lat;
        const currentSmoothLng = this.smoothTargetLatLng.lng;
        const targetLat = this.targetLatLng.lat;
        const targetLng = this.targetLatLng.lng;

        // 計算目標點位置差異
        const latDiff = targetLat - currentSmoothLat;
        const lngDiff = targetLng - currentSmoothLng;
        const distance = Math.sqrt(latDiff * latDiff + lngDiff * lngDiff);

        // 計算目標角度差異 - 使用優化的角度差異計算
        const angleDiff = this._calculateAngleDifference(this.targetAngle, this.smoothTargetAngle);

        // 如果差異很小，直接設定到目標
        if (distance <= this.minDistance && Math.abs(angleDiff) <= this.minAngleThreshold) {
            this.smoothTargetLatLng = L.latLng(targetLat, targetLng);
            this.smoothTargetAngle = this.targetAngle;
        } else {
            // 計算目標點平滑係數
            const smoothFactor = Math.min(1.0, this.targetSmoothSpeed * delta);

            // 平滑目標位置
            if (distance > this.minDistance) {
                const newLat = currentSmoothLat + latDiff * smoothFactor;
                const newLng = currentSmoothLng + lngDiff * smoothFactor;
                this.smoothTargetLatLng = L.latLng(newLat, newLng);
            }

            // 平滑目標角度 - 使用更精確的閾值
            if (Math.abs(angleDiff) > this.minAngleThreshold) {
                this.smoothTargetAngle += angleDiff * smoothFactor;
                this.smoothTargetAngle = this._normalizeAngle(this.smoothTargetAngle);
            }
        }
    }

    // 線性插值更新（現在朝向平滑目標點移動）
    updateWithLinearInterpolation(delta) {
        const currentLat = this.latlng.lat;
        const currentLng = this.latlng.lng;
        const targetLat = this.smoothTargetLatLng.lat;
        const targetLng = this.smoothTargetLatLng.lng;

        // 計算位置差異
        const latDiff = targetLat - currentLat;
        const lngDiff = targetLng - currentLng;
        const distance = Math.sqrt(latDiff * latDiff + lngDiff * lngDiff);

        // 計算角度差異 - 使用優化的角度差異計算
        const angleDiff = this._calculateAngleDifference(this.smoothTargetAngle, this.angle);

        // 優化的角度和位置更新邏輯
        if (distance <= this.minDistance && Math.abs(angleDiff) <= this.minAngleThreshold) {
            this.latlng = L.latLng(targetLat, targetLng);
            this.angle = this.smoothTargetAngle;
        } else {
            // 計算插值係數
            const lerpFactor = Math.min(1.0, this.lerpSpeed * delta);

            // 位置插值
            if (distance > this.minDistance) {
                const newLat = currentLat + latDiff * lerpFactor;
                const newLng = currentLng + lngDiff * lerpFactor;
                this.latlng = L.latLng(newLat, newLng);
            }

            // 優化的角度插值 - 使用動態速度調整
            if (Math.abs(angleDiff) > this.minAngleThreshold) {
                // 根據角度差異調整插值速度，角度差異越小速度越慢，實現平滑過渡
                let dynamicLerpFactor = lerpFactor;
                if (Math.abs(angleDiff) < this.angleThreshold) {
                    // 當接近目標角度時，降低插值速度以實現更平滑的過渡
                    const proximityFactor = Math.abs(angleDiff) / this.angleThreshold;
                    dynamicLerpFactor = lerpFactor * Math.max(0.1, proximityFactor);
                }

                this.angle += angleDiff * dynamicLerpFactor;
                this.angle = this._normalizeAngle(this.angle);
            }
        }
    }




}
