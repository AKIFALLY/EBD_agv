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

        // 計算目標角度差異
        let angleDiff = this.targetAngle - this.smoothTargetAngle;
        if (angleDiff > 180) angleDiff -= 360;
        if (angleDiff < -180) angleDiff += 360;

        // 如果差異很小，直接設定到目標
        if (distance <= this.minDistance && Math.abs(angleDiff) <= 1) {
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

            // 平滑目標角度
            if (Math.abs(angleDiff) > 1) {
                this.smoothTargetAngle += angleDiff * smoothFactor;

                // 確保角度在 0-360 範圍內
                if (this.smoothTargetAngle < 0) this.smoothTargetAngle += 360;
                if (this.smoothTargetAngle >= 360) this.smoothTargetAngle -= 360;
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

        // 計算角度差異
        let angleDiff = this.smoothTargetAngle - this.angle;
        if (angleDiff > 180) angleDiff -= 360;
        if (angleDiff < -180) angleDiff += 360;

        // 如果距離和角度差異都很小，直接設定到目標
        if (distance <= this.minDistance && Math.abs(angleDiff) <= 1) {
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

            // 角度插值
            if (Math.abs(angleDiff) > 1) {
                this.angle += angleDiff * lerpFactor;

                // 確保角度在 0-360 範圍內
                if (this.angle < 0) this.angle += 360;
                if (this.angle >= 360) this.angle -= 360;
            }
        }
    }




}
