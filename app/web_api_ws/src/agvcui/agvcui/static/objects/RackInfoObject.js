import { BaseObject } from './BaseObject.js';
export class RackInfoObject extends BaseObject {
  constructor(map, latlng, id, rackName = "RackId") {
    let html = `
<div class="rack">
  <!-- 統一的切換按鈕 -->
  <div class="rack-toggle-btn" id="rack-toggle-${id}" title="切換顯示模式">
    ⇄
  </div>

  <!-- 最小化顯示 -->
  <div id="rack-info-${id}-min" class="rack-moving hidden">
    <span class="rack-name">${rackName}</span>
  </div>

  <!-- 詳細顯示 -->
  <div id="rack-info-${id}-max" class="rack-static ">

    <div class="rack-label has-radius-top-left has-radius-top-right span3">
      <span class="product-name"></span>
      </div>
    <div class="rack-label has-radius-bottom-left">
      <span class="size"></span>
      </div>
    <div class="rack-label has-radius-bottom-right span2">
      <span class="count"></span>
      <span class="slash">/</span>
      <span class="total"></span>
      </div>
    <div class="rack-title has-radius-top-left has-radius-top-right span3">
      <span class="rack-name">${rackName}</span>
      </div>
  </div>
</div>
`;

    // 降低 z-index，讓 AGV 可以在上層
    super(map, latlng, html, [96, 96], [48, 48], 1500);
    this.id = `rack-info-${id}`;
    this.rackId = id;
    this.isMiniMode = true;
    this.attachedAgvId = null;
    this.pendingRackData = null;

    // 使用 setTimeout 確保 DOM 已經插入到頁面中
    setTimeout(() => {
      this.initializeDomElements();
    }, 0);
  }

  // 初始化 DOM 元素
  initializeDomElements() {
    console.debug('Initializing DOM elements for rack:', this.rackId);

    // 通過 this.el 查找子元素（this.el 是 BaseObject 提供的根元素）
    if (this.el) {
      this.minDom = this.el.querySelector(`#rack-info-${this.rackId}-min`);
      this.maxDom = this.el.querySelector(`#rack-info-${this.rackId}-max`);
      this.toggleBtn = this.el.querySelector(`#rack-toggle-${this.rackId}`);

      console.debug('DOM elements found:', {
        minDom: !!this.minDom,
        maxDom: !!this.maxDom,
        toggleBtn: !!this.toggleBtn
      });

      if (this.minDom && this.maxDom) {
        this.rackNameDomMin = this.minDom.querySelector('.rack-name');
        this.rackNameDomMax = this.maxDom.querySelector('.rack-name');
        this.productNameDom = this.maxDom.querySelector('.product-name');
        this.sizeDom = this.maxDom.querySelector('.size');
        this.countDom = this.maxDom.querySelector('.count');
        this.totalDom = this.maxDom.querySelector('.total');

        // 設置切換按鈕的點擊事件
        this.setupToggleButtons();

        // 設置初始狀態
        this.setMini(true);

        // 如果有暫存的資料，立即更新
        if (this.pendingRackData) {
          console.debug('Applying pending rack data for:', this.rackId);
          this.update(this.pendingRackData);
        }

        console.debug('Rack DOM initialization completed for:', this.rackId);
      } else {
        console.error('Failed to find min/max DOM elements for rack:', this.rackId);
      }
    } else {
      console.error('Root element (this.el) not found for rack:', this.rackId);
    }
  }

  // 設置切換按鈕事件
  setupToggleButtons() {
    console.debug('Setting up toggle button for rack:', this.rackId);

    // 設置統一的切換按鈕
    if (this.toggleBtn) {
      this.toggleBtn.addEventListener('click', (e) => {
        e.stopPropagation();
        e.preventDefault();
        console.debug('Toggle button clicked for rack:', this.rackId);
        this.setMini(!this.isMiniMode);
      });
      console.debug('Toggle button setup completed');
    } else {
      console.error('Toggle button not found for rack:', this.rackId);
    }
  }

  update(rack) {
    // 檢查 DOM 元素是否已初始化，如果沒有則暫存資料並稍後更新
    if (!this.rackNameDomMin || !this.rackNameDomMax || !this.productNameDom ||
      !this.sizeDom || !this.countDom || !this.totalDom) {
      console.debug('DOM elements not ready for rack:', this.rackId, 'storing data for later update');
      this.pendingRackData = rack;
      return;
    }

    this.rackNameDomMin.textContent = rack.name || '-';
    this.rackNameDomMax.textContent = rack.name || '-';
    this.productNameDom.textContent = rack.product_name || '-';
    this.sizeDom.textContent = rack.size || '-';
    this.countDom.textContent = rack.count || 0;
    this.totalDom.textContent = rack.total || 0;

    // 清除暫存的資料
    this.pendingRackData = null;
  }

  setMini(isMiniMode) {
    console.debug('setMini called for rack:', this.rackId, 'isMiniMode:', isMiniMode);

    if (!this.minDom || !this.maxDom) {
      console.error('DOM elements not ready for rack:', this.rackId);
      return;
    }

    this.isMiniMode = isMiniMode;

    if (isMiniMode) {
      // 顯示最小化模式，隱藏詳細模式
      this.minDom.classList.remove('hidden');
      this.maxDom.classList.add('hidden');
      // 調整 marker 大小為較小尺寸，避免擋住 AGV
      // iconAnchor 設為圖標大小的一半，確保居中
      this.updateMarkerSize([40, 40], [20, 20], '2.5rem', '2.5rem');
      console.debug('Switched to mini mode for rack:', this.rackId);
    } else {
      // 顯示詳細模式，隱藏最小化模式
      this.minDom.classList.add('hidden');
      this.maxDom.classList.remove('hidden');
      // 恢復 marker 原始大小
      // iconAnchor 設為圖標大小的一半，確保居中
      this.updateMarkerSize([96, 96], [48, 48], '6rem', '6rem');
      console.debug('Switched to detailed mode for rack:', this.rackId);
    }
  }

  // 更新 marker 的圖標大小
  updateMarkerSize(iconSize, iconAnchor, rackWidth, rackHeight) {
    if (this.marker && this.el) {
      // 確保 iconAnchor 是 iconSize 的一半，保持居中
      const calculatedAnchor = iconAnchor || [iconSize[0] / 2, iconSize[1] / 2];

      // 創建新的圖標，保持原有的 HTML 內容
      const newIcon = L.divIcon({
        className: '',
        html: this.el.outerHTML,
        iconSize,
        iconAnchor: calculatedAnchor
      });

      // 更新 marker 的圖標
      this.marker.setIcon(newIcon);

      // 重新獲取 DOM 元素引用
      this.el = this.marker.getElement()?.firstElementChild;

      // 重新初始化 DOM 元素引用並更新 CSS 大小，因為 HTML 被重新創建了
      setTimeout(() => {
        this.reinitializeDomElements();
        // 在 DOM 元素重新初始化後，更新 .rack 元素的 CSS 大小
        if (rackWidth && rackHeight) {
          this.updateRackElementSize(rackWidth, rackHeight);
        }
      }, 0);

      console.debug('Updated marker size for rack:', this.rackId, 'to:', iconSize, 'anchor:', calculatedAnchor, 'CSS size:', rackWidth, 'x', rackHeight);
    }
  }

  // 重新初始化 DOM 元素引用
  reinitializeDomElements() {
    if (this.el) {
      this.minDom = this.el.querySelector(`#rack-info-${this.rackId}-min`);
      this.maxDom = this.el.querySelector(`#rack-info-${this.rackId}-max`);
      this.toggleBtn = this.el.querySelector(`#rack-toggle-${this.rackId}`);

      if (this.minDom && this.maxDom) {
        this.rackNameDomMin = this.minDom.querySelector('.rack-name');
        this.rackNameDomMax = this.maxDom.querySelector('.rack-name');
        this.productNameDom = this.maxDom.querySelector('.product-name');
        this.sizeDom = this.maxDom.querySelector('.size');
        this.countDom = this.maxDom.querySelector('.count');
        this.totalDom = this.maxDom.querySelector('.total');

        // 重新設置切換按鈕事件
        this.setupToggleButtons();

        console.debug('DOM elements reinitialized for rack:', this.rackId);
      }
    }
  }

  // 更新 rack 元素的 CSS 大小
  updateRackElementSize(width, height) {
    console.debug('updateRackElementSize called for rack:', this.rackId, 'with size:', width, 'x', height);

    // 獲取 marker 的完整元素（包含 .rack）
    const markerElement = this.marker.getElement();
    console.debug('Marker element:', markerElement);

    if (markerElement) {
      // 找到 .rack 元素（最外層容器）
      const rackElement = markerElement.querySelector('.rack');
      console.debug('Found rack element:', rackElement);

      if (rackElement) {
        console.debug('Before update rack - width:', rackElement.style.width, 'height:', rackElement.style.height);
        rackElement.style.width = width;
        rackElement.style.height = height;
        console.debug('After update rack - width:', rackElement.style.width, 'height:', rackElement.style.height);
        console.debug('Updated rack element size for:', this.rackId, 'to:', width, 'x', height);
      } else {
        console.error('Could not find .rack element for rack:', this.rackId);
        console.debug('Available elements in markerElement:', markerElement.innerHTML);
      }
    } else {
      console.error('markerElement is null for rack:', this.rackId);
    }
  }
}
