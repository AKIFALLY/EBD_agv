import { BaseObject } from './BaseObject.js';
export class RackInfoObject extends BaseObject {
  constructor(map, latlng, id, rackName = "RackId") {
    let html = `
<div class="rack">
  <div id="rack-info-${id}-min" class="rack-moving hidden">
      <span class="rack-name">${rackName}</span>
  </div>
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

    super(map, latlng, html, [96, 96], [48, 48], 2000);
    this.id = `rack-info-${id}`;
    this.isMiniMode = true;
    this.attachedAgvId = null;
    this.minDom = document.getElementById(`rack-info-${id}-min`);
    this.maxDom = document.getElementById(`rack-info-${id}-max`);

    this.rackNameDomMin = this.minDom.querySelector('.rack-name');
    this.rackNameDomMax = this.maxDom.querySelector('.rack-name');
    this.productNameDom = this.maxDom.querySelector('.product-name');
    this.sizeDom = this.maxDom.querySelector('.size');
    this.countDom = this.maxDom.querySelector('.count');
    this.totalDom = this.maxDom.querySelector('.total');


    // 點擊事件將由 mapObjectManager 統一處理
    // 保留 setMini 方法供外部調用
    // this.addClickHandler(() => {
    //   this.setMini(!this.isMiniMode);
    // });
    this.setMini(true);
  }

  update(rack) {
    this.rackNameDomMin.textContent = rack.name || '-';
    this.rackNameDomMax.textContent = rack.name || '-';
    this.productNameDom.textContent = rack.product_name || '-';
    this.sizeDom.textContent = rack.size || '-';
    this.countDom.textContent = rack.count || 0;
    this.totalDom.textContent = rack.total || 0;

  }

  setMini(isMiniMode) {
    this.isMiniMode = isMiniMode;
    this.minDom.classList.toggle('hidden', !isMiniMode);
    this.maxDom.classList.toggle('hidden', isMiniMode);
  }
}
