import { BaseObject } from './BaseObject.js';
export class EqpInfoObject extends BaseObject {
  constructor(map, latlng, id, title = "EqpTitle", with_counter = false) {
    let html = '';
    if (with_counter) {
      html = `
<div id="info-${id}" class="info-static">
  <div class="info-title has-radius-top-left has-radius-top-right">
    <span>${title}</span>   
    </div>
  <div class="info-label has-radius-bottom-left has-radius-bottom-right">
    
    <span class="mdi mdi-safety-goggles"></span>
    <span class="mdi mdi-close"></span>
    <span class="count">100</span>
    </div>
</div>
`;
    } else {
      html = `
<div id="info-${id}" class="info-static">
  <div class="info-title has-radius-top-left has-radius-top-right has-radius-bottom-left has-radius-bottom-right">
    <span>${title}</span>   
    </div>
</div>
`;
    }
    super(map, latlng, html, [120, 72], [60, 36], 2000);
    this.id = `info-${id}`;
  }

  updateEqpCount(count) {
    const infoElem = document.getElementById(this.id);
    if (infoElem) {
      const countElem = infoElem.querySelector('.count');
      if (countElem) {
        countElem.textContent = count;
      }
    }
  }
}
