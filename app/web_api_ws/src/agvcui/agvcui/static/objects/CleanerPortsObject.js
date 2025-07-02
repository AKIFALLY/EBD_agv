import { BaseObject } from './BaseObject.js';
export class CleanerPortsObject extends BaseObject {
    constructor(map, latlng, id) {
        const html = `
<div id="cleaner-ports-${id}" class="cleaner-ports-static">
  <div class="cleaner-top cell has-radius-top-left" data-port="2">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="cleaner-top cell has-radius-top-right" data-port="1">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="cleaner-bottom cell has-radius-bottom-left" data-port="4">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="cleaner-bottom cell has-radius-bottom-right" data-port="3">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
</div>
`;
        super(map, latlng, html, [80, 80], [40, 40]);
        this.id = `cleaner-ports-${id}`;
        this.eqp_id = id;

        if (!this.el) return;

        this.el.querySelectorAll('.cleaner-ports-static .cleaner-top').forEach((el, index) => {
            if (index < 1) {
                el.style.transform = 'translateY(-0.25rem) translateX(-0.25rem)';
            } else {
                el.style.transform = 'translateY(-0.25rem)';
            }
        });
        this.el.querySelectorAll('.cleaner-ports-static .cleaner-bottom').forEach((el, index) => {
            if (index < 1) {
                el.style.transform = 'translateX(-0.25rem)';
            } else {
                el.style.transform = '';
            }
        });
    }
}
