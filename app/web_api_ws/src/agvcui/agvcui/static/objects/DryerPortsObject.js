import { BaseObject } from './BaseObject.js';
export class DryerPortsObject extends BaseObject {
  constructor(map, latlng, id) {
    const html = `
<div id="dryer-ports-${id}" class="dryer-ports-static">
  <div class="dryer-top cell has-radius-top-left" data-port="4">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="dryer-bottom cell has-radius-top-right"  data-port="8">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="dryer-top cell" data-port="3">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="dryer-bottom cell" data-port="7">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="dryer-top cell" data-port="2">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="dryer-bottom cell" data-port="6">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="dryer-top cell has-radius-bottom-left" data-port="1">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="dryer-bottom cell has-radius-bottom-right" data-port="5">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
</div>
`;
    super(map, latlng, html, [80, 160], [40, 80]);
    this.id = `dryer-ports-${id}`;
    this.eqp_id = id;

    if (!this.el) return;

    this.el.querySelectorAll('.dryer-ports-static .dryer-top').forEach((el, index) => {
      if (index < 2) {
        el.style.transform = 'translateY(-0.75rem) translateX(-0.25rem)';
      } else {
        el.style.transform = 'translateY(-0.5rem) translateX(-0.25rem)';
      }
    });
    this.el.querySelectorAll('.dryer-ports-static .dryer-bottom').forEach((el, index) => {
      if (index < 2) {
        el.style.transform = 'translateY(-0.25rem)';
      } else {
        el.style.transform = '';
      }
    });
  }
}
