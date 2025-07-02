import { BaseObject } from './BaseObject.js';
export class OvenPortsObject extends BaseObject {
  constructor(map, latlng, id) {
    const html = `
<div id="oven-ports-${id}" class="oven-ports-static">
  <div class="oven-top cell has-radius-top-left" data-port="4">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="oven-top cell " data-port="3">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="oven-top cell " data-port="2">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="oven-top cell has-radius-top-right" data-port="1">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="oven-bottom cell has-radius-bottom-left" data-port="8">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="oven-bottom cell " data-port="7">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="oven-bottom cell " data-port="6">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="oven-bottom cell has-radius-bottom-right" data-port="5">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
</div>
`;
    super(map, latlng, html, [160, 80], [80, 40]);
    this.id = `oven-ports-${id}`;
    this.eqp_id = id;

    if (!this.el) return;

    this.el.querySelectorAll('.oven-ports-static .oven-top').forEach((el, index) => {
      if (index < 2) {
        el.style.transform = 'translateY(-0.25rem) translateX(-0.25rem)';
      } else {
        el.style.transform = 'translateY(-0.25rem)';
      }
    });
    this.el.querySelectorAll('.oven-ports-static .oven-bottom').forEach((el, index) => {
      if (index < 2) {
        el.style.transform = 'translateX(-0.25rem)';
      } else {
        el.style.transform = '';
      }
    });
  }
}
