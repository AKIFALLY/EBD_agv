import { BaseObject } from './BaseObject.js';
export class SoakingPortsObject extends BaseObject {
  constructor(map, latlng, id) {
    const html = `
<div id="soaking-ports-${id}" class="soaking-ports-static">
  <div class="cell has-radius-top-left  has-radius-bottom-left" data-port="6">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="cell" data-port="5">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="cell" data-port="4">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="cell" data-port="3">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="cell" data-port="2">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
  <div class="cell has-radius-top-right has-radius-bottom-right" data-port="1">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span></div>
</div>
`;
    super(map, latlng, html, [240, 40], [120, 20]);
    this.id = `soaking-ports-${id}`;
    this.eqp_id = id;
  }
}
