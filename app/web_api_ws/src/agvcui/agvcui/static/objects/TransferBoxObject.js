import { BaseObject } from './BaseObject.js';
export class TransferBoxObject extends BaseObject {
  constructor(map, latlng, id) {
    const html = `
<div id="transfer-box-${id}" class="transfer-box-static">
  <div class="door-inside-latch door cell has-radius-left InnerTop" data-door="down" data-door-state="closed">
    <span class="movable-dot"></span>
    </div>
  <div class="cell" data-port="2">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span>
    </div>
  <div class="cell" data-port="1">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span>
    </div>
  <div class="door cell has-radius-right door-outside-latch OuterTop" data-door="up"  data-door-state="opened">
    <span class="movable-dot"></span>
  </div>
  <div class="door-inside-latch door cell has-radius-left InnerBottom" data-door="down" data-door-state="opened">
    <span class="movable-dot"></span>
  </div>
  <div class="cell" data-port="4">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span>
    </div>
  <div class="cell" data-port="3">
    <span class="mdi mdi-transfer-down hidden"></span>
    <span class="mdi mdi-transfer-up hidden"></span>
    <span class="mdi mdi-safety-goggles"></span>
    </div>
  <div class="door-outside-latch door cell has-radius-right OuterBottom" data-door="up" data-door-state="closed">
    <span class="movable-dot"></span>
    </div>
</div>
`;
    super(map, latlng, html, [120, 80], [60, 40]);
    this.id = `transfer-box-${id}`;
    this.eqp_id = id;

    if (!this.el) return;

    const doors = this.el.querySelectorAll('.door');
    doors.forEach((door, i) => {
      switch (i) {
        case 0:
          door.style.transform = 'translateY(-0.25rem)';
          break;
        case 1:
          door.style.transform = 'translateY(-0.5rem)';
          break;
        case 2:
          door.style.transform = 'translateY(0.25rem)';
          break;
        case 3:
          door.style.transform = 'translateY(0)';
          break;
        default:
          door.style.transform = '';
      }
    });
    const ports = this.el.querySelectorAll('[data-port]');
    ports.forEach((port, i) => {
      if (i < 2) {
        port.style.transform = 'translateY(-0.25rem)';
      } else {
        port.style.transform = 'translateY(0.25rem)';
      }
    });
  }
  updateEqpSignals(signalMap) {
    super.updateEqpSignals(signalMap);

    ['InnerTop', 'InnerBottom', 'OuterTop', 'OuterBottom'].forEach(signal_name => {
      //const signal = signalMap[`${this.eqp_id}_${signal_name}`];
      const signal = signalMap.get(`${this.eqp_id}_${signal_name}`);

      if (!signal) return;
      const cell = this.el.querySelector(`.${signal_name}`);
      if (!cell) return;
      switch (signal.value) {
        case "1":
          cell.dataset.door = 'up';
          cell.dataset.doorState = signal_name.includes('Top') ? 'opened' : 'closed';
          break;
        case "0":
          cell.dataset.door = 'mid';
          cell.dataset.doorState = 'closed';
          break;
        case "-1":
          cell.dataset.door = 'down';
          cell.dataset.doorState = signal_name.includes('Bottom') ? 'opened' : 'closed';
          break;
      }
    });

  }
}
