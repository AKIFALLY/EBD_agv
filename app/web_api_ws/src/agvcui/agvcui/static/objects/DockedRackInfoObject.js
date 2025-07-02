import { BaseObject } from './BaseObject.js';
export class DockedRackInfoObject extends BaseObject {
  constructor(map, latlng, id, locationName = "停靠區") {
    let html = `
<div id="docked-rack-info-${id}" class="docked-rack-info">
  <div class="docked-rack-title">${locationName}</div>
  <ul class="docked-rack-list">
    <li></li>
    <li></li>
    <li></li>
    <li></li>
    <li></li>
  </ul>
</div>
`;

    super(map, latlng, html, [96, 64], [48, 32], 2000);
    this.id = `docked-rack-info-${id}`;
    this.rackInfoDom = document.getElementById(`docked-rack-info-${id}`);
    this.rackListDom = this.rackInfoDom.querySelector('.docked-rack-list');
  }

  update(racks) {
    const rackListItems = this.rackListDom.querySelectorAll('li');
    if (racks && racks.length > 0) {
      rackListItems.forEach((li, index) => {
        if (index < racks.length) {
          li.textContent = racks[index].name;
          li.style.display = 'list-item';
        } else {
          li.style.display = 'none';
        }
      });
    }
  }
}
