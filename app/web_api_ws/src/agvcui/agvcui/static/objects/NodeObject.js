import { BaseObject } from './BaseObject.js';
export class NodeObject extends BaseObject {
  constructor(map, latlng, id, className = '') {
    const html = `
<div id="node-${id}" class="node-static">
<div class="node-icon ${className}"></div>
</div>
`;
    super(map, latlng, html, [20, 20], [10, 10]);
    this.id = `node-${id}`;
    this.nodeId = id;

    // 點擊事件將由 mapObjectManager 統一處理
  }
}
