/**
 * bitmapEditor.js - 32位位图可视化编辑器
 * 用于编辑 carrier_bitmap 和 carrier_enable_bitmap 字段
 *
 * 位映射关系：
 * - slot 1 = 位 0（二进制最右边）→ 十六进制最右边字符的最低位
 * - slot 32 = 位 31（二进制最左边）→ 十六进制最左边字符的最高位
 *
 * 示例：
 * - FFFFFFFF = 所有 slot (1-32) 启用
 * - 00000001 = 只有 slot 1 启用
 * - FFFFFFF0 = slot 1-4 关闭，5-32 启用
 */

// 当前编辑状态
let currentEditingField = null;
let currentBitmapState = [];

/**
 * 十六进制字符串转二进制数组 (32位)
 * @param {string} hex - 8位十六进制字符串 (如 "FFFFFFFF")
 * @returns {Array<number>} - 32位二进制数组，binary[0] = 位31，binary[31] = 位0
 */
function hexToBinary(hex) {
    // 确保输入为 8 位十六进制
    const paddedHex = hex.padStart(8, '0').toUpperCase();

    // 将每个十六进制字符转换为 4 位二进制
    // binary[0] 对应位 31（最高位），binary[31] 对应位 0（最低位）
    let binary = [];
    for (let i = 0; i < 8; i++) {
        const hexChar = paddedHex[i];
        const nibble = parseInt(hexChar, 16);

        // 提取 4 位二进制（从高位到低位）
        for (let j = 3; j >= 0; j--) {
            binary.push((nibble >> j) & 1);
        }
    }

    return binary;
}

/**
 * 二进制数组转十六进制字符串
 * @param {Array<number>} binary - 32位二进制数组
 * @returns {string} - 8位十六进制字符串
 */
function binaryToHex(binary) {
    let hex = '';

    // 每 4 位二进制转换为 1 位十六进制
    for (let i = 0; i < 32; i += 4) {
        const nibble = (binary[i] << 3) | (binary[i+1] << 2) | (binary[i+2] << 1) | binary[i+3];
        hex += nibble.toString(16).toUpperCase();
    }

    return hex;
}

/**
 * Slot 编号转 binary 数组索引
 * @param {number} slotNumber - Slot 编号 (1-32)
 * @returns {number} - binary 数组索引 (0-31)
 */
function slotToBinaryIndex(slotNumber) {
    // slot 1 = 位 0 = binary[31]
    // slot 32 = 位 31 = binary[0]
    return 32 - slotNumber;
}

/**
 * 打开位图编辑器
 * @param {string} fieldId - 输入框的 ID (carrier_bitmap 或 carrier_enable_bitmap)
 */
function openBitmapEditor(fieldId) {
    currentEditingField = fieldId;

    // 读取当前输入框的值
    const input = document.getElementById(fieldId);
    const currentValue = input.value || '00000000';

    // 转换为二进制数组
    currentBitmapState = hexToBinary(currentValue);

    // 创建或显示 Modal
    createBitmapModal();

    // 渲染格子
    renderBitmapGrid();

    // 显示 Modal
    const modal = document.getElementById('bitmap-editor-modal');
    modal.classList.add('is-active');
}

/**
 * 创建位图编辑器 Modal（如果不存在）
 */
function createBitmapModal() {
    // 检查是否已存在
    if (document.getElementById('bitmap-editor-modal')) {
        return;
    }

    // 创建 Modal HTML
    const modalHTML = `
        <div id="bitmap-editor-modal" class="modal">
            <div class="modal-background" onclick="closeBitmapEditor()"></div>
            <div class="modal-card">
                <header class="modal-card-head">
                    <p class="modal-card-title">
                        <span class="icon">
                            <i class="mdi mdi-grid"></i>
                        </span>
                        位圖編輯器 (32 槽位)
                    </p>
                    <button class="delete" aria-label="close" onclick="closeBitmapEditor()"></button>
                </header>
                <section class="modal-card-body">
                    <div class="content">
                        <p class="subtitle is-6">點擊格子切換狀態</p>

                        <!-- 位图格子容器：左右两区 -->
                        <div id="bitmap-grid-wrapper" class="bitmap-grid-wrapper">
                            <!-- 左区：slot 1-16 -->
                            <div class="bitmap-zone">
                                <div class="zone-label">Slot 1-16 (A面)</div>
                                <div id="bitmap-grid-left" class="bitmap-grid"></div>
                            </div>

                            <!-- 右区：slot 17-32 -->
                            <div class="bitmap-zone">
                                <div class="zone-label">Slot 17-32 (B面)</div>
                                <div id="bitmap-grid-right" class="bitmap-grid"></div>
                            </div>
                        </div>

                        <!-- 当前十六进制值显示 -->
                        <div class="field mt-4">
                            <label class="label">當前十六進制值</label>
                            <div class="control">
                                <input type="text" id="bitmap-hex-preview" class="input is-family-monospace"
                                    readonly style="font-size: 1.2em; text-align: center; letter-spacing: 0.2em;">
                            </div>
                            <p class="help">
                                說明：最右邊字符對應 slot 1-4，最左邊字符對應 slot 29-32
                            </p>
                        </div>

                        <!-- 操作按钮 -->
                        <div class="field is-grouped mt-4">
                            <div class="control">
                                <button class="button is-small is-info is-light" onclick="setAllBits(1)">
                                    <span class="icon"><i class="mdi mdi-checkbox-multiple-marked"></i></span>
                                    <span>全選</span>
                                </button>
                            </div>
                            <div class="control">
                                <button class="button is-small is-info is-light" onclick="setAllBits(0)">
                                    <span class="icon"><i class="mdi mdi-checkbox-multiple-blank-outline"></i></span>
                                    <span>全清</span>
                                </button>
                            </div>
                        </div>
                    </div>
                </section>
                <footer class="modal-card-foot">
                    <button class="button is-success" onclick="saveBitmapEdit()">
                        <span class="icon"><i class="mdi mdi-content-save"></i></span>
                        <span>保存</span>
                    </button>
                    <button class="button" onclick="closeBitmapEditor()">取消</button>
                </footer>
            </div>
        </div>
    `;

    // 插入到页面
    document.body.insertAdjacentHTML('beforeend', modalHTML);
}

/**
 * 渲染位图格子（左右两区，每区 4x4）
 */
function renderBitmapGrid() {
    const leftGrid = document.getElementById('bitmap-grid-left');
    const rightGrid = document.getElementById('bitmap-grid-right');

    if (!leftGrid || !rightGrid) return;

    leftGrid.innerHTML = '';
    rightGrid.innerHTML = '';

    // 左区：slot 1-16 (4行 x 4列)
    for (let row = 0; row < 4; row++) {
        const rowDiv = document.createElement('div');
        rowDiv.className = 'bitmap-row';

        for (let col = 0; col < 4; col++) {
            const slotNumber = row * 4 + col + 1; // 1-16
            const slot = createSlotElement(slotNumber);
            rowDiv.appendChild(slot);
        }

        leftGrid.appendChild(rowDiv);
    }

    // 右区：slot 17-32 (4行 x 4列)
    for (let row = 0; row < 4; row++) {
        const rowDiv = document.createElement('div');
        rowDiv.className = 'bitmap-row';

        for (let col = 0; col < 4; col++) {
            const slotNumber = row * 4 + col + 17; // 17-32
            const slot = createSlotElement(slotNumber);
            rowDiv.appendChild(slot);
        }

        rightGrid.appendChild(rowDiv);
    }

    // 更新十六进制预览
    updateHexPreview();
}

/**
 * 创建单个槽位元素
 * @param {number} slotNumber - 槽位编号 (1-32)
 * @returns {HTMLElement} - 槽位 DOM 元素
 */
function createSlotElement(slotNumber) {
    const binaryIndex = slotToBinaryIndex(slotNumber);
    const slot = document.createElement('div');
    slot.className = 'bitmap-slot';
    slot.dataset.slot = slotNumber;

    // 根据当前状态设置样式
    if (currentBitmapState[binaryIndex] === 1) {
        slot.classList.add('is-active');
    }

    // 显示槽位编号
    slot.innerHTML = `<span class="slot-number">${slotNumber}</span>`;

    // 点击事件
    slot.onclick = () => toggleSlot(slotNumber);

    return slot;
}

/**
 * 切换指定槽位的状态
 * @param {number} slotNumber - 槽位编号 (1-32)
 */
function toggleSlot(slotNumber) {
    const binaryIndex = slotToBinaryIndex(slotNumber);

    // 切换状态
    currentBitmapState[binaryIndex] = currentBitmapState[binaryIndex] === 1 ? 0 : 1;

    // 更新视觉效果
    const slot = document.querySelector(`[data-slot="${slotNumber}"]`);
    if (currentBitmapState[binaryIndex] === 1) {
        slot.classList.add('is-active');
    } else {
        slot.classList.remove('is-active');
    }

    // 更新十六进制预览
    updateHexPreview();
}

/**
 * 设置所有槽位为指定值
 * @param {number} value - 0 或 1
 */
function setAllBits(value) {
    currentBitmapState = new Array(32).fill(value);
    renderBitmapGrid();
}

/**
 * 更新十六进制预览
 */
function updateHexPreview() {
    const hex = binaryToHex(currentBitmapState);
    const preview = document.getElementById('bitmap-hex-preview');
    if (preview) {
        preview.value = hex;
    }
}

/**
 * 保存位图编辑结果
 */
function saveBitmapEdit() {
    if (!currentEditingField) return;

    // 转换为十六进制
    const hex = binaryToHex(currentBitmapState);

    // 更新输入框的值
    const input = document.getElementById(currentEditingField);
    input.value = hex;

    // 触发 change 事件（如果有监听器）
    input.dispatchEvent(new Event('change', { bubbles: true }));

    // 关闭 Modal
    closeBitmapEditor();
}

/**
 * 关闭位图编辑器
 */
function closeBitmapEditor() {
    const modal = document.getElementById('bitmap-editor-modal');
    if (modal) {
        modal.classList.remove('is-active');
    }

    // 清理状态
    currentEditingField = null;
    currentBitmapState = [];
}

// 键盘快捷键支持
document.addEventListener('keydown', (e) => {
    const modal = document.getElementById('bitmap-editor-modal');
    if (modal && modal.classList.contains('is-active')) {
        // ESC 关闭
        if (e.key === 'Escape') {
            closeBitmapEditor();
        }
        // Enter 保存
        if (e.key === 'Enter' && e.ctrlKey) {
            saveBitmapEdit();
        }
    }
});
