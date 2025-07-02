#!/usr/bin/env python3
"""
批量更新剩餘頁面的腳本
將 racks, products, rosout_logs, runtime_logs 頁面統一改成新風格
"""

import os
import re

def update_racks_page():
    """更新 racks 頁面"""
    template_path = "../../../agvcui/templates/racks.html"
    
    new_content = '''{% extends "base.html" %}
{% block title %}貨架管理{% endblock %}

{% block content %}
<section class="section">
    <div class="container">
        <div class="level">
            <div class="level-left">
                <div class="level-item">
                    <h1 class="title">貨架管理</h1>
                </div>
            </div>
            <div class="level-right">
                <div class="level-item">
                    {% if current_user and current_user.role in ['operator', 'admin'] %}
                    <a class="button is-primary" href="/racks/create">
                        <span class="icon">
                            <i class="mdi mdi-plus"></i>
                        </span>
                        <span>新增貨架</span>
                    </a>
                    {% endif %}
                </div>
            </div>
        </div>

        <div class="box">
            <table class="table is-fullwidth is-hoverable">
                <thead>
                    <tr>
                        <th>ID</th>
                        <th>房間 ID</th>
                        <th>貨架名稱</th>
                        <th>描述</th>
                        <th>狀態</th>
                        {% if current_user and current_user.role in ['operator', 'admin'] %}
                        <th>操作</th>
                        {% endif %}
                    </tr>
                </thead>
                <tbody>
                    {% for rack in racks %}
                    <tr>
                        <td><strong>{{ rack.id }}</strong></td>
                        <td>
                            {% if rack.room_id %}
                            <span class="tag is-info">{{ rack.room_id }}</span>
                            {% else %}
                            <span class="tag is-light">未設置</span>
                            {% endif %}
                        </td>
                        <td>
                            <strong>{{ rack.name or '未命名' }}</strong>
                            {% if rack.description %}
                            <br><small class="has-text-grey">{{ rack.description[:30] }}{% if rack.description|length > 30 %}...{% endif %}</small>
                            {% endif %}
                        </td>
                        <td>{{ rack.description or '-' }}</td>
                        <td>
                            {% if rack.status_id == 1 %}
                            <span class="tag is-success">可用</span>
                            {% elif rack.status_id == 2 %}
                            <span class="tag is-warning">使用中</span>
                            {% elif rack.status_id == 3 %}
                            <span class="tag is-danger">維護中</span>
                            {% else %}
                            <span class="tag is-light">{{ rack.status_id or '未知' }}</span>
                            {% endif %}
                        </td>
                        {% if current_user and current_user.role in ['operator', 'admin'] %}
                        <td>
                            <div class="buttons are-small">
                                <a class="button is-info is-small" href="/racks/{{ rack.id }}/edit">
                                    <span class="icon">
                                        <i class="mdi mdi-pencil"></i>
                                    </span>
                                    <span>編輯</span>
                                </a>
                                {% if current_user.role == 'admin' %}
                                <button class="button is-danger is-small" onclick="deleteRack({{ rack.id }}, '{{ rack.name or rack.id }}')">
                                    <span class="icon">
                                        <i class="mdi mdi-delete"></i>
                                    </span>
                                    <span>刪除</span>
                                </button>
                                {% endif %}
                            </div>
                        </td>
                        {% endif %}
                    </tr>
                    {% endfor %}
                </tbody>
            </table>
        </div>

        <p class="has-text-grey">最後查詢時間: {{ latest_query }}</p>

        <!-- 分頁 -->
        {% if total_pages > 1 %}
        <nav class="pagination is-centered mt-4" role="navigation" aria-label="pagination">
            {% if current_page > 1 %}
            <a class="pagination-previous" href="/racks?page={{ current_page - 1 }}">上一頁</a>
            {% endif %}
            {% if current_page < total_pages %}
            <a class="pagination-next" href="/racks?page={{ current_page + 1 }}">下一頁</a>
            {% endif %}
            
            <ul class="pagination-list">
                {% for page_num in range(1, total_pages + 1) %}
                    {% if page_num == current_page %}
                    <li><a class="pagination-link is-current">{{ page_num }}</a></li>
                    {% else %}
                    <li><a class="pagination-link" href="/racks?page={{ page_num }}">{{ page_num }}</a></li>
                    {% endif %}
                {% endfor %}
            </ul>
        </nav>
        {% endif %}
    </div>
</section>

<!-- 刪除確認模態框 -->
{% if current_user and current_user.role == 'admin' %}
<div class="modal" id="deleteModal">
    <div class="modal-background"></div>
    <div class="modal-card">
        <header class="modal-card-head">
            <p class="modal-card-title">確認刪除</p>
            <button class="delete" aria-label="close" onclick="closeDeleteModal()"></button>
        </header>
        <section class="modal-card-body">
            <p>您確定要刪除貨架 <strong id="deleteRackName"></strong> 嗎？</p>
            <p class="has-text-danger">此操作無法撤銷！</p>
        </section>
        <footer class="modal-card-foot">
            <form id="deleteForm" method="post">
                <button class="button is-danger" type="submit">確認刪除</button>
                <button class="button" type="button" onclick="closeDeleteModal()">取消</button>
            </form>
        </footer>
    </div>
</div>

<script>
function deleteRack(rackId, rackName) {
    document.getElementById('deleteRackName').textContent = rackName;
    document.getElementById('deleteForm').action = `/racks/${rackId}/delete`;
    document.getElementById('deleteModal').classList.add('is-active');
}

function closeDeleteModal() {
    document.getElementById('deleteModal').classList.remove('is-active');
}
</script>
{% endif %}
{% endblock %}'''
    
    return new_content


def update_products_page():
    """更新 products 頁面"""
    return '''{% extends "base.html" %}
{% block title %}產品管理{% endblock %}

{% block content %}
<section class="section">
    <div class="container">
        <div class="level">
            <div class="level-left">
                <div class="level-item">
                    <h1 class="title">產品管理</h1>
                </div>
            </div>
            <div class="level-right">
                <div class="level-item">
                    {% if current_user and current_user.role in ['operator', 'admin'] %}
                    <a class="button is-primary" href="/products/create">
                        <span class="icon">
                            <i class="mdi mdi-plus"></i>
                        </span>
                        <span>新增產品</span>
                    </a>
                    {% endif %}
                </div>
            </div>
        </div>

        <div class="box">
            <table class="table is-fullwidth is-hoverable">
                <thead>
                    <tr>
                        <th>ID</th>
                        <th>產品名稱</th>
                        <th>描述</th>
                        <th>狀態</th>
                        <th>創建時間</th>
                        {% if current_user and current_user.role in ['operator', 'admin'] %}
                        <th>操作</th>
                        {% endif %}
                    </tr>
                </thead>
                <tbody>
                    {% for product in products %}
                    <tr>
                        <td><strong>{{ product.id }}</strong></td>
                        <td>
                            <strong>{{ product.name or '未命名' }}</strong>
                            {% if product.description %}
                            <br><small class="has-text-grey">{{ product.description[:30] }}{% if product.description|length > 30 %}...{% endif %}</small>
                            {% endif %}
                        </td>
                        <td>{{ product.description or '-' }}</td>
                        <td>
                            {% if product.status == 'active' %}
                            <span class="tag is-success">啟用</span>
                            {% elif product.status == 'inactive' %}
                            <span class="tag is-danger">停用</span>
                            {% else %}
                            <span class="tag is-light">{{ product.status or '未知' }}</span>
                            {% endif %}
                        </td>
                        <td>{{ product.created_at.strftime('%Y-%m-%d %H:%M') if product.created_at else '-' }}</td>
                        {% if current_user and current_user.role in ['operator', 'admin'] %}
                        <td>
                            <div class="buttons are-small">
                                <a class="button is-info is-small" href="/products/{{ product.id }}/edit">
                                    <span class="icon">
                                        <i class="mdi mdi-pencil"></i>
                                    </span>
                                    <span>編輯</span>
                                </a>
                                {% if current_user.role == 'admin' %}
                                <button class="button is-danger is-small" onclick="deleteProduct({{ product.id }}, '{{ product.name or product.id }}')">
                                    <span class="icon">
                                        <i class="mdi mdi-delete"></i>
                                    </span>
                                    <span>刪除</span>
                                </button>
                                {% endif %}
                            </div>
                        </td>
                        {% endif %}
                    </tr>
                    {% endfor %}
                </tbody>
            </table>
        </div>

        <p class="has-text-grey">最後查詢時間: {{ latest_query }}</p>

        <!-- 分頁 -->
        {% if total_pages > 1 %}
        <nav class="pagination is-centered mt-4" role="navigation" aria-label="pagination">
            {% if current_page > 1 %}
            <a class="pagination-previous" href="/products?page={{ current_page - 1 }}">上一頁</a>
            {% endif %}
            {% if current_page < total_pages %}
            <a class="pagination-next" href="/products?page={{ current_page + 1 }}">下一頁</a>
            {% endif %}
            
            <ul class="pagination-list">
                {% for page_num in range(1, total_pages + 1) %}
                    {% if page_num == current_page %}
                    <li><a class="pagination-link is-current">{{ page_num }}</a></li>
                    {% else %}
                    <li><a class="pagination-link" href="/products?page={{ page_num }}">{{ page_num }}</a></li>
                    {% endif %}
                {% endfor %}
            </ul>
        </nav>
        {% endif %}
    </div>
</section>

<!-- 刪除確認模態框 -->
{% if current_user and current_user.role == 'admin' %}
<div class="modal" id="deleteModal">
    <div class="modal-background"></div>
    <div class="modal-card">
        <header class="modal-card-head">
            <p class="modal-card-title">確認刪除</p>
            <button class="delete" aria-label="close" onclick="closeDeleteModal()"></button>
        </header>
        <section class="modal-card-body">
            <p>您確定要刪除產品 <strong id="deleteProductName"></strong> 嗎？</p>
            <p class="has-text-danger">此操作無法撤銷！</p>
        </section>
        <footer class="modal-card-foot">
            <form id="deleteForm" method="post">
                <button class="button is-danger" type="submit">確認刪除</button>
                <button class="button" type="button" onclick="closeDeleteModal()">取消</button>
            </form>
        </footer>
    </div>
</div>

<script>
function deleteProduct(productId, productName) {
    document.getElementById('deleteProductName').textContent = productName;
    document.getElementById('deleteForm').action = `/products/${productId}/delete`;
    document.getElementById('deleteModal').classList.add('is-active');
}

function closeDeleteModal() {
    document.getElementById('deleteModal').classList.remove('is-active');
}
</script>
{% endif %}
{% endblock %}'''


def main():
    """主函數"""
    print("批量更新剩餘頁面腳本")
    print("此腳本提供了 racks 和 products 頁面的完整模板")
    print("請手動複製內容到對應的模板文件中")
    
    print("\n=== racks.html 模板 ===")
    print("文件路徑: agvcui/templates/racks.html")
    print("請將以下內容替換整個文件:")
    print(update_racks_page())
    
    print("\n=== products.html 模板 ===")
    print("文件路徑: agvcui/templates/products.html")
    print("請將以下內容替換整個文件:")
    print(update_products_page())
    
    print("\n=== 日誌頁面說明 ===")
    print("rosout_logs.html 和 runtime_logs.html 主要是日誌顯示")
    print("建議保持簡潔，只需要:")
    print("1. 使用 level 布局")
    print("2. 表格包裹在 box 中")
    print("3. 統一分頁")
    print("4. 不需要編輯/刪除操作")


if __name__ == "__main__":
    main()
