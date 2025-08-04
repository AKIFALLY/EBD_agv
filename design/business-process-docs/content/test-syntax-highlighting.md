# 語法高亮測試與展示

## 🎨 Prism.js 語法高亮展示

本頁面展示了 RosAGV 文檔系統中整合的 Prism.js 語法高亮功能，支援多種程式語言的代碼美化顯示。

## 🐍 Python 代碼範例

### ROS 2 Python 節點

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class AGVController(Node):
    """AGV 控制節點"""
    
    def __init__(self):
        super().__init__('agv_controller')
        
        # 創建發布者和訂閱者
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_subscription = self.create_subscription(
            String, 'agv_status', self.status_callback, 10)
        
        # 定時器
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('AGV Controller 已啟動')
    
    def status_callback(self, msg):
        """處理狀態訊息"""
        self.get_logger().info(f'收到狀態: {msg.data}')
    
    def control_loop(self):
        """主控制迴圈"""
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 0.5
        self.cmd_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    controller = AGVController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('收到中斷信號')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### PLC 通訊範例

```python
class KeyencePlcCom:
    """Keyence PLC 通訊類別"""
    
    def __init__(self, ip: str, port: int = 8501):
        self.ip = ip
        self.port = port
        self.socket = None
        self.connected = False
        
        # 錯誤碼對應
        self.error_messages = {
            "E0": "E0:元件編號異常",
            "E1": "E1:指令異常", 
            "E4": "E4:禁止寫入"
        }
    
    async def connect(self) -> bool:
        """建立 PLC 連接"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            await self.socket.connect((self.ip, self.port))
            self.connected = True
            return True
        except Exception as e:
            self.logger.error(f"PLC 連接失敗: {e}")
            return False
    
    def read_data(self, device: str, address: str) -> str:
        """讀取 PLC 資料"""
        command = f"RD {device}{address}\r\n"
        response = self._send_command(command)
        
        if response.startswith(('E0', 'E1', 'E4')):
            raise PLCError(self.error_messages.get(response[:2], response))
        
        return response.strip()

# 使用範例
async def main():
    plc = KeyencePlcCom("192.168.2.101", 8501)
    
    if await plc.connect():
        try:
            value = plc.read_data("DM", "2990")
            print(f"讀取到的值: {value}")
        except PLCError as e:
            print(f"PLC 錯誤: {e}")
        finally:
            plc.disconnect()
```

## 🟨 JavaScript 代碼範例

### 導航管理器

```javascript
class NavigationManager {
    constructor() {
        this.currentPath = '';
        this.navigationConfig = this.getDefaultConfig();
        this.currentRole = 'all';
        
        this.initRouter();
        this.bindEvents();
    }

    /**
     * 載入內容
     * @param {string} path - 文件路徑
     */
    async loadContent(path) {
        const contentDiv = document.getElementById('main-content');
        
        try {
            // 顯示載入中
            contentDiv.innerHTML = '<div class="loading">📄 載入中...</div>';
            
            // 載入 Markdown 內容
            const html = await window.contentLoader.loadMarkdown(path);
            contentDiv.innerHTML = `<div class="prose max-w-none">${html}</div>`;
            
            // 觸發語法高亮
            if (typeof Prism !== 'undefined') {
                Prism.highlightAllUnder(contentDiv);
            }
            
        } catch (error) {
            console.error('載入內容失敗:', error);
            this.showError(path, error);
        }
    }
    
    navigateTo(path) {
        if (path.endsWith('.md')) {
            window.location.hash = path;
        } else {
            window.location.hash = path;
        }
    }
}

// 全域實例
document.addEventListener('DOMContentLoaded', () => {
    window.navigationManager = new NavigationManager();
});
```

## 🐚 Bash 腳本範例

### Docker 容器管理

```bash
#!/bin/bash
# AGVC 系統管理腳本

set -e  # 遇到錯誤立即退出

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日誌函數
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# AGVC 系統啟動函數
agvc_start() {
    log_info "啟動 AGVC 系統..."
    
    # 檢查 Docker 是否運行
    if ! docker info >/dev/null 2>&1; then
        log_error "Docker 未運行，請先啟動 Docker"
        return 1
    fi
    
    # 啟動服務
    docker compose -f docker-compose.agvc.yml up -d
    
    if [ $? -eq 0 ]; then
        log_success "AGVC 系統啟動成功"
        
        # 等待服務就緒
        log_info "等待服務初始化..."
        sleep 10
        
        # 檢查服務狀態
        agvc_status
    else
        log_error "AGVC 系統啟動失敗"
        return 1
    fi
}

# 檢查服務狀態
agvc_status() {
    log_info "檢查 AGVC 系統狀態..."
    
    # 檢查容器狀態
    containers=$(docker compose -f docker-compose.agvc.yml ps --format "table {{.Name}}\t{{.Status}}")
    echo "$containers"
    
    # 檢查端口
    ports=(8000 8001 8002 5432 5050)
    for port in "${ports[@]}"; do
        if ss -tuln | grep -q ":$port "; then
            log_success "端口 $port 正在監聽"
        else
            log_error "端口 $port 未開放"
        fi
    done
}

# 主程式
case "$1" in
    start)
        agvc_start
        ;;
    stop)
        log_info "停止 AGVC 系統..."
        docker compose -f docker-compose.agvc.yml down
        log_success "AGVC 系統已停止"
        ;;
    restart)
        log_info "重啟 AGVC 系統..."
        docker compose -f docker-compose.agvc.yml restart
        log_success "AGVC 系統重啟完成"
        ;;
    status)
        agvc_status
        ;;
    *)
        echo "使用方法: $0 {start|stop|restart|status}"
        exit 1
        ;;
esac
```

## 📄 YAML 配置範例

### Docker Compose 配置

```yaml
version: '3.8'

services:
  agvc_server:
    build:
      context: .
      dockerfile: Dockerfile.agvc
    container_name: agvc_server
    hostname: agvc_server
    networks:
      agvc_network:
        ipv4_address: 192.168.100.100
    ports:
      - "8000:8000"  # API Server
      - "8001:8001"  # AGVCUI
      - "8002:8002"  # OPUI
    volumes:
      - ./app:/app
      - /etc/timezone:/etc/timezone:ro
      - /etc/localtime:/etc/localtime:ro
    environment:
      - ROS_DOMAIN_ID=42
      - RMW_IMPLEMENTATION=rmw_zenohd
      - POSTGRES_HOST=postgres
      - POSTGRES_DB=agvc
      - POSTGRES_USER=agvc
      - POSTGRES_PASSWORD=agvc123
    depends_on:
      - postgres
    restart: unless-stopped
    stdin_open: true
    tty: true

  postgres:
    image: postgres:16
    container_name: postgres_container
    hostname: postgres
    networks:
      agvc_network:
        ipv4_address: 192.168.100.254
    ports:
      - "5432:5432"
    volumes:
      - postgres_data:/var/lib/postgresql/data
      - ./scripts/init-db.sql:/docker-entrypoint-initdb.d/init-db.sql
    environment:
      - POSTGRES_DB=agvc
      - POSTGRES_USER=agvc
      - POSTGRES_PASSWORD=agvc123
      - POSTGRES_INITDB_ARGS=--encoding=UTF-8 --lc-collate=C --lc-ctype=C
    restart: unless-stopped

  nginx:
    image: nginx:alpine
    container_name: nginx
    hostname: nginx
    networks:
      agvc_network:
        ipv4_address: 192.168.100.200
    ports:
      - "80:80"
    volumes:
      - ./nginx/default.conf:/etc/nginx/conf.d/default.conf
    depends_on:
      - agvc_server
    restart: unless-stopped

networks:
  agvc_network:
    driver: bridge
    ipam:
      config:
        - subnet: "192.168.100.0/24"
          gateway: "192.168.100.1"

volumes:
  postgres_data:
    driver: local
```

## 🔧 JSON 配置範例

### Zenoh Router 配置

```json
{
  "mode": "router",
  "connect": {
    "endpoints": []
  },
  "listen": {
    "endpoints": [
      "tcp/0.0.0.0:7447"
    ]
  },
  "scouting": {
    "multicast": {
      "enabled": true,
      "address": "224.0.0.224:7446",
      "interface": "auto",
      "ttl": 1
    }
  },
  "timestamping": {
    "enabled": true,
    "drop_future_timestamp": false
  },
  "queries_default_timeout": 10000,
  "routing": {
    "face": {
      "unicast": {
        "accept_timeout": 10000,
        "max_sessions": 1000
      }
    }
  },
  "transport": {
    "unicast": {
      "lowlatency": true,
      "qos": {
        "enabled": true
      }
    },
    "multicast": {
      "qos": {
        "enabled": true
      }
    }
  }
}
```

## 🗄️ SQL 查詢範例

### 資料庫結構

```sql
-- AGV 狀態表
CREATE TABLE agv_status (
    id SERIAL PRIMARY KEY,
    agv_id VARCHAR(50) NOT NULL,
    status VARCHAR(20) NOT NULL,
    position_x DECIMAL(10,3),
    position_y DECIMAL(10,3),
    orientation DECIMAL(10,3),
    battery_level INTEGER,
    current_task_id INTEGER,
    last_updated TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- 任務表
CREATE TABLE tasks (
    id SERIAL PRIMARY KEY,
    task_type VARCHAR(50) NOT NULL,
    priority INTEGER DEFAULT 5,
    status VARCHAR(20) DEFAULT 'pending',
    agv_id VARCHAR(50),
    source_location VARCHAR(100),
    destination_location VARCHAR(100),
    payload_info JSONB,
    estimated_duration INTEGER,
    actual_duration INTEGER,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    started_at TIMESTAMP,
    completed_at TIMESTAMP
);

-- 複雜查詢範例
WITH agv_stats AS (
    SELECT 
        agv_id,
        COUNT(*) as total_tasks,
        AVG(actual_duration) as avg_duration,
        SUM(CASE WHEN status = 'completed' THEN 1 ELSE 0 END) as completed_tasks
    FROM tasks 
    WHERE created_at >= CURRENT_DATE - INTERVAL '7 days'
    GROUP BY agv_id
),
battery_stats AS (
    SELECT 
        agv_id,
        AVG(battery_level) as avg_battery,
        MIN(battery_level) as min_battery
    FROM agv_status 
    WHERE last_updated >= CURRENT_DATE - INTERVAL '1 day'
    GROUP BY agv_id
)
SELECT 
    a.agv_id,
    a.total_tasks,
    a.avg_duration,
    a.completed_tasks,
    ROUND((a.completed_tasks::DECIMAL / a.total_tasks) * 100, 2) as success_rate,
    b.avg_battery,
    b.min_battery
FROM agv_stats a
LEFT JOIN battery_stats b ON a.agv_id = b.agv_id
ORDER BY success_rate DESC, total_tasks DESC;
```

## 🎯 支援的語言清單

Prism.js 自動載入器支援以下語言的語法高亮：

- **Python** (`python`, `py`)
- **JavaScript** (`javascript`, `js`)
- **TypeScript** (`typescript`, `ts`)
- **Bash** (`bash`, `sh`)
- **YAML** (`yaml`, `yml`)
- **JSON** (`json`)
- **SQL** (`sql`)
- **CSS** (`css`)
- **HTML** (`html`)
- **XML** (`xml`)
- **C++** (`cpp`, `c++`)
- **C** (`c`)
- **Java** (`java`)
- **Go** (`go`)
- **Rust** (`rust`)
- **Docker** (`dockerfile`)
- **Nginx** (`nginx`)

## 💡 使用技巧

### 指定語言
在 Markdown 中使用三個反引號加語言名稱：

```
```python
# 這裡寫 Python 代碼
```

### 無語言標識
如果不指定語言，將顯示為普通代碼塊：

```
這是沒有語法高亮的純文字代碼塊
適合顯示配置文件或純文字內容
```

### 內聯代碼
使用單個反引號的內聯代碼：`import rclpy` 會保持簡潔的樣式。

---

🎨 **語法高亮已完全整合**：現在所有的代碼塊都會根據指定的語言自動套用相應的語法高亮，讓技術文檔更加專業美觀！