import threading
import time
import rclpy.logging
from keyence_plc.keyence_plc_com import KeyencePlcCom

# 常數定義
MIN_POOL_SIZE = 1
MAX_POOL_SIZE = 5
RECONNECT_INTERVAL = 5  # 重試間隔 (秒)


class KeyencePlcPool:
    """基於 Semaphore 管理 PLC 連線池，支援錯誤連線的等待重連"""

    def __init__(self, ip, port, max_connections=MAX_POOL_SIZE):
        self.logger = rclpy.logging.get_logger('keyence_plc_pool')
        self.ip = ip
        self.port = port
        self.max_connections = max_connections
        self.connections = [KeyencePlcCom(ip, port) for _ in range(max_connections)]
        self.lost_connections = []  # 儲存需要重連的連線
        self.lock_pool = threading.Lock()  # 保護連線池操作
        self.lock_lost_pool = threading.Lock()  # 保護連線池操作
        self.semaphore = threading.Semaphore(max_connections)  # 限制最大連線數量
        
        self._running = True  # <- 新增旗標
        self.daemon_thread = threading.Thread(
            target=self._pool_daemon, daemon=True
        )
        self.daemon_thread.start()

    def _pool_daemon(self):
        """Pool 背景執行緒，持續嘗試重新連線"""
        while self._running:  # <- 用旗標控制迴圈
            time.sleep(RECONNECT_INTERVAL)
            while True:
                #self.logger.info(
                    #f"[plc pool conn:{len(self.connections)},lost:{len(self.lost_connections)}]"
                #)
                with self.lock_lost_pool:
                    if not self.lost_connections:
                        break  # 如果沒有要重連的 PLC，結束迴圈

                    plc = self.lost_connections.pop(0)  # 取出第一個 PLC 進行重連

                try:
                    if plc.connect() and plc.connect_test():  # 重試連線
                        self.logger.info(f"PLC {plc.ip}:{plc.port} 重新連線成功")
                        self._release_connection(plc)
                    else:
                        # 如果連線失敗，把它加回等待清單
                        with self.lock_lost_pool:
                            self.lost_connections.append(plc)
                except Exception as e:
                    self.logger.warn(f"try connect except: {e}")
                    # 如果連線中異常，把它加回等待清單
                    with self.lock_lost_pool:
                        plc = KeyencePlcCom(self.ip, self.port)
                        self.lost_connections.append(plc)

    def _get_connection(self):
        """取得一條可用的連線"""
        # 等待可用的連線，若達到最大連線數則阻塞
        self.semaphore.acquire()
        try:
            with self.lock_pool:
                if self.connections:
                    con = self.connections.pop()
                    if con.connect():
                        return con

            # 如果沒有可用連線，創建新連線
            con = KeyencePlcCom(self.ip, self.port)
            if con.connect():
                return con

            # 若無法建立連線，丟出例外
            raise ConnectionError(f"無法連接到 PLC ({self.ip}:{self.port})")
        except Exception as e:
            raise e  # 重新丟出例外

    def _release_connection(self, plc):
        """釋放一條連線。如果連線可用則放回池中，否則加入等待重連池"""
        if plc:
            #self.logger.debug(f"[plc _release_connection] back to pool: {len(self.connections)}")
            with self.lock_pool:
                if len(self.connections) < self.max_connections :
                    self.connections.append(plc)
                    self.semaphore.release()
                else:
                    self.logger.warn("Max connections")
        else:
            # plc is None 代表斷線,異常
            plc = KeyencePlcCom(self.ip, self.port)
            self.logger.warn(f"plc = None[plc pool _release_connection]PLC斷線 加入重連池")
            with self.lock_lost_pool:
                self.lost_connections.append(plc)
        #self.logger.debug(
        #    f"[plc pool _release_connection][conn:{len(self.connections)},reconn:{len(self.lost_connections)}]"
        #)

    def execute(self, command):
        """執行 PLC 命令並返回結果"""
        plc = None
        try:
            plc = self._get_connection()
            return plc.send_command(command)
        except Exception as e:  # 通常是OSError (網路線問題)
            if plc:
                plc.disconnect()
            plc = None
            self.logger.warn(f"[pool execute]Error: {e}")
            raise  # 重新丟出例外
        finally:
            self._release_connection(plc)

    def close_connection(self):
        """同步關閉所有 PLC 連線並釋放資源"""
        with self.lock_pool:
            for plc in self.connections:
                plc.disconnect()  # 假設 plc.disconnect() 是同步方法
                plc = None
            self.connections.clear()  # 清空 connections 列表

        # 在 lock_lost_pool 鎖範圍內，也可以考慮清除或重設重連池
        with self.lock_lost_pool:            
            for plc in self.connections:
                plc.disconnect()  # 假設 plc.disconnect() 是同步方法
                plc = None
            self.lost_connections.clear()  # 清空重連池，釋放失敗的連線

    def stop(self):
        self._running = False
        self.thread.join(timeout=2.0)  # 等待背景執行緒結束
