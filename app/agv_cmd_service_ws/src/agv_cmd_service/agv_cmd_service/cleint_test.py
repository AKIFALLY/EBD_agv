import rclpy
from agv_cmd_service.agv_cmd_client_node import AgvCommandClient
import time
from rclpy.node import Node

# 初始化 ROS 2
rclpy.init()

# 創建節點物件
client = AgvCommandClient(Node())

# 計算指令發送時間
print("發送100次開始")
start_time = time.time()  # 記錄開始時間

for i in range(100):
    # 發送手動前進指令
    success = client.send_manual_command("forward", True)
    # print(f"手動前進 ON 指令結果: {success}")

# 計算耗時
end_time = time.time()  # 記錄結束時間
elapsed_time = end_time - start_time  # 計算耗時
average_time = elapsed_time / 100  # 計算平均每次耗時

print(f"發送 100 次指令耗時: {elapsed_time:.2f} 秒")
print(f"平均每次指令耗時: {average_time:.4f} 秒")

# 發送自動模式指令
success = client.send_general_command("auto", "on,100,200,300")
print(f"自動模式 ON 指令結果: {success}")

# 清理節點
client.destroy_node()

# 關閉 ROS 2
rclpy.shutdown()

print("程式執行完成")
