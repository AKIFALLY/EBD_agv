import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
from plc_proxy.plc_client import PlcClient
from ecs.door_controller_config import DoorControllerConfig
from ecs.door_logic import DoorLogic
import time

# 必需使用非同步的方式來處理 PLC 的狀態查詢，因為這樣可以避免阻塞主執行緒(MQTTLoop中觸發的callback一樣需要非同步)


class DoorControllerNodeMQTT(Node):
    def __init__(self):
        super().__init__('door_controller_node_mqtt')
        namespace = self.get_namespace()
        self.get_logger().info(f"🔧 服務命名空間: {namespace}")

        self.declare_parameter('broker_host', '192.168.10.3')
        self.declare_parameter('broker_port', 2883)
        self.declare_parameter('username', 'DsH8vSx2uhTao1hlc9vx')
        self.declare_parameter('sub_topic', 'request/to/agvc/door')
        self.declare_parameter('pub_topic', 'response/to/kukaecs/door')
        self.declare_parameter('doors', ['1,MR,100,DM,5000'])

        self.broker_host = self.get_parameter(
            'broker_host').get_parameter_value().string_value
        self.broker_port = self.get_parameter(
            'broker_port').get_parameter_value().integer_value
        username = self.get_parameter(
            'username').get_parameter_value().string_value
        sub_topic = self.get_parameter(
            'sub_topic').get_parameter_value().string_value
        pub_topic = self.get_parameter(
            'pub_topic').get_parameter_value().string_value

        self.get_logger().info(f"🔧 self.broker_host...{self.broker_host}")
        self.get_logger().info(f"🔧 self.broker_port...{self.broker_port}")
        doors = self.get_parameter(
            'doors').get_parameter_value().string_array_value

        plc_client = PlcClient(self)
        self.config = DoorControllerConfig.from_list(doors)
        self.door_logic = DoorLogic(plc_client, self.config)

        self.mqtt_client = mqtt.Client()
        if username:
            self.mqtt_client.username_pw_set(username)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_client.on_message = self.on_mqtt_message

        self.pub_topic = pub_topic
        self.sub_topic = sub_topic
        self.mqtt_connecting = False
        self.subscribed = False

        # 初始化門狀態和追蹤字典
        self.door_status = {}
        self.mqtt_controlled_doors = {}  # 用來追蹤哪些門曾被 MQTT 控制過
        for door_id in self.config.doors.keys():
            try:
                state_info = self.door_logic.state_door(door_id)
                self.door_status[door_id] = state_info["state"]
                self.mqtt_controlled_doors[door_id] = False  # 初始為未被控制
                self.get_logger().info(
                    f"初始化門 {door_id} 狀態為 {state_info['state']}")
            except Exception as e:
                self.get_logger().error(f"初始化門 {door_id} 狀態失敗: {e}")
                self.door_status[door_id] = None

        try:
            self.mqtt_client.connect(
                self.broker_host, self.broker_port, keepalive=10)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"❌ MQTT 初始連線失敗: {e}")

        self.get_logger().info("🧪 建立計時器...")
        self.timer = self.create_timer(1.0, self.check_door_status)

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("✅ MQTT 連線成功")
            if not self.subscribed:
                self.mqtt_client.subscribe(self.sub_topic)
                self.subscribed = True
                self.get_logger().info(f"📡 訂閱主題: {self.sub_topic}")
        else:
            self.get_logger().error(f"❌ MQTT 連線失敗，返回碼: {rc}")

    def on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warning(f"⚠️ MQTT 斷線，返回碼: {rc}")
        self.subscribed = False

    def reconnect_mqtt(self):
        while rclpy.ok():
            time.sleep(5)
            if not self.mqtt_client.is_connected() and not self.mqtt_connecting:
                self.mqtt_connecting = True
                try:
                    self.get_logger().info("🔁 嘗試重連 MQTT...")
                    self.mqtt_client.connect(
                        self.broker_host, self.broker_port, keepalive=10)
                except Exception as e:
                    self.get_logger().error(f"MQTT 重連失敗: {e}")
                finally:
                    self.mqtt_connecting = False

    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            if 'doorId' not in payload or 'isOpen' not in payload:
                self.get_logger().warn(f"❗️收到不完整訊息: {payload}")
                return

            door_id = int(payload['doorId'])
            is_open = payload['isOpen'] == "true"
            self.get_logger().info(
                f"📥 收到控制訊息 doorId={door_id}, isOpen={is_open}")

            if door_id not in self.door_status:
                self.get_logger().info(f"self.door_status[door_id] = None")
                self.door_status[door_id] = None

            # 設定該門為已被 MQTT 控制
            self.mqtt_controlled_doors[door_id] = True
            # 控制門的開關狀態
            self.door_logic.async_control_door(door_id, is_open)
            # 先回一次現在狀態
            self.publish_door_state(door_id, self.door_status[door_id])
            # 使用 async_state_door 非同步查詢門狀態
            self.door_logic.async_state_door(
                door_id,
                lambda state_info, door_id=door_id: self._handle_door_status_update(
                    door_id, state_info)
            )

            self.get_logger().info(
                f"狀態: doorId={door_id}, isOpen={is_open}, currentState={self.door_status[door_id]}")

        except Exception as e:
            self.get_logger().error(f"❌ 處理訊息錯誤: {e}")

    def check_door_status(self):
        """
        定期檢查門的狀態，並更新狀態變更。
        """

        for door_id in self.door_status.keys():
            try:
                current_state = self.door_status[door_id]
                if current_state is None:
                    continue

                # 使用 async_state_door 非同步查詢門狀態
                self.door_logic.async_state_door(
                    door_id,
                    lambda state_info, door_id=door_id: self._handle_door_status_update(
                        door_id, state_info)
                )
            except Exception as e:
                self.get_logger().error(f"檢查門 {door_id} 狀態錯誤: {e}")

    def _handle_door_status_update(self, door_id, state_info):
        """
        處理門狀態更新的回呼函數。
        :param door_id: 門的識別 ID
        :param state_info: 門的狀態資訊字典
        """
        try:
            if not state_info["success"]:
                self.get_logger().error(f"門 {door_id} 狀態查詢失敗")
                return

            actual_status = state_info["state"]
            current_state = self.door_status[door_id]

            # 僅當門狀態變化且該門曾被 MQTT 控制過時，才發佈狀態
            if actual_status != current_state and self.mqtt_controlled_doors.get(door_id, False):
                self.get_logger().info(f"門 {door_id} 狀態變更為 {actual_status}")
                self.publish_door_state(door_id, actual_status)
                self.door_status[door_id] = actual_status
            elif actual_status == current_state and self.mqtt_controlled_doors.get(door_id, False):
                # 如果狀態未變化且該門曾被 MQTT 控制過，重置為未被控制狀態
                self.mqtt_controlled_doors[door_id] = False
        except Exception as e:
            self.get_logger().error(f"處理門 {door_id} 狀態更新錯誤: {e}")

    def publish_door_state(self, door_id, state):
        self.get_logger().info("📤 發佈前")
        payload = json.dumps({
            "doorId": str(door_id),
            "state": state
        })
        self.mqtt_client.publish(self.pub_topic, payload)
        self.get_logger().info(f"📤 發佈狀態: {payload}")

    def destroy_node(self):
        self.get_logger().info("🔚 關閉節點與 MQTT...")
        try:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        except Exception as e:
            self.get_logger().error(f"關閉 MQTT 時出錯: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DoorControllerNodeMQTT()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Received Keyboard Interrupt (Ctrl+C), shutting down...")
    finally:
        node.destroy_node()  # Ensure we only call this once
        rclpy.shutdown()  # Ensure we only call shutdown once


if __name__ == '__main__':
    main()
