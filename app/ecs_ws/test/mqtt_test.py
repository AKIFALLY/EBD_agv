import paho.mqtt.client as mqtt

# MQTT 參數
broker_host = "192.168.11.206"
broker_port = 2883
username = "DsH8vSx2uhTao1hlc9vx"
password = None  # 如果沒有密碼可以留空或設為 None
sub_topic = "request/to/agvc/door"
pub_topic = "response/to/kukaecs/door"

# 當連線成功時呼叫
def on_connect(client, userdata, flags, rc):
    print("已連線，回傳碼 =", rc)
    if rc == 0:
        client.subscribe(sub_topic)
        print(f"已訂閱主題: {sub_topic}")
    else:
        print("連線失敗")

# 當收到訊息時呼叫
def on_message(client, userdata, msg):
    print(f"[收到訊息] 主題: {msg.topic}, 訊息: {msg.payload.decode()}")
    # 回應訊息
    response = "門控已收到指令"
    client.publish(pub_topic, response)
    print(f"[已回應] 主題: {pub_topic}, 訊息: {response}")

# 建立 MQTT 客戶端
client = mqtt.Client()
client.username_pw_set(username, password)

client.on_connect = on_connect
client.on_message = on_message

# 連線到 broker 並啟動 loop
client.connect(broker_host, broker_port, 60)
client.loop_forever()
