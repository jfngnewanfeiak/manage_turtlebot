from paho.mqtt import client as mqtt
from socket import gethostname,gethostbyname

# ip取得
your_ip = gethostbyname(gethostname())

class MQTT_SUB:
    def __init__(self) -> None:
        self.__broker = ""
        self.__port = 1883 #default settings
        self.__topic = ""
        self.__username = "user"
        self.__password = "password"
        self.__callback = None

    # セッター
    def borker_setter(self,broker_ip:str):
        self.__broker = broker_ip
    
    def topic_setter(self,topic_name:str):
        self.__topic = topic_name
    
    # ブローカに接続
    def __connect_mqtt(self)->mqtt:
        def __on_connect(client,ud,flag,rc):
            if rc == 0:
                print("Connected to Mqtt Broker")
            else:
                print("Failed to connect mqtt broker")
        
        client = mqtt.Client()
        client.username_pw_set(self.__username,self.__password)
        client.on_connect = __on_connect
        client.connect(self.__broker,self.__port)
        
        return client


    def __subscribe(self,client:mqtt):
        def __on_message(client,ud,msg):
            print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
            # self.__callback(msg.payload.decode())
            self.__callback()
        
        client.subscribe(self.__topic)
        client.on_message = __on_message
    
        
    def sub_run(self,broker_ip:str,topic_name:str,cb):
        # 設定
        self.borker_setter(broker_ip=broker_ip)
        self.topic_setter(topic_name=topic_name)

        client = self.__connect_mqtt()
        self.__subscribe(client=client)
        self.__callback = cb
        # client.loop_start()
        client.loop_forever()
    def callback(self):
        print("callback")


if __name__ == "__main__":
    mqtt_SUBSCRIBER = MQTT_SUB()
    # ここは引数に合わせて設定
    mqtt_SUBSCRIBER.sub_run(broker_ip=your_ip,topic_name="req/manage",cb=mqtt_SUBSCRIBER.callback)

