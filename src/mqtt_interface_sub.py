from paho.mqtt import client as mqtt

class MQTT_SUB:
    def __init__(self) -> None:
        self.broker = ""
        self.port = 1883 #default settings
        self.topic = ""
        self.username = "user"
        self.password = "password"

    # セッター
    def borker_setter(self,broker_ip:str):
        self.broker = broker_ip
    
    def topic_setter(self,topic_name:str):
        self.topic = topic_name
    
    # ブローカに接続
    def __connect_mqtt(self)->mqtt:
        def __on_connect(client,ud,flag,rc):
            if rc == 0:
                print("Connected to Mqtt Broker")
            else:
                print("Failed to connect mqtt broker")
        
        client = mqtt.Client()
        client.username_pw_set(self.username,self.password)
        client.on_connect = __on_connect
        client.connect(self.broker,self.port)
        return client


    def __subscribe(self,client:mqtt):
        def __on_message(client,ud,msg):
            print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
        
        client.subscribe(self.topic)
        client.on_message = __on_message
    
        
    def sub_run(self,broker_ip:str,topic_name:str):
        # 設定
        self.borker_setter(broker_ip=broker_ip)
        self.topic_setter(topic_name=topic_name)

        client = self.__connect_mqtt()
        self.__subscribe(client=client)
        client.loop_forever()


if __name__ == "__main__":
    mqtt_SUBSCRIBER = MQTT_SUB()
    # ここは引数に合わせて設定
    mqtt_SUBSCRIBER.sub_run(broker_ip="192.168.11.2",topic_name="test/mqtt")

