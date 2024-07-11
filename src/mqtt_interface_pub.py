from paho.mqtt import client as mqtt


class MQTT_PUB:
    def __init__(self) -> None:
        self.broker = ""
        self.port = 1883
        self.topic = ""
        self.pubmsg = ""
        self.username = ""
        self.password = "password"
        self.client = None
        
    def __broker_setter(self,broker_ip:str):
        self.broker = broker_ip
    
    def __topic_setter(self,topic_name:str):
        self.topic = topic_name
    
    def pubmsg_setter(self,pubmsg):
        self.pubmsg = pubmsg
    
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
    
    
    def pub_con(self,broker_ip:str,topic_name:str,pubmsg:str)->mqtt:
        def __on_connect(client,userdata,flag,rc):
            if rc == 0:
                print("Connected to Mqtt Broker")
            else:
                print("Failed to connect mqtt broker")
            
        # 設定
        self.__broker_setter(broker_ip=broker_ip)
        self.__topic_setter(topic_name=topic_name)
        self.pubmsg_setter(pubmsg=pubmsg)

        self.client = mqtt.Client()
        self.client.username_pw_set(self.username,self.password)
        self.client.on_connect = __on_connect
        self.client.connect(self.broker,self.port)
        return self.client
    
    def pub_run(self):
        result = self.client.publish(self.topic,self.pubmsg)

        status = result[0]

        if status == 0:
            print(f"Send `{self.pubmsg}` to topic `{self.topic}`")
        else:
            print(f"Failed to send msg to topic {self.topic}")


if __name__ == "__main__":
    mqtt_PUBLISHER = MQTT_PUB()
    mqtt_PUBLISHER.pub_con(broker_ip="192.168.11.2",topic_name="test/mqtt",pubmsg="Hello mqtt World!")
    mqtt_PUBLISHER.pub_run()
    mqtt_PUBLISHER.pubmsg_setter("変えてみた")
    mqtt_PUBLISHER.pub_run()
    pass