#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 17 11:54:44 2016

@author: takahisa
"""
import paho.mqtt.client as mqtt
from subprocess import call

host = '192.168.250.10'
port = 1883
topic = '/shutdown'

flag = False

def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))
    client.subscribe(topic)
    
def on_message(client, userdata, msg):
    global flag

    if flag:
        if msg.payload == '1':
            call("~/catkin_ws/src/m34_agv_host/scripts/shutdown_script.sh", shell=True)
    else:
        if msg.payload == '0':
            flag = True
    
if __name__ == '__main__':
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.on_connect = on_connect
    client.on_message = on_message
    
    client.connect(host, port=port, keepalive=60)
    
    client.loop_forever()
    
