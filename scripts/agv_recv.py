#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 17 11:54:44 2016

@author: takahisa
"""

import paho.mqtt.client as mqtt
from time import sleep, localtime, asctime, time
import rospy
from std_msgs.msg import String, Float32
from m34_agv_host.msg import agv_msgs
from threading import Thread, BoundedSemaphore

host = '192.168.250.10'
port = 1883
topic = '/agv'

mes = '0:0:0:0:0'
wdf = False
wdt = 5

semaphore = BoundedSemaphore(1)

def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))
    client.subscribe(topic)
    
def on_message(client, userdata, msg):
    global mes, wdf, wdt

    semaphore.acquire()
    mes = msg.payload
    wdt = 0
    wdf = True
    semaphore.release()

def im920callback(s):
    global mes, wdf

    semaphore.acquire()
    if wdf == False:
        mes = s.data
    semaphore.release()

def mqtt_loop():
        client.loop_forever()
    
if __name__ == '__main__':
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port=port, keepalive=60)

    rospy.init_node('agv_recv')
    pub = rospy.Publisher('/agv', agv_msgs, queue_size=1)
    pub_velAY = rospy.Publisher('/agv/velAY', Float32, queue_size=1)
    pub_velBX = rospy.Publisher('/agv/velBX', Float32, queue_size=1)
    pub_voltM = rospy.Publisher('/agv/voltM', Float32, queue_size=1)
    sub = rospy.Subscriber('im920', String, im920callback)

    rate = rospy.Rate(20)
    Thread(target=mqtt_loop).start()

    agv = agv_msgs()

    while not rospy.is_shutdown():
        semaphore.acquire()
        wdt += 1
        if wdt >= 20:
            wdt = 20
            wdf = False
        mel = mes.split(':')
        mef = wdf
        semaphore.release()

        print '[', wdf, ':', wdt,'] ', mes
        agv.velAY = float(mel[0])
        agv.velBX = float(mel[1])
        agv.voltM = float(mel[2])
        agv.adrs = int(mel[3])
        agv.pote = int(mel[4])
        agv.lan = mef
        pub.publish(agv)

        s = agv.velAY
        if abs(s) < 20:
            s = 0
        pub_velAY.publish(s)
        s = agv.velBX
        if abs(s) < 20:
            s = 0
        pub_velBX.publish(s)

        pub_voltM.publish(agv.voltM)

        rate.sleep()

