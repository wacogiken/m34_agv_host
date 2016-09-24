#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import rospy
import ast
from std_msgs.msg import String
from threading import Thread, BoundedSemaphore, Event
from time import sleep, time

class im920Manager:
    def __init__(self):
        rospy.init_node('im920_node', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        driver_item = rospy.get_param('~driver_item', 'im920')
        driver_name = rospy.get_param('~driver_name', 'no1')

        self.pub = rospy.Publisher('/im920', String, queue_size=1)

        port_name = rospy.get_param('~serial_ports/port_name', '/dev/im920')
        port_baud = rospy.get_param('~serial_ports/baud_rate', 38400)
        im920_id = rospy.get_param('~im920/id', '0E65')
        self.ser = serial.Serial(port_name, port_baud, timeout=1)
        self.semaphore = semaphore
        
        self.ret_val=''
        self.ret_str=''
        self.recv_fifo=''
        self.packet_fifo=''

        self.ser.flush()

        self.stop_event = Event()
        self.thread = Thread(target = self.recv)
        self.thread.start()

        num=1

        while True:
           s = self.send('rdid')
           if s == im920_id:
               break;
        self.send('rdnn')
        self.send('rrid')
        self.send('rdch')
        self.send('rdpo')
        self.send('rdrt')
        self.send('rdvr')

        rate = rospy.Rate(1)
        s = ''
        old_time = time()
        while not rospy.is_shutdown():
            while True:
                c = self.read()
                if c != '':
                    if c == '\n':
                        new_time = time()
                        self.pub.publish(s)
                        old_time = new_time
                        s = ''
                    else:
                        s += c
                else:
                    break;

    def shutdown(self):
        self.ser.close()
        self.stop_event.set()

    def read(self):
        if len(self.recv_fifo):
            self.semaphore.acquire()
            c = self.recv_fifo[0]
            self.recv_fifo = self.recv_fifo[1:]
            self.semaphore.release()
        else:
            c=''

        return c

    def recv(self):
        while not self.stop_event.is_set():
            s = (self.ser.readline()).strip('\n\r')
            if s=='':
                sleep(0.01)
            else:
                t = s.split(':')
                if len(t)==1:
                    self.ret_val = s
                else:
                    print '{'+t[0]+'} ',
                    s = t[1]
                    for i in s.split(','):
                        c = chr(ast.literal_eval('0x'+i))
                        if c == '\x02':
                            self.packet_fifo = ''
                        elif c == '\x03':
                            sum=0
                            for i in self.packet_fifo[:-2]:
                                sum += ord(i)
                            chk = ast.literal_eval('0x'+self.packet_fifo[-2:])
                            if (sum + chk) & 0xff == 0:
                                self.semaphore.acquire()
                                self.recv_fifo += self.packet_fifo[:-2]
                                self.semaphore.release()
                        else:
                            self.packet_fifo += c

    def send(self, command):
        print '{send:'+command+'}'
        self.ret_val = ''
        self.ser.write(command+'\n\r')
        i=0
        while self.ret_val=='':
            i += 1
            if i > 10:
                self.ret_val = 'TO'
                break
            sleep(0.1)
        return self.ret_val

    def write(self, command):
        i=0
        for t in command:
            i = i+ord(t)
        t=hex(65536-i)
        s = chr(0x02)+command+t[len(t)-2:].upper()+chr(0x03)
        t = 'txda '
        for i in s:
            t += ('0'+hex(ord(i))[2:])[-2:]
        s = self.send(t)
        return s

    def im920_handler(self, msg):
        self.semaphore.acquire()
        d = dict()
        d['spdA']=msg.spdA
        d['spdB']=msg.spdB
        d['pote']=msg.pote
        d['volt']=msg.volt
        d['adrs']=msg.adrs
        s=str(d)

        self.semaphore.release()

        abh3 = abh3_msgs()
        for i in range(drv_num):
            self.semaphore.acquire()
            cmd = abh3_header + drv_cmd[i]
            self.semaphore.release()
            if flag:
                drv_value[i] = int(ret, 16)

        abh3.vel, abh3.trq = drv_value
        self.pub.publish(abh3)
    
if __name__ == '__main__':
    try:
        semaphore = BoundedSemaphore(1)
        manager = im920Manager()
    except rospy.ROSInterruptException:
        manager.shutdown()

