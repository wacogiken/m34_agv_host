#!/usr/bin/env python

import roslib
roslib.load_manifest("view_controller_msgs")

import rospy
from math import *
from view_controller_msgs.msg import CameraPlacement
from geometry_msgs.msg import Point, Vector3, Quaternion, Pose
from std_msgs.msg import Int16MultiArray
from jsk_rviz_plugins.msg import Pictogram
from sensor_msgs.msg import Joy
from m34_agv_host.msg import agv_msgs
from time import sleep
from subprocess import call

view0 = [ 0.0, -3.0,  3.0]
view1 = [-1.6, -1.6, -2.0]
view2 = [ 2.0,  2.0,  0.5]
view3 = [ 1.0,  0.0, -2.0]
view  = [view0, view1, view2, view3]

pi=3.1415926

posd = [[     0,      0,       0, ' ', False],  #  0
        [ 0.975,  0.867,    pi/2, 'A', False],  #  1
        [     0,      0,       0, ' ', False],  #  2
        [ 1.130,  1.000,    pi/2, 'B',  True],  #  3
        [     0,      0,       0, ' ', False],  #  4
        [ 1.130,  1.000,      pi, 'C',  True],  #  5
        [     0,      0,       0, ' ', False],  #  6
        [-0.699,  1.067,      pi, 'D', False],  #  7
        [     0,      0,       0, ' ', False],  #  8
        [-0.780,  1.000,      pi, 'E',  True],  #  9
        [     0,      0,       0, ' ', False],  # 10
        [-0.780,  1.000,    pi/2, 'F',  True],  # 11
        [     0,      0,       0, ' ', False],  # 12
        [-0.436, -0.153,  3*pi/4, 'G', False],  # 13
        [     0,      0,       0, ' ', False],  # 14
        [-0.190, -0.340,  3*pi/4, 'H',  True],  # 15
        [     0,      0,       0, ' ', False],  # 16
        [-0.190, -0.340,  5*pi/4, 'I',  True],  # 17
        [     0,      0,       0, ' ', False],  # 18
        [ 0.217,  0.217,  5*pi/4, 'J', False],  # 19
        [     0,      0,       0, ' ', False],  # 20
        [ 0.560,  0.410,  5*pi/4, 'K',  True],  # 21
        [     0,      0,       0, ' ', False],  # 22
        [ 0.217,  0.217,  5*pi/4, 'L', False],  # 23
        [     0,      0,       0, ' ', False],  # 24
        [-0.273, -0.273,  5*pi/4, 'M', False],  # 25
        [     0,      0,       0, ' ', False],  # 26
        [-0.900, -0.900,  5*pi/4, 'N', False],  # 27
        [     0,      0,       0, ' ', False],  # 28
        [-0.850, -1.000,  5*pi/4, 'O',  True],  # 29
        [     0,      0,       0, ' ', False],  # 30
        [-0.850, -1.000,       0, 'P',  True],  # 31
        [     0,      0,       0, ' ', False],  # 32
        [ 1.130, -0.200,    pi/2, 'Q',  True]   # 33
       ]

p = 0
p_old = 1
posn = 0

move0 = {'theta':0.0, 'phi':0.0, 'r':1.0}
move1 = {'x':0.0, 'y':0.0, 'z':0.0}

old_adrs = 99
pp = Pose()
pict = Pictogram()

auto_cnt = 0
auto_p = 0
autof = resetf = False

auto_focus_point = Point(0, 0, 0)
adrsf = False

auto = True
adrs = True
rate_float = 2

def callback(message):
    global move0, move1
    global auto, autof, rate_float, resetf
    global adrs, adrsf

    inp = message.data[0]

    if inp & 1:
        call("~/catkin_ws/src/m34_agv_host/scripts/shutdown_script.sh", shell=True)

    if inp & 4096:
        if autof == False:
            if auto:
                auto = False
                resetf = True
                rate_float = 9
            else:
                auto = True
                rate_float = 2
            autof = True
    else:
        autof = False

    if inp & 256:
        if adrsf == False:
            adrs ^= True
            adrsf = True
    else:
        adrsf = False

    t = move0['theta']
    p = move0['phi']
    r = move0['r']

    x = move1['x']
    y = move1['y']

    if inp & 8:
        t += 5.0*pi/360.0
    if inp & 2:
        t -= 5.0*pi/360.0
    if inp & 4:
        p += 5.0*pi/360.0
    if inp & 16:
        p -= 5.0*pi/360.0
    if inp & 128:
        r += 5.0*0.002
    if inp & 2048:
        r -= 5.0*0.002

    if inp & 512:
        x += 5.0*0.01
    if inp & 32:
        x -= 5.0*0.01
    if inp & 1024:
        y += 5.0*0.01
    if inp & 64:
        y -= 5.0*0.01

    if resetf:
        t = p = 0.0
        r = 1.0
        x = y = 0.0
        resetf = False

    move0['theta'] = t
    move0['phi']   = p
    move0['r']     = r

    move1['x'] = x
    move1['y'] = y

def agvCallback(agv):
    global old_adrs, posd, pp, pict
    global auto_focus_point

    if agv.adrs >= 0 and agv.adrs < len(posd):
        if agv.adrs != old_adrs:
            posn = agv.adrs
            if posn >=len(posd):
                posn = 0
            px, py, pz, ps, pf = posd[posn]
            if pf:
                pp.position = auto_focus_point = Point(px, py, 0)
                pp.orientation = Quaternion(0, 0, pz, 0)
                pos.publish(pp)

            pict.pose.position = Point(px, py, 0.8)
            pict.character = ps
            pic.publish(pict)

            old_adrs = posn

def moveto0(point, m):
    x, y, z = point.x, point.y, point.z
    theta = atan2(sqrt(x**2+y**2), z) + m['theta']
    phi = atan2(y, x) + m['phi']
    r = sqrt(x**2+y**2+z**2) * m['r']

    if theta >= pi:
        theta -= 2*pi
    if theta <= -pi:
        theta += 2*pi
    if phi >= pi:
        phi -= 2*pi
    if phi <= -pi:
        phi += 2*pi

    xx = r * sin(theta) * cos(phi)
    yy = r * sin(theta) * sin(phi)
    zz = r * cos(theta)
    p = Point(xx, yy, zz)
    
    return p

def moveto1(point, m):
    x, y, z = point.x, point.y, point.z
    x += m['x']
    y += m['y']
    z += m['z']
    p = Point(x, y, z)
    
    return p

if __name__ == '__main__':
    rospy.init_node("camera_test", anonymous = True)

    pub = rospy.Publisher("/rviz/camera_placement", CameraPlacement, queue_size = 1)
    pos = rospy.Publisher("/agv/cur_odom", Pose, queue_size = 1)
    pic = rospy.Publisher("/pict", Pictogram, queue_size = 1)
    sub = rospy.Subscriber('/arduino', Int16MultiArray , callback)
    agv = rospy.Subscriber('/agv', agv_msgs , agvCallback)

    rate = rospy.Rate(10)

    px, py, pz, ps, pf = posd[0]

    sleep(5)
    pp.position = Point(px, py, 0)
    pp.orientation = Quaternion(0, 0, pz, 0)
    pos.publish(pp)

    pict.header.frame_id = 'odom'
    pict.pose.position = Point(px, py, 0.8)
    pict.pose.orientation = Quaternion(0.5, 0.5, 0.5, -0.5)
    pict.action = 4
    pict.mode = 1
    pict.character = ps
    pict.size = 1
    pict.speed = 0.25
    pict.color.r = 1.0
    pict.color.g = 0.0
    pict.color.b = 0.0
    pict.color.a = 1.0
    pic.publish(pict)

    cp = CameraPlacement()
    cp.target_frame = 'odom'
    cp.time_from_start = rospy.Duration(1.0/rate_float)
    cp.eye.header.frame_id = 'odom'
    cp.eye.point = eye_point = auto_eye_point = Point(view[0][0], view[0][1], view[0][2])
    cp.focus.header.frame_id = 'odom'
    cp.focus.point = focus_point = auto_focus_point = Point(0, 0, 0)
    cp.up.header.frame_id = 'odom'
    cp.up.vector = Vector3(0, 0, 0)

    while not rospy.is_shutdown():

        t = rospy.get_time()
        r = 10

        if auto:
            cp.eye.point = auto_eye_point
        else:
            cp.eye.point = moveto1(moveto0(eye_point, move0), move1)

        if adrs:
            cp.focus.point = auto_focus_point
        else:
            cp.focus.point = moveto1(focus_point, move1)

        cp.time_from_start = rospy.Duration(1.0/rate_float)
        pub.publish(cp)

        if auto:
            auto_cnt = auto_cnt + 1
            if auto_cnt >= 100:
                auto_cnt = 0
                auto_p = auto_p + 1
                if auto_p >= len(view):
                    auto_p = 0
                auto_eye_point = Point(view[auto_p][0], view[auto_p][1], view[auto_p][2])

        rate.sleep()

