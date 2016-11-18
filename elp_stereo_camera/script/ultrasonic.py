#! /usr/bin/env python

import rospy
import serial
import time

from std_msgs.msg import Int8

def talker():
    pub = rospy.Publisher('ultrasonic_range', Int8, queue_size=10)
    rospy.init_node('ultrasonic_talker', anonymous=True)
    #rate = rospy.Rate(100) # 10hz
    ser = serial.Serial('/dev/ttyUSB0', baudrate=57600, timeout=0.01)  # open serial port
    start_time = time.time()
    while not rospy.is_shutdown():
        #rospy.loginfo(hello_str)
        
        
        start_time = time.time()
        txt = ser.read(10)
        elapsed_time = time.time() - start_time
        print "read time:", elapsed_time, txt
        start_time = time.time()
        try:
            if txt[0] == 'R':
                data = int(txt[1:4])
                #print data
                start_time = time.time()
                pub.publish(data)
                elapsed_time = time.time() - start_time
                print "publish time:", elapsed_time
                #rate.sleep()
        except: 
            continue


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
