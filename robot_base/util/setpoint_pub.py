#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

from dynamic_reconfigure.server import Server
from robot_base.cfg import SetpointConfig


value = 0


def callback(config, level):
    global value
    value = config['setpoint']
    return config


def publish():
    pub = rospy.Publisher(rospy.get_param('~topic'), Float64, queue_size=10)
    rate = rospy.Rate(rospy.get_param('~rate'))
    while not rospy.is_shutdown():
        pub.publish(value)
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('setpoint_pub')
        srv = Server(SetpointConfig, callback)
        publish()
    except rospy.ROSInterruptException:
        pass

