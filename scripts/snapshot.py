#!/usr/bin/env python
import rospy
import std_srvs.srv
import std_msgs.msg
import sensor_msgs.msg
import time

class Snapshot:
    def __init__(self):
        rospy.init_node('snapshot')
        rospy.Subscriber("/usb_cam/image_raw", sensor_msgs.msg.Image, self.image_callback)
        rospy.wait_for_service('image_saver/save')
        self.saver = rospy.ServiceProxy('image_saver/save', std_srvs.srv.Empty)
        self.warn_pub = rospy.Publisher('operation_str', std_msgs.msg.String, queue_size=5)

    def image_callback(self, data):
        self.saver()

        #self.warn_pub.publish(std_msgs.msg.String(""))

        time.sleep(2)

if __name__ == "__main__":
    snapshot = Snapshot()

    rospy.spin()