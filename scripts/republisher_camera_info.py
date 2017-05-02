#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
External computer

Editer : Tatsuya Aoki
"""
import rospy
from sensor_msgs.msg import CameraInfo

class Republisher(object):
    def __init__(self):
        # publisher & subscriber for RGB image 
        topic_name = "/hsrb/head_rgbd_sensor/rgb/camera_info"
        self.sub_rgb_ =  rospy.Subscriber(topic_name, CameraInfo, self.cb_republish_rgb_camera_info, queue_size=10)
        self.pub_rgb =  rospy.Publisher("/ext/camera/rgb/camera_info",CameraInfo, queue_size=10)

        # publisher & subscriber for Depth image
        #topic_name = "/hsrb/head_rgbd_sensor/depth_registered/camera_info"
        #self.sub_depth =  rospy.Subscriber(topic_name, CameraInfo, self.cb_republish_depth_camera_info, queue_size=10)
        #self.pub_depth =  rospy.Publisher("/ext/camera/depth_registered/camera_info", CameraInfo, queue_size=10)
        
 
    def cb_republish_rgb_camera_info(self, msg):
        """
        subscribe   "/hsrb/head_rgbd_sensor/rgb/camera_info" 
        publish     "/ext/camera/rgb/camera_info"
        
        Params
        -----
        msg : sensor_msgs.msg : CameraInfo
        """
        self.pub_rgb.publish(msg)


    def cb_republish_depth_camera_info(self, msg):
        """
        subscribe   "/hsrb/head_rgbd_sensor/camera/depth_registered/camera_info"
        publish     "/ext/camera/depth_registered/camera_info"
        
        Params
        -----
        msg : sensor_msgs.msg : CameraInfo
        """
        self.pub_depth.publish(msg)

if __name__ == "__main__":
    node_name = "republisher_camera_info"
    rospy.init_node(node_name)
    republisher = Republisher()
    rospy.loginfo("{0} Start.".format(node_name))
    while not rospy.is_shutdown():
    	rospy.spin()
    rospy.loginfo("{0} End.".format(node_name))
