#!/usr/bin/env python
#coding: utf-8
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode

class QRCodeDecoder():
    def __init__(self):
        self.sub_img_name = rospy.get_param("sub_img_name", "/usb_cam/image_raw")
        self.decode_ctrl = rospy.get_param("decode_ctrl", True)

        self.sub_img = rospy.Subscriber(self.sub_img_name, Image, self.sub_img_CB)
        self.sub_decode_ctrl = rospy.Subscriber("decode_ctrl", Bool, self.sub_decode_ctrl_CB)

        self.pub_decode_data = rospy.Publisher("decode_data", String, queue_size=10)

    def sub_decode_ctrl_CB(self, msg):
        self.decode_ctrl = msg.data


    def sub_img_CB(self, msg):
        if( self.decode_ctrl == True ):
            try:
                cv_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
                qr_data = decode(cv_img)

                if( len(qr_data) != 0 ):
                    pub_msg = String()
                    pub_msg.data = str(qr_data[0][0])
                    self.pub_decode_data.publish(pub_msg)

            except CvBridgeError, e:
                rospy.logerror("Failed to Subscribe Image Topic")


if __name__ == "__main__":
    rospy.init_node("qrcode_decoder")

    QCD = QRCodeDecoder()

    rospy.spin()
