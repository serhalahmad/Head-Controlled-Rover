#!/usr/bin/env python3

import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
 

def callback(data):
 
  br = CvBridge()
  rospy.loginfo("receiving rover frame")
  current_frame = br.imgmsg_to_cv2(data)
  cv2.imshow("Rover", current_frame)
  cv2.waitKey(1)
      

def receive_message():
 
  rospy.init_node('rover_frames', anonymous=True)
  rospy.Subscriber('rover_py', Image, callback)
  rospy.spin()
  cv2.destroyAllWindows()
  


if __name__ == '__main__':
  receive_message()
