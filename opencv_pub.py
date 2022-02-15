#!/usr/bin/env python3


import rospy # Python library for ROS
from capstone.msg import steering # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import mediapipe
import cv2 # OpenCV library
import find_face_mesh


def publish_message():

  pub = rospy.Publisher('video_frames', steering, queue_size=10)

  rospy.init_node('video_pub_py', anonymous=True)

  rate = rospy.Rate(30) # 30hz

  detector = find_face_mesh.FaceMeshDetector()
  #cap = cv2.VideoCapture("https://192.168.137.130:8080/video")
  cap=cv2.VideoCapture()
  cap.open("http://192.168.43.143:8000/")
  cap.set(3, 320)
  cap.set(4, 240)
  br = CvBridge()


  while not rospy.is_shutdown():

      ret, frame = cap.read()

      if ret == True:
        detector.find_face_mesh(frame, draw=False)
        action, direction = detector.get_head_position(5)
        rospy.loginfo('Publishing video frame and steering instructions')
        msg=steering()
        # msg.frame=br.cv2_to_imgmsg(frame)
        msg.action=action
        msg.direction=direction
        #cv2.imshow('Frame', frame)
        #cv2.waitKey(1)
        pub.publish(msg)

      rate.sleep()


def main():
  try:
      publish_message()
  except rospy.ROSInterruptException:
      pass


if __name__ == '__main__':
	main() 
