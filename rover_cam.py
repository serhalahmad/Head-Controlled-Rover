#!//home/pi/Desktop/Gaze/bin/python3

import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

def publish_message():

	pub = rospy.Publisher('rover_frames', Image, queue_size=10)
	rospy.init_node('rover_pub_py', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	cap = cv2.VideoCapture(0)
	br = CvBridge()

	while not rospy.is_shutdown():
		ret, frame = cap.read()
		if ret == True:
			rospy.loginfo('publishing rover frame')
			pub.publish(br.cv2_to_imgmsg(frame))
		rate.sleep()


def main():
	try:
		publish_message()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
 main()
