#!//home/pi/Desktop/Gaze/bin/python3

import rospy # Python library for ROS
from capstone.msg import steering # Image is the message type
from std_msgs.msg import String
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library



def callback(data):

  # Used to convert between ROS and OpenCV images
  br = CvBridge()

  # Output debugging information to the terminal
  rospy.loginfo("Receiving steering instructions")
  rospy.loginfo("\nAction:\t"+data.action+"\nDirection:\t"+data.direction)
  # Convert ROS Image message to OpenCV image
#  current_frame = br.imgmsg_to_cv2(data.frame)
  # Display image
#  cv2.imshow("camera", current_frame)
#  cv2.waitKey(1)

def receive_message():

  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('video_sub_py',anonymous=True)

  # Node is subscribing to the video_frames topic
  rospy.Subscriber('video_frames', steering, callback)

  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

  # Close down the video stream when done
  cv2.destroyAllWindows()

if __name__ == '__main__':
  receive_message()
