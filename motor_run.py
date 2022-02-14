#!/home/pi/Desktop/Gaze/bin/python3

# Import the necessary libraries
import rospy # Python library for ROS
import motor
from capstone.msg import steering # Image is the message type
from std_msgs.msg import Float64 
#from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
#import cv2 # OpenCV library


class Motor_control:
	
	def __init__(self):
		self.face=None
		self.ultrasonic=None

	def head(self, data):
		self.face = data
		self.control_motor()

	def ultra(self, data):
		self.ultrasonic=data.data
		self.control_motor()
	 
	def control_motor(self):
		action = self.face.action
		direction = self.face.direction
		sensor = self.ultrasonic
		rospy.loginfo("Receiving steering instructions and running motor")
		rospy.loginfo("\nAction:\t"+action+"\nDirection:\t"+direction)
	  	
		if action == 'forward':
			if direction == 'left':
				m.turn_left()
		  		#                 turn on left motor
			elif direction == 'right':
				m.turn_right()
		 		#                 turn on right motor
			elif sensor > 15:
				m.move_forward()
				#                 turn on both motors
			else:
				m.stop()
		else:
			m.stop()
	  	# Convert ROS Image message to OpenCV image
		#  current_frame = br.imgmsg_to_cv2(data.frame)
	  	# Display image
		#  cv2.imshow("camera", current_frame)
		#  cv2.waitKey(1)

def receive_message():

  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('motor_run_py', anonymous=True)
  motor_control=Motor_control()
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('video_frames', steering, motor_control.head)
  rospy.Subscriber('ultrasonic_pub', Float64, motor_control.ultra)

  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

  # Close down the video stream when done
#  cv2.destroyAllWindows()


if __name__ == '__main__':
    m=motor.Motors(13,12,23,22,17,27,50)
    receive_message()
