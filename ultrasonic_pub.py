#!/home/pi/Desktop/Gaze/bin/python3

import rospy
import ultrasonic
from std_msgs.msg import Float64


def publish_message():

  pub = rospy.Publisher('ultrasonic_pub', Float64, queue_size=10)

  rospy.init_node('ultrasonic_py', anonymous=True)

  rate = rospy.Rate(10) #10 Hz
  
  ultra = ultrasonic.Ultrasonic(24,25)
  # While ROS is still running.
  while not rospy.is_shutdown():
      rospy.loginfo('Publishing Ultrasonic data')
      try:
      	distance=ultra.get_distance()
      except:
      	distance=1000
      pub.publish(distance)
      rate.sleep()


def main():
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()

