import numpy as np
import rospy
from std_msgs.msg import Float64

def callback(data):
    print(data)

rospy.init_node('listener', anonymous=True)
print('Subscribing to "motor" topic')
# robot.step(timeStep)
rospy.Subscriber("/dingo_controller/FR_theta1/command", Float64, callback)
rospy.spin()