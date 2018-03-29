#!/usr/bin/env python
import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDrive

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    # UTV
    # speed = data.axes[3] * 18
    # steer = data.axes[2]

    # RC Car
    speed = data.axes[1] * 26.8
    steer = data.axes[2]
    accel = 0
    # print "%.4f  %.4f" % (speed, steer)

    # convert to ackermann msg

    # ack_cmd = AckermannDrive()
    ack_cmd.speed = speed
    ack_cmd.steering_angle = steer
    ack_cmd.acceleration = accel
    # pub.publish(ack_cmd)
    pub.publish(ack_cmd)





if __name__ == '__main__':

    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=10)
    rospy.Subscriber('joy', Joy, callback)

    speed = 0
    steer = 0
    accel = 0
    ack_cmd = AckermannDrive()

    while not rospy.is_shutdown():
    # hello_str = "hello world %s" % rospy.get_time()
    # rospy.loginfo(hello_str)
        # rate.sleep()
        rospy.spin()
