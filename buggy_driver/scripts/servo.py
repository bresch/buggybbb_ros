#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
import Adafruit_BBIO.PWM as PWM

servo_pin = "P8_13"
duty_min = 6.6
duty_max = 12
duty_span = duty_max - duty_min

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %d", data.data)
    angle = data.data
    if angle == 0:
        PWM.stop(servo_pin)
        PWM.cleanup()
        return
    angle_f = float(angle)
    duty = ((angle_f / 180) * duty_span + duty_min)
    PWM.set_duty_cycle(servo_pin, duty)

def servo():

    rospy.init_node('servo', anonymous=True)
    rospy.Subscriber("servo_angle", Int16, callback)
    
    PWM.start(servo_pin, duty_span * 0.5 + duty_min, 60.0)

    rospy.spin()


if __name__ == '__main__':
    try:
        servo()
    except rospy.ROSInterruptException:
        pass
