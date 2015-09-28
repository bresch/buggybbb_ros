#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
import Adafruit_BBIO.PWM as PWM

# Servo PPM configuration
servo_pin = "P8_13"
duty_servo_min = 6.6
duty_servo_max = 12
duty_servo_span = duty_servo_max - duty_servo_min

# Motor PPM configuration
motor_pin = "P8_14"
duty_motor_min = 6
duty_motor_max = 12
duty_motor_span = duty_motor_max - duty_motor_min

def setServoAngle(angle_f):
    duty = ((-angle_f/2 + 0.5) * duty_servo_span + duty_servo_min)
    PWM.set_duty_cycle(servo_pin, duty)

def setMotorSpeed(speed_f):
    duty = ((-speed_f/2 + 0.5) * duty_motor_span + duty_motor_min)
    PWM.set_duty_cycle(motor_pin, duty)
    
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    angle_f = float(data.angular.z)
    speed_f = float(data.linear.x)
    setServoAngle(angle_f)
    setMotorSpeed(speed_f)

def driver():
    rospy.init_node('driver', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    
    PWM.start(servo_pin, duty_servo_span * 0.5 + duty_servo_min, 60.0)
    PWM.start(motor_pin, duty_motor_span * 0.5 + duty_motor_min, 60.0)

    rospy.spin()


if __name__ == '__main__':
    try:
        driver()
    except rospy.ROSInterruptException:
        pass
