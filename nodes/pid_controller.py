#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server
from husky_pid.cfg import PIDConfig


class PID:
    def __init__(self, Kp, Kd, Ki, dt):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.dt = dt
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0
        self.desired_distance = 0

    def srv_callback(self, config, level):
        # Update gains from dynamic reconfigure values
        self.Kp = config.Kp
        self.Kd = config.Kd
        self.Ki = config.Ki
        self.desired_distance = config.Distance
        
        print("P: %2f, I: %2f, D: %2f" % (self.Kp, self.Ki, self.Kd))
        print("Desired distance: %2f" % (self.desired_distance))
        return config
        

    def update_control(self, current_error, reset_prev=False):
    	# Proportional control
        p_control = self.Kp * self.curr_error

        # Integral control
        self.sum_error += (self.curr_error) * self.dt
        i_control = self.sum_error * self.Ki
        
        # Derivative control
        self.curr_error_deriv = (self.curr_error - self.prev_error) / self.dt
        d_control = self.Kd * self.curr_error_deriv

        # Control command = P + I + D
        angle = p_control + d_control + i_control
        self.control = angle

        # update error
        self.prev_error = self.curr_error
        self.curr_error = current_error
        self.prev_error_deriv = self.curr_error_deriv
        
        # print("P: %2f, I: %2f, D: %2f" % (self.Kp, self.Ki, self.Kd))
        # print("P: %2f, I: %2f, D: %2f" % (p_gain, i_gain, d_gain))
        # print("--------------------------------------")
        return self.control 


class Control:
    def __init__(self):
        rospy.init_node('pid_controller', anonymous=True)

        self.forward_speed = rospy.get_param("~forward_speed")
        self.desired_distance = rospy.get_param("~desired_distance")
        self.hz = 5
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback)

        # PID init with dynamic reconfigure
        self.pid = PID(0, 0, 0, 0.2)
        self.pid.desired_distance = self.desired_distance
        srv = Server(PIDConfig, self.pid.srv_callback)


    def laser_scan_callback(self, msg):
        # compute cross-track error
        cte_pub = rospy.Publisher('/cte', Float32, queue_size=1000)
        
        cte = min(msg.ranges) - self.pid.desired_distance
        cte = self.pid.desired_distance - min(msg.ranges)
        
        cte_pub.publish(cte)
        
        # This is where the magic happens!
        # Steering angle is the only control mechanism here
        angle = self.pid.update_control(cte)
        
        linear = Vector3(self.forward_speed, 0, 0)
        angular = Vector3(0, 0, angle)
        
        # Send new twist to husky
        twist = Twist(linear, angular)
        self.cmd_pub.publish(twist)


    def run(self):
        rate = rospy.Rate(self.hz)
        rospy.spin()
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    ctl = Control()
    ctl.run()