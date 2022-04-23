#!/usr/bin/env python

import rospy
import subprocess, shlex, psutil
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int64
import numpy as np

# Does this work with xbox controller?

# Should get rid of teensy arming sequence - unsure if this is using the arming protocol properly

# Is it even useful to have a max velocity limit? Already max accel limits, natural limits from vehicle, limits to the joystick output, time delay of PID, alpha filter, hard to keep track

ROSBAG_RECORD_DURATION = 120 # Maximum rosbag recording duration 
ROSBAG_RECORD_TOPICS = [
    "/d455/color/camera_info",
    "/d455/color/image_raw",
    "/d455/aligned_depth_to_color/camera_info"
    "/d455/aligned_depth_to_color/image_raw",
    "/d455/depth/color/points",
    "/t265/odom/metadata",
    "/t265/odom/sample",
    "/cmd_vel",
    "/joy",
    "/Teensy/contact1",
    "/Teensy/contact2",
    "/Teensy/speed",
    "/rtabmap/(.*)",
    "/tf",
    "/tf_static"
]

class Control:
    def __init__(self):
        self.node_name = rospy.get_name()

        self.dt = 0.02
        self.arm = True
        self.manual = True
        self.vel = Twist()
        self.last_vel = Twist()

        # Rover ros parameters
        self.lin_vel_max = rospy.get_param("~lin_vel_max", 5)
        self.ang_vel_max = rospy.get_param("~ang_vel_max", 5)
        self.lin_acc_max = rospy.get_param("~lin_acc_max", 2)
        self.ang_acc_max = rospy.get_param("~ang_acc_max", 2)

        # Controller ros parameters
        # Defaults set to xbox 360 wireless for linux
        self.green = rospy.get_param("~green", 0)
        self.red = rospy.get_param("~red", 1)
        self.blue = rospy.get_param("~blue", 2)
        self.yellow = rospy.get_param("~yellow", 3)
        self.record = rospy.get_param("~record", 7)
        self.slow_trigger = rospy.get_param("~slow_trigger", 4)
        self.fast_trigger = rospy.get_param("~fast_trigger", 5)
        self.forward = rospy.get_param("~forward", 1)
        self.turn = rospy.get_param("~turn", 2)

        # Rosbag recording variables
        self.is_recording = False # Bool used to track whether a bag is currently being recorded
        self.recording_start = None # Timestamp for when the current recording started
        self.record_process = None # Pointer to the subprocess running the rosbag record command

        self.low_speed_linear = rospy.get_param("~low_speed_linear",0.5)
        self.high_speed_linear = rospy.get_param("~high_speed_linear",1.0)
        self.low_speed_angular = rospy.get_param("~low_speed_angular",1.0)
        self.high_speed_angular = rospy.get_param("~high_speed_angular",4.0)

        # Alpha filter ros parameters
        # ARBITRARILY CHOSEN -- NEED TO FIND ACTUAL TIME CONSTANTS
        self.lin_time_constant = rospy.get_param("~lin_time_constant", 0.2)
        self.ang_time_constant = rospy.get_param("~ang_time_constant", 0.2)

        # Comms
        self.last_joy_stamp = None
        self.last_auto_stamp = None
        self.joy_comms_timeout = 0.05
        self.auto_comms_timeout = 1

        # Pub & Sub
        self.pub_arm = rospy.Publisher('/NUC_topic', Int64, queue_size=10)
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.joy_cb, queue_size=1) ## Global, change to local namespace later
        self.sub_auto = rospy.Subscriber("vel_cmd_auto", Twist, self.auto_cb, queue_size=1)

        # Arming sequences
        self.arm_start = 2000 # 
        self.arm_stop = 1000 # tells rover to stop
        self.arm_shutdown = 1001 # sent from NUC during endinghook
        self.arm_neutral = 1401 # tells teensy to start

        # Arm teensy initially
        self.pub_arm.publish(self.arm_neutral)

        # Shutdown procedure
        rospy.on_shutdown(self.on_shutdown)

        # Run loop callback every dt
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.loop_cb)
    

    def loop_cb(self, event):
        
        if self.manual:

            # Connected?
            if self.last_joy_stamp is None:
                rospy.loginfo("[rover] Haven't received any joystick msgs yet.")
                self.pub_vel.publish(Twist())
                self.last_vel = Twist()
                return

            # Timeout
            if (rospy.Time.now() - self.last_joy_stamp).to_sec() >= self.joy_comms_timeout:
                # Some controllers only send msg when something on the controller changes, can we do this with manual control?
                # self.pub_vel.publish(Twist())
                # self.last_vel = Twist()
                pass

        else: # Auto mode

            # Connected?
            if self.last_auto_stamp is None:
                rospy.loginfo("[rover] Haven't received any autonomous cmd_vel msgs yet.")
                self.pub_vel.publish(Twist())
                self.last_vel = Twist()
                return

            # Timeout
            if (rospy.Time.now() - self.last_auto_stamp).to_sec() >= self.auto_comms_timeout:
                rospy.logwarn("[rover] In autonomous mode, but haven't received messages in a while. Sending 0 velocity.")
                self.pub_vel.publish(Twist())
                self.last_vel = Twist()
                return

        # Smooth out the velocity discontinuity with alpha filter
        smooth_vel = Twist()
        smooth_vel.linear.x = self.alpha(self.vel.linear.x, self.last_vel.linear.x, self.dt, self.lin_time_constant)
        smooth_vel.angular.z = self.alpha(self.vel.angular.z, self.last_vel.angular.z, self.dt, self.ang_time_constant)

        # Are we armed?
        if self.arm:
            self.pub_vel.publish(smooth_vel)
            self.last_vel = smooth_vel
        else:
            self.pub_vel.publish(Twist())
            self.last_vel = Twist()


    def joy_cb(self, data):
        if data.header.seq == 0: # first joystick msg is usually garbage
            return

        self.last_joy_stamp = data.header.stamp

        # Read joystick input
        green = data.buttons[self.green]
        red = data.buttons[self.red]
        blue = data.buttons[self.blue]
        yellow = data.buttons[self.yellow]
        record = data.buttons[self.record]
        slow_trigger = data.buttons[self.slow_trigger]
        fast_trigger = data.buttons[self.fast_trigger]
        forward = data.axes[self.forward]
        turn = data.axes[self.turn]

        if green == 1:
            self.arm = True
            self.pub_arm.publish(self.arm_start)
            self.vel = Twist()
            rospy.loginfo('Rover armed')

        if red == 1:
            self.arm = False
            self.pub_arm.publish(self.arm_neutral)
            self.vel = Twist()
            rospy.loginfo('Rover disarmed')

        if blue == 1:
            self.manual = True
            rospy.loginfo('Rover in manual control mode')

        if yellow == 1:
            self.manual = False
            rospy.loginfo('Rover in autonomous control mode')

        if self.is_recording and rospy.Time.now() - self.recording_start >= rospy.Duration(ROSBAG_RECORD_DURATION):
            self.rosbag_stop_recording()

        if record == 1:
            if self.is_recording and rospy.Time.now() - self.recording_start > rospy.Duration(1):
                self.rosbag_stop_recording()
            elif not self.is_recording:
                self.rosbag_start_recording()

        if self.manual:
            if fast_trigger:
                linear_speed = self.high_speed_linear
                angular_speed = self.high_speed_angular
            else:
                linear_speed = self.low_speed_linear
                angular_speed = self.low_speed_angular

            vx = 0
            if abs(forward)>=0.1:
                vx = linear_speed*forward
            vw = 0
            if abs(turn)>=0.1:
                vw = angular_speed*turn

            vx, vw = self.cap_speed(vx, vw)

            vel = Twist()
            vel.linear.x = vx
            vel.angular.z = vw
            self.vel = vel

    def rosbag_start_recording(self):
        self.recording_start = rospy.Time.now()
        self.is_recording = True
        command = ['rosbag', 'record',
                '--duration=%d' % ROSBAG_RECORD_DURATION,
                '--output-name=/home/acl/code/vision_ws/bags/%s' % str(self.recording_start), 
                '--regex'] + ROSBAG_RECORD_TOPICS
        self.record_process = subprocess.Popen(command)
        
    def rosbag_stop_recording(self):
        #for proc in psutil.process_iter():
        #    if "record" in proc.name() and set(self.ROSBAG_RECORD_TOPICS).issubset(proc.cmdline()):
        #        proc.send_signal(subprocess.signal.SIGINT)

        self.record_process.send_signal(subprocess.signal.SIGINT)
        self.record_process.terminate()
        self.is_recording = False
        self.recording_start = None

    def auto_cb(self, auto_vel):
        self.last_auto_stamp = rospy.Time.now()

        if not self.manual:
            # Impose speed limits
            vel = Twist()
            vx, vw = self.cap_speed(auto_vel.linear.x, auto_vel.angular.z)
            vel.linear.x = vx
            vel.angular.z = vw
            self.vel = vel


    def alpha(self, new, old, dt, tau):
        """
        Alpha filter for smoothing velocity inputs
        https://en.wikipedia.org/wiki/Low-pass_filter#RC_filter
        """
        alpha = dt / (dt + tau)
        vel = alpha * new + (1 - alpha) * old
        if abs(vel) <= 0.01:
            vel = 0
        return vel


    def cap_speed(self, vx, vw):

        if vx >= self.lin_vel_max:
            vx = self.lin_vel_max
        elif vx <= -self.lin_vel_max:
            vx = -self.lin_vel_max

        if vw >= self.ang_vel_max:
            vw = self.ang_vel_max
        elif vw <= -self.ang_vel_max:
            vw = -self.ang_vel_max

        return vx, vw


    def on_shutdown(self):
        # Disarm
        self.pub_vel.publish(Twist())
        self.pub_arm.publish(self.arm_shutdown) 

        rospy.loginfo("[%s] shutting down.\n"%self.node_name)
        pass


if __name__ == '__main__':
    rospy.init_node('rover', anonymous=False)

    node = Control()

    rospy.spin()
