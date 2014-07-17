#!/usr/bin/env python
import rospy
from laser_assembler.srv import *

class AssembleCaller:
    cloud_pub = None
    joint_sub = None
    assemble_srv = None
    command_pub = None
    prev_angle = None
    max_angle = None
    min_angle = None
    lower_threshold = None
    upper_threshold = None
    scan_time = None

    def init(self):
        rospy.init_node('test_assemble')
        self.cloud_pub = rospy.Publisher('assemble_cloud', sensor_msgs.msg.PointCloud2)
        self.command_pub = rospy.Publisher('/tilt_controller/command', std_msgs.msg.Float64)
        self.joint_sub = rospy.Subscriber('joint_states', sensor_msgs.msg.JointState, self.joint_callback)
        self.assemble_srv = rospy.ServiceProxy('assemble_scans2', AssembleScans2)

        if not self.max_angle:
            self.max_angle = 0.7
        if not self.min_angle:
            self.min_angle = -0.95
        if not self.scan_time:
            self.scan_time = 10.0
        if not self.lower_threshold:
            self.lower_threshold = self.min_angle + 0.05
        if not self.upper_threshold:
            self.upper_threshold = self.max_angle - 0.05

    def move_to_angle(self, angle):
        self.command_pub.publish(std_msgs.msg.Float64(angle))

    def joint_callback(self, msg):
        pos = None
        try:
            pos = msg.position[msg.name.index('tilt_joint')]
            ##print 'pos %f'%pos
        except:
            print 'exept'

        if pos:
            if not self.prev_angle:
                self.prev_angle = pos
                return
            if pos > self.max_angle:
                self.move_to_angle(self.min_angle - 0.02)

            if self.prev_angle < self.upper_threshold and pos > self.upper_threshold:
                self.scan_and_publish(self.scan_time)

            if self.prev_angle > self.lower_threshold and pos < self.lower_threshold:
                self.scan_and_publish(self.scan_time)

            if pos < self.min_angle:
                self.move_to_angle(self.max_angle + 0.02)

            self.prev_angle = pos

    def scan_and_publish(self, sec):
        tm = rospy.get_rostime()
        print 'scan rostime : %d %d'%(tm.secs, tm.nsecs)
        req = AssembleScans2Request()
        req.begin = rospy.Time.from_sec(tm.to_sec() - sec)
        req.end = tm

        ret = self.assemble_srv(req.begin, req.end)
        ## print ret.cloud
        if ret:
            print 'publish'
            self.cloud_pub.publish(ret.cloud)

    def spin(self):
        rospy.spin()

if __name__=='__main__':
    a = AssembleCaller()
    a.init()
    a.spin()
