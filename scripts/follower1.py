#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel',Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)
        self.twist = Twist()
        rospy.on_shutdown(self.clean_shutdown)
       
        #rospy.loginfo(self.twist)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([ 5, 100, 100])
        upper_yellow = numpy.array([135, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        rospy.loginfo(M)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            err = cx - w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
            rospy.loginfo("Inside the if statement")
        
        cv2.imshow("window", image)
        cv2.waitKey(3)
        rospy.loginfo(self.twist)

    def scan_callback(self, msg):
        thr1 = 0.6 # Laser scan range threshold
        thr2 = 0.6
        if msg.ranges[0]<thr1 or msg.ranges[15]<thr2 or msg.ranges[345]<thr2:
            self.twist.linear.x = 0.0 # stop
            self.twist.angular.z = 0.5 # rotate counter-clockwise
            self.cmd_vel_pub.publish(self.twist)
            rospy.loginfo("Obstacle Ahead")

    
    def clean_shutdown(self):
        ''' Stop robot when shutting down '''

        rospy.loginfo("System is shutting down. Stopping robot...")
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)


rospy.init_node('follower')
follower = Follower()
rospy.spin()