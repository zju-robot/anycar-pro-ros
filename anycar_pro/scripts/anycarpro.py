from threading import Thread, Event
from time import sleep

import geometry_msgs.msg
import nav_msgs.msg
import rospy
import serial
import tf_conversions

import messages as msgs


class Client:
    def __init__(self, port: str) -> None:
        self.__ser = serial.Serial(port, 115200)
        self.__ser.close()
        self.__link = msgs.MAVLink(self.__ser)
        self.__stop_event = Event()
        self.__listen_thread = Thread(target=self.__listen)
        self.__connect_thread = Thread(target=self.__heartbeat)
        self.__odom_publisher = \
            rospy.Publisher("odom", nav_msgs.msg.Odometry, queue_size=10)
        self.__twist_listener = \
            rospy.Subscriber("cmd_vel", geometry_msgs.msg.Twist,
                             self.__twist_received)

    def __listen(self) -> None:
        while not self.__stop_event.wait(0.01):
            try:
                if self.__ser.readable():
                    byte = self.__ser.read(1)
                    msg = self.__link.parse_char(byte)
                    if msg is not None:
                        self.__publish(msg)
            except Exception as e:
                rospy.logwarn(e)

    def __heartbeat(self) -> None:
        while not self.__stop_event.wait(0.1):
            self.__link.heartbeat_send(0)

    def __publish(self, msg: msgs.MAVLink_motion_state_message) -> None:
        if type(msg) == msgs.MAVLink_heartbeat_message:
            pass
        elif type(msg) == msgs.MAVLink_motion_state_message:
            odom = nav_msgs.msg.Odometry()

            odom.header.frame_id = "odom"
            odom.header.stamp = rospy.Time.now()
            odom.child_frame_id = "base_footprint"

            odom.pose.pose.position.x = msg.x
            odom.pose.pose.position.y = msg.y
            odom.pose.pose.position.z = 0.

            q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.yaw)
            odom.pose.pose.orientation = geometry_msgs.msg.Quaternion(*q)

            odom.twist.twist.linear.x = msg.xVel
            odom.twist.twist.linear.y = odom.twist.twist.linear.z = 0.
            odom.twist.twist.angular.z = msg.yawRate
            odom.twist.twist.angular.x = odom.twist.twist.angular.y = 0.

            self.__odom_publisher.publish(odom)

    def __twist_received(self, twist: geometry_msgs.msg.Twist) -> None:
        self.__link.set_speed_send(twist.linear.x, twist.angular.z)

    def open(self) -> None:
        self.__ser.open()
        self.__stop_event.clear()
        self.__connect_thread.start()
        self.__listen_thread.start()

    def close(self) -> None:
        self.__stop_event.set()
        self.__listen_thread.join()
        self.__connect_thread.join()
        self.__ser.close()
