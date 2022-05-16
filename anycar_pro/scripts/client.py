#!/usr/bin/env python3

import rospy

from anycarpro import Client

if __name__ == '__main__':
    rospy.init_node("anycar_pro", anonymous=True)

    port: str = rospy.get_param('~port')

    client = Client(port)
    rospy.on_shutdown(client.close)

    client.open()
    rospy.spin()
