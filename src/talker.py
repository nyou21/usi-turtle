#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    nome_persona_che_parla = 'nadia'
    nome_persona_che_ascolta = 'giovanni'

    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # while not rospy.is_shutdown():
    #     hello_str = "ciao %s sono %s" % (nome_persona_che_ascolta, nome_persona_che_parla)
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #     rate.sleep()

    hello_str = "wellaaa %s come staiii %s" % (nome_persona_che_ascolta, nome_persona_che_parla)
    pub.publish(hello_str)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass