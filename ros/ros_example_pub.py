import rospy
from std_msgs.msg import String, Int32


def main():
    rospy.init_node("ros_example_pub")

    str_pub = rospy.Publisher("/str", String, queue_size=1)
    int_pub = rospy.Publisher("/int", Int32, queue_size=1)
    cnt = 0

    loop_rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        # string
        str_msg = String()
        str_msg.data = "hello"
        str_pub.publish(str_msg)
        # int32
        int_msg = Int32()
        int_msg.data = cnt
        int_pub.publish(int_msg)
        cnt += 1
        # sleep
        loop_rate.sleep()


if __name__ == "__main__":
    main()