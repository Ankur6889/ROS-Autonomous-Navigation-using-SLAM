#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from com2009_msgs.srv import TimedMovement,TimedMovementResponse

service_name = "move_service"

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
vel = Twist()

def callback_function(service_request):

    service_response = TimedMovementResponse()

    if service_request.movement_request == 'forward':
        print(f"The '{service_name}' Server received a 'forward' request and the robot will now move forward for '{service_request.duration}' seconds...")

        StartTime = rospy.get_rostime()

        vel.linear.x = 0.1
        pub.publish(vel)
        rospy.loginfo('Published the velocity command to /cmd_vel')
        while (rospy.get_rostime().secs - StartTime.secs) < service_request.duration:
            continue

        rospy.loginfo(f"Duration '{service_request.duration}' seconds have elapsed, stopping the robot...")

        vel.linear.x = 0.0
        pub.publish(vel)

        service_response.success = True
    elif service_request.movement_request=='backwards':
        print(f"The '{service_name}' Server received a 'backwards' request and the robot will now move backwards for '{service_request.duration}' seconds...")
        StartTime = rospy.get_rostime()

        vel.linear.x = -0.1
        pub.publish(vel)

        rospy.loginfo('Published the velocity command to /cmd_vel')
        while (rospy.get_rostime().secs - StartTime.secs) < service_request.duration:
            continue

        rospy.loginfo(f"Duration '{service_request.duration}' seconds have elapsed, stopping the robot...")

        vel.linear.x = 0.0
        pub.publish(vel)

        service_response.success = True
    elif service_request.movement_request=='left':
        print(f"The '{service_name}' Server received a 'left' request and the robot will now turn left for '{service_request.duration}' seconds...")
        StartTime = rospy.get_rostime()

        vel.linear.x = 0
        vel.angular.z=0.1
        pub.publish(vel)

        rospy.loginfo('Published the velocity command to /cmd_vel')
        while (rospy.get_rostime().secs - StartTime.secs) < service_request.duration:
            continue

        rospy.loginfo(f"Duration '{service_request.duration}' seconds have elapsed, stopping the robot...")

        vel.linear.x = 0.0
        vel.angular.z=0.0
        pub.publish(vel)

        service_response.success = True
    elif service_request.movement_request=='right':
        print(f"The '{service_name}' Server received a 'right' request and the robot will now turn right for '{service_request.duration}' seconds...")
        StartTime = rospy.get_rostime()

        vel.linear.x = 0
        vel.angular.z=-0.1
        pub.publish(vel)

        rospy.loginfo('Published the velocity command to /cmd_vel')
        while (rospy.get_rostime().secs - StartTime.secs) < service_request.duration:
            continue

        rospy.loginfo(f"Duration '{service_request.duration}' seconds have elapsed, stopping the robot...")

        vel.linear.x = 0.0
        vel.angular.z=0.0
        pub.publish(vel)

        service_response.success = True
    else:
        service_response.success = False
    return service_response

rospy.init_node(f"{service_name}_server")
my_service = rospy.Service(service_name, TimedMovement, callback_function)
rospy.loginfo(f"the '{service_name}' Server is ready to be called...")
rospy.spin()