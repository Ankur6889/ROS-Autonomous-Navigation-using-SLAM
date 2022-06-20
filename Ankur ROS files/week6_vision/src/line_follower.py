#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from tb3 import Tb3Move

class LineFollower(object):
    def __init__(self):
        node_name = "line_follower"
        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(5)

        self.cvbridge_interface = CvBridge()
        self.img_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_cb)
        self.robot_controller = Tb3Move()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_cb(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        height, width, _ = cv_img.shape
        crop_width = 400
        crop_height = 200
        crop_x = int((width / 2) - (crop_width / 2))
        crop_z0=int((height)-300)
        cropped_img = cv_img[crop_z0:crop_z0+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        lower = (150, 180, 100)
        upper = (165, 255, 255)
        mask = cv2.inRange(hsv_img, lower, upper)
        res = cv2.bitwise_and(cropped_img, cropped_img, mask = mask)

        m = cv2.moments(mask)
        cy = m['m10'] / (m['m00'] + 1e-5)
        cz = m['m01'] / (m['m00'] + 1e-5)
                
        cv2.circle(res, (int(cy), int(cz)), 10, (255, 0, 0), 2)
        cv2.imshow("filtered image", res)
        cv2.waitKey(1)
                
        y_error = cy - (width / 2)
        kp = 1.0 / 5000.0

        fwd_vel = 0.1
        ang_vel = kp * y_error
        if ang_vel>1.50:
            ang_vel=1
            print(f"Warning: The angular speed invalid, Speed set to 1rad/s")
        if abs(y_error)== (width/2):
            ang_vel=-ang_vel*5
        print(f"Y-error = {y_error:.3f} pixels, ang_vel = {ang_vel:.3f} rad/s")
        crop_width = 400
        crop_height = 10
        crop_x = int((width / 2) - (crop_width / 2))
        crop_z0=int((height)-10)
        cropped_img = cv_img[crop_z0:crop_z0+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        lowerR=(0,220,100)
        upperR=(6,255,255)
        maskR=cv2.inRange(hsv_img,lowerR,upperR)
        res = cv2.bitwise_and(cropped_img, cropped_img, mask = maskR)
        mR = cv2.moments(maskR)
        cyR = mR['m10'] / (mR['m00'] + 1e-5)
        czR = mR['m01'] / (mR['m00'] + 1e-5)
        if cyR > 0:
            if cy >0:
                fwd_vel=0.5
                ang_vel=-ang_vel
            else:
                print(f"Finish line reached, Robot stopping")
                fwd_vel=0.0
                ang_vel=0.0
        self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
        self.robot_controller.publish()  # publish the velocity command

    def main(self):
        while not self.ctrl_c:
            self.rate.sleep()

if __name__ == '__main__':
    lf_instance = LineFollower()
    try:
        lf_instance.main()
    except rospy.ROSInterruptException:
        pass