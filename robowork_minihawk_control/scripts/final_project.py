#!/usr/bin/python

import rospy
from mavros_msgs.srv import SetMode, CommandBool
from apriltag_ros.msg import AprilTagDetectionArray
from mavros_msgs.msg import OverrideRCIn

class FinalProject:
    def __init__(self):
        self.apriltag_data = None
        self.apriltag_detection = False
        self.rate = rospy.Rate(10)  # 10 Hz update

        rospy.init_node('final_project_node', anonymous=True)

        rospy.wait_for_service('/minihawk_SIM/mavros/set_mode')
        rospy.wait_for_service('/minihawk_SIM/mavros/cmd/arming')

        self.apriltag_subscriber = rospy.Subscriber(
            '/minihawk_SIM/MH_usb_camera_link_optical/tag_detections', 
            AprilTagDetectionArray, 
            self.apriltag_return
        )

        self.rc_pub = rospy.Publisher(
            '/minihawk_SIM/mavros/rc/override', 
            OverrideRCIn, queue_size=10
        )

        self.set_auto_mode()
        self.arm_motors()
        self.wait_for_apriltag()
        self.finetune_position()
        self.set_land_mode()

    def set_auto_mode(self):
        try:
            set_mode = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
            set_mode(0, 'AUTO')
        except rospy.ServiceException as e:
            print('Error setting AUTO mode:', e)

    def arm_motors(self):
        try:
            armed = rospy.ServiceProxy('/minihawk_SIM/mavros/cmd/arming', CommandBool)
            armed(True)
        except rospy.ServiceException as e:
            print('Error arming motors:', e)

    def apriltag_return(self, message):
        if len(message.detections) > 0:
            self.apriltag_data = message.detections[0]
            self.apriltag_detection = True
        else:
            self.apriltag_data = None
            self.apriltag_detection = False

    def get_apriltag_offsets(self):
        """Returns (x_offset, y_offset) if available, else None."""
        if not self.apriltag_data:
            return None, None
        pose = self.apriltag_data.pose.pose.pose
        return pose.position.x, pose.position.y

    def wait_for_apriltag(self):
        while not rospy.is_shutdown():
            if self.apriltag_detection:
                rospy.loginfo("AprilTag detected!")
                break
            self.rate.sleep()
        rospy.sleep(1)  # short pause

    def finetune_position(self):
        try:
            set_mode = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
            set_mode(0, 'QLOITER')
        except rospy.ServiceException as e:
            print('Error setting QLOITER mode:', e)

        tolerance = 0.1  # You can set this lower if you want higher precision
        max_iterations = 100
        iteration = 0

        while not rospy.is_shutdown() and iteration < max_iterations:
            x_offset, y_offset = self.get_apriltag_offsets()
            if x_offset is not None and y_offset is not None:
                print("Offsets x: {:.3f}  y: {:.3f}".format(x_offset, y_offset))

                # Check if within tolerance
                if abs(x_offset) < tolerance and abs(y_offset) < tolerance:
                    print("Arrived at center of AprilTag! Offsets ~ 0")
                    break

                # RC calculations
                P = 200  # gain (tune as necessary for your setup)
                roll = int(max(1000, min(2000, 1500 + P*y_offset)))
                pitch = int(max(1000, min(2000, 1500 - P*x_offset)))
                throttle = 1500
                yaw = 1500
                rc_override = OverrideRCIn()
                rc_override.channels = [roll, pitch, throttle, yaw] + [0]*14

                self.rc_pub.publish(rc_override)
            else:
                print("No AprilTag detected, holding position.")

            self.rate.sleep()
            iteration += 1

    def set_land_mode(self):
        try:
            set_mode = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
            set_mode(0, 'QLAND')
            print("Landing...")
        except rospy.ServiceException as e:
            print('Error setting QLAND mode:', e)


if __name__ == '__main__':
    try:
        FinalProject()
    except rospy.ROSInterruptException:
        pass