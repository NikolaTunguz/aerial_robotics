#!/usr/bin/python

import rospy
from mavros_msgs.srv import SetMode, CommandBool
from apriltag_ros.msg import AprilTagDetectionArray
from mavros_msgs.msg import OverrideRCIn


class FinalProject:
    def __init__(self):
        #variables for storing information
        self.apriltag_data = None
        self.apriltag_detection = False

        #creating nodes
        rospy.init_node('final_project_node', anonymous = True)
        
        #establishing services
        rospy.wait_for_service('/minihawk_SIM/mavros/set_mode')
        rospy.wait_for_service('/minihawk_SIM/mavros/cmd/arming')


        #calling services
        #print('\n\n\n\n\n\n\n\n\n\n\TESTTESTTESTTEST\n\n\n\n\n\n')
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
            print('Errored: ', e)

    def arm_motors(self):
        try:
            armed = rospy.ServiceProxy('/minihawk_SIM/mavros/cmd/arming', CommandBool)
            armed(True)
        except rospy.ServiceException as e:
            print('Errored: ', e)

    def apriltag_return(self, message):
        if len(message.detections) > 0:
            self.apriltag_data = message.detections[0]
            self.apriltag_detection = True
            #print('\n\nnew apriltag data\n\n')

    def get_apriltag_position(self, pose):
        while hasattr(pose, "pose"):
            pose = pose.pose
        return pose.position

    def wait_for_apriltag(self):
        apriltag_subscriber = rospy.Subscriber('/minihawk_SIM/MH_usb_camera_link_optical/tag_detections', AprilTagDetectionArray, self.apriltag_return)
        while not self.apriltag_detection and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.sleep(5)

    def finetune_position(self):
        #setting to qloiter
        try:
            set_mode = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
            set_mode(0, 'QLOITER')  
        except rospy.ServiceException as e:
            print('Errored: ', e)

        publish_control = rospy.Publisher('/minihawk_SIM/mavros/rc/override', OverrideRCIn, queue_size = 10)

        #adjusting position
        while True:
            if self.apriltag_data:
                #take data
                apriltag_position = self.get_apriltag_position(self.apriltag_data.pose)#self.apriltag_data.pose.pose.position
                apriltag_x_offset = apriltag_position.x
                apriltag_y_offset = apriltag_position.y

                print(apriltag_x_offset, apriltag_y_offset)

                if abs(apriltag_x_offset) < 1 and abs(apriltag_y_offset) < 1:
                    #break
                    print('it tried to break')
                

                #calculate roll
                p = 50
                roll = max(1000, min(2000, int (1500 + p * apriltag_y_offset)))

                #calculate pitch
                p = 50
                pitch = max(1000, min(2000, int (1500 - p * apriltag_x_offset)))

                throttle = 1500
                yaw = 1500

                control = OverrideRCIn()
                control.channels = [roll, pitch, throttle, yaw, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

                publish_control.publish(control)

                rospy.sleep(0.5)


    def set_land_mode(self):
        try:
            set_mode = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
            set_mode(0, 'QLAND')

        except rospy.ServiceException as e:
            print('Errored: ', e)


if __name__ == '__main__':
    program = FinalProject()