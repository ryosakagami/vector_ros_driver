#!/usr/bin/env python3.6

import rospy
import rospkg
import anki_vector
import cv_bridge
import numpy
import yaml

from sensor_msgs.msg import Image, CameraInfo

class Camera(object):
    def __init__(self, async_robot, publish_rate=30, image_frame_id='camera_link'):
        self.async_robot = async_robot
        self.rate = rospy.Rate(publish_rate)
        self.image_frame_id = image_frame_id

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('vector_ros_driver')

        # Camera calibration file
        camera_calibration_path = rospy.get_param(
            '~camera_calibration_path', pkg_path + "/configs/camera_calibration.yaml")
        with open(camera_calibration_path, "r") as file_handle:
            calib_data = yaml.load(file_handle)

        # CV bridge
        self.bridge = cv_bridge.CvBridge()

        # Publishers
        self.image_raw_publisher = rospy.Publisher("~camera/image_raw", Image, queue_size=1)
        self.camera_info_publisher = rospy.Publisher("~camera/camera_info", CameraInfo, queue_size=1)

        # Camera information
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = self.image_frame_id
        self.camera_info.height = calib_data["image_height"]
        self.camera_info.width = calib_data["image_width"]
        self.camera_info.K = calib_data["camera_matrix"]["data"]
        self.camera_info.D = calib_data["distortion_coefficients"]["data"]
        self.camera_info.R = calib_data["rectification_matrix"]["data"]
        self.camera_info.P = calib_data["projection_matrix"]["data"]
        self.camera_info.distortion_model = calib_data["camera_model"]


        # Start publishing camera topics
        self.publish_camera_feed()

    def publish_camera_feed(self):


        while not rospy.is_shutdown():
            image = self.bridge.cv2_to_imgmsg(numpy.asarray(self.async_robot.camera.latest_image), encoding="rgb8") # convert PIL.Image to ROS Image
            self.camera_info.header.stamp = rospy.Time.now()
            image.header.stamp = rospy.Time.now()
            image.header.frame_id = self.image_frame_id
            self.image_raw_publisher.publish(image)
            self.camera_info_publisher.publish(self.camera_info)

            # make sure to publish at required rate
            self.rate.sleep()

if __name__=="__main__":
    rospy.init_node("camera")
    async_robot = anki_vector.AsyncRobot(enable_camera_feed=True)
    async_robot.connect()
    Camera(async_robot)
    rospy.spin()

