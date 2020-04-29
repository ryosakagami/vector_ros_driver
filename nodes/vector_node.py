#!/usr/bin/env python3.6

import rospy
import anki_vector
import threading
import os
import sys

from vector_ros_driver.vector import Vector
from vector_ros_driver.anim import Animation
from vector_ros_driver.drive import Drive
from vector_ros_driver.camera import Camera
from vector_ros_driver.behavior import Behavior
from vector_ros_driver.tf import JointStatesPublisher
from vector_ros_driver.nav_map import NavMap

if __name__=="__main__":
    rospy.init_node("vector")

    # use mock robot object if required
    if rospy.get_param("~use_mock", False):
        rospy.loginfo("using mock!")
        sys.path.append(os.path.join(os.path.dirname(__file__), "..", "test"))
        import mock_robot
        async_robot = mock_robot.MockRobot()

    else:
        with anki_vector.AsyncRobot() as async_robot:
            async_robot.camera.init_camera_feed()
            async_robot.nav_map.init_nav_map_feed()

    # connect to Vector
    async_robot.connect()

    # start all using shared AsyncRobot object
    Vector(async_robot)
    Animation(async_robot)
    Behavior(async_robot)

    # these services require threads to run(and publish) in parallel
    drive_thread = threading.Thread(target=Drive, args=(async_robot,))
    drive_thread.start()

    camera_thread = threading.Thread(target=Camera, args=(async_robot,))
    camera_thread.start()

    tf_thread = threading.Thread(target=JointStatesPublisher, args=(async_robot,))
    tf_thread.start()

    nav_map_thread = threading.Thread(target=NavMap, args=(async_robot,))
    nav_map_thread.start()

    rospy.spin()