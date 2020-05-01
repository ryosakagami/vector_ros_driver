#!/usr/bin/env python3.6

import rospy
import random
import numpy as np

import anki_vector
from anki_vector.events import Events
from anki_vector.nav_map import NavMapGrid, NavNodeContentTypes
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from grid_map_msgs.msg import GridMap


class NavMap(object):
    def __init__(self, async_robot, publish_rate=1):
        self.async_robot = async_robot
        self.rate = rospy.Rate(publish_rate)

        # Publishers
        self.grid_map_publisher = rospy.Publisher("~grid_map", GridMap, queue_size=1)

        # Subscribers
        # self.async_robot.events.subscribe(self._on_nav_map_update, Events.nav_map_update)

        # Start publishing NavMap topic
        self.publish_nav_map()

    # This function is attributed to
    # https://github.com/neka-nat/ros_np_multiarray/blob/master/src/ros_np_multiarray/ros_np_multiarray.py
    def _numpy_to_multiarray(self, np_array):
        multiarray = Float32MultiArray()
        multiarray.layout.dim = [MultiArrayDimension('dim%d' % i,
                                                    np_array.shape[i],
                                                    np_array.shape[i] * np_array.dtype.itemsize) for i in range(np_array.ndim)]
        multiarray.data = np_array.reshape([1, -1])[0].tolist()
        return multiarray

    def publish_nav_map(self):
        while not rospy.is_shutdown():
            try:
                latest_nav_map = self.async_robot.nav_map.latest_nav_map
            except anki_vector.exceptions.VectorPropertyValueNotReadyException as e:
                print(e)
                print("Early return and retry after sleeping...")
                self.rate.sleep()
                continue

            grid_map = self.nav_map_grid_to_grid_map(latest_nav_map)
            self.grid_map_publisher.publish(grid_map)
            self.rate.sleep()

    def nav_map_grid_to_grid_map(self, nav_map_grid):
        grid_map = GridMap()
        grid_map.layers = ['nav_map']

        # GridMapInfo
        grid_map.info.header.stamp = rospy.Time.now()
        grid_map.info.header.frame_id = 'base_link'
        grid_map.info.resolution = 0.01  # 10mm / cell
        grid_map.info.length_x = nav_map_grid.size / 1000.
        grid_map.info.length_y = nav_map_grid.size / 1000.
        grid_map.info.pose.position.x = nav_map_grid.center.x / 1000.
        grid_map.info.pose.position.y = nav_map_grid.center.y / 1000.
        grid_map.info.pose.position.z = nav_map_grid.center.z / 1000.

        height = int(grid_map.info.length_x / grid_map.info.resolution)
        width = int(grid_map.info.length_y / grid_map.info.resolution)

        def _index_to_pose(row_ix, col_ix):
            x = nav_map_grid.center.x - nav_map_grid.size + row_ix * grid_map.info.resolution * 1000  # unit: mm
            y = nav_map_grid.center.y + nav_map_grid.size - col_ix * grid_map.info.resolution * 1000  # unit: mm
            return x, y  # unit: mm

        np_array = np.zeros((height, width))
        for i in range(height):
            for j in range(width):
                x, y = _index_to_pose(i, j)
                num = nav_map_grid.get_content(x, y)
                if type(num) == int:
                    value = float(num)
                else:
                    value = float(num.value)
                # np_array[i, j] = float(value)
                np_array[i, j] = float(i)
        # for some reason, if we set row_index to dim[0].label, visualizer complains.
        # thus, we first transpose np_array
        data = self._numpy_to_multiarray(np_array.transpose())
        data.layout.dim[0].label = "column_index"
        data.layout.dim[1].label = "row_index"

        grid_map.data.append(data)

        # Return grid map
        return grid_map

