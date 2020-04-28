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
    def __init__(self, async_robot):
        self.async_robot = async_robot

        # Publishers
        self.grid_map_publisher = rospy.Publisher("~grid_map", GridMap, queue_size=1)

        # Subscribers
        self.async_robot.events.subscribe(self._on_nav_map_update, Events.nav_map_update)

    def _on_nav_map_update(self, _robot, _event_type, msg):
        grid_map = self.nav_map_grid_to_grid_map(msg)
        self.grid_map_publisher.publish(grid_map)

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

        # Data initialization
        data = Float32MultiArray()
        data.layout.dim.append(MultiArrayDimension())
        data.layout.dim.append(MultiArrayDimension())
        data.layout.dim[0].label = "height"
        data.layout.dim[1].label = "width"
        height = int(grid_map.info.length_y / grid_map.info.resolution)
        width = int(grid_map.info.length_x / grid_map.info.resolution)
        data.layout.dim[0].size = height
        data.layout.dim[1].size = width
        data.layout.dim[0].stride = height * width
        data.layout.dim[1].stride = width
        data.layout.data_offset = 0
        data.data = [NavNodeContentTypes.Unknown]*9

        # Fill in data
        dstride0 = data.layout.dim[0].stride
        dstride1 = data.layout.dim[1].stride
        offset = data.layout.data_offset
        tmpmat = np.zeros((height, width))
        for i in range(height):
            for j in range(width):
                num = random.randrange(0,10)
                data.data[offset + i + dstride1*j] = num
                tmpmat[i,j] = num
        grid_map.data.append(data)

        # Return grid map
        return grid_map

