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

    # def _on_nav_map_update(self, _robot, _event_type, msg):
    #     grid_map = self.nav_map_grid_to_grid_map(msg)
    #     self.grid_map_publisher.publish(grid_map)

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
        data.layout.dim[0].label = "column_index"  # height
        data.layout.dim[1].label = "row_index"  # width
        height = int(grid_map.info.length_y / grid_map.info.resolution)
        width = int(grid_map.info.length_x / grid_map.info.resolution)
        data.layout.dim[0].size = height
        data.layout.dim[1].size = width
        data.layout.dim[0].stride = height * width
        data.layout.dim[1].stride = width
        data.layout.data_offset = 0
        data.data = [NavNodeContentTypes.Unknown] * height * width

        # Fill in data
        dstride0 = data.layout.dim[0].stride
        dstride1 = data.layout.dim[1].stride
        offset = data.layout.data_offset

        def _center_to_indices(center, half_size):
            x_cen = (nav_map_grid.size / 2. + (center.x - nav_map_grid.center.x)) / 1000.  # unit: m
            x_min = x_cen - half_size / 1000.
            x_max = x_cen + half_size / 1000.
            y_cen = (nav_map_grid.size / 2. - (center.y - nav_map_grid.center.y)) / 1000.  # unit: m
            y_min = y_cen - half_size / 1000.
            y_max = y_cen + half_size / 1000.
            try:
                assert x_min >= 0 and x_max <= grid_map.info.length_x \
                    and y_min >= 0 and y_max <= grid_map.info.length_y
            except AssertionError as e:
                print("x_min: {} or x_max: {} or y_min: {} or y_max: {} is outside of nav_map".format(x_min, x_max, y_min, y_max))
                print("Points out of map will be ignored")
            column_index_min = int(x_min / grid_map.info.resolution)
            column_index_max = int(x_max / grid_map.info.resolution)
            row_index_min = int(y_min / grid_map.info.resolution)
            row_index_max = int(y_max / grid_map.info.resolution)
            if column_index_min < 0:
                column_index_min = 0
                print("Set column_index_min = 0")
            if column_index_max > width:
                column_index_max = width
                print("Set column_index_max = {}".format(width))
            if row_index_min < 0:
                row_index_min = 0
                print("Set row_index_min = 0")
            if row_index_max > height:
                row_index_max = height
                print("Set row_index_max = {}".format(height))
            return list(range(row_index_min, row_index_max)), list(range(column_index_min, column_index_max))

        def _recursive_draw(grid_node: anki_vector.nav_map.NavMapGridNode):
            if grid_node.children is not None:
                for child in grid_node.children:
                    _recursive_draw(child)
            else:
                # leaf node
                cen = grid_node.center
                half_size = grid_node.size * 0.5
                row_indices, column_indices = _center_to_indices(cen, half_size)
                for row_index in row_indices:
                    for column_index in column_indices:
                        try:
                            data.data[offset + row_index + dstride1 * column_index] = float(grid_node.content)
                        except IndexError:
                            print("row_index {} or column_index {} is out of range".format(row_index, column_index))
                            print("len(data.data) is {}, but offset + row_index + dstride1 * column_index is {}".format(len(data.data), offset + row_index + dstride1 * column_index))

        _recursive_draw(nav_map_grid.root_node)

        # Dummy data for test
        """
        tmpmat = np.zeros((height, width))
        for i in range(height):
            for j in range(width):
                num = random.randrange(0,8)
                data.data[offset + i + dstride1*j] = num
                tmpmat[i, j] = num
        """

        grid_map.data.append(data)

        # Return grid map
        return grid_map

