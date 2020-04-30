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
        # TODO: implement here, borrowing codes from vector sdk
        """
        def _recursive_draw(grid_node: nav_map.NavMapGridNode):
            if grid_node.children is not None:
                for child in grid_node.children:
                    _recursive_draw(child)
            else:
                # leaf node - render as a quad
                map_alpha = 0.5
                cen = grid_node.center
                half_size = grid_node.size * 0.5

                # Draw outline
                glColor4f(*color_light_gray, 1.0)  # fully opaque
                glBegin(GL_LINE_STRIP)
                glVertex3f(cen.x + half_size, cen.y + half_size, cen.z)
                glVertex3f(cen.x + half_size, cen.y - half_size, cen.z)
                glVertex3f(cen.x - half_size, cen.y - half_size, cen.z)
                glVertex3f(cen.x - half_size, cen.y + half_size, cen.z)
                glVertex3f(cen.x + half_size, cen.y + half_size, cen.z)
                glEnd()

                # Draw filled contents
                glColor4f(*color_for_content(grid_node.content), map_alpha)
                glBegin(GL_TRIANGLE_STRIP)
                glVertex3f(cen.x + half_size, cen.y - half_size, fill_z)
                glVertex3f(cen.x + half_size, cen.y + half_size, fill_z)
                glVertex3f(cen.x - half_size, cen.y - half_size, fill_z)
                glVertex3f(cen.x - half_size, cen.y + half_size, fill_z)
                glEnd()

        _recursive_draw(new_nav_map.root_node)
        """
        dstride0 = data.layout.dim[0].stride
        dstride1 = data.layout.dim[1].stride
        offset = data.layout.data_offset
        tmpmat = np.zeros((height, width))
        for i in range(height):
            for j in range(width):
                num = random.randrange(0,8)
                data.data[offset + i + dstride1*j] = num
                tmpmat[i,j] = num
        grid_map.data.append(data)

        # Return grid map
        return grid_map

