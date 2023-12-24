#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from math import sin, cos
import numpy as np

map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

def laser_scan_callback(scan_data):
    angle_min = scan_data.angle_min
    angle_increment = scan_data.angle_increment
    ranges = scan_data.ranges

    map_resolution = 0.05
    map_width = 300
    map_height = 300
    map_origin_x = -5.0
    map_origin_y = -5.0
    max_range = max(ranges)
    gridmap = np.full((map_height, map_width), -1, dtype=np.int8)
    max_x = -1
    max_y = -1

    for i, range_value in enumerate(ranges):
        if range_value > 0:
            angle = angle_min + i * angle_increment
            x = range_value * cos(angle)
            y = range_value * sin(angle)
            if x > max_x:
                max_x = x
            if y > max_y:
                max_y = y
            grid_x = int((x - map_origin_x) / map_resolution)
            grid_y = int((y - map_origin_y) / map_resolution)

            if 0 <= grid_x < map_width and 0 <= grid_y < map_height:
                if range_value < max_range:
                    gridmap[map_height - grid_y - 1, grid_x] = 100
                else:
                    gridmap[map_height - grid_y - 1, grid_x] = 0

    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.stamp = rospy.Time.now()
    occupancy_grid.header.frame_id = "laser_tilt_link"
    occupancy_grid.info.map_load_time = rospy.Time.now()
    occupancy_grid.info.resolution = map_resolution
    occupancy_grid.info.width = map_width
    occupancy_grid.info.height = map_height
    occupancy_grid.info.origin.position.x = map_origin_x
    occupancy_grid.info.origin.position.y = map_origin_y
    occupancy_grid.info.origin.position.z = 0.0
    occupancy_grid.info.origin.orientation.x = 0.0
    occupancy_grid.info.origin.orientation.y = 0.0
    occupancy_grid.info.origin.orientation.z = 0.0
    occupancy_grid.info.origin.orientation.w = 1.0
    occupancy_grid.data = gridmap.flatten().tolist()

    map_pub.publish(occupancy_grid)


def main():
    rospy.init_node('single_scan_to_map_node')

    rospy.Subscriber('/tilt_scan', LaserScan, laser_scan_callback)

    rospy.spin()

if __name__ == '__main__':
    main()