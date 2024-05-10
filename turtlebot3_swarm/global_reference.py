#! /usr/bin/env python3

from sensor_msgs.msg import LaserScan

import math
import numpy as np
# import pyquaternion

class globalReference:
    def __init__(self, disc_size=0.1):
        self.disc_size = disc_size
        self.path = None
        
    def plan(self, position:np.array, goal:np.array, scan:LaserScan):
        '''
        position: numpy.array - robot position [x, y] in world frame
        goal: numpy.array - goal position [x, y] in world frame
        scan: LaserScan - scan data
        '''
        dist_to_goal = np.abs(goal-position)
        if scan.range_max > dist_to_goal[0]:
            cols = int(scan.range_max*2/self.disc_size)
        else:
            cols = int(dist_to_goal[0]*2/self.disc_size)
        if scan.range_max > dist_to_goal[1]:
            rows = int(scan.range_max*2/self.disc_size)
        else:
            rows = int(dist_to_goal[1]*2/self.disc_size)
        sensing_map = self.scanDataToMapArray(scan, rows, cols)
        target = self.worldToMap(position, goal, rows, cols)

        # TO DO: Run JPS to find the path to goal
        # TO DO: Split path to get n point

    def worldToMap(self, position:np.array, goal:np.array, rows:int, cols:int):
        mx = int(goal[0] - position[0])/self.disc_size + cols/2
        my = int(goal[1] - position[1])/self.disc_size + rows/2
        return np.array([mx, my])

    def scanDataToMapArray(self, scan:LaserScan, rows:int, cols:int):
        num_pts = len(scan.ranges)
        xy_scan = np.zeros((num_pts,2))
        
        blank_image = np.zeros((rows,cols), dtype=np.uint8)
        for i in range(num_pts):
            # Check that distance is not longer than it should be
            if (scan.ranges[i] > scan.range_max) or (math.isnan(scan.ranges[i])):
                continue
            else:
                # Calculate angle of point and calculate X,Y position
                angle = scan.angle_min + float(i)*scan.angle_increment
                xy_scan[i][0] = float(scan.ranges[i]*math.cos(angle))
                xy_scan[i][1] = float(scan.ranges[i]*math.sin(angle))

        # Loop through all points plot in blank_image
        for i in range(num_pts):
            pt_x = xy_scan[i,0]
            pt_y = xy_scan[i,1]
            if (pt_x < scan.range_max) or (pt_x > -1 * (scan.range_max-self.disc_size)) or \
                (pt_y < scan.range_max) or (pt_y > -1 * (scan.range_max-self.disc_size)):
                pix_x = int(math.floor((pt_x + scan.range_max) / self.disc_size))
                pix_y = int(math.floor((scan.range_max - pt_y) / self.disc_size))
                if (pix_x > cols) or (pix_y > rows):
                    continue
                else:
                    blank_image[pix_y,pix_x] = 255
        return blank_image

# def quaternion_to_euler(q):
#     q = pyquaternion.Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
#     yaw, pitch, roll = q.yaw_pitch_roll
#     return [roll, pitch, yaw]