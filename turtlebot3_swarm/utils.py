#! /usr/bin/env python

from sensor_msgs.msg import LaserScan

import math
import numpy as np
import pyquaternion

def scan_data_to_array(scan:LaserScan, disc_size = .1):
    image_size = int(scan.range_max*2/disc_size)

    minAngle = scan.angle_min
    angleInc = scan.angle_increment
    
    num_pts = len(scan.ranges)
    xy_scan = np.zeros((num_pts,2))
    
    blank_image = np.zeros((image_size,image_size), dtype=np.uint8)
    for i in range(num_pts):
        # Check that distance is not longer than it should be
        if (scan.ranges[i] > scan.range_max) or (math.isnan(scan.ranges[i])):
            continue
        else:
            # Calculate angle of point and calculate X,Y position
            angle = minAngle + float(i)*angleInc
            xy_scan[i][0] = float(scan.ranges[i]*math.cos(angle))
            xy_scan[i][1] = float(scan.ranges[i]*math.sin(angle))

    # Loop through all points plot in blank_image
    for i in range(num_pts):
        pt_x = xy_scan[i,0]
        pt_y = xy_scan[i,1]
        if (pt_x < scan.range_max) or (pt_x > -1 * (scan.range_max-disc_size)) or \
            (pt_y < scan.range_max) or (pt_y > -1 * (scan.range_max-disc_size)):
            pix_x = int(math.floor((pt_x + scan.range_max) / disc_size))
            pix_y = int(math.floor((scan.range_max - pt_y) / disc_size))
            if (pix_x > image_size) or (pix_y > image_size):
                continue
            else:
                blank_image[pix_y,pix_x] = 255
    return blank_image

def quaternion_to_euler(q):
    q = pyquaternion.Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
    yaw, pitch, roll = q.yaw_pitch_roll
    return [roll, pitch, yaw]