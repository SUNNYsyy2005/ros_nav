#!/usr/bin/env python3

'''
xvslam.py : BreezySLAM Python with GetSurreal XV Lidar
                 
Copyright (C) 2016 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10
LIDAR_DEVICE            = '/dev/ttyACM0'

from breezyslam.algorithms import RMHC_SLAM

from breezyslam.sensors import Lidar1 as LaserModel
ii = -1
#from roboviz import MapVisualizer
class Lidar():
        def __init__(self, device):
            self.device = device
        def getScan(self):
            # 这里虚拟了10个扫描点作为示例
            scans = [
                # 第1组：小车刚开始移动，周围障碍物较远
                [
                    (2.0, 0), (2.5, 30), (3.0, 60), (3.5, 90),
                    (4.0, 120), (3.5, 150), (3.0, 180), (2.5, 210),
                    (2.0, 240), (1.5, 270)
                ],
                # 第2组：小车向左转，左侧障碍物变近
                [
                    (2.5, 0), (3.0, 30), (3.5, 60), (2.0, 90),
                    (2.5, 120), (3.0, 150), (3.5, 180), (4.0, 210),
                    (4.5, 240), (5.0, 270)
                ],
                # 第3组：小车继续前进，前方障碍物变近
                [
                    (1.5, 0), (2.0, 30), (2.5, 60), (3.0, 90),
                    (3.5, 120), (3.0, 150), (2.5, 180), (2.0, 210),
                    (1.5, 240), (1.0, 270)
                ],
                # 第4组：小车向右转，右侧障碍物变近
                [
                    (3.0, 0), (3.5, 30), (4.0, 60), (4.5, 90),
                    (5.0, 120), (4.5, 150), (4.0, 180), (3.5, 210),
                    (3.0, 240), (2.5, 270)
                ],
                # 第5组：小车继续前进，障碍物均匀分布
                [
                    (2.0, 0), (2.5, 30), (3.0, 60), (3.5, 90),
                    (4.0, 120), (3.5, 150), (3.0, 180), (2.5, 210),
                    (2.0, 240), (1.5, 270)
                ]
            ]
            global ii
            ii+=1
            #print(ii,scans[ii%5])
            return scans[ii%5]
if __name__ == '__main__':

    # Connect to Lidar unit
    lidar = Lidar(LIDAR_DEVICE)

    # Create an RMHC SLAM object with a laser model and optional robot model
    slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # Set up a SLAM display
    #viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')

    # Initialize an empty trajectory
    trajectory = []

    # Initialize empty map
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    iii = 0
    while True:

        # Update SLAM with current Lidar scan, using first element of (scan, quality) pairs
        slam.update([pair[0] for pair in lidar.getScan()])

        # Get current robot position
        x, y, theta = slam.getpos()

        # Get current map bytes as grayscale
        slam.getmap(mapbytes)
        iii+=1
        if iii%1000==0:
            print(x,y,theta)
        # Display map and robot pose, exiting gracefully if user closes it
        #if not viz.display(x/1000., y/1000., theta, mapbytes):
        #    exit(0)
