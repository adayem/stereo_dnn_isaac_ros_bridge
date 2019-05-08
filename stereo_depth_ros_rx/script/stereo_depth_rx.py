#!/usr/bin/env python

from __future__ import print_function

import sys
import threading
import numpy as np
import argparse
from matplotlib import pyplot as plt

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension


class StereoDnn:
    def __init__(self, min_depth=0, max_depth=10):
        self.lock = threading.Lock()
        
        self.dist_min = min_depth
        self.dist_max = max_depth
        self.im_shape = [1, 1]
        
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.plot = self.ax.imshow(
                np.zeros((1, 1)),
                vmin=self.dist_min,
                vmax=self.dist_max,
                #interpolation='nearest',
                cmap="jet_r")
        self.fig.colorbar(self.plot, ticks=np.linspace(self.dist_min, self.dist_max, 5))
        plt.show()

        ## Message handlers
        self.sub = rospy.Subscriber("/stereoDnn/depth", Float32MultiArray, self.depthCB, queue_size=1)

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            ## Draw image in main thread
            plt.draw()
            self.ax.set_aspect(float(self.im_shape[0])/self.im_shape[1])
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            rate.sleep()
        
    
    def depthCB(self, msg):
        if self.lock.locked():
            return

        with self.lock:
            self.im_shape = [x.size for x in msg.layout.dim]
            im = np.array(msg.data, dtype=np.float32)
            im = im.reshape(self.im_shape)
            im = np.clip(im, self.dist_min, self.dist_max)
            
            self.plot.set_data(im)
            

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Receive depth data from an Nvidia Isaac nodelet and display the data.')
    parser.add_argument('--min_depth', metavar='<float>', type=float, default=0.0,
                        help='Minimum distance to display in meters (default: 0.0)')
    parser.add_argument('--max_depth', metavar='<float>',type=float, default=10.0,
                        help='Maximum distance to display in meters (default: 10.0)')
    args = parser.parse_args()
    
    rospy.init_node("stereo_depth_rx")
    app = StereoDnn(min_depth=args.min_depth, max_depth=args.max_depth)
    rospy.spin()
