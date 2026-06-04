#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import geometry_msgs.msg
import nav_msgs.msg

import sensor_msgs_py.point_cloud2 as pc2

from scipy.spatial import Delaunay
import numpy as np
import sys, math

def distance_2d( a, b ):
    return math.sqrt( (a.position.x-b.position.x)**2 + (a.position.y-b.position.y)**2 )

class SimplePath(Node):
    def __init__(self):
        super().__init__('SimplePath')         

        self.__sub = self.create_subscription(
            geometry_msgs.msg.PoseArray,
            'centers', self.__centers_callback, 10 )
 
        self.__pub = self.create_publisher(
            nav_msgs.msg.Path, 'path', 10 )

    def __centers_callback(self, msg):
        self.get_logger().info( "Callback" )

        path = nav_msgs.msg.Path()

        centers = np.array( [ ( p.position.x, p.position.y ) for p in msg.poses ] )
        #print( centers, file=sys.stderr )

        for p in msg.poses:
            self.get_logger().info( f"{p.position.x}, {p.position.y}, {p.orientation.x}, {p.orientation.y}, {p.orientation.z}, {p.orientation.w}" )

        return 

        #cones = np.array( [ list(p) for p in pc2.read_points(msg, field_names=("x", "y", "rgba"), skip_nans=True) ] )

        tri = Delaunay( centers )
        #unassigned = list(range(1,len(centers)))
        path = { i: [] for i in range(len(centers)) }

        for simplex in tri.simplices:


            for i in range(3):
                path[simplex[i]].append( simplex[(i+1)%3] )


        print( tri, file=sys.stderr )

        self.__pub.publish( path )

def main(args=None):
    rclpy.init(args=args)

    node = SimplePath()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()