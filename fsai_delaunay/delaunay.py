#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import sensor_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg

import sensor_msgs_py.point_cloud2 as pc2

from scipy.spatial import Delaunay
import numpy as np
import sys

# color values are the raw values from the rgba field of the point cloud
BLUE   = 4278190335
YELLOW = 4294967040
ORANGE = 4294934272

def get_edges( tri ):
    less_first = lambda a, b: [a,b] if a < b else [b,a]

    edges = []
    for triangle in tri.simplices:
        for e1, e2 in [[0,1],[1,2],[2,0]]: # for all edges of triangle
            edges.append(less_first(triangle[e1],triangle[e2])) # always lesser index first
    return np.unique( edges, axis=0 )

def to_coords( cones, edges ):
    """Convert edge indices to coordinates
    
    Returns:
        generator of ( (x1, y1, color1), (x2, y2, color2) ) tuples"""

    for a, b in edges:
        yield cones[a], cones[b]

def get_gates( coords ):
    for a, b in coords:
        if a[2] != b[2] and a[2] in ( BLUE, YELLOW ) and b[2] in ( BLUE, YELLOW ):
            yield a, b

def to_point( coord ) -> geometry_msgs.msg.Point:
    p = geometry_msgs.msg.Point()
    p.x, p.y, p.z = coord[0], coord[1], 0.0
    return p

class DelaunayNode(Node):
    def __init__(self):
        super().__init__('DelaunayNode')         

        self.__sub = self.create_subscription(
            sensor_msgs.msg.PointCloud2,
            'cones', self.__cones_callback, 10 )
        
        self.__vizPub = self.create_publisher(
            visualization_msgs.msg.MarkerArray, 'rviz', 10 )
        
        self.__pub = self.create_publisher(
            geometry_msgs.msg.PoseArray, 'gates', 10 )

    def __cones_callback(self, msg):
        self.get_logger().info( "Callback" )

        cones = np.array( [ list(p) for p in pc2.read_points(msg, field_names=("x", "y", "rgba"), skip_nans=True) ] )

        tri = Delaunay( cones[:,:2] )
        
        # extract unique edges from the simplices
        edges = get_edges( tri )

        # convert edge indices to coordinates, i.e. (x, y, color) pairs
        coords = list( to_coords( cones, edges ) )
        
        # extract gates from the coords        
        gates = list( get_gates( coords ) )

        self.__publish_gates( msg.header, gates )

        self.__publish_viz( msg.header, coords, "edges", [1.0,0.0,0.0] )
        self.__publish_viz( msg.header, gates, "gates", [0.0,1.0,0.0] )

    def __publish_gates( self, header, gates ):
        self.get_logger().debug( "Publish gates" )

        if self.__pub.get_subscription_count() == 0:
            self.get.logger().debug( "No subscribers" )
            return

        msg = geometry_msgs.msg.PoseArray()

        msg.header = header

        for a, b in gates:
            # gates should be ordered BLUE, YELLOW
            if a[2] == YELLOW and b[2] == BLUE:
                a, b = b, a

            pose = geometry_msgs.msg.Pose()
            pose.position.x = (a[0]+b[0])/2
            pose.position.y = (a[1]+b[1])/2
            pose.position.z = 0.0

            # calculate angle
            angle = np.arctan2( b[1]-a[1], b[0]-a[0] )

            # rotate 90 degrees to direction of travel
            angle += np.pi/2

            # convert to quaternion
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = np.sin( angle/2 )
            pose.orientation.w = np.cos( angle/2 )


            msg.poses.append( pose )

        self.__pub.publish( msg )

    def __publish_viz( self, header, coords, ns, color ):
        self.get_logger().debug( "Publish visuals" )

        if self.__vizPub.get_subscription_count() == 0:
            self.get.logger().debug( "No subscribers" )
            return

        msg = visualization_msgs.msg.MarkerArray()

        msg.markers = [
            visualization_msgs.msg.Marker(), visualization_msgs.msg.Marker()
        ]

        msg.markers[0].header = header
        msg.markers[0].ns = ns
        msg.markers[0].id = 0
        msg.markers[0].type = visualization_msgs.msg.Marker.DELETEALL

        msg.markers[1].header = header
        msg.markers[1].ns = ns
        msg.markers[1].id = 1
        msg.markers[1].type = visualization_msgs.msg.Marker.LINE_LIST
        msg.markers[1].action = visualization_msgs.msg.Marker.ADD
        msg.markers[1].scale.x = 0.1
        msg.markers[1].color.r, msg.markers[1].color.g, msg.markers[1].color.b = color
        msg.markers[1].color.a = 1.0

        for a, b in coords:
            msg.markers[1].points.append( to_point( a ) )
            msg.markers[1].points.append( to_point( b ) )

        self.__vizPub.publish( msg )

def main(args=None):
    rclpy.init(args=args)

    node = DelaunayNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()