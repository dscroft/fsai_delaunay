#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import rclpy.time
import std_msgs.msg
import tf2_ros.buffer
import tf2_ros.transform_listener
import geometry_msgs.msg
import visualization_msgs.msg
import tf2_geometry_msgs # do not remove, needed by tf2
from tf2_geometry_msgs import do_transform_pose

import sys, math

class Pursuit(Node):
    def __init__(self):
        super().__init__('Pursuit')         

        self.declare_parameter('target_frame', 'base_link')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)
      
        self.__sub = self.create_subscription(
            geometry_msgs.msg.PoseArray,
            'centers', self.__centers_callback, 10 )
 
        self.__pub = self.create_publisher(
            std_msgs.msg.Float32, 'steer_angle', 10 )
                
        self.__vizPub = self.create_publisher(
            visualization_msgs.msg.MarkerArray, 'rviz', 10 )
        
        self.__timer = self.create_timer( 0.1, self.__timer_callback )

        self.__centers = None

    def __publish_viz( self, header, poses, ns, color, scale=0.1 ):
        self.get_logger().debug( "Publish visuals" )

        if self.__vizPub.get_subscription_count() == 0:
            self.get_logger().debug( f"No subscribers for {ns}" )
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
        msg.markers[1].type = visualization_msgs.msg.Marker.POINTS
        msg.markers[1].action = visualization_msgs.msg.Marker.ADD
        msg.markers[1].scale.x = msg.markers[1].scale.y = msg.markers[1].scale.z = scale
        msg.markers[1].color.r, msg.markers[1].color.g, msg.markers[1].color.b = color
        msg.markers[1].color.a = 1.0

        for pose in poses:
            msg.markers[1].points.append( pose.position )

        self.__vizPub.publish( msg )

    def __timer_callback(self):
        self.get_logger().debug( "Timer callback", throttle_duration_sec=1.0 )

        if self.__centers == None:
            self.get_logger().warn( "No path", throttle_duration_sec=1.0 )
            return
        
        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame, self.__centers.header.frame_id, rclpy.time.Time() )#self.__centers.header.stamp )
        except Exception as e:
            self.get_logger().warn( f"Failed to lookup transform: {e}" )
            return

        localCenters = [ do_transform_pose( p, trans ) for p in self.__centers.poses ]
        
        infront = [ center for center in localCenters if center.position.x > 0 ]
        nearby = [ center for center in infront if ( dist := math.sqrt(center.position.x**2 + center.position.y**2) ) <= 10 ]
        viable = [ center for center in nearby if math.sqrt(center.position.x**2 + center.position.y**2) > 3.0 ]
        
        # rviz visualization
        header = std_msgs.msg.Header()
        header.stamp = trans.header.stamp
        header.frame_id = self.target_frame
        self.__publish_viz( header, infront, f"{self.get_name()}/infront", [0.0,0.0,1.0], scale=0.15 )
        self.__publish_viz( header, nearby, f"{self.get_name()}/nearby", [1.0,1.0,0.0], scale=0.2 )
        self.__publish_viz( header, viable, f"{self.get_name()}/viable", [0.0,1.0,0.0], scale=0.25 )

        if len(viable) == 0:
            self.get_logger().warn( "No viable targets", throttle_duration_sec=1.0 )
            return
        
        target = sorted( viable, key=lambda x: math.sqrt(x.position.x**2 + x.position.y**2) )[0]     
        angle = math.atan2( target.position.y, target.position.x )

        msg = std_msgs.msg.Float32()
        msg.data = angle        
        self.__pub.publish( msg )

        # rviz visualization
        self.__publish_viz( header, [target], f"{self.get_name()}/target", [1.0,1.0,1.0], scale=0.3 )
        
    def __centers_callback(self, msg):
        self.get_logger().info( "Got path" )
        self.__centers = msg

def main(args=None):
    rclpy.init(args=args)

    node = Pursuit()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()