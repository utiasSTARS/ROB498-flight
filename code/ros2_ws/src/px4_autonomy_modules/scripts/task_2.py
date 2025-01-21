#!/usr/bin/env python3

from copy import deepcopy
import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.qos
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped


# Main communication node for ground control
class CommNode(Node):
    def __init__(self):
        node_name = 'rob498_drone_#'
        super().__init__(node_name)
        self.get_logger().info("\n\n ******Startig comms node!***** \n\n")
        #
        # Services to interact with ground control station
        self.srv_launch = self.create_service(Empty, "~/comm/launch", self.callback_launch)
        self.srv_test = self.create_service(Empty, "~/comm/test", self.  callback_test)
        self.srv_land = self.create_service(Empty, "~/comm/land", self.  callback_land)
        self.srv_aboard = self.create_service(Empty, "~/comm/aboard", self.  callback_abort)
        #
        # Your code goes here

    # Service callbacks
    def callback_launch(self, request:Empty.Request, response:Empty.Response):
        # Your code goes here
        return response

    def callback_test(self, request:Empty.Request, response:Empty.Response):
        # Your code goes here
        return response

    def callback_land(self, request:Empty.Request, response:Empty.Response):
        # Your code goes here
        return response

    def callback_abort(self, request:Empty.Request, response:Empty.Response):
        # Your code goes here
        return response


def main(args=None):
    try:
        rclpy.init()
        comm_node = CommNode()
        rclpy.spin(comm_node)
        comm_node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()