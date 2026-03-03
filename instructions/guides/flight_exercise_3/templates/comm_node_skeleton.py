import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_srvs.srv import Trigger

STATE = 'Init'
WAYPOINTS = None
WAYPOINTS_RECEIVED = False

# Callback handlers
def handle_launch():
    global STATE
    STATE = 'Launch'
    print('Launch Requested.')

def handle_test():
    global STATE
    STATE = 'Test'
    print('Test Requested.')

def handle_land():
    global STATE
    STATE = 'Land'
    print('Land Requested.')

def handle_abort():
    global STATE
    STATE = 'Abort'
    print('Abort Requested.')

# Service callbacks
def callback_launch(request, response):
    handle_launch()
    return response

def callback_test(request, response):
    handle_test()
    return response

def callback_land(request, response):
    handle_land()
    return response

def callback_abort(request, response):
    handle_abort()
    return response

def callback_waypoints(msg):
    global WAYPOINTS_RECEIVED, WAYPOINTS
    if WAYPOINTS_RECEIVED:
        return
    print('Waypoints Received')
    WAYPOINTS_RECEIVED = True
    WAYPOINTS = np.empty((0,3))
    for pose in msg.poses:
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        WAYPOINTS = np.vstack((WAYPOINTS, pos))

class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_00')  # Change 00 to your team ID
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_00/comm/launch', callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_00/comm/test', callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_00/comm/land', callback_land)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_00/comm/abort', callback_abort)
        self.sub_waypoints = self.create_subscription(PoseArray, 'rob498_drone_00/comm/waypoints', callback_waypoints, 10)

def main(args=None):
    global STATE, WAYPOINTS, WAYPOINTS_RECEIVED

    rclpy.init(args=args)
    node = CommNode()
    print('This is a dummy drone node to test communication with the ground control')
    while rclpy.ok():
        rclpy.spin_once(node)
        if WAYPOINTS_RECEIVED:
            print('Waypoints:\n', WAYPOINTS)

        # Your code goes here
        if STATE == 'Launch':
            print('Comm node: Launching...')
        elif STATE == 'Test':
            print('Comm node: Testing...')
        elif STATE == 'Land':
            print('Comm node: Landing...')
        elif STATE == 'Abort':
            print('Comm node: Aborting...')

        node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.2))

    rclpy.shutdown()

if __name__ == "__main__":
    main()