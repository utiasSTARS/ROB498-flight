import rospy
from std_srvs.srv import Empty, EmptyResponse

# Callback handlers
def handle_launch():
    print('Launch Requested. Your drone should take off.')

def handle_test():
    print('Test Requested. Your drone should perform the required tasks. Recording starts now.')

def handle_land():
    print('Land Requested. Your drone should land.')

def handle_abort():
    print('Abort Requested. Your drone should land immediately due to safety considerations')

# Service callbacks
def callback_launch(request):
    handle_launch()
    return EmptyResponse()

def callback_test(request):
    handle_test()
    return EmptyResponse()

def callback_land(request):
    handle_land()
    return EmptyResponse()

def callback_abort(request):
    handle_abort()
    return EmptyResponse()

# Main communication node for ground control
def comm_node():
    print('This is a dummy drone node to test communication with the ground control')
    print('node_name should be rob498_drone_TeamID. Service topics should follow the name convention below')
    print('The TAs will test these service calls prior to flight')
    print('Your own code should be integrated into this node')
    
    node_name = 'rob498_drone_XX'
    rospy.init_node(node_name) 
    srv_launch = rospy.Service(node_name + '/comm/launch', Empty, callback_launch)
    srv_test = rospy.Service(node_name + '/comm/test', Empty, callback_test)
    srv_land = rospy.Service(node_name + '/comm/land', Empty, callback_land)
    srv_abort = rospy.Service(node_name + '/comm/abort', Empty, callback_abort)

    # Your code goes below

    rospy.spin()

if __name__ == "__main__":
    comm_node()
