# Author : Khantaphon Chaiyo & 

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#from std_msgs.msg import String

class Turtlebot3Controller(Node):

    def __init__(self):
        super().__init__('turtlebot3_controller')   #node name
        #TODO: on shutdown?
        self.cmdVelPublisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.scanSubscriber = self.create_subscription(LaserScan, 'scan', self.scanCallback, 1)
        self.batteryStateSubscriber = self.create_subscription(BatteryState, 'battery_state', self.batteryStateCallback, 1)
        self.odomSubscriber = self.create_subscription(Odometry, 'odom', self.odomCallback, 1)
        self.valueLaserRaw = {
            'range_min':0.0,
            'range_max':0.0,
            'ranges':[0.0]*360,
        }
        self.valueBatteryState = None
        self.valueOdometry = {
            'position':None,        #Datatype: geometry_msg/Point   (x,y,z)
            'orientation':None,     #Datatype: geometry_msg/Quaternion (x,y,z,w)
            'linearVelocity':None,  #Datatype: geometry_msg/Vector3 (x,y,z)
            'angularVelocity':None, #Datatype: geometry_msg/Vector3 (x,y,z)
        }

    def publishVelocityCommand(self, linearVelocity, angularVelocity):
        msg = Twist()
        msg.linear.x = linearVelocity
        msg.angular.z = angularVelocity
        self.cmdVelPublisher.publish(msg)
        #self.get_logger().info('Publishing cmd_vel: "%s", "%s"' % linearVelocity, angularVelocity)

    def scanCallback(self, msg):
        self.valueLaserRaw = {
            'range_min':msg.range_min,
            'range_max':msg.range_max,
            'ranges':list(msg.ranges),
        }

    def batteryStateCallback(self, msg):
        self.valueBatteryState = msg

    def odomCallback(self, msg):
        self.valueOdometry = {
            'position':msg.pose.pose.position,
            'orientation':msg.pose.pose.orientation,
            'linearVelocity':msg.twist.twist.linear,
            'angularVelocity':msg.twist.twist.angular,
        }
def main(args=None):
    rclpy.init(args=args)
    tb3ControllerNode = Turtlebot3Controller()
    ##Method 1: Setups the callbacks then block the main thread.
    #setup callbacks
    #rclpy.spin(tb3ControllerNode)

    ##Method 2: Sync the node's work with our looping code in main thread.
    try:
        while rclpy.ok():
            rclpy.spin_until_future_complete(tb3ControllerNode)
            #read sensors values
            print(tb3ControllerNode.valueBatteryState)
            print(tb3ControllerNode.valueLaserRaw)
            print(tb3ControllerNode.valueOdometry)
            #calculate command movement
            linearVelocity = 0.1 #m/s
            angularVelocity = 0.05 #rad/s
            tb3ControllerNode.publishVelocityCommand(linearVelocity,angularVelocity)
    except KeyboardInterrupt:
        tb3ControllerNode.publishVelocityCommand(0.0,0.0)

    tb3ControllerNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
