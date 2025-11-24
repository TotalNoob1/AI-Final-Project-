#!/home/edward/final_ros_ws/finalenv/bin/python3

#IMPORT LIBRARIES----------------------------------------------------------------------------------
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String #recieves gesture commands
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint #commands joints

#NODE CLASS----------------------------------------------------------------------------------------
class A1_Subscriber(Node):
    def __init__(self):
        super().__init__('a1_subscriber')

        #SUBCRIBER UNITRE COMMAND__________________________________________________________________
        #listens for commands
        self.subscription = self.create_subscription(
            String,
            '/unitree/command',
            self.command_callback,
            10 #QoS history depth
        )

        #PUBLISHER_________________________________________________________________________________
        #publisher sends msgs to joints
        self.joint_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        #log info
        self.get_logger().info("A1 Command Subscriber Ready")

    #CALLBACK FUNC---------------------------------------------------------------------------------
    def command_callback(self, msg):
        #make string lowercase
        command = msg.data.lower()

        #create velocity message
        twist = Twist()

        #command mappings
        if command == "stop":
            # walk foward
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.x = 0.0
            twist.angular.x = 0.0
            self.get_logger().info("STOP - Fred stopped")

        elif command == "come":
            #turn left in place
            twist.linear.x = 0.3
            twist.angular.z = 0.0
            self.get_logger().info("COME - Fred is coming back")

        elif command == "get_back":
            #walk backwards
            twist.linear.x = -0.3
            twist.angular.z = 0.0
            self.get_logger().info("GET BACK - Fred is backing up")

        elif command == "spin_left":
            #turn left in place
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.get_logger().info("SPIN LEFT - Fred is spinning left")

        elif command == "spin_right":
            #turn right in place
            twist.linear.x = 0.0
            twist.angular.z = -0.5
            self.get_logger().info("SPIN RIGHT - Fred is spinning right")

        else:
            self.get_logger().warn(f"Unknown command: {command}")
            return

        #PUBLISH___________________________________________________________________________________
        self.joint_pub.publish(twist)
        self.get_logger().info(f"Command executed: {command}")

#MAIN FUNC-----------------------------------------------------------------------------------------
def main():
    rclpy.init() #initialize ros
    node = A1_Subscriber() #create node
    rclpy.spin(node) #keep node alive
    
    node.destroy_node() #cleanup
    rclpy.shutdown() #shutdown ros

if __name__ == "__main__":
    main()