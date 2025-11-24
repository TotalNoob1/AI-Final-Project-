#!/home/edward/final_ros_ws/finalenv/bin/python3

#IMPORT LIBRARIES----------------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from std_msgs.msg import String #recieves gesture commands
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint #commands joints

#JOINT NAMES---------------------------------------------------------------------------------------
JOINT_NAMES = [
    "FRHJ", "FRTJ", "FRCJ", #front right hip, thigh, and calf joint
    "FLHJ", "FLTJ", "FLCJ", #front left hip, thigh, and calf joint
    "RRHJ", "RRTJ", "RRCJ", #rear right hip, thigh, and calf joint
    "RLHP", "RLTJ", "RLCJ" #rear left hip, thigh, and calf joint
]

#POSES---------------------------------------------------------------------------------------------
#may need to adjust depending on how it looks under simulation
POSES = {
    "stand": [0.0, 0.7, -1.4, 0.0, 0.7, -1.4, 0.0, 0.7, -1.4, 0.0, 0.7, -1.4], 
    "sit": [0.0, 0.4, -0.6, 0.0, 0.4, -0.6, 0.0, 0.6, -1.0, 0.0, 0.6, -1.0],
    "lay_down": [0.0, 0.1, -0.1, 0.0, 0.1, -0.1, 0.0, 0.1, -0.1, 0.0, 0.1, -0.1],
}

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
            JointTrajectory,
            'test',
            10
        )

        #log info
        self.get_logger().info("A1 Command Subscriber Ready")

    #CALLBACK FUNC---------------------------------------------------------------------------------
    def command_callback(self, msg):
        #make string lowercase
        command = msg.data.lower()

        #stop command to stop in position
        if command == "stop":
            self.get_logger().info("STOP command received, holding current position")
            return #exit w/o pub
        
        #check for known commands
        if command not in POSES:
            self.get_logger().warn(f"Unknown command: {command}")
            return #ignore
        
        #JOINT TRAJECTORY MSG______________________________________________________________________
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        #create single point
        point = JointTrajectoryPoint()
        point.positions = POSES[command] #assign pos for pose
        point.time_from_start.sec = 1 #move over 1 sec
        traj.points.append(point) #add point to traj

        #PUBLISH___________________________________________________________________________________
        self.joint_pub.publish(traj)
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