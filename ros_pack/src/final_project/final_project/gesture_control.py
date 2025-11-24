#!/home/edward/final_ros_ws/finalenv/bin/python3

# IMPORT LIBRARIES----------------------------------------------------------------------------------
import cv2 as cv
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# START MEDIAPIPE HAND TRACKING--------------------------------------------------------------------
mp_hands = mp.solutions.hands # accesses mediapipe hands model package

#create hand detector
hands = mp_hands.Hands(
    max_num_hands = 1, #only track 1 hand
    min_detection_confidence = 0.7, #must be 70% confident to detect
    min_tracking_confidence = 0.7 #tracking threshold
    )
mp_draw = mp.solutions.drawing_utils #draw hand skeleton on video

# ROS2 NODE FOR PUBLICATION, WEBCAM, AND AI--------------------------------------------------------
class GestureController(Node):
    def __init__(self):
        #constructor to run when node starts
        #set up publisher, webcam, and timer callback
        super().__init__("gesture_controller") # name inside ros2

        #publisher send string command to robot topic
        self.publisher = self.create_publisher(String, "/unitree/command", 10)

        #start webcam captures 0 for laptop camera
        self.cap = cv.VideoCapture(0)
        self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        if self.cap.isOpened():
            self.get_logger().info('Open')
        #timer so processing runs continously about 30 frames ps
        self.timer = self.create_timer(0.3, self.process_frame)

        self.get_logger().info("Getsure Controller Node Active")

    # FUNC FOR EXTENDED FINGERS ___________________________________________________________________
    def count_fings(self, hand_landmarks):
        #return number of extended fingers
        #compare y of fingertip vs wrist
        
        ##mediapipe fingertip indexes for 4 fings no thumb
        tip_id = [8, 12, 16, 20] #pointer, middle, ring, and pinky
        dip_id = [7, 11, 15, 19]
        wrist_y = hand_landmarks.landmark[0].y #wrist index = 0

        fingers_up = 0
        for x,tip in enumerate(tip_id):
            if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[dip_id[x]].y:
                fingers_up += 1 #if finger is higher it is extended
        self.get_logger().info(str(fingers_up))
        return fingers_up
    
    #FUNC FOR PALM TILT____________________________________________________________________________
    def hand_tilt_down(self, hand_landmarks):
        #if hand facing down lay down
        #see if pointer below wrist

        wrist_y = hand_landmarks.landmark[0].y
        pointer_tip_y = hand_landmarks.landmark[8].y
        self.get_logger().info("wrist_y")
        intStr = wrist_y
        self.get_logger().info(str(wrist_y))
        self.get_logger().info("pointer_tip_y")
        self.get_logger().info(str(pointer_tip_y))
        return pointer_tip_y > wrist_y #true for palm down
    
    #LOOP FOR WEBCAM READ, HAND DETECTION, AND PUBLISHING ROS______________________________________
    def process_frame(self):
        #run about 0.03 seconds
        #read webcam frame, then mediapipe, then interpret gesture, then publish ros

        ret, frame = self.cap.read() #read webcam frame
        if not ret:
            self.get_logger().warn('Not returning images')
            return # no frame, skip
        
        #convert fromat opencv to mediapipe
        frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

        #do landmark on frame
        results = hands.process(frame_rgb)
        # print(results)
        # cv.imshow("Ros2 Gesture Control View", frame) # This is just here so that you know it is working. 
        # cv.waitKey(20) # remove it these two line 
        #if hand detected
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                #count ext fings
                fingers_up = self.count_fings(hand_landmarks)

                #check palm
                tilted_down = self.hand_tilt_down(hand_landmarks)

                #msg to publish
                msg = String()

                #GESTURE RULES=====================================================================
                if fingers_up == 0: #fist
                    msg.data = "sit" 
                    self.get_logger().info("sit")
                elif fingers_up == 4 and not tilted_down: #hand upright
                    msg.data = "stand"
                    self.get_logger().info("stand")
                elif fingers_up == 1 and tilted_down: #hand down
                    msg.data = "lay_down"
                    self.get_logger().info("lay_down")
                elif fingers_up == 5: #hand spread
                    msg.data = "stop"
                    self.get_logger().info("stop")
                else: #did not recognize
                    msg.data = ""
                    self.get_logger().info("I ain't got a clue")
                #publish command
                if msg.data != "":
                    self.publisher.publish(msg)
                    self.get_logger().info(f"COMMAND SENT: {msg.data}")

                #draw hand for debug
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            #display webcam
            cv.imshow("Ros2 Gesture Control View", frame) 
            #press x to exit
            test = cv.waitKey(1)
            # if  test == ord("x"):
            #     self.cap.release()
            # cv.destroyAllWindows()
            # rclpy.shutdown()

#ROS NODE ENTRY------------------------------------------------------------------------------------
def main():
    rclpy.init() #start ros
    node = GestureController() #create gesture node
    rclpy.spin(node) #node stay active
    node.destroy_node() #cleanup
    rclpy.shutdown() #turn off ros

if __name__ == "__main__":
    main()