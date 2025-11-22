#!/usr/bin/env python3

# IMPORT LIBRARIES----------------------------------------------------------------------------------
import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# START MEDIAPIPE HAND TRACKING--------------------------------------------------------------------
mp_hands = mp.solutions.hands # accesses mediapipe hands model package

#create hand detector
hands = mp_hands.Hands(
    max_hands = 1, #only track 1 hand
    min_det_conf = 0.7, #must be 70% confident to detect
    min_trac_conf = 0.7 #tracking threshold
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
        self.cap = cv2.VideoCapture(0)

        #timer so processing runs continously about 30 frames ps
        self.timer = self.create_timer(0.03, self.process_frame)
        self.get_logger().info("Getsure Controller Node Active")

    # FUNC FOR EXTENDED FINGERS ___________________________________________________________________
    def count_fings(self, hand_landmarks):
        #return number of extended fingers
        #compare y of fingertip vs wrist
        
        ##mediapipe fingertip indexes for 4 fings no thumb
        tip_id = [8, 12, 16,20] #pointer, middle, ring, and pinky
        wrist_y = hand_landmarks.landmark[0].y #wrist index = 0

        fingers_up = 0
        for tip in tip_id:
            if hand_landmarks.landmark[tip].y < wrist_y:
                fingers_up += 1 #if finger is higher it is extended

        return fingers_up
    
    #FUNC FOR PALM TILT____________________________________________________________________________
    def hand_tilt_down(self, hand_landmarks):
        #if hand facing down lay down
        #see if pointer below wrist

        wrist_y = hand_landmarks.landmark[0].y
        pointer_tip_y = hand_landmarks.landmark[8].y

        return pointer_tip_y > wrist_y #true for palm down
    
    #LOOP FOR WEBCAM READ, HAND DETECTION, AND PUBLISHING ROS______________________________________
    def process_frame(self):
        #run about 0.03 seconds
        #read webcam frame, then mediapipe, then interpret gesture, then publish ros

        ret, frame = self.cap.read() #read webcam frame
        if not ret:
            return # no frame, skip
        
        #convert fromat opencv to mediapipe
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        #do landmark on frame
        results = hands.process(frame_rgb)

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

                elif fingers_up == 4 and not tilted_down: #hand upright
                    msg.data = "stand"
                
                elif fingers_up == 4 and tilted_down: #hand down
                    msg.data = "lay_down"

                elif fingers_up == 5: #hand spread
                    msg.data = "stop"

                else: #did not recognize
                    msg.data = ""
                
                #publish command
                if msg.data != "":
                    self.publisher.publish(msg)
                    self.get_logger().info(f"COMMAND SENT: {msg.data}")

                #draw hand for debug
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            #display webcam
            cv2.imshow("Ros2 Gesture Control View", frame)

            #press x to exit
            if cv2.waitKey(1) & 0xFF == ord("x"):
                self.cap.release()
                cv2.destroAllWindows()
                rclpy.shutdown()

#ROS NODE ENTRY------------------------------------------------------------------------------------
def main():
    rclpy.init() #start ros
    node = GestureController() #create gesture node
    rclpy.spin(node) #node stay active
    node.destroy_node() #cleanup
    rclpy.shutdown() #turn off ros

if __name__ == "__main__":
    main()