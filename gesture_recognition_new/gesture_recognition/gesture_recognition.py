import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
import time

class GestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('gesture_recognition_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.main_loop)
        self.cap = cv2.VideoCapture(0)
        self.drawing = mp.solutions.drawing_utils
        self.hands = mp.solutions.hands
        self.hand_obj = self.hands.Hands(max_num_hands=1)
        self.prev = -1
        self.start_init = False
        self.start_time = time.time()

    def count_fingers(self, lst):
        cnt = 0
        # Check if the index finger is raised
        if lst.landmark[self.hands.HandLandmark.INDEX_FINGER_TIP].y < lst.landmark[self.hands.HandLandmark.INDEX_FINGER_MCP].y:
            cnt += 1
        # Check if the middle finger is raised
        if lst.landmark[self.hands.HandLandmark.MIDDLE_FINGER_TIP].y < lst.landmark[self.hands.HandLandmark.MIDDLE_FINGER_MCP].y:
            cnt += 1
        # Check if the ring finger is raised
        if lst.landmark[self.hands.HandLandmark.RING_FINGER_TIP].y < lst.landmark[self.hands.HandLandmark.RING_FINGER_MCP].y:
            cnt += 1
        # Check if the pinky finger is raised
        if lst.landmark[self.hands.HandLandmark.PINKY_TIP].y < lst.landmark[self.hands.HandLandmark.PINKY_MCP].y:
            cnt += 1

        return cnt

    def main_loop(self):
        _, frm = self.cap.read()
        frm = cv2.flip(frm, 1)
        res = self.hand_obj.process(cv2.cvtColor(frm, cv2.COLOR_BGR2RGB))

        if res.multi_hand_landmarks:
            hand_keyPoints = res.multi_hand_landmarks[0]
            cnt = self.count_fingers(hand_keyPoints)
            self.get_logger().info(f"Finger count: {cnt}")  # Log finger count

            if self.prev != cnt:
                if not self.start_init:
                    self.start_time = time.time()
                    self.start_init = True
                elif (time.time() - self.start_time) > 0.5:  # Increase delay to 0.5 seconds
                    direction = self.update_turtlebot(cnt)
                    self.prev = cnt
                    self.start_init = False
                    
                    # Display the direction on the camera feed
                    cv2.putText(frm, direction, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            self.drawing.draw_landmarks(frm, hand_keyPoints, self.hands.HAND_CONNECTIONS)

        cv2.imshow('Gesture Recognition', frm)
        cv2.waitKey(1)

    def update_turtlebot(self, count):
        twist = Twist()

        if count == 1:
            twist.linear.x = 0.2  # Move forward
            self.get_logger().info("Moving forward")
            direction = "Forward"
        elif count == 2:
            twist.linear.x = -0.2  # Move backward
            self.get_logger().info("Moving backward")
            direction = "Backward"
        elif count == 3:
            twist.angular.z = 0.5  # Turn left with higher angular velocity
            self.get_logger().info("Turning left")
            direction = "Left"
        elif count == 4:
            twist.angular.z = -0.5  # Turn right with higher angular velocity
            self.get_logger().info("Turning right")
            direction = "Right"
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Stopping")
            direction = "Stop"

        self.publisher_.publish(twist)
        return direction

def main(args=None):
    rclpy.init(args=args)
    gesture_recognition_node = GestureRecognitionNode()
    rclpy.spin(gesture_recognition_node)

    gesture_recognition_node.cap.release()
    cv2.destroyAllWindows()
    gesture_recognition_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
