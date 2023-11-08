#!/usr/bin/env python
import cv2
import mediapipe as mp
import rospy
import time
from std_msgs.msg import String
import serial
def send_gesture(gesture):
    gesture_msg = String()
    gesture_msg.data = gesture
    gesture_pub.publish(gesture_msg)
# Initialize ROS node
rospy.init_node('hand_gesture_detection')
ser = serial.Serial('/dev/ttyACM0', 9600)
# ROS publisher for sending gestures
gesture_pub = rospy.Publisher('gesture', String, queue_size=10)
def detect_gesture(hand_landmarks, hand_type):
    thumb_tip = hand_landmarks.landmark[mpHands.HandLandmark.THUMB_TIP]
    thumb_mcp = hand_landmarks.landmark[mpHands.HandLandmark.THUMB_MCP]
    index_finger_tip = hand_landmarks.landmark[mpHands.HandLandmark.INDEX_FINGER_TIP]
    index_finger_base = hand_landmarks.landmark[mpHands.HandLandmark.INDEX_FINGER_MCP]
    middle_finger_tip = hand_landmarks.landmark[mpHands.HandLandmark.MIDDLE_FINGER_TIP]
    middle_finger_base = hand_landmarks.landmark[mpHands.HandLandmark.MIDDLE_FINGER_MCP]
    ring_finger_tip = hand_landmarks.landmark[mpHands.HandLandmark.RING_FINGER_TIP]
    ring_finger_base = hand_landmarks.landmark[mpHands.HandLandmark.RING_FINGER_MCP]
    pinky_finger_tip = hand_landmarks.landmark[mpHands.HandLandmark.PINKY_TIP]
    pinky_finger_base = hand_landmarks.landmark[mpHands.HandLandmark.PINKY_MCP]
    pinky_finger_dip = hand_landmarks.landmark[mpHands.HandLandmark.PINKY_DIP]
     
    if index_finger_tip.y < index_finger_base.y and \
      middle_finger_tip.y > middle_finger_base.y and \
      ring_finger_tip.y > ring_finger_base.y and \
      pinky_finger_tip.y > pinky_finger_base.y:
      print("Forward")
      return "One"  

# Two fingers pointing up (Backward)
    elif index_finger_tip.y < index_finger_base.y and \
      middle_finger_tip.y < middle_finger_base.y and \
      ring_finger_tip.y > ring_finger_base.y and \
      pinky_finger_tip.y > pinky_finger_base.y:
     print("Backward")
     return "Two"
    

    # Three fingers pointing up
    elif index_finger_tip.y < index_finger_base.y and \
      middle_finger_tip.y < middle_finger_base.y and \
      ring_finger_tip.y < ring_finger_base.y and \
      pinky_finger_tip.y > pinky_finger_dip.y:
       
        print("Right")
        return "Three"

    # Four fingers pointing up
    elif index_finger_tip.y < index_finger_base.y and \
      middle_finger_tip.y < middle_finger_base.y and \
      ring_finger_tip.y < ring_finger_base.y and \
      pinky_finger_tip.y < pinky_finger_dip.y and \
        thumb_tip.x < thumb_mcp.x:
        print("Left")
        return "Four"

    # Five fingers pointing up
    elif index_finger_tip.y < index_finger_base.y and \
      middle_finger_tip.y < middle_finger_base.y and \
      ring_finger_tip.y < ring_finger_base.y and \
      pinky_finger_tip.y < pinky_finger_base.y and \
       thumb_tip.x > thumb_mcp.x:
        print("Five")
        return "Five"
    elif index_finger_tip.y < index_finger_base.y and \
      middle_finger_tip.y > middle_finger_base.y and \
      ring_finger_tip.y > ring_finger_base.y and \
      pinky_finger_tip.y < pinky_finger_base.y:
        print("Yo")
        return "Yo"
    elif index_finger_tip.y > index_finger_base.y and \
      middle_finger_tip.y < middle_finger_base.y and \
      ring_finger_tip.y > ring_finger_base.y and \
      pinky_finger_tip.y > pinky_finger_base.y:
        print("WTF")
        return "Noo"
    else:
        print("No gesture detected")
        return "0"

        
cap = cv2.VideoCapture(0)

mpHands = mp.solutions.hands
hands = mpHands.Hands(static_image_mode=False,
                      max_num_hands=2,
                      min_detection_confidence=0.5,
                      min_tracking_confidence=0.5)
mpDraw = mp.solutions.drawing_utils

pTime = 0
cTime = 0
        
while True:
    success, img = cap.read()
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)
    right_hand_detected = False
    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks:
          hand_type = "Right" if handLms.landmark[mpHands.HandLandmark.WRIST].x < 0.5 else "Left"
          if hand_type == "Right":
            gesture = detect_gesture(handLms, hand_type)
            send_gesture(gesture)
            right_hand_detected = True 

            for id, lm in enumerate(handLms.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                cv2.circle(img, (cx, cy), 3, (255, 0, 255), cv2.FILLED)

            mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)
            x_min, x_max, y_min, y_max = w, 0, h, 0
            for lm in handLms.landmark:
                cx, cy = int(lm.x * w), int(lm.y * h)
                if cx < x_min:
                    x_min = cx
                if cx > x_max:
                    x_max = cx
                if cy < y_min:
                    y_min = cy
                if cy > y_max:
                    y_max = cy

            box_color = (0, 255, 0) if hand_type == "Right" else (255, 0, 0)
            cv2.rectangle(img, (x_min - 10, y_min - 10), (x_max + 10, y_max + 10), box_color, 2)
            cv2.putText(img, hand_type, (x_min - 10, y_min - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)
            cv2.putText(img, gesture, (x_min - 10, y_min - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)
    else:
        send_gesture("0")
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

    cv2.imshow("Image", img)
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
