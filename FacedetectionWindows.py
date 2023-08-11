import cv2
import mediapipe as mp
import time
import serial

def send_gesture(gesture):
    ser.write(gesture.encode()) 

def detect_gesture(hand_landmarks, hand_type):
    thumb_tip = hand_landmarks.landmark[mpHands.HandLandmark.THUMB_TIP]
    thumb_mcp = hand_landmarks.landmark[mpHands.HandLandmark.THUMB_MCP]
    index_finger_tip = hand_landmarks.landmark[mpHands.HandLandmark.INDEX_FINGER_TIP]
    middle_finger_tip = hand_landmarks.landmark[mpHands.HandLandmark.MIDDLE_FINGER_TIP]
    ring_finger_tip = hand_landmarks.landmark[mpHands.HandLandmark.RING_FINGER_TIP]
    pinky_finger_tip = hand_landmarks.landmark[mpHands.HandLandmark.PINKY_TIP]
    
    # One finger pointing up
    if index_finger_tip.y < middle_finger_tip.y and index_finger_tip.y < ring_finger_tip.y and index_finger_tip.y < pinky_finger_tip.y and index_finger_tip.y < thumb_tip.y:
        send_gesture("F")
        print("f")
        return "One"
    
    # Two fingers pointing up
    elif index_finger_tip.y < thumb_tip.y and middle_finger_tip.y < thumb_tip.y and thumb_tip.y < ring_finger_tip.y and thumb_tip.y < pinky_finger_tip.y:
        send_gesture("B")
        print("b")
        return "Two"

    # Three fingers pointing up
    elif index_finger_tip.y < thumb_tip.y and middle_finger_tip.y < thumb_tip.y and pinky_finger_tip.y > thumb_tip.y and thumb_tip.y > ring_finger_tip.y:
        send_gesture("R")
        print("r")
        return "Three"

    # Four fingers pointing up
    elif index_finger_tip.y < thumb_tip.y and middle_finger_tip.y < thumb_tip.y and ring_finger_tip.y < thumb_tip.y and pinky_finger_tip.y < thumb_tip.y and thumb_mcp.x > thumb_tip.x:
        if hand_type == "Right":
            send_gesture("B")
            print("B")
            return "Four"
        else:
            send_gesture("L")
            print("L")
            return "Five"

    # Five fingers pointing up
    elif index_finger_tip.y < thumb_tip.y and middle_finger_tip.y < thumb_tip.y and ring_finger_tip.y < thumb_tip.y and pinky_finger_tip.y < thumb_tip.y and thumb_mcp.x < thumb_tip.x:
        if hand_type == "Right":
            send_gesture("L")
            print("L")
            return "Five"
        else:
            send_gesture("B")
            print("B")
            return "Four"
    elif thumb_tip.y < middle_finger_tip.y and thumb_tip.y < ring_finger_tip.y and thumb_tip.y < pinky_finger_tip.y and thumb_tip.y < index_finger_tip.y:
        send_gesture("S")
        print("s")
        return "Fist"
    else:  
        send_gesture("S")
        print("s")
        return "No Gesture"# Your gesture detection code here

# Establish serial connection
ser = serial.Serial('COM5', 9600)  # Replace with the appropriate port name
ser.timeout = 1

cap = cv2.VideoCapture(0)

mpHands = mp.solutions.hands
hands = mpHands.Hands(static_image_mode=False,
                      max_num_hands=1,
                      min_detection_confidence=0.5,
                      min_tracking_confidence=0.5)
mpDraw = mp.solutions.drawing_utils

pTime = 0
cTime = 0

while True:
    success, img = cap.read()
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)

    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks:
            hand_type = "Right" if handLms.landmark[mpHands.HandLandmark.WRIST].x < 0.5 else "Left"
            gesture = detect_gesture(handLms, hand_type)
            send_gesture(gesture)

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

            # Display the detected gesture
            cv2.putText(img, gesture, (x_min - 10, y_min - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)
            # Rest of the code for displaying hand information
    
    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)

    cv2.imshow("Image", img)
    if cv2.waitKey(1) == ord('q'):
        break

ser.close()
cv2.destroyAllWindows()
