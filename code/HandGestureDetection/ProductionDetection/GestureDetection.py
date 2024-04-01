import mediapipe
import cv2
from utils import findpostion, print_coords
import time


cameraCapture = cv2.VideoCapture(0)
tips = [8, 12, 16, 20]  # 20 landmarks total and these are all the tip of each finger
joints = [6, 10, 14, 18]
fingers_name = ["Index", "Middle", "Ring", "Pinky"]

gestures = {
    "rock_roll": [1, 0, 0, 1],
    "peace": [1, 1, 0, 0],
    "index_up": [1, 0, 0, 0],
    "all_fingers_up": [1, 1, 1, 1],
    "pinky": [0, 0, 0, 1],
    "closed_hand": [0, 0, 0, 0],
}

gestures_id = {
    "rock_roll": 1,
    "peace": 2,
    "index_up": 3,
    "all_fingers_up": 4,
    "pinky": 5,
    "closed_hand": 0,
}


def match_gesture(fingers):
    for gesture, pattern in gestures.items():
        if fingers == pattern:
            return gesture, gestures_id[gesture]
    return "unknown", -1


curr_state = "Nothing"
gpio_value = 0

while True:
    ret, frame = cameraCapture.read()  # Gets a frame
    frame1 = cv2.resize(frame, (640, 480))
    a = findpostion(frame1)
    # print_coords(a)

    # if hand exists
    if len(a) > 0:
        fingers = []
        for id in range(0, 4):
            tip = a[tips[id]][2:]
            joint = a[joints[id]][2:]
            if tip < joint:
                fingers.append(1)
            else:
                fingers.append(0)

        up, down = 0, 0
        for i in range(4):
            value = fingers[i]
            if value:
                finger = "up"
                up += 1
            else:
                finger = "down"
                down += 1
        #     print(f"{fingers_name[i]} is {finger}")
        # print(f"Total Fingers Up: {up}  Down: {down}")

        gpio_temp, state = match_gesture(fingers)

        if gpio_temp == -1:
            continue
        else:
            gpio_value = gpio_temp
            curr_state = state
            print(f"New gesture detected {curr_state}")
            
    print(f"Current state: {curr_state} current gpio: {gpio_value}")

