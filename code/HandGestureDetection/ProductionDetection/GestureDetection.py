import mediapipe
import cv2
from utils import findpostion, print_coords
import time


cameraCapture = cv2.VideoCapture(0)
tips = [8, 12, 16, 20]  # 20 landmarks total and these are all the tip of each finger
joints = [6, 10, 14, 18]
fingers_name = ["Index", "Middle", "Ring", "Pinky"]


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
            print(f"{fingers_name[i]} is {finger}")
        print(f"Total Fingers Up: {up}  Down: {down}")

    time.sleep(1)
