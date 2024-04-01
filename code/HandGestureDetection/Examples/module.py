# This is a support Module mainly for the speech from text and also to aid the
# System into knowing when a finger is up or when a finger is down based on the
# joints

import cv2
import mediapipe as mp

import os

drawingModule = mp.solutions.drawing_utils
handsModule = mp.solutions.hands

# mod = handsModule.Hands()


h = 480
w = 640


def findpostion(frame1):
    list = []
    with handsModule.Hands(static_image_mode=False, min_detection_confidence=0.7, t=0.7, max_num_hands=2) as mod:
        results = mod.process(cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB))
        if results.multi_hand_landmarks != None:
            for handLandmarks in results.multi_hand_landmarks:
                drawingModule.draw_landmarks(
                    frame1, handLandmarks, handsModule.HAND_CONNECTIONS
                )
                list = []
                for id, pt in enumerate(handLandmarks.landmark):
                    x = int(pt.x * w)
                    y = int(pt.y * h)
                    list.append([id, x, y])

    return list


def findnameoflandmark(frame1):
    list = []
    results = mod.process(cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB))
    if results.multi_hand_landmarks != None:
        print(f" Number of landmarks {len(results.multi_hand_landmarks)}")
        for handLandmarks in results.multi_hand_landmarks:
            for point in handsModule.HandLandmark:
                pointName =   int(str(point))
                list.append(
                    str(point)
                    .replace("< ", "")
                    .replace("HandLandmark.", "")
                    .replace("_", " ")
                    .replace("[]", "")
                )
    return list
