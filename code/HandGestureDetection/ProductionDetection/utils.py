# This is a support Module mainly for the speech from text and also to aid the
# System into knowing when a finger is up or when a finger is down based on the
# joints

import cv2
import mediapipe as mp
import os

drawingModule = mp.solutions.drawing_utils
handsModule = mp.solutions.hands

mod = handsModule.Hands()

LandmarkNames = [
    "Wrist",
    "Thumb_CMC",
    "Thumb_MCP",
    "Thumb_IP",
    "Thumb_TIP",
    "Index_Finger_MCP",
    "Index_Finger_PIP",
    "Index_Finger_DIP",
    "Index_Finger_TIP",
    "Middle_Finger_MCP",
    "Middle_Finger_PIP",
    "Middle_Finger_DIP",
    "Middle_Finger_TIP",
    "Ringer_Finger_MCP",
    "Ringer_Finger_PIP",
    "Ringer_Finger_DIP",
    "Ringer_Finger_TIP",
    "Pinky_MCP",
    "Pinky_MCP",
    "Pinky_MCP",
    "Pinky_MCP",
]

h = 480
w = 640

# Credit https://core-electronics.com.au/guides/hand-identification-raspberry-pi/
def findpostion(frame1):
    list = []
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


def print_coords(landmarks):
    for landmark in landmarks:
        id, x, y = landmark
        print(f"{LandmarkNames[id]} x: {x} y: {y}")
