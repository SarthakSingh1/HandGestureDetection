import mediapipe
import cv2
from utils import findpostion, print_coords
import time


cameraCapture = cv2.VideoCapture(0)
tips = [8, 12, 16, 20] # 20 landmarks total and these are all the tip of each finger 
joints = [] 


while True:
    ret, frame = cameraCapture.read()  # Gets a frame
    frame1 = cv2.resize(frame, (640, 480))
    a = findpostion(frame1)
    print_coords(a)
    time.sleep(1)
