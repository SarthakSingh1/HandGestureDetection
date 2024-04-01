import mediapipe
import cv2
from utils import findpostion, print_coords
import RPi.GPIO as GPIO
import time

# GPIO Basic initialization
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# GPIO port numbers
bit_1 = 17
bit_2 = 27
bit_3 = 22

bit_1_value = 0
bit_2_value = 0
bit_3_value = 0

GPIO.setup(bit_1, GPIO.OUT)
GPIO.setup(bit_2, GPIO.OUT)
GPIO.setup(bit_3, GPIO.OUT)


def set_gpio(number):
    global bit_1_value, bit_2_value, bit_3_value
    if number > 7 or number < 0:
        print("Cannot represent")
        return
    else:
        binary_number = format(number, "03b")
        bit_1_value = int(binary_number[0])
        bit_2_value = int(binary_number[1])
        bit_3_value = int(binary_number[2])

        GPIO.output(bit_1, bit_1_value)
        GPIO.output(bit_2, bit_2_value)
        GPIO.output(bit_3, bit_3_value)


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

        if gpio_temp != -1:
            if gpio_temp != gpio_value:
                gpio_value = gpio_temp
                curr_state = state
                print(f"New gesture detected {curr_state}")
                set_gpio(gpio_value)

    print(f"Current state: {curr_state} current gpio: {gpio_value}")
