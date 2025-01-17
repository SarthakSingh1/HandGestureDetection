# Sarthak Singh

# 2024-02-06
Set up the github today to hold the computer vision code as well as the notebooks for all members of our project. 

# 2024-02-13 
Met with our TA to discuss our project proposal and steps to take next. Main concerns with the report were high level objectives, block diagram blue lines, and  having a more concrete structure for the substem description section. We inquired about if the camera part we need is already in the lab. We now have access to a rasberry pi3 and all the supporting hardware to install the OS on the Pi. 

# 2024-02-22 
Design doc was finished and submitted today. Main design decision made from my end was that we would send information from the PI to the PCB using GPIO pins with 3 pins representing 3 bits. Furthermore, I ordered the Pi camera which is the only external part I need

# 2024-02-26 
We gave our design presentation with main critiques being that we should make a decision between a pre tained model or our own for hand gesture detection. We found a package called mediapipe used by google which has a hand training model. The camera has arrived and I can begin testing

# 2024-02-31
I am dealing with a myriad of issues due to the low ram on the rasberry pi. Because of its limited compute, I cannot compile the binary of openCV - it takes well over 5 hours and then fails. I need to consider cross compiling or another alternative. I was able to test that the camera works as expected though. Using the command `raspiStill image.jpg` I can take a picture with the camera and it is fairly high quality. 

# 2024-03-06
We needed to order additional SMT development boards as they have run out. I was also unsucessful in being able to install openCV on the Pi and am considering purchasing a better Pi. I will continue to try to get this one to work until we have an estimate of the price of the other parts. I have also ordered a heat sink in hopes of improving the performance of the Pi

# 2024-03-18
The team and I discussed the posibility of upgrading the PI within our budget. Given we have only 60 dollars left and thats how much the pi would cost us, we are trying to find alternative solutions. I did some testing with a voltmeter to see if the Pi was getting enough currrent and why it was sturggle to compile the binary 

# 2024-03-19
Was able to install the binary by cross compiling on my mac and loading it onto the Pi. I was able to get a video feed going and run a simple openCV program. I can now start building the actual gesture detection. I have setup a folder in the github and am starting to push code. Also our audit passed today we were able to order our PCB


# 2024-03-20
We got out feedback for our design document and made the relevant changes to re submit it for more points. Furthermore, I was able get the X and Y coordinate of joints on the hand to print to the console. I devised an algorithm for figuring out if a finger is up or down. 

![See hand landmarks here](/notebooks/sarthak/HandLandmarks.png)

The top joint on each hand and 2 joints under are going to have a comparison done on them. Whichever one is closer to the top of the screen will indicate if a finger is up or down. 

# 2024-03-23
I formally was able use the X Y coordinates to see if a hand was up or down using the following code 

```
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
```

The coordinates are the distance from the top and left of the screen which is why we use a less than sign rather than a greater than sigh for the tip < joint


# 2024-03-25
I was able get an overlay on the screen to show the joins and detect if different joins are up or down. The thumb is buggy due to the difference in it and the other hands. Currently we are considering disregarding the thumb and using the other 4 fingers for our gesture. 

<img src="/notebooks/sarthak/HandGestureOverlay.png" alt="Hand Gesture Overlay" width="300">

```
This many fingers are up -  5
This many fingers are down -  0
 Number of landmarks 1

This many fingers are up -  2
This many fingers are down -  3
 Number of landmarks 1


This many fingers are up -  2
This many fingers are down -  3
This many fingers are up -  2
```

As you can see I can for the most part detect properly. I am a bit unclear on how to handle the case with two hands so we are limited the scope of the project to a single hand within the view. 

# 2024-04-01
Began getting to work on the 3d model for our enclosure. The idea is that there will be a top and a bottom with the bottom containing the speaker and the camera while the top has the LCD. Also decided our final gestures and gave them coresponding GPIO values. Next step on hand gesture detection is to output to gpio and test using a breadboard. 

```
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
```


# 2024-04-07
Got initial vision for the 3d models was completed. The camera was put on the top part as that made more sense for 3d printing. We will discuss as a group and make edits to it for our project. 

<img src="/notebooks/sarthak/OriginalBottomVision.png" alt="Bottom First Draft" width="300">
<img src="/notebooks/sarthak/OriginalTopVision.png" alt="Top First Draft" width="300">

# 2024-04-09
After discussion with the team, there were a few issues with the design. The first was that it would take too long to print and was too large to fit on the bed of the printer. Also the camera cable is not long enough to make it to the top of this box. Also an important realization is that we only need one side of the top so we make changed to the design. Also ordered fillament for 3d printing the rest of our materials 


<img src="/notebooks/sarthak/finalBottom.png" alt="Bottom Final draft" width="300">
<img src="/notebooks/sarthak/finalTop.png" alt="Upper Final draft" width="300">



# 2024-04-15
GPIO was completed and tested on a breadboard. We used LED's to show that the correct signals are showing up when the proper gesture is being done. See Below

<img src="/notebooks/sarthak/raspberryPi.jpg" alt="Pi Connection" width="300">
<img src="/notebooks/sarthak/breadboard.jpg" alt="Upper Final draft" width="300">

You can see there is also a pus button which terminates our code. That is just used for when power goes out, we want the program to not be running to prvent disk access when shutting down 

Furthermore, I made it so the code begins on startup instead of me needing to manually connect to a monitor and run the code. 

# 2024-04-18
The printed 3d model had a few issues after printing. The first were the distance between the camera holes were incorrect, and the soldered LCD within our breakout board is not flush with the backround. Also, the Pi and PCB did not fit into the mounting holes and those also need to be smaller. As of now we will work with the current 3d print and I will begin making small edits

# 2024-04-18
Small edits needed to made to the 3d model and they need to be printed. Main issue was the hols were too big so I needed to re design them and re print. I am currently waiting to 3d print the final enclosure. It should be done in 24 hours from now and then I will screw in all the components. 

# 2024-04-19
The parts are printed and now I need to screw the pieces on. THe holes for the camera were a bit small and I needed to use sandpaper to open up the hole a little bit. All the pieces ended up fitting. See below for the final print with parts screwed in. 

<img src="/notebooks/sarthak/finalEnclosure.jpeg" alt="Upper Final " width="300">

# 2024-04-20
My subsystems are used as inputs for the other subsystems however, it seems like our PCB has some issues since Sergio was not able to program our PCB. Therfore, to showcase that we can communicate externally, I need to build a circuit board connected to the gpio pins so worst case I can showcase the gesture system on a breadboard. 


# 2024-04-22 
We tried to program the PCB a few more times, but there is issue after issue. Even the dev board is causing us problems so we are assuming our breaboard will be needed to communicate the performance of the gesture control subsystem. 

# 2024-04-23
Over night, sergio was able fix the LCD code and so the UI subsystem works. I went into the lab to intergrate my gesture control subsystem with it. We were sucessfully able to translate a new gesture to a state change on the LCD. The breadboard is only needed for a push button to shut down the code, so we will try connecting the button directly to ground on the Pi to gain an extra 10 points. The code was being wierd when we did that so we are using a small circuit board for our demo 

# 2024-4-24
After finishing our demo, we fully finished the development of our prototype. Today we met up in the lab to record videos for our extra credit video. We got videos of the full aparatus and explanations on our overall project. I spend a few hours today in final cut pro stitching together the videos and realized we didnt have a video of the power subsystem. I will go back tomorrow to re record that. 

# 2024-4-25 
I got the final clips needed and finished up editing the video and submitted it. I also did some manual verification of the gesture constrol subsystem for our final report. I made 50 gestures and times how long it took to recognize that. I also wrote down how accurate the hand gesture was, I found on average it took 3 seconds while it was 98% accurate. 

# 2024-5-1
I just wrapped up writing all my sections in the final report and this is the last entry in my notebook. As a summary, I was responsible for the gesture control subsystem, the 3d modeling and printing of our enclosure, and making our extra credit video. I beleive I put in my fair share of work and all of my assigned tasks were completed sucessfully and on time. I learned alot from building this product that I will apply in the future. 