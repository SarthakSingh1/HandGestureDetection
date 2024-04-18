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
I am dealing with a myriad of issues due to the low ram on the rasberry pi. Because of its limited compute, I cannot compile the binary of openCV - it takes well over 5 hours and then fails. I need to consider cross compiling or another alternative. I was able to test that the camera works as expected though. 

# 2024-03-06
We needed to order additional SMT development boards as they have run out. I was also unsucessful in being able to install openCV on the Pi and am considering purchasing a better Pi. I will continue to try to get this one to work until we have an estimate of the price of the other parts. I have also ordered a heat sink in hopes of improving the performance of the Pi

# 2024-03-18
The team and I discussed the posibility of upgrading the PI within our budget. Given we have only 60 dollars left and thats how much the pi would cost us, we are trying to find alternative solutions. I did some testing with a voltmeter to see if the Pi was getting enough currrent and why it was sturggle to compile the binary 

# 2024-03-19
Was able to install the binary by cross compiling on my mac and loading it onto the Pi. I was able to get a video feed going and run a simple openCV program. I can now start building the actual gesture detection. I have setup a folder in the github and am starting to push code. Also our audit passed today we were able to order our PCB

# 2024-03-20
We got out feedback for our design document and made the relevant changes to re submit it for more points. Furthermore, I was able get the X and Y coordinate of joints on the hand to print to the console. I devised an algorithm for figuring out if a finger is up or down. 

![See hand landmarks here](/notebooks/sarthak/HandLandmarks.png)

The top joint on each hand and 2 joints under are going to have a comparison done on them. Whichever one is closer to the top of the screen will indicate if a hand is up or down. 

# 2024-03-25
I was able get an overlay on the screen to show the joins and detect if different joins are up or down. The thumb is buggy due to the difference in it and the other hands. Currently we are considering disregarding the thumb and using the other 4 fingers for our gesture. 

![See hand overlay here](/notebooks/sarthak/HandLandmarks.png)

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

# 2024-04-07
Got initial vision for the 3d models was completed. The camera was put on the top part as that made more sense for 3d printing. We will discuss as a group and make edits to it for our project. 

![Bottom First draft](/notebooks/sarthak/OriginalBottomVision.png)
![Upper First draft](/notebooks/sarthak/OriginalTopVision.png)


