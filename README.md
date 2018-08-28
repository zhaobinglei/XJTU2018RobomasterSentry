# XJTU2018RobomasterSentry
This project is the computer vision algorithm of Xi'an Jiaotong University using for sentry's automatic aiming in the 2018 Robomaster. This project finally send target's coordinate(x, y of image coordinate system) to PLC through serial port.

If you find any problem, you can contact me using WeChat. My WeChat ID: lesile_Perte912
<br>

### @Software Requirements: 


* OpenCV3.3.1

* Ubuntu14.01

* ROS1.0 Kinetic

* Cmake2.8.3 version above
<br>

### @How to setup?

1.build Kinetic initial condition

2.put all of the folder in src

3.catkin_make in the terminal of current path
<br>

### @How to run?

1.Open terminal and input command: &roscore

2.Open terminal of your created ros condition folder and input follow command

3.&source devel/setup.bash

4.&rosrun package-name executable-name

### @Change the parameter

1./sentry_4.2/opencv_testcam.cpp -lightness threshold 
 
 you can find two_threshval and two_number_threshval(global variable)

2./sentry_4.2/contours.h -color(select blue or red)
 
 you can change color in the last if-statement in the full_small_contours(..)-function 
