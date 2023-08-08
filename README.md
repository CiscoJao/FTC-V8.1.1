
# Object Detector Bot for FTC

This repo holds code created by Francisco Jao during the summer of 2023. The code is designed for a mecanum drive robot with a camera and 3 odometry pods. The goal is to program the robot to autonomously recognize an object and have it move towards the object.


## Robot Components
These are the main subsystems used to successfully detect an object and move towards it.

* Mecanum Drive
* Odometry System
* Camera
* Sensor
* PID Controller

## Brief Overview

In order to detect the desired object, OpenCV was used to filter the images from the camera. In `ContourPipline.java` this is achieved by isolating a specific solid colour from the camera feed. The pipeline then records where the object is in the frame and is able to give that information to `PoleDetectionAuto.java`.