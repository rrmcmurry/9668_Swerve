# 9668_Swerve 

### Malfunctionz - West High School Robotics club
#### West High School, Knoxville, TN

## Description

This is an ongoing project under development during the off season.  We have two robot drivetrains.  We have a swerve drive and a tank drive. This is the swerve drive. 

This project is based on a template project for an FRC swerve drive train that uses REV MAXSwerve Modules. See [REVrobotics MAXSwerve Java Template](https://github.com/REVrobotics/MAXSwerve-Java-Template/) for more information.

## Current objectives:
- Learn how to detect AprilTags with a [parrallel processor](https://github.com/rrmcmurry/WestPi/) - DONE
- Get Swerve Drive to work with an XBox controller - DONE
- Get field oriented remote control driving to work - DONE
- Use the parrallel processor to control to the Swerve Drive in Autonomous mode - In Progress
    - Have the robot turn towards an AprilTag
    - Have the robot follow an AprilTag around like a puppy
    - Have the robot find a specific pose based on the pose of an AprilTag
- Find a method of tracking position
- Pass position information from Robot to Raspberry Pi
- Use flow fields with position info in the raspberry pi to drive to specific locations and orientations 
- Use AprilTags in known positions to error correct position information.
- Put lots of AprilTags in a game field in known positions and use flow fields to change objectives\
- Modify the flowfields based on detection of other robots and/or obstacles
- Pass off from flowfields to AprilTag pose for target orientation 
- Contemplate how to deal with "waves" assuming they use speed bumps in the flow field design
- Add some layer of complexity like have an arm do something while driving.. 
- Find and detect objects as targets with unknown or random locations

If we ever get this far... work on speed.

