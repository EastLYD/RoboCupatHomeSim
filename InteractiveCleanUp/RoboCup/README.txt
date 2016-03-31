Interactve CleanUp

This task is a temtative task for RoboCup Japan Open 2016, which will be held on March 2016.
A real human (referee) logon to the avatar to demonstrate pointing gesture in order to instruct the robot which object should be the target of cleanup.

OculusRift and Kinect sensor are used for the avatar control.


cleanUpUser.cpp:  normal controller for the avatar
cleanUpUser2.cpp: another controller for the avatar, which uses spherical linear interpolation on quaternion

cleanUpRobot.cpp  controller for the robot

Original source is SIGVerse/sample/CleanUpHumanInteraction; modification for RoboCup should be progressed on this directory.
