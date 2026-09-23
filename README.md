# Smart-Rock-Testing-Rig

## Design Overview:

This system will act as a testing harness for the Smart Rock, both before and after deployment, to test the functionality and overall accuracy of the electronics and sensors on board the Smart Rock.

This rig will test for power, communication, and high-level (non-granular) accuracy and effectiveness of each sensor by using an ESP-32 to control test code injection. The ESP-32 will monitor the health of the Smart Rock as it is tested through various stages.

🎯 The goal of this testing rig is to track down sensor defects, microcontroller defects, calibration errors, or any other kind of baseline hardware issue inside the Smart Rock before in-situ testing and more advanced calibration.

A very high-level diagram of the logic system is shown below.
![[Pasted image 20260819100611.png]]
