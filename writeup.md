# Code Write Ups
### 2025 May 22 
I just pushed the code. We (@Alex McMinn, @Elizabeth Nies, and myself) this morning managed to fix the code. We learned 3 things about it.

* We also learned that you just give the drivetrain (CTRE) the vision measurement without a filter. We were implementing our own filter and then giving it to CTRE to filter it again.
* We were not calling the ```drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds),getEstimationStdDevs());``` in the periodic. Only once in the robot init
* We were using the wrong time measurement for the pose estimations. We were using ```Timer.getFPGATimestamp()``` and we needed to use ```Utils.fpgaToCurrentTime(est.timestampSeconds)```, which is a different unit

Now it is just time to tune this in a bit and then begin to think about how we want to tackle how to make auto-align as easy to update at a comp where maybe the tag is just a bit left or right from what we are used to. My thinking is that we can define the coordinates for the tags themselves (this is built in using ```AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)```) and then we just have a translation for each section of the reef. The challenge is that the reef is at some weird angles, so any ideas you guys have are great! Let me know if you have any questions or concerns



During lunch, I rewrote the code to make it easier to add new cameras. It has all of the same logic and actually runs faster. Now it's 3 lines of code, and a new camera is added to the program. This is assuming that we have the same coprocessor and the same standard deviations. But if we need to change it is not hard to add.

One of the things that I removed to really help with the speed of the processing is I removed the option to view the raw vision and filtered vision fields in Elastic. This should no longer be needed and greatly reduce the amount of logic and complexity needed. We can easily add this back later if needed.

The way that the program works at the moment, when you want it to go to a specific spot on the field, is you are able to just use the ```makeGoToPose(5.89, 4.026, 180)``` Then Path Planner creates an on-the-fly optimized path to reach the destination. This makes it super easy to change where it goes. The robot does not align with the April Tag, but aligns with the exact coordinates that you give it.



I just went ahead and added a bunch of new logic. This mainly surrounds the addition of automatically getting the nearest tag and then moving either left or right.

The idea is that the robot is always updating which April Tag is the closest. With that tag it is then able to get the exact coordinates of it. Then it uses ```tag.getRotation().plus(Rotation2d.fromDegrees(180))``` to change the coordinates so that it moves the offset to the left or right. On the edges with angles at 30º or 60º is will change, but the x and the y based on the orientation of the angle.

Then for the commands for the button I have ```makeGoToTag(vision.getLastSeenTagId(), TagSide.LEFT, 0.164338)``` This creates a command that should make it move directly to the position. I have not yet tested it as I do not have the robot, but I am planning to come in early and hopefully test it.

Tuesday will be the true test of how well it works. We also have to get on making mounts for the Trifty Cams so that we can mount them on top of the swerves. If anyone wants to start working on those let me know 


### 2025 May 24

EagleEye is a go! I have integrated the 2 Thrifty Bot Cameras into the code and have them working. I also went ahead and calibrated each camera with over 600 images, really locking them in.

The mount that they are on is at a 10º inward tilt with a 20º tilt backwards. After physically messing with the camera before finalizing the mount, I found that this seemed like a pretty good compromise between total POV and seeing the tag up close.

On the coding front, it was just a bit more challenging to add the multiple cameras, but nothing crazy. I had to add the translations for where the cameras are on the bot, and was able to make that nice and precise using the CAD. I took the mounts and put them into the practice bot CAD and got the translation of each camera.

I also realized on Friday that when it calls ```Pose2d tagPose = getTagPose2d(tagId)```, it returns the exact coordinates of the tag, and when the bot is told to go to the spot, it tries to put the center of the robot at that. This is an issue and so I added some more logic using Trig to make it offset even on the hexagonal sides. It also has left and right offsets for all sides as well.

I also tuned the vision standard deviations. So now it is much more confident in the vision poses. It's now so much better with multiple cameras and really isn't "glitching" around like it does when the vision is too confident. I also tested with other tags, and things seem to be working just as intended even with the other angles

We should now be good to go to test on the reef at Carmel on Tuesday!!

### 2025 May 25

Today I did not really do much, but I was not sure of what else needed to be done. I went ahead and added some nice things that I think are good things to add instead. One thing that I added was the exponential drive. I think that this could really help our driving, and if we do not like it then it is super easy to remove/turn off. I also inverted the stick drive so that the robot is automatically moving in the correct direction based on the field orientation. 

Another thing that I did was I added some code to allow the driver to override the auto-align command. I made it so that if the driver uses over 20% stick in any direction, it will cancel and give control back to the driver. 

I also tested the entire blue side of the reef, ensuring every side worked. There were a few issues, notably the issues on id 21 and 18 (the straight on ones), the front offset was about 0.15m off. So I added that and think I got everything dialed.

Also, when testing the bot, I found that one camera, especially, really liked to disconnect. I know that Thrifty makes the locking USB-C connectors, and I would say that those are super important because things are noticeably less accurate when one of them disconnects.

 I am currently just starting to look into how to add the start of autons automatically to the starting pose of the robot. If you guys can think of anything that I am forgetting to add or should look into, let me know.
