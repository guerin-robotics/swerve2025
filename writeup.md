### Code Write Ups
# 2025 May 22 
I just pushed the code. We (
@Alex McMinn, @Elizabeth Nies, and myself) this morning managed to fix the code. We learned 3 things about it.
We also learned that you just give the drivetrain (CTRE) the vision measurement without a filter. We were implementing our own filter and then giving it to CTRE to filter it again.
We were not calling the drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds),getEstimationStdDevs()); in the periodic. Only once in the robot init
We were using the wrong time measurement for the pose estimations. We were using Timer.getFPGATimestamp() and we needed to use Utils.fpgaToCurrentTime(est.timestampSeconds), which is a different unit
Now it is just time to tune this in a bit and then begin to think about how we want to tackle how to make auto-align as easy to update at a comp where maybe the tag is just a bit left or right from what we are used to. My thinking is that we can define the coordinates for the tags themselves (this is built in using AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)) and then we just have a translation for each section of the reef. The challenge is that the reef is at some weird angles, so any ideas you guys have are great! Let me know if you have any questions or concerns