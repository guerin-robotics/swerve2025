# Code Write Ups

### 2025 June 12
Good morning, all! I have been looking over the code and have made some changes.
I would say that the biggest one is using a hash map. A hash map is a data structure that stores data as key-value pairs, allowing you to quickly retrieve a value using its key. It works by passing the key through a hash function, which converts it into a number (the hash), then uses that number to find the correct index in an internal array where the value is stored. This makes access time almost instant, much faster than searching through every item one by one. Hash maps are efficient, especially for large datasets, because they avoid unnecessary comparisons by jumping directly to where the data should be.
I also believe to have added the jumping around between poses. The robot now gets the distance to all tags and finds the closest one. Then, with that, it is able to drive to that, but if the stick is moved left or right by 20% within 5 seconds (just an arbitrary number), it will switch the tag either clockwise or counterclockwise to the next "node".
Also, in hopes of making vision even better, I added some logic that makes the slandered deviations ramp up steeper, so hopefully vision can still help over 2 m without too much glittering (edited) 


### 2025 July 3

Hello everyone. I have just committed a bunch of changes that have been made to the robot! I will say that I know this is long message but there are some big changes and things that I wanted to share with everyone so everyone knows what is going on.

The first big one is that I have the active intake working and programed. to make the active intake work better it moves up 0.5 rotations to make it flow in better. The rear wheels are running at 20% speed and the front ones are running at 40%. This makes the intake super fast. 

@RJ Martin also told me how other teams were using a lock and load system to make sure that the coral is in the proper place in the intake every time. After testing I realized that this is something that we probably want to do, because sometimes the coral will be detected later. The way that it works now is it intakes to the spot where it is detected and then runs it in 2 more rotations in so that if the elevator would be moved up it would not break things. Then it runs the outtake out till it can not see it then run it back in. To do this I made a couple of different commands to make this super seamless and built into the in button and the auto command. 

For the auto paths I added a 3 piece side auto and 2 one piece center autos. I still need to change where they start but the idea is that we actually only need right auto not red right and blue right. Path planner automatically flips the path based on the alliance station. I am going to move the bot further forward and then turn it more to the left. I also was looking into maybe breaking up the path to have the elevator go up before the bot arrives but I'm not sure that the bot can do that without flipping so I am not going to mess with that a whole lot.

I also saw that in our autos the outtake is what is mainly the slowest thing and so I added code that essentially just runs the intake out until it cant see it any more and then 3 more rotations so that should greatly increase the speed of things. 

The issues of the in and out commands locking up the robot is fixed with this code structure.
buttonPanel.button(Constants.buttonPanel.coral.In)
    .onTrue(
        sequence(
	        // 1) move the elevator up to position 1
	        new InstantCommand(() -> Elevator.toPosition(
	                        Constants.elevator.level.intake),
	                        m_elevator),
	        // 2) run intake until coral arrives, 1s timeout, or
	        // cancel button pressed
	        new ParallelRaceGroup(
	                        new StartEndCommand(
	                                        m_effector::startIntake,
	                                        m_effector::stopIntake,
	                                        m_effector),
	                        waitUntil(m_effector::isCoralDetected),
	                        waitSeconds(5.0),
	                        waitUntil(() -> buttonPanel.button(
	                                        Constants.buttonPanel.intake.cancel)
	                                        .getAsBoolean())),

	        new ParallelRaceGroup(
	                        new StartEndCommand(
	                                        m_effector::startLock,
	                                        m_effector::stopIntake,
	                                        m_effector),
	                        waitUntil(m_effector::isCoralNotDetected),
	                        waitSeconds(3.0),
	                        waitUntil(() -> buttonPanel.button(
	                                        Constants.buttonPanel.intake.cancel)
	                                        .getAsBoolean())),
	        // 3) bump the wheels 3 rotations (always runs after the
	        // intake group)
	        new InstantCommand(() -> m_effector.bumpRotations(
	                        Constants.intake.lockRotations),
	                        m_effector),
	        new InstantCommand(() -> Elevator.toPosition(
	                        Constants.elevator.level.L1),
	                        m_elevator)));
```
So this allows the intake to start running as a command and then it has multiple end conditions for the Race Group. The idea is instead of running for a fixed time in the code itself the command just knows that until the coral is detected or 3 seconds has passed or the cancel button is pressed. Then it runs the locking series. 

I also found that during matches when the heights would be messed up changing the height every time we tried to score was annoying and slow so I decided to add memory to it. When ever a command is executed to move the elevator is updates a variable with that level and then when the manual tick is used it will store the value with the new height allowing it to stay the same for each independent level for each match. 

One last small thing I made it so that the Hang will only go in and out in less than 7 seconds. This seems to be reliable to faster than the 11 seconds that full extension is.

Happy 4th!! 