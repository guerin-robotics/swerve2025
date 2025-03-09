package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;

import au.grapplerobotics.CanBridge;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    Timer intakeTimer = new Timer();
    static Timer liftTimer = new Timer();
    
    public Robot() {
        // Initialize robot components
        m_robotContainer = new RobotContainer();
        CameraServer.startAutomaticCapture();
        // elevator = new Elevator();

        CanBridge.runTCP();
    }

    @Override
    public void robotPeriodic() {
        // Runs the scheduler for commands
        CommandScheduler.getInstance().run();
    }


    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        // Start the selected autonomous command
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void disabledInit() {
    }
}
