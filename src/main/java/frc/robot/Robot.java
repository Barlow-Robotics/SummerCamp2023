// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sim.PhysicsSim;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    private Field2d gameField;

    static long startTime = System.currentTimeMillis();

    static HashMap<Command, Long> startTimes = new HashMap();

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

        // DriverStation.silenceJoystickConnectionWarning(true) ;

        gameField = new Field2d();
        SmartDashboard.putData("Field", gameField);

        CommandScheduler.getInstance().onCommandInitialize(Robot::reportCommandStart);
        CommandScheduler.getInstance().onCommandFinish(Robot::reportCommandFinish);
        CommandScheduler.getInstance().onCommandInterrupt(this::handleInterrupted);
    }

    @Override
    public void robotPeriodic() {
        
        SmartDashboard.putData(CommandScheduler.getInstance());
        SmartDashboard.putData(robotContainer.driveSub);

        SmartDashboard.putData(robotContainer.driveSub);
        SmartDashboard.putData(robotContainer.visionSub);
        SmartDashboard.putData(robotContainer.shooterSub);
        // SmartDashboard.putData(robotContainer.indexSub);

        if (robotContainer.getCurrentTrajectory() != null) {
            gameField.getObject("traj").setTrajectory(robotContainer.getCurrentTrajectory());
        }
        gameField.setRobotPose(robotContainer.driveSub.getPose());

        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
            System.out.println("Auto Command is " + autonomousCommand);
        } else {
            System.out.println("Auto Command is null");
        }
    }

    // @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {
        robotContainer.driveSub.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        PhysicsSim.getInstance().run();
    }

    static public void reportCommandStart(Command c) {
        double deltaTime = ((double) System.currentTimeMillis() - startTime) / 1000.0;
        System.out.println(deltaTime + ": Started " + c.getName());
        startTimes.putIfAbsent(c, System.currentTimeMillis());
    }

    static public void reportCommandFinish(Command c) {
        if (startTimes.containsKey(c)) {
            long currentTime = System.currentTimeMillis();
            double deltaTime = ((double) currentTime - startTime) / 1000.0;
            double elapsedTime = (double) (currentTime - startTimes.get(c)) / 1000.0;
            System.out.println(deltaTime + ": Finished (elapsed time " + elapsedTime + ")" + c.getName());
            startTimes.remove(c);
        }
    }

    private void handleInterrupted(Command c) {
        System.out.println("Commmand " + c + " named " + c.getName() + " was interrupted");
    }
}
