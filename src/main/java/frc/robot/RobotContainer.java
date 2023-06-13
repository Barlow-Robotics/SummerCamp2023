// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveRobot;
import frc.robot.commands.InstrumentedSequentialCommandGroup;
import frc.robot.commands.StartFlyWheel;
import frc.robot.commands.StartShooter;
import frc.robot.commands.StopFlyWheel;
import frc.robot.commands.StopShooter;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    /* Subsystems */
    public final Drive driveSub = new Drive();
    public final Shooter shooterSub = new Shooter();
    public final Vision visionSub = new Vision();

    /* Commands */
    private final StartShooter startShooterCmd = new StartShooter(shooterSub);
    private final StopShooter stopShooterCmd = new StopShooter(shooterSub);
    private final StartFlyWheel startFlyWheelCmd = new StartFlyWheel(shooterSub);
    private final StopFlyWheel stopFlyWheelCmd = new StopFlyWheel(shooterSub);

    /* Controllers */
    Joystick driverController; // Joystick 1
    Joystick operatorController; // Joystick 2

    /* Buttons */
    private JoystickButton alignWithTargetButton;
    private JoystickButton shooterButton;
    private JoystickButton flyWheelButton;

    /* Drive Movement Axis */
    public int throttleJoystickID;
    public int turnJoystickID;

    /* Shuffleboard */
    final SendableChooser<String> autoChooser = new SendableChooser<String>();

    /* Autonomous */
    PathPlannerTrajectory path;
    PathPlannerTrajectory currentTrajectory = null;

    public RobotContainer() {
        configureButtonBindings();
        setupInstrumentation();
        buildAutoOptions();

        driveSub.setDefaultCommand(
                new DriveRobot(
                        driveSub, visionSub, alignWithTargetButton, driverController, throttleJoystickID,
                        turnJoystickID));
    }

    private void configureButtonBindings() {
        /* Setting Controller Ports */
        if (driverController == null) {
            System.out.println("Null driver controller, using joystick 1");
            driverController = new Joystick(1);
        }

        if (operatorController == null) {
            System.out.println("Null operator controller, using joystick 2");
            operatorController = new Joystick(2);
        }

        /* Setting Buttons on Controllers */
        alignWithTargetButton = new JoystickButton(driverController, Constants.LogitechDualAction.RightTrigger);
        shooterButton = new JoystickButton(operatorController, Constants.LogitechDualAction.RightTrigger);
        flyWheelButton = new JoystickButton(operatorController, Constants.LogitechDualAction.LeftTrigger);

        throttleJoystickID = Constants.LogitechDualAction.LeftStickY;
        turnJoystickID = Constants.LogitechDualAction.RightStickX;

        /* Setting Commands of Buttons */
        shooterButton.onTrue(startShooterCmd).onFalse(stopShooterCmd);
        // flyWheelButton.onTrue(startFlyWheelCmd).onFalse(stopFlyWheelCmd); // remove commands, added in shooter 
    }

    /************** PATHPLANNER & AUTO **************/

    private PathPlannerTrajectory loadPath(String name, double velocity, double accel, boolean reverse) {
        PathPlannerTrajectory temp = PathPlanner.loadPath(
                name,
                new PathConstraints(velocity, accel),
                reverse);
        return PathPlannerTrajectory.transformTrajectoryForAlliance(temp, DriverStation.getAlliance());
    }

    /* CREATE LINE AUTO COMMAND */

    InstrumentedSequentialCommandGroup createLineAutoCmd() {
        InstrumentedSequentialCommandGroup theCmd = new InstrumentedSequentialCommandGroup();

        path = loadPath(
                "Line", DriveConstants.DefaultAutoVelocity, DriveConstants.DefaultAutoAccel, true);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("FirstBase", new PrintCommand("***************MISSION SHOOT FRISBEE IS A GO****************"));
        eventMap.put("shoot", startShooterCmd);
        eventMap.put("stop shoot", stopShooterCmd);
        eventMap.put("Final event",
                new PrintCommand("**************MISSION SHOOT FRISBEE IS A SUCCESS****************"));

        PPRamseteCommand basePathCmd = new PPRamseteCommand(
                path,
                driveSub::getPose,
                new RamseteController(),
                new DifferentialDriveKinematics(DriveConstants.TrackWidth),
                driveSub::setWheelSpeeds,
                driveSub);

        FollowPathWithEvents eventPathCmd = new FollowPathWithEvents(
                basePathCmd,
                path.getMarkers(),
                eventMap);

        theCmd.addCommands(new InstantCommand(() -> this.currentTrajectory = path));
        theCmd.addCommands(new InstantCommand(() -> driveSub.setOdometry(path.getInitialPose()), driveSub));
        theCmd.addCommands(basePathCmd);

        theCmd.onCommandInitialize(Robot::reportCommandStart);
        theCmd.onCommandFinish(Robot::reportCommandFinish);

        return theCmd;
    }

    /* CREATE BIG CURVE AUTO COMMAND */

    InstrumentedSequentialCommandGroup createBigCurveAutoCmd() {
        InstrumentedSequentialCommandGroup theCmd = new InstrumentedSequentialCommandGroup();

        path = loadPath(
                "BigCurve", DriveConstants.DefaultAutoVelocity, DriveConstants.DefaultAutoAccel, false);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("FirstBase", new PrintCommand("***************MISSION SHOOT FRISBEE IS A GO****************"));
        eventMap.put("shoot", startShooterCmd);
        eventMap.put("stop shoot", stopShooterCmd);
        eventMap.put("Final event",
                new PrintCommand("**************MISSION SHOOT FRISBEE IS A SUCCESS****************"));

        PPRamseteCommand basePathCmd = new PPRamseteCommand(
                path,
                driveSub::getPose,
                new RamseteController(),
                new DifferentialDriveKinematics(DriveConstants.TrackWidth),
                driveSub::setWheelSpeeds,
                driveSub);

        FollowPathWithEvents eventPathCmd = new FollowPathWithEvents(
                basePathCmd,
                path.getMarkers(),
                eventMap);

        theCmd.addCommands(new InstantCommand(() -> this.currentTrajectory = path));
        theCmd.addCommands(new InstantCommand(() -> driveSub.setOdometry(path.getInitialPose()), driveSub));
        theCmd.addCommands(basePathCmd);

        theCmd.onCommandInitialize(Robot::reportCommandStart);
        theCmd.onCommandFinish(Robot::reportCommandFinish);

        return theCmd;
    }

    /* CREATE ARC AUTO COMMAND */

    InstrumentedSequentialCommandGroup createArcAutoCmd() {
        InstrumentedSequentialCommandGroup theCmd = new InstrumentedSequentialCommandGroup();

        path = loadPath(
                "Arc", DriveConstants.DefaultAutoVelocity, DriveConstants.DefaultAutoAccel, false);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("startPrint", new PrintCommand("***************MISSION SHOOT FRISBEE IS A GO****************"));
        eventMap.put("startFlyWheel", startFlyWheelCmd);
        eventMap.put("startShooting", startShooterCmd);
        eventMap.put("stopShooting", stopShooterCmd);
        eventMap.put("stopFlyWheel", stopFlyWheelCmd);
        eventMap.put("endPrint", new PrintCommand("**************MISSION SHOOT FRISBEE IS A SUCCESS****************"));

        PPRamseteCommand basePathCmd = new PPRamseteCommand(
                path,
                driveSub::getPose,
                new RamseteController(),
                new DifferentialDriveKinematics(DriveConstants.TrackWidth),
                driveSub::setWheelSpeeds,
                driveSub);

        FollowPathWithEvents eventPathCmd = new FollowPathWithEvents(
                basePathCmd,
                path.getMarkers(),
                eventMap);

        theCmd.addCommands(new InstantCommand(() -> this.currentTrajectory = path));
        theCmd.addCommands(new InstantCommand(() -> driveSub.setOdometry(path.getInitialPose()), driveSub));
        theCmd.addCommands(eventPathCmd);

        theCmd.onCommandInitialize(Robot::reportCommandStart);
        theCmd.onCommandFinish(Robot::reportCommandFinish);

        return theCmd;
    }

    /* CREATE SQUARE AUTO COMMAND */

    InstrumentedSequentialCommandGroup createSquareAutoCmd() {
        InstrumentedSequentialCommandGroup theCmd = new InstrumentedSequentialCommandGroup();

        path = loadPath(
                "Square", DriveConstants.DefaultAutoVelocity, DriveConstants.DefaultAutoAccel, false);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("FirstBase", new PrintCommand("***************MISSION SHOOT FRISBEE IS A GO****************"));
        eventMap.put("shoot", startShooterCmd);
        eventMap.put("stop shoot", stopShooterCmd);
        eventMap.put("Final event",
                new PrintCommand("**************MISSION SHOOT FRISBEE IS A SUCCESS****************"));

        PPRamseteCommand basePathCmd = new PPRamseteCommand(
                path,
                driveSub::getPose,
                new RamseteController(),
                new DifferentialDriveKinematics(DriveConstants.TrackWidth),
                driveSub::setWheelSpeeds,
                driveSub);

        FollowPathWithEvents eventPathCmd = new FollowPathWithEvents(
                basePathCmd,
                path.getMarkers(),
                eventMap);

        theCmd.addCommands(new InstantCommand(() -> this.currentTrajectory = path));
        theCmd.addCommands(new InstantCommand(() -> driveSub.setOdometry(path.getInitialPose()), driveSub));
        theCmd.addCommands(eventPathCmd);

        theCmd.onCommandInitialize(Robot::reportCommandStart);
        theCmd.onCommandFinish(Robot::reportCommandFinish);

        return theCmd;
    }

    /* CREATE FANCY AUTO COMMAND */

    InstrumentedSequentialCommandGroup createFancyAutoCmd() {
        InstrumentedSequentialCommandGroup theCmd = new InstrumentedSequentialCommandGroup();

        path = loadPath(
                "Fancy", DriveConstants.DefaultAutoVelocity, DriveConstants.DefaultAutoAccel, false);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("FirstBase", new PrintCommand("***************MISSION SHOOT FRISBEE IS A GO****************"));
        eventMap.put("shoot", startShooterCmd);
        eventMap.put("stop shoot", stopShooterCmd);
        eventMap.put("Final event",
                new PrintCommand("**************MISSION SHOOT FRISBEE IS A SUCCESS****************"));

        PPRamseteCommand basePathCmd = new PPRamseteCommand(
                path,
                driveSub::getPose,
                new RamseteController(),
                new DifferentialDriveKinematics(DriveConstants.TrackWidth),
                driveSub::setWheelSpeeds,
                driveSub);

        FollowPathWithEvents eventPathCmd = new FollowPathWithEvents(
                basePathCmd,
                path.getMarkers(),
                eventMap);

        theCmd.addCommands(new InstantCommand(() -> this.currentTrajectory = path));
        theCmd.addCommands(new InstantCommand(() -> driveSub.setOdometry(path.getInitialPose()), driveSub));
        theCmd.addCommands(eventPathCmd);

        theCmd.onCommandInitialize(Robot::reportCommandStart);
        theCmd.onCommandFinish(Robot::reportCommandFinish);

        return theCmd;
    }

    public PathPlannerTrajectory getCurrentTrajectory() {
        return currentTrajectory;
    }

    private void setupInstrumentation() {
        PPRamseteCommand.setLoggingCallbacks(
                (PathPlannerTrajectory traj) -> {
                    this.currentTrajectory = traj;
                },
                (Pose2d targetPose) -> {
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/X")
                            .setDouble(targetPose.getX());
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/Y")
                            .setDouble(targetPose.getY());
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/degrees")
                            .setDouble(targetPose.getRotation().getDegrees());
                },
                (ChassisSpeeds setpointSpeeds) -> {
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vx")
                            .setDouble(setpointSpeeds.vxMetersPerSecond);
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vy")
                            .setDouble(setpointSpeeds.vyMetersPerSecond);
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/omega")
                            .setDouble(setpointSpeeds.omegaRadiansPerSecond);
                },
                (Translation2d translationError, Rotation2d rotationError) -> {
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/X")
                            .setDouble(translationError.getX());
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/Y")
                            .setDouble(translationError.getY());
                    NetworkTableInstance.getDefault().getEntry("pathPlanner/rotationError/degrees")
                            .setDouble(rotationError.getDegrees());
                });

        NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/X").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/Y").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("pathPlanner/targetPose/degrees").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vx").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/vy").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("pathPlanner/setpointSpeeds/omega").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/X").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("pathPlanner/translationError/Y").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("pathPlanner/rotationError/degrees").setDouble(0.0);
    }

    private void buildAutoOptions() {
        autoChooser.setDefaultOption("Line", "line");
        autoChooser.addOption("Big Curve", "bigCurve");
        autoChooser.addOption("Arc", "arc");
        autoChooser.addOption("Square", "square");
        autoChooser.addOption("Fancy", "fancy");

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        this.currentTrajectory = new PathPlannerTrajectory();

        String choice = autoChooser.getSelected();
        if (choice == "line") {
            return createLineAutoCmd(); // create error
        } else if (choice == "bigCurve") {
            return createBigCurveAutoCmd();
        } else if (choice == "arc") {
            return createArcAutoCmd();
        } else if (choice == "square") {
            return createSquareAutoCmd();
        } else if (choice == "fancy") {
            return createFancyAutoCmd();
        } else {
            return null;
        }
    }
}
