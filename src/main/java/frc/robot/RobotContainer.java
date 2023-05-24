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
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveRobot;
import frc.robot.commands.InstrumentedSequentialCommandGroup;
import frc.robot.commands.Pivot;
import frc.robot.commands.StartIndexAndShooter;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    public final Drive driveSub = new Drive();
    public final Shooter shooterSub = new Shooter();
    public final Index indexSub = new Index();
    // private final UnderGlow m_underGlow = new UnderGlow();
    private final Vision visionSub = new Vision();

    private final StartIndexAndShooter indexAndShooterCom = new StartIndexAndShooter(shooterSub, indexSub);
    private final DriveDistance driveDistanceCom = new DriveDistance(driveSub, 2, 1);
    private final Pivot pivotCom = new Pivot(driveSub, 180, 0.5);
    // private final TurnOffUnderGlow turnOffUnderGlowCommand = new
    // TurnOffUnderGlow(m_underGlow);
    // private final TurnOnUnderGlow turnOnUnderGlowCommand = new
    // TurnOnUnderGlow(m_underGlow);

    Joystick driverController; // Joystick 1
    Joystick operatorController; // Joystick 2

    private JoystickButton alignWithTargetButton;
    private JoystickButton indexAndShooterButton;
    private JoystickButton driveDistanceButton;
    private JoystickButton pivotButton;

    public int throttleJoystickID;
    public int turnJoystickID;

    SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    final SendableChooser<String> stringChooser = new SendableChooser<String>();

    PathPlannerTrajectory path;

    // The current trajectory that will be sent to the filed object for
    // debug/instrumentation
    PathPlannerTrajectory currentTrajectory = null;

    // HashMap<String, PathPlannerTrajectory> trajectories;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        // loadTrajectories();
        // createAutonomousCommands();

        driveSub.setDefaultCommand(
                new DriveRobot(
                        driveSub, visionSub, alignWithTargetButton, driverController, throttleJoystickID,
                        turnJoystickID));

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        if (driverController == null) {
            System.out.println("Null driver controller, using joystick 1");
            driverController = new Joystick(1);
        }

        if (operatorController == null) {
            System.out.println("Null operator controller, using joystick 2");
            operatorController = new Joystick(2);
        }

        String controllerType = driverController.getName();
        System.out.println("The controller name is " + controllerType);
        // boolean controllerFound = false;

        alignWithTargetButton = new JoystickButton(driverController, Constants.Logitech_Dual_Action.LeftBumper);
        indexAndShooterButton = new JoystickButton(driverController, Constants.Logitech_Dual_Action.RightTrigger);
        driveDistanceButton = new JoystickButton(driverController, Constants.Logitech_Dual_Action.ButtonY);
        pivotButton = new JoystickButton(driverController, Constants.Logitech_Dual_Action.ButtonX);

        throttleJoystickID = Constants.Logitech_Dual_Action.LeftStickY;
        turnJoystickID = Constants.Logitech_Dual_Action.RightStickX;

        indexAndShooterButton.whileTrue(indexAndShooterCom);
        driveDistanceButton.onTrue(driveDistanceCom);
        pivotButton.onTrue(pivotCom);
    }

    InstrumentedSequentialCommandGroup createAutoCmd() {
        /* Place Game Piece on Bottom Row, Reverse Out of Community */
        InstrumentedSequentialCommandGroup theCmd = new InstrumentedSequentialCommandGroup();

        HashMap<String, Command> eventMap = new HashMap<>();
            eventMap.put("FirstBase", new PrintCommand("***************Passed first leg****************"));
            eventMap.put("shoot", new InstantCommand(() -> shooterSub.startShooting()));
            eventMap.put("Final event", new PrintCommand("**************Shoot stuff****************"));

        path = loadPath(
                "Path", DriveConstants.DefaultAutoVelocity, DriveConstants.DefaultAutoAccel, true);

        theCmd.addCommands(new InstantCommand(() -> this.currentTrajectory = path));
        theCmd.addCommands(new InstantCommand(() -> driveSub.setOdometry(path.getInitialPose()), driveSub));


        PPRamseteCommand getPathFollowingCmd = new PPRamseteCommand(
            path,
            driveSub::getPose,
            new RamseteController(),
            new DifferentialDriveKinematics(DriveConstants.WheelBase), 
            driveSub::setWheelSpeeds,
            driveSub);

        FollowPathWithEvents pathCmd = new FollowPathWithEvents(
            getPathFollowingCmd,
            path.getMarkers(),
            eventMap);

        theCmd.addCommands(pathCmd);

        // theCommand.onCommandInitialize(Robot::reportCommandStart);
        // theCommand.onCommandFinish(Robot::reportCommandFinish);

        return theCmd;
    }

    private PathPlannerTrajectory loadPath(String name, double velocity, double accel, boolean reverse) {
        PathPlannerTrajectory temp = PathPlanner.loadPath(
                name,
                new PathConstraints(velocity, accel),
                reverse);
        return PathPlannerTrajectory.transformTrajectoryForAlliance(temp, DriverStation.getAlliance());
    }

    public Command getAutonomousCommand() {

        this.currentTrajectory = new PathPlannerTrajectory();
        return createAutoCmd();
        // String choice = stringChooser.getSelected();
        // if (choice == "autoCom") {
        //     return createAutoCmd();
        // } else {
        //     return null;
        // }
    }
}
