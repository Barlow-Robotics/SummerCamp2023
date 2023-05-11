// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    PathPlannerTrajectory reversePath;
    PathPlannerTrajectory engagePath;
    PathPlannerTrajectory shortSideGamePiecePath1;
    PathPlannerTrajectory shortSideGamePiecePath2;
    PathPlannerTrajectory longSideGamePiecePath1;
    PathPlannerTrajectory longSideGamePiecePath2;

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

    InstrumentedSequentialCommandGroup createPlaceTopAndReverseCommand() {
        /* Place Game Piece on Bottom Row, Reverse Out of Community */
        InstrumentedSequentialCommandGroup theCommand = new InstrumentedSequentialCommandGroup();

        reversePath = loadPath(
                "Reverse", DriveConstants.DefaultAutoVelocity, DriveConstants.DefaultAutoAccel, true);

        theCommand.addCommands(new InstantCommand(() -> this.currentTrajectory = reversePath));
        theCommand
                .addCommands(new InstantCommand(() -> driveSub.resetOdometry(reversePath.getInitialPose()), driveSub));

        // theCommand.addCommands(new PPRamseteCommand(
        // reversePath,
        // driveSub::getPose,
        // new RamseteController(),
        // new DifferentialDriveKinematics(Constants.DriveConstants.TrackWidth),
        // driveSub::setSpeeds,
        // false,
        // driveSub));

        // theCommand.onCommandInitialize(Robot::reportCommandStart);
        // theCommand.onCommandFinish(Robot::reportCommandFinish);

        return theCommand;
    }

    private PathPlannerTrajectory loadPath(String name, double velocity, double accel, boolean reverse) {
        PathPlannerTrajectory temp = PathPlanner.loadPath(
                name,
                new PathConstraints(velocity, accel),
                reverse);
        return PathPlannerTrajectory.transformTrajectoryForAlliance(temp, DriverStation.getAlliance());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    // public Command getAutonomousCommand() {
    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that will be used by all path following commands.
    // HashMap<String, Command> eventMap = new HashMap<>();
    // eventMap.put("FirstBase", new PrintCommand("Passed first leg"));
    // eventMap.put("half way", new PrintCommand("half way there"));
    // eventMap.put("done", new PrintCommand("arrived at detination"));
    // eventMap.put("intakeDown", new IntakeDown());

    // This will load the file "Example Path.path" and generate it with a max
    // velocity of 4 m/s and a max acceleration of 3 m/s^2
    // PathPlannerTrajectory traj = PathPlanner.loadPath("SquarePath", new
    // PathConstraints(1, 1));
    // PathPlannerTrajectory traj = PathPlanner.loadPath("FancyPath", new
    // PathConstraints(2, 1));

    // Command ic = new InstantCommand(() -> {
    // // Reset odometry for the first path you run during auto
    // m_drive.resetEncoders();
    // m_drive.resetOdometry(traj.getInitialPose());
    // });

    // RamseteController controller = new RamseteController();

    // Command pathFollowingCommand = new PPRamseteCommand(
    // traj,
    // m_drive::getPose, // Pose supplier
    // controller,
    // new DifferentialDriveKinematics(0.75), // wpk need to put in correct chassis
    // width (wheel base)
    // m_drive::setWheelSpeeds,
    // eventMap, // This argument is optional if you don't use event markers
    // m_drive // Requires this drive subsystem
    // );
    // return new SequentialCommandGroup(ic, pathFollowingCommand);
    // }

}
