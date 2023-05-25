// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class DriveRobot extends CommandBase {

    Drive driveSub;
    Vision visionSub;

    // private boolean lastAutoSteer = false;
    private float yawMultiplier = 1.0f;
    private double error;
    private double leftVelocity;
    private double rightVelocity;
    // private int missedFrames = 0;
    private double adjustment;

    Trigger autoAlignButton;
    Trigger toggleTargetButton;
    Joystick driverController;
    int controllerThrottleID;
    int controllerTurnID;

    public PIDController pid;

    public DriveRobot(
            Drive d, Vision v, Trigger autoAlignButton, Joystick driverController, int throttleID, int turnID) {

        driveSub = d;
        visionSub = v;

        this.autoAlignButton = autoAlignButton;
        this.driverController = driverController;
        this.controllerThrottleID = throttleID;
        this.controllerTurnID = turnID;

        pid = new PIDController(
                Constants.DriveConstants.kPAutoAlign,
                Constants.DriveConstants.kIAutoAlign,
                Constants.DriveConstants.kDAutoAlign);

        addRequirements(driveSub);
    }

    @Override
    public void initialize() {
        pid.reset();
        // missedFrames = 0;
    }

    @Override
    public void execute() {
        boolean autoAlignEnabled = autoAlignButton.getAsBoolean();

        SmartDashboard.putBoolean("Auto Align Enabled", autoAlignEnabled);

        double x = -driverController.getRawAxis(controllerThrottleID);
        if (Math.abs(x) < 0.01) {
            x = 0.0;
        }
        double yaw = -driverController.getRawAxis(controllerTurnID);
        if (Math.abs(yaw) < 0.01) {
            yaw = 0.0;
        }
        double speed = -x;
        double turn = -yaw;

        if (!autoAlignEnabled) {
            yaw = -turn;

            // if (armSub.getAngle() > 60.0) {
            // speed = speed * 0.45;
            // yaw = yaw * 0.6;
            // }

            // yawMultiplier = (float) (0.3 + Math.abs(speed) * 0.2f);
            yawMultiplier = 0.5f;
            yaw = Math.signum(yaw) * (yaw * yaw) * yawMultiplier;

            if (Math.abs(yaw) < 0.02f) {
                yaw = 0.0f;
            }

            // lastAutoSteer = false;
            // } else {
            // if (!lastAutoSteer) {
            // pid.reset();
            // }

            // // if (toggleTarget == true) { /* switch indicates game piece with switch
            // value
            // // of 1 (maybe or 0?) */

            // if (visionSub.aprilTagIsVisible()) {
            // error = visionSub.aprilTagDistanceFromCenter();
            // adjustment = pid.calculate(error);
            // adjustment = Math.signum(adjustment)
            // * Math.min(Math.abs(adjustment),
            // Constants.DriveConstants.CorrectionRotationSpeed / 4.0);
            // leftVelocity = Constants.DriveConstants.CorrectionRotationSpeed - adjustment;
            // rightVelocity = Constants.DriveConstants.CorrectionRotationSpeed +
            // adjustment;

            // driveSub.setWheelSpeeds(leftVelocity, rightVelocity);
            // } else {
            // missedFrames++;
            // }
            // }
        } else { /* switch indicates april tag with switch value of -1 */

            if (visionSub.aprilTagDetected()) {
                error = visionSub.aprilTagBearing();
                adjustment = pid.calculate(error);
                adjustment = Math.signum(adjustment)
                        * Math.min(Math.abs(adjustment),
                                Constants.DriveConstants.CorrectionRotationSpeed / 4.0);
                leftVelocity = Constants.DriveConstants.CorrectionRotationSpeed - adjustment;
                rightVelocity = Constants.DriveConstants.CorrectionRotationSpeed +
                        adjustment;

                driveSub.setWheelSpeeds(leftVelocity, rightVelocity);
            // } else {
            //     missedFrames++;
            }
            yaw = pid.calculate(visionSub.aprilTagBearing());
            // lastAutoSteer = true;
        }

        NetworkTableInstance.getDefault().getEntry("drive/speed").setDouble(-speed);
        NetworkTableInstance.getDefault().getEntry("drive/speed").setDouble(-speed);
        NetworkTableInstance.getDefault().getEntry("drive/yaw").setDouble(yaw);

        driveSub.drive(-speed, yaw * 0.8, true);

    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}