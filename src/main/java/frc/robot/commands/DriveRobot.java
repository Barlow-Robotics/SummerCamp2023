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

    private boolean autoSteering = false;
    private float yawMultiplier = 1.0f;
    private double error;
    private double leftVelocity;
    private double rightVelocity;
    private int missedFrames = 0;
    private double adjustment;

    Trigger autoAlignButton;
    Trigger toggleTargetButton;
    Joystick driverController;
    int controllerThrottleID;
    int controllerTurnID;

    public PIDController pid;

    public DriveRobot(
            Drive driveSub, Vision visionSub, Trigger autoAlignButton, Joystick driverController, int throttleID,
            int turnID) {

        this.driveSub = driveSub;
        this.visionSub = visionSub;
        this.autoAlignButton = autoAlignButton;
        this.driverController = driverController;
        this.controllerThrottleID = throttleID;
        this.controllerTurnID = turnID;

        pid = new PIDController(
                Constants.DriveConstants.kPAutoAlign, //EHP fix PID values
                Constants.DriveConstants.kIAutoAlign,
                Constants.DriveConstants.kDAutoAlign);

        addRequirements(driveSub);
    }

    @Override
    public void initialize() {
        pid.reset();
        missedFrames = 0;
    }

    @Override
    public void execute() {
        boolean autoAlignEnabled = autoAlignButton.getAsBoolean(); //right trigger (driver controller)

        SmartDashboard.putBoolean("Auto Align Enabled", autoAlignEnabled); // fix

        double throttle = driverController.getRawAxis(controllerThrottleID);
        if (Math.abs(throttle) < 0.005) {
            throttle = 0.0;
        }

        double yaw = driverController.getRawAxis(controllerTurnID);
        if (Math.abs(yaw) < 0.005) {
            yaw = 0.0;
        }

        double speed = throttle;
        double turn = yaw;

        // if (!autoAlignEnabled || !autoSteering) {
        if  (!autoAlignEnabled ) {
            yaw = -turn;

            // yawMultiplier = (float) (0.3 + Math.abs(speed) * 0.2f);
            // yawMultiplier = 0.5f;
            // yaw = Math.signum(yaw) * (yaw * yaw) * yawMultiplier;

            if (Math.abs(yaw) < 0.02f) {
                yaw = 0.0f;
            }
            driveSub.drive(-speed * 0.8, yaw * 0.6, true);

        } else {
            if (visionSub.getAprilTagDetected()) {
                missedFrames = 0;
                autoSteering = true;
                error = visionSub.getAprilTagDistToCenter();
                adjustment = pid.calculate(error);
                // adjustment = Math.signum(adjustment)
                //         * Math.min(Math.abs(adjustment), Constants.DriveConstants.CorrectionRotationSpeed / 4.0);
                // leftVelocity = Constants.DriveConstants.CorrectionRotationSpeed - adjustment;
                // rightVelocity = Constants.DriveConstants.CorrectionRotationSpeed + adjustment;
                leftVelocity = adjustment;
                rightVelocity = -adjustment;

                driveSub.setWheelSpeeds(leftVelocity, rightVelocity);
            } else {
                missedFrames++;
            }

            if (missedFrames >= 10) {
                autoSteering = false;
                pid.reset();
            }

            // yaw = pid.calculate(visionSub.getAprilTagDistToCenter());
        }
        
        NetworkTableInstance.getDefault().getEntry("drive/speed").setDouble(-speed);
        NetworkTableInstance.getDefault().getEntry("drive/speed").setDouble(-speed);
        NetworkTableInstance.getDefault().getEntry("drive/yaw").setDouble(yaw);

        // driveSub.drive(-speed * 0.8, yaw * 0.6, true);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}