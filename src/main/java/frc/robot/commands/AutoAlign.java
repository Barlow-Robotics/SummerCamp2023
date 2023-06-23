// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class AutoAlign extends CommandBase {

    Drive driveSub;
    Vision visionSub;

    private double error;
    private double leftVelocity;
    private double rightVelocity;
    private int missedFrames;
    private double adjustment;

    public PIDController pid;

    public AutoAlign(Drive driveSub, Vision visionSub) {
        this.driveSub = driveSub;
        this.visionSub = visionSub;

        pid = new PIDController(
                Constants.DriveConstants.kPAutoAlign, // EHP fix PID values
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
        if (visionSub.getAprilTagDetected()) {
            missedFrames = 0;
            error = visionSub.getAprilTagDistToCenter();
            adjustment = pid.calculate(error);
            adjustment = Math.signum(adjustment)
                    * Math.min(Math.abs(adjustment), Constants.DriveConstants.CorrectionRotationSpeed / 4.0);
            leftVelocity = Constants.DriveConstants.CorrectionRotationSpeed - adjustment;
            rightVelocity = Constants.DriveConstants.CorrectionRotationSpeed + adjustment;

            driveSub.setWheelSpeeds(leftVelocity, rightVelocity);
        } else {
            missedFrames++;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return (missedFrames > 10) || (Math.abs(error) < 5);
    }
}
