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

    private double pixelOffset;
    static private double setPoint = 28 ;
    private double leftVelocity;
    private double rightVelocity;
    private int missedFrames;
    private double adjustment;
    boolean autoAlignEnabled;

    public PIDController pid;

    public AutoAlign(Drive driveSub, Vision visionSub) {
        this.driveSub = driveSub;
        this.visionSub = visionSub;

        pid = new PIDController(
                Constants.DriveConstants.kPAutoAlign, // EHP fix PID values
                Constants.DriveConstants.kIAutoAlign,
                Constants.DriveConstants.kDAutoAlign);

        pid.setSetpoint(28);

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
            autoAlignEnabled = true;
            missedFrames = 0;
            pixelOffset = visionSub.getAprilTagDistToCenter();
            adjustment = pid.calculate(pixelOffset);
            adjustment = Math.signum(adjustment) 
                            * Math.min(Math.abs(adjustment), 2 * Constants.DriveConstants.SlowTurnVelocity);
            leftVelocity = adjustment;
            rightVelocity = -adjustment;

            driveSub.setWheelSpeeds(leftVelocity, rightVelocity);
        } else {
            missedFrames++;
        }

        if (missedFrames >= 10) {
            autoAlignEnabled = false;
            pid.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return (missedFrames > 10) || (Math.abs(pixelOffset - setPoint) < 10);
    }
}
