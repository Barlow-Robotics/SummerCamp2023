// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

public class StartIndexAndShooter extends CommandBase {

    private Shooter shooterSub;
    private Index indexSub;

    public StartIndexAndShooter(Shooter s, Index i) {
        shooterSub = s;
        indexSub = i;
        addRequirements(shooterSub, indexSub);
    }

    @Override
    public void initialize() {
        int counter = 0;
    }

    @Override
    public void execute() {
        shooterSub.startFlyWheel();

        if (shooterSub.flyWheelUpToSpeed()) {
            indexSub.startHopper();
            shooterSub.startPaddle();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (!shooterSub.hallEffectTrue()) {
            shooterSub.startPaddle();
        }
        // counter++;
        // if (counter >= 500) { // 10 sec
        // shooter.stopShooterIndex();
        // } //courtesy of brian
    }

    @Override
    public boolean isFinished() {
        if (shooterSub.hallEffectTrue()) {
            return true;
        } else {
            return false;
        }
    }
}
