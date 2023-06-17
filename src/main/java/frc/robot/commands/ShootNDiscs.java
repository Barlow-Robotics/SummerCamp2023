// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class ShootNDiscs extends CommandBase {
    /** Creates a new ShootNDiscs. */

    int numDiscs;
    Shooter shooterSub ;

    public ShootNDiscs(int d, Shooter s) {
        // Use addRequirements() here to declare subsystem dependencies.
        numDiscs = d;
        shooterSub = s ;
        addRequirements(s);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooterSub.startShooter(numDiscs);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSub.stopShooter();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (shooterSub.getRemainingDiscs() == 0) 
            && (shooterSub.getShooterState() == ShooterState.IndexingPaddle ||  shooterSub.getShooterState() == ShooterState.Stopped);
    }
}
