// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

public class StartIndexAndShooter extends CommandBase {

  private Shooter shooterSub;
  private Index indexSub;

  /** Creates a new StartShooting. */
  public StartIndexAndShooter(Shooter s, Index i) {
    shooterSub = s;
    indexSub = i;
    addRequirements(shooterSub, indexSub);
    // Use addRequirements() here to dclare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int wpk = 1 ;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSub.startShooting();

    if(shooterSub.getVelocity() >= (0.95 * Constants.ShooterConstants.Flywheel.FlyWheelVelocity)) {
      indexSub.startHopper();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSub.stopShooting();
    indexSub.stopHopper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
