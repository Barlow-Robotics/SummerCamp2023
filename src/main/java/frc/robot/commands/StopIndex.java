// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;

public class StopIndex extends CommandBase {

  Index indexSub;

  public StopIndex(Index indexSub) {
    this.indexSub = indexSub;
    addRequirements(indexSub);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    indexSub.stopIndex();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
