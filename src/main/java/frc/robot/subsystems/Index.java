// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {
  WPI_TalonFX hopperMotor;

  /** Creates a new Index. */
  public Index() {
    hopperMotor = new WPI_TalonFX(Constants.IndexConstants.HopperMotorID);
    setMotorConfig(hopperMotor);
    hopperMotor.configFactoryDefault();
  }

  public void startHopper() {
    hopperMotor.set(TalonFXControlMode.Velocity, Constants.IndexConstants.HopperMotorSpeed);
  }

  public void stopHopper() {
    hopperMotor.set(TalonFXControlMode.Velocity, 0);
  }

  private void setMotorConfig(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.configClosedloopRamp(Constants.IndexConstants.ClosedVoltageRampingConstant);
    motor.configOpenloopRamp(Constants.IndexConstants.ManualVoltageRampingConstant);
    motor.config_kF(0, Constants.IndexConstants.kF);
    motor.config_kP(0, Constants.IndexConstants.kP);
    motor.config_kI(0, Constants.IndexConstants.kI);
    motor.config_kD(0, Constants.IndexConstants.kD);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
