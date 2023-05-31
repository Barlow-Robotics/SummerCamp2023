// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {
  WPI_TalonSRX hopperMotor;

  public Index() {
    hopperMotor = new WPI_TalonSRX(Constants.IndexConstants.HopperMotorID);
    setMotorConfig(hopperMotor);
    hopperMotor.configFactoryDefault();
  }

  @Override
  public void periodic() {
  }

  public void startIndex() {
    hopperMotor.set(TalonSRXControlMode.Velocity, Constants.IndexConstants.HopperMotorSpeed);
  }

  public void stopIndex() {
    hopperMotor.set(TalonSRXControlMode.Velocity, 0);
  }

  private void setMotorConfig(WPI_TalonSRX motor) {
    motor.configFactoryDefault();
    motor.configClosedloopRamp(Constants.IndexConstants.ClosedVoltageRampingConstant);
    motor.configOpenloopRamp(Constants.IndexConstants.ManualVoltageRampingConstant);
    motor.config_kF(0, Constants.IndexConstants.kF);
    motor.config_kP(0, Constants.IndexConstants.kP);
    motor.config_kI(0, Constants.IndexConstants.kI);
    motor.config_kD(0, Constants.IndexConstants.kD);
    motor.setNeutralMode(NeutralMode.Brake);
  }
}
