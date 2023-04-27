// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  WPI_TalonFX flyWheelMotor;

  boolean isShooting = false;
  boolean simulationInitialized = false;

  /** Creates a new Shooter. */

  public Shooter() {
    flyWheelMotor = new WPI_TalonFX(Constants.ShooterConstants.Flywheel.flyWheelMotorID);
    setMotorConfig(flyWheelMotor);
    flyWheelMotor.configFactoryDefault();
  }

  public void setFlyWheelVelocity(double velocity) {
    flyWheelMotor.set(TalonFXControlMode.Velocity, velocity * Constants.ShooterConstants.Flywheel.RPM);
  }

  public double getFlyWheelVelocity() {
    double velocity = flyWheelMotor.getSelectedSensorVelocity();
    return velocity;
  }

  private void setMotorConfig(WPI_TalonFX motor) {
    motor.configFactoryDefault();
    motor.configClosedloopRamp(Constants.DriveConstants.closedVoltageRampingConstant);
    motor.configOpenloopRamp(Constants.DriveConstants.manualVoltageRampingConstant);
    motor.config_kF(0, Constants.DriveConstants.kF);
    motor.config_kP(0, Constants.DriveConstants.kP);
    motor.config_kI(0, Constants.DriveConstants.kI);
    motor.config_kD(0, Constants.DriveConstants.kD);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  public boolean flyWheelIsUpToSpeed(){
    if (getFlyWheelVelocity() == Constants.ShooterConstants.Flywheel.RPM) {
      return true;
    } else {
    return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
