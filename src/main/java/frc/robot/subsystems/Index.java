// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {
    WPI_TalonSRX hopperMotor;

    double hopperDirection = 1.0;
    double hopperSpeed = 0.0;

    public Index() {
        hopperMotor = new WPI_TalonSRX(Constants.IndexConstants.HopperMotorID);
        setMotorConfig(hopperMotor);
        hopperMotor.configFactoryDefault();
    }

    @Override
    public void periodic() {
        if (Math.abs(getSupplyCurrent()) > 0.1) {
            hopperDirection = -hopperDirection;
        }
        if (hopperDirection > 0.0) {
            hopperMotor.setInverted(InvertType.None);
        } else {
            hopperMotor.setInverted(InvertType.InvertMotorOutput);
        }
        hopperMotor.set(TalonSRXControlMode.PercentOutput, hopperSpeed);
        // System.out.println("current in index periodic is " + getSupplyCurrent()) ;
    }

    public void startIndex() {
        hopperSpeed = Constants.IndexConstants.HopperMotorSpeed;
    }

    public void stopIndex() {
        hopperSpeed = 0.0;
        // hopperMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    private double getSupplyCurrent() {
        return this.hopperMotor.getOutputCurrent();
        // return this.hopperMotor.getSupplyCurrent();
    }

    private double getStatorCurrent() {
        return this.hopperMotor.getStatorCurrent();
    }

    public double getOutput() {
        return hopperMotor.getMotorOutputPercent();
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Index Subsystem");

        builder.addDoubleProperty("Percent Output", this::getOutput, null);
        builder.addDoubleProperty("Supply Current", this::getSupplyCurrent, null);
        builder.addDoubleProperty("Stator Current", this::getStatorCurrent, null);
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
