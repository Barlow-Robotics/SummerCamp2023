// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;

public class Shooter extends SubsystemBase {
    private static final TalonSRX hopperMotor = null;

    WPI_TalonFX flyWheelMotor;

    Solenoid solenoid;
    Solenoid extendSolenoid;

    boolean isShooting = false;
    boolean simulationInitialized = false;

    boolean extended = false;

    public Shooter() {
        flyWheelMotor = new WPI_TalonFX(Constants.ShooterConstants.Flywheel.FlyWheelMotorID);
        setMotorConfig(flyWheelMotor);
        flyWheelMotor.configFactoryDefault();

        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM /* <- This probably needs to change */,
                Constants.ShooterConstants.RetractSolenoidID);
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
        motor.configClosedloopRamp(Constants.DriveConstants.ClosedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.DriveConstants.ManualVoltageRampingConstant);
        motor.config_kF(0, Constants.DriveConstants.kF);
        motor.config_kP(0, Constants.DriveConstants.kP);
        motor.config_kI(0, Constants.DriveConstants.kI);
        motor.config_kD(0, Constants.DriveConstants.kD);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public boolean flyWheelIsUpToSpeed() {
        if (getFlyWheelVelocity() == Constants.ShooterConstants.Flywheel.RPM) {
            return true;
        } else {
            return false;
        }
    }

    public void retract() {
        extendSolenoid.set(true);
        solenoid.set(false);
        extended = false;
    }

    public void open() {
        extendSolenoid.set(false);
        solenoid.set(true);
        extended = true;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void startShooting() {
        flyWheelMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.Flywheel.FlyWheelVelocity);
        isShooting = true;
    }

    public void stopShooting() {
        flyWheelMotor.set(TalonFXControlMode.Velocity, 0);
        isShooting = false;
    }

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonSRX(hopperMotor, 0.5, 6800);
    }

}
