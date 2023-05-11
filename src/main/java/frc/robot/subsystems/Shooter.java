// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;

public class Shooter extends SubsystemBase {
    // private static final TalonSRX hopperMotor = null;

    WPI_TalonFX flyWheelMotor;
    WPI_TalonFX paddleMotor;

    // Solenoid retractSolenoid;
    // Solenoid extendSolenoid;

    boolean isShooting = false;
    boolean simulationInitialized = false;

    boolean extended = false;

    public Shooter() {
        flyWheelMotor = new WPI_TalonFX(Constants.ShooterConstants.FlyWheelMotorID);
        setMotorConfig(flyWheelMotor);

        paddleMotor = new WPI_TalonFX(Constants.ShooterConstants.PaddleMotorID);
        setMotorConfig(paddleMotor);

        // retractSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM /* <- This
        // probably needs to change */,
        // Constants.ShooterConstants.RetractSolenoidID);
        // extendSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM /* <- This
        // probably needs to change */,
        // Constants.ShooterConstants.ExtendSolenoidID);

        simulationInit();
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Shooter Closed Loop Error", getClosedLoopError());
        SmartDashboard.putNumber("Shooter Velocity", getVelocity());

    }

    public void setVelocity(double velocity) {
        flyWheelMotor.set(TalonFXControlMode.Velocity, velocity * Constants.ShooterConstants.RPM);
    }

    public double getVelocity() {
        double velocity = flyWheelMotor.getSelectedSensorVelocity();
        return velocity;
    }

    public void startShooting() {
        flyWheelMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.FlyWheelVelocity);
        paddleMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.PaddleVelocity);
        isShooting = true;
    }

    public void stopShooting() {
        flyWheelMotor.set(TalonFXControlMode.Velocity, 0);
        paddleMotor.set(TalonFXControlMode.Velocity, 0);
        isShooting = false;
    }

    public boolean flyWheelIsUpToSpeed() {
        if (getVelocity() == Constants.ShooterConstants.RPM) {
            return true;
        } else {
            return false;
        }
    }

    // public void retract() {
    // extendSolenoid.set(true);
    // retractSolenoid.set(false);
    // extended = false;
    // }

    // public void open() {
    // extendSolenoid.set(false);
    // retractSolenoid.set(true);
    // extended = true;
    // }

    private void setMotorConfig(WPI_TalonFX motor) {
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.ShooterConstants.ClosedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.ShooterConstants.ManualVoltageRampingConstant);
        motor.config_kF(0, Constants.ShooterConstants.kF); // Need to make these not the drive constants
        motor.config_kP(0, Constants.ShooterConstants.kP);
        motor.config_kI(0, Constants.ShooterConstants.kI);
        motor.config_kD(0, Constants.ShooterConstants.kD);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(flyWheelMotor, 0.5, 6800);
    }

    private double getClosedLoopError() {
        return this.flyWheelMotor.getClosedLoopError();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.setSmartDashboardType("Shooter Subsystem");

        builder.addDoubleProperty("Error", this::getClosedLoopError, null);
    }
}
