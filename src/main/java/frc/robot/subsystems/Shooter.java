// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;
import edu.wpi.first.wpilibj.DigitalInput;


public class Shooter extends SubsystemBase {
    // private static final TalonSRX hopperMotor = null;

    WPI_TalonFX flyWheelMotor;
    WPI_TalonSRX paddleMotor;
    
    DigitalInput hallEffect;

    public boolean isShooting = false;
    boolean simulationInitialized = false;

    public Shooter() {
        flyWheelMotor = new WPI_TalonFX(Constants.Shooter.FlyWheel.FlyWheelMotorID);
        setMotorConfig(flyWheelMotor);

        paddleMotor = new WPI_TalonSRX(Constants.Shooter.Paddle.PaddleMotorID);
        setMotorConfig(paddleMotor);

        hallEffect = new DigitalInput(Constants.Shooter.Paddle.HallEffectID); 

        CreateNetworkTableEntries();
        simulationInit();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Closed Loop Error", getClosedLoopError());
        SmartDashboard.putNumber("Fly Wheel Velocity", getVelocity(flyWheelMotor));
        SmartDashboard.putBoolean("Hall Effect Detected", hallEffectTrue());

        NetworkTableInstance.getDefault().getEntry("shooter/fly_wheel_velocity").setDouble(getVelocity(flyWheelMotor));
        NetworkTableInstance.getDefault().getEntry("shooter/paddle_velocity").setDouble(getVelocity(paddleMotor));
    }

    public void startPaddle() {
        paddleMotor.set(TalonSRXControlMode.Velocity, Constants.Shooter.Paddle.Velocity);
    }

    public void stopPaddle() {
        paddleMotor.set(TalonSRXControlMode.Velocity, 0.0);
        isShooting = false;
    }
    public void startFlyWheel() {
        flyWheelMotor.set(TalonFXControlMode.Velocity, Constants.Shooter.FlyWheel.Velocity);
        isShooting = true;
    }

    public void stopFlyWheel() {
        flyWheelMotor.set(TalonFXControlMode.Velocity, 0.0);
        isShooting = false;
    }
    

    public double getVelocity(TalonFX motor) {
        double velocity = motor.getSelectedSensorVelocity();
        return velocity;
    }

    public double getVelocity(TalonSRX motor) {
        double velocity = motor.getSelectedSensorVelocity();
        return velocity;
    }
    
    public boolean hallEffectTrue() {
        if (hallEffect.get() == false) {
            return true;
        }
        else {
            return false;
        }
    }

    public boolean flyWheelUpToSpeed() {
        if (getVelocity(flyWheelMotor) == (0.95 * Constants.Shooter.FlyWheel.Velocity)) {
            return true;
        } else {
            return false;
        }
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

    private void CreateNetworkTableEntries() {
        NetworkTableInstance.getDefault().getEntry("shooter/fly_wheel_velocity").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("shooter/paddle_velocity").setDouble(0.0);
    }
    
    private void setMotorConfig(WPI_TalonFX motor) {
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.Shooter.FlyWheel.ClosedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.Shooter.FlyWheel.ManualVoltageRampingConstant);
        motor.config_kF(0, Constants.Shooter.FlyWheel.kF); // Need to make these not the drive constants
        motor.config_kP(0, Constants.Shooter.FlyWheel.kP);
        motor.config_kI(0, Constants.Shooter.FlyWheel.kI);
        motor.config_kD(0, Constants.Shooter.FlyWheel.kD);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    private void setMotorConfig(WPI_TalonSRX motor) {
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.Shooter.Paddle.ClosedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.Shooter.Paddle.ManualVoltageRampingConstant);
        motor.config_kF(0, Constants.Shooter.Paddle.kF); // Need to make these not the drive constants
        motor.config_kP(0, Constants.Shooter.Paddle.kP);
        motor.config_kI(0, Constants.Shooter.Paddle.kI);
        motor.config_kD(0, Constants.Shooter.Paddle.kD);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(flyWheelMotor, 0.5, 6800);
    }

    @Override
    public void simulationPeriodic() {
   
    }
}
