// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.sim.PhysicsSim;
import edu.wpi.first.wpilibj.DigitalInput;


public class Shooter extends SubsystemBase {
    // private static final TalonSRX hopperMotor = null;

    WPI_TalonFX flyWheelMotor;
    WPI_TalonSRX paddleMotor;
    
    DigitalInput HallEffect;

    // Solenoid retractSolenoid;
    // Solenoid extendSolenoid;

    boolean isShooting = false;
    boolean simulationInitialized = false;

    boolean extended = false;

    public Shooter() {
        flyWheelMotor = new WPI_TalonFX(Constants.ShooterConstants.FlyWheelMotorID);
        setMotorConfig(flyWheelMotor);

        paddleMotor = new WPI_TalonSRX(Constants.ShooterConstants.PaddleMotorID);
        setMotorConfig(paddleMotor);

        simulationInit();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Closed Loop Error", getClosedLoopError());
        SmartDashboard.putNumber("Fly Wheel Velocity", getVelocity(flyWheelMotor));
    }

    public void spinPaddle() {
        paddleMotor.configMotionCruiseVelocity(1200 * ShooterConstants.DegreesPerSecToCountsPer100MSec);
        // paddleMotor.configMotionAcceleration(1200 * ShooterConstants.DegreesPerSecToCountsPer100MSec / accelerationTime);

        // double ff = Math.sin(Math.toRadians(getAngle())) * rotationFeedForward();

        // double setAngle = 180 * ShooterConstants.CountsPerArmDegree;

        //  System.out.println("Setting arm angle to " + desiredAngle + "( " +  setAngle + " ) with feed forward "+ ff ) ;
        // paddleMotor.set(TalonSRXControlMode.MotionMagic, setAngle, DemandType.ArbitraryFeedForward, ff);
    }

    public double getVelocity(TalonFX motor) {
        double velocity = motor.getSelectedSensorVelocity();
        return velocity;
    }

    public void startShooterIndex() {
        flyWheelMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.FlyWheelVelocity);
        
        if (flyWheelIsUpToSpeed()) {
            
        }
        // wait until wheel i at full speed
        // run the spinner
        // do {
        // while
    // }
    // until (button is clicked to stop and spinner result == false)
        // than, turn off wheel

        // timer for failsame
        
        isShooting = true;
    }
    // public void start() {
    //     startShooting();
    //     if (flyWheelIsUpToSpeed()) {
    //         spinPaddle();
    //         if (shoterbuttonunclicked && HallEffect.get() == false)
                


    //     }
    // }

    public void stopShooterIndex() {
        flyWheelMotor.set(TalonFXControlMode.Velocity, 0.0);
        paddleMotor.set(TalonSRXControlMode.Velocity, 0.0);
        isShooting = false;
    }

    public boolean flyWheelIsUpToSpeed() {
        if (getVelocity(flyWheelMotor) == (0.95 * Constants.ShooterConstants.FlyWheelVelocity)) {
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

    private void setMotorConfig(WPI_TalonSRX motor) {
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
