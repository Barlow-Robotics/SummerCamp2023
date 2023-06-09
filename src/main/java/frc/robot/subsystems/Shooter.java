// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
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

    public enum ShooterState {
        Stopped, SpinningUpFlywheel, AdvancingPaddle, IndexingPaddle
    }

    ShooterState shooterState = ShooterState.Stopped;

    public Shooter() {
        flyWheelMotor = new WPI_TalonFX(Constants.Shooter.FlyWheel.FlyWheelMotorID);
        setMotorConfig(flyWheelMotor);

        paddleMotor = new WPI_TalonSRX(Constants.Shooter.Paddle.PaddleMotorID);
        setMotorConfig(paddleMotor);

        hallEffect = new DigitalInput(Constants.Shooter.Paddle.HallEffectID);

        simulationInit();
    }

    @Override
    public void periodic() {
        manageShooterState();
    }

    /******** SHOOTER STATE MACHINE ********/

    private void manageShooterState() {
        switch (shooterState) {
            case Stopped:
                stopPaddle();
                stopFlyWheel();
                break;

            case SpinningUpFlywheel:
                startFlyWheel();
                if (flyWheelUpToSpeed()) {
                    shooterState = ShooterState.AdvancingPaddle;
                }
                break;

            case AdvancingPaddle:
                if (paddleAtIndexPosition() && !flyWheelUpToSpeed()) {
                    stopPaddle();
                    shooterState = ShooterState.SpinningUpFlywheel;
                } else {
                    startPaddle();
                }
                break;

            case IndexingPaddle:
                startPaddle();
                if (paddleAtIndexPosition()) {
                    shooterState = ShooterState.Stopped;
                }
                break;
        }
    }

    public void startShooter() {
        if (shooterState != ShooterState.IndexingPaddle) {
            shooterState = ShooterState.SpinningUpFlywheel;
        }
    }

    public void stopShooter() {
        if (shooterState == ShooterState.SpinningUpFlywheel) {
            shooterState = ShooterState.Stopped;
            // temporarily commented out until hall effects installed.
            // } else if (shooterState == ShooterState.AdvancingPaddle) {
            // shooterState = ShooterState.IndexingPaddle;
        } else {
            shooterState = ShooterState.Stopped;
        }
    }

    public String getShooterState() {
        return shooterState.toString();
    }

    public boolean isShooting() {
        return isShooting;
    }

    /******** FLYWHEEL ********/

    public void startFlyWheel() {
        flyWheelMotor.set(TalonFXControlMode.Velocity, Constants.Shooter.FlyWheel.Velocity);
        isShooting = true;
    }

    public void stopFlyWheel() {
        flyWheelMotor.set(TalonFXControlMode.Velocity, 0.0);
        isShooting = false;
    }

    public double getFlyWheelVelocity() {
        double velocity = flyWheelMotor.getSelectedSensorVelocity();
        return velocity;
    }

    public boolean flyWheelUpToSpeed() {
        return getFlyWheelVelocity() >= (0.95 * Constants.Shooter.FlyWheel.Velocity); // EHP change 0.95 later
    }

    private double getFlyWheelClosedLoopError() {
        return this.flyWheelMotor.getClosedLoopError();
    }

    /******** PADDLE ********/

    public void startPaddle() {
        paddleMotor.set(TalonSRXControlMode.PercentOutput, Constants.Shooter.Paddle.PercentOutput);
    }

    public void stopPaddle() {
        paddleMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
        isShooting = false;
    }

    public double getPaddlePercentOutput() {
        double velocity = paddleMotor.getMotorOutputPercent();
        return velocity;
    }

    public boolean paddleAtIndexPosition() {
        return !hallEffect.get();
    }

    /******** SHUFFLEBOARD ********/

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.setSmartDashboardType("Shooter Subsystem");

        builder.addStringProperty("State", this::getShooterState, null);
        builder.addDoubleProperty("Fly Wheel Velocity", this::getFlyWheelVelocity, null);
        builder.addDoubleProperty("Fly Wheel Error", this::getFlyWheelClosedLoopError, null);
        builder.addDoubleProperty("Paddle Percent Output", this::getPaddlePercentOutput, null);
        builder.addBooleanProperty("Paddle at Index Position", this::paddleAtIndexPosition, null);
    }

    /******** MOTOR CONFIGURATION ********/

    private void setMotorConfig(WPI_TalonFX motor) {
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.Shooter.FlyWheel.ClosedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.Shooter.FlyWheel.ManualVoltageRampingConstant);
        motor.config_kF(0, Constants.Shooter.FlyWheel.kF); //EHP fix PID values
        motor.config_kP(0, Constants.Shooter.FlyWheel.kP);
        motor.config_kI(0, Constants.Shooter.FlyWheel.kI);
        motor.config_kD(0, Constants.Shooter.FlyWheel.kD);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    private void setMotorConfig(WPI_TalonSRX motor) {
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.Shooter.Paddle.ClosedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.Shooter.Paddle.ManualVoltageRampingConstant);
        motor.config_kF(0, Constants.Shooter.Paddle.kF); //EHP fix PID values
        motor.config_kP(0, Constants.Shooter.Paddle.kP);
        motor.config_kI(0, Constants.Shooter.Paddle.kI);
        motor.config_kD(0, Constants.Shooter.Paddle.kD);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    /******** SIMULATION ********/

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(flyWheelMotor, 0.2, 21777);
        PhysicsSim.getInstance().addTalonSRX(paddleMotor, 0.2, 21777);
    }
}
