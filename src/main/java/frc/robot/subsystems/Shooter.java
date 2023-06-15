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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;

public class Shooter extends SubsystemBase {
    // private static final TalonSRX hopperMotor = null;

    Vision visionSub;

    WPI_TalonFX flyWheelMotor;
    WPI_TalonSRX paddleMotor;

    DigitalInput hallEffect;
    private JoystickButton flyWheelButton;
    Joystick operatorController;
    public boolean isShooting = false;
    boolean simulationInitialized = false;

    double[][] distAndVelocityArray = { { 1, 2, 3, 4, 5 }, { 6000, 7000, 8000, 9000, 8000 } }; // Values are arbitrary,
                                                                                               // need
    // to test

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

        if (operatorController == null) {
            System.out.println("Null operator controller, using joystick 2");
            operatorController = new Joystick(2);
        }

        flyWheelButton = new JoystickButton(operatorController, Constants.LogitechDualAction.LeftTrigger);

        simulationInit();
    }

    @Override
    public void periodic() {
        manageShooterState();
        if (flyWheelButton.getAsBoolean()) {
            startFlyWheel();
        } else {
            // stopFlyWheel();
        }
    }

    /******** SHOOTER STATE MACHINE ********/

    private void manageShooterState() {
        switch (shooterState) {
            case Stopped:
                stopPaddle();
                // stopFlyWheel();
                break;

            case SpinningUpFlywheel:
                // startFlyWheel();
                if (flyWheelUpToSpeed()) {
                    shooterState = ShooterState.AdvancingPaddle;
                }
                break;

            case AdvancingPaddle:
                if (paddleAtIndexPosition() && !flyWheelUpToSpeed()) {
                    // System.out.println("AdvancingPaddle:Paddle at index but flywheel not up to
                    // speed") ;
                    stopPaddle();
                    shooterState = ShooterState.SpinningUpFlywheel;
                } else {
                    // System.out.println("AdvancingPaddle:Paddle at index is " +
                    // paddleAtIndexPosition() + " and flywheel up to speed is " +
                    // flyWheelUpToSpeed()) ;
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
        } else if (shooterState == ShooterState.AdvancingPaddle) {
            shooterState = ShooterState.IndexingPaddle;
        } else {
            shooterState = ShooterState.Stopped;
        }
    }

    public String getShooterState() {
        return shooterState.toString();
    }

    /******** FLYWHEEL ********/

    public double flyWheelVelocity() {
        double v;
        if (visionSub.aprilTagDetected()) {
            int closestIndex = 0;
            for (int i = 0; i < 5; i++) { // should run until the max length of the distAndVelocityArray
                if (visionSub.distanceToAprilTag() >= distAndVelocityArray[0][i]
                        && visionSub.distanceToAprilTag() < distAndVelocityArray[0][i + 1]) {
                    closestIndex = i;
                    break;
                }
            }
            v = distAndVelocityArray[1][closestIndex]
                    + (((visionSub.distanceToAprilTag() - distAndVelocityArray[0][closestIndex])
                            * (distAndVelocityArray[1][closestIndex + 1] - distAndVelocityArray[1][closestIndex]))
                            / (distAndVelocityArray[0][closestIndex + 1] - distAndVelocityArray[0][closestIndex]));
        } else {
            v = Constants.Shooter.FlyWheel.DefaultVelocity;
        }
        return v;
    }

    public void startFlyWheel() {
        flyWheelMotor.set(TalonFXControlMode.Velocity, flyWheelVelocity());
        isShooting = true;
    }

    public void stopFlyWheel() {
        flyWheelMotor.set(TalonFXControlMode.Velocity, 0.0);
        isShooting = false;
    }

    public boolean isShooting() {
        return isShooting;
    }

    public double getFlyWheelVelocity() {
        double velocity = flyWheelMotor.getSelectedSensorVelocity();
        return velocity;
    }

    public boolean flyWheelUpToSpeed() {
        return getFlyWheelVelocity() >= (0.95 * Constants.Shooter.FlyWheel.DefaultVelocity); // EHP change 0.95 later
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

    public void spinPaddleOnce() {
        boolean pastFirstTrue = false;
        startPaddle();
        if (!paddleAtIndexPosition()) {
            pastFirstTrue = true;
        }
        if (pastFirstTrue) {
            if (paddleAtIndexPosition()) {
                stopPaddle();
            }
        }
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
        motor.config_kF(0, Constants.Shooter.FlyWheel.kF); // EHP fix PID values
        motor.config_kP(0, Constants.Shooter.FlyWheel.kP);
        motor.config_kI(0, Constants.Shooter.FlyWheel.kI);
        motor.config_kD(0, Constants.Shooter.FlyWheel.kD);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    private void setMotorConfig(WPI_TalonSRX motor) {
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.Shooter.Paddle.ClosedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.Shooter.Paddle.ManualVoltageRampingConstant);
        motor.config_kF(0, Constants.Shooter.Paddle.kF); // EHP fix PID values
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
