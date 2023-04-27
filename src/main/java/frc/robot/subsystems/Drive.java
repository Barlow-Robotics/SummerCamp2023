// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;

public class Drive extends SubsystemBase {
    WPI_TalonSRX leftLeader;
    WPI_VictorSPX leftFollower;
    WPI_TalonSRX rightLeader;
    WPI_VictorSPX rightFollower;

    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private final ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(gyro);

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
            Constants.DriveConstants.kTrackWidth);

    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d(),
            getLeftDistance(), getRightDistance());

    private final DifferentialDrive diffDrive = new DifferentialDrive(leftLeader, rightLeader);

    boolean simulationInitialized = false;

    // private final SimpleMotorFeedforward m_feedforward = new
    // SimpleMotorFeedforward(1, 3);

    public Drive() {

        leftLeader = new WPI_TalonSRX(Constants.DriveConstants.ID_leftLeaderMotor);
        leftFollower = new WPI_VictorSPX(Constants.DriveConstants.ID_leftFollowerMotor);
        rightLeader = new WPI_TalonSRX(Constants.DriveConstants.ID_rightLeaderMotor);
        rightFollower = new WPI_VictorSPX(Constants.DriveConstants.ID_rightFollowerMotor);

        // Config Motors
        setMotorConfig(leftLeader);
        setMotorConfig(rightLeader);

        leftFollower.configFactoryDefault();
        rightFollower.configFactoryDefault();

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        rightLeader.setInverted(InvertType.InvertMotorOutput);
        rightFollower.setInverted(InvertType.FollowMaster);
        leftFollower.setInverted(InvertType.FollowMaster);

        leftLeader.setSensorPhase(true);
        rightLeader.setSensorPhase(false);

        setDefaultNeutralMode();
        gyro.reset();
        CreateNetworkTableEntries();
    }

    private void setMotorConfig(WPI_TalonSRX motor) {
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.DriveConstants.closedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.DriveConstants.manualVoltageRampingConstant);
        motor.config_kF(0, Constants.DriveConstants.kF);
        motor.config_kP(0, Constants.DriveConstants.kP);
        motor.config_kI(0, Constants.DriveConstants.kI);
        motor.config_kD(0, Constants.DriveConstants.kD);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public void periodic() {
        odometry.update(
                gyro.getRotation2d(),
                getLeftDistance(),
                getRightDistance());

        Pose2d pose = odometry.getPoseMeters();
        NetworkTableInstance.getDefault().getEntry("drive/pose/x").setDouble(pose.getX());
        NetworkTableInstance.getDefault().getEntry("drive/pose/y").setDouble(pose.getY());
        NetworkTableInstance.getDefault().getEntry("drive/pose/rotation").setDouble(pose.getRotation().getDegrees());

    }

    public void setDefaultNeutralMode() {
        leftLeader.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
    }

    private double getLeftSpeed() {
        double s = leftLeader.getSelectedSensorVelocity() * 10.0 * Constants.DriveConstants.Meters_Per_Count;
        return (s);
    }

    private double getRightSpeed() {
        double s = -rightLeader.getSelectedSensorVelocity() * 10.0 * Constants.DriveConstants.Meters_Per_Count;
        return (s);
    }

    public double getLeftDistance() {
        double d = (leftLeader.getSelectedSensorPosition() / Constants.DriveConstants.Counts_Per_Revolution)
                * Constants.DriveConstants.Meters_Per_Revolution;
        return (d);
    }

    public double getRightDistance() {
        double d = (-rightLeader.getSelectedSensorPosition() / Constants.DriveConstants.Counts_Per_Revolution)
                * Constants.DriveConstants.Meters_Per_Revolution;
        return (d);
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param xSpeed       Linear velocity in m/s.
     * @param rot          Angular velocity in rad/s.
     * @param squareInputs Decreases input sensitivity at low speeds.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double rot, boolean squareInputs) {
        NetworkTableInstance.getDefault().getEntry("drive/xSpeed").setDouble(leftLeader.getSelectedSensorVelocity());
        NetworkTableInstance.getDefault().getEntry("drive/rot").setDouble(leftLeader.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("drive/distance").setDouble(getLeftDistance());

        DifferentialDrive.WheelSpeeds relativeSpeeds = DifferentialDrive.arcadeDriveIK(xSpeed, rot, squareInputs);
        DifferentialDrive.WheelSpeeds absoluteSpeeds = new DifferentialDrive.WheelSpeeds(
                relativeSpeeds.left * Constants.DriveConstants.kMaxSpeed, // MaxAttainableVelocity
                relativeSpeeds.right * Constants.DriveConstants.kMaxSpeed);
        setWheelSpeeds(absoluteSpeeds);
    }

    public void resetEncoders() {
        leftLeader.setSelectedSensorPosition(0);
        rightLeader.setSelectedSensorPosition(0);
    }

    public double getGyroHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360.0);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getGyroHeading());
    }

    public void resetHeading() {
        gyro.reset();
    }

    public void setWheelSpeeds(DifferentialDrive.WheelSpeeds speeds) {
        leftLeader.set(TalonSRXControlMode.Velocity,
                speeds.left * Constants.DriveConstants.Meters_Per_Second_to_Counts_per_100_mSec);
        rightLeader.set(TalonSRXControlMode.Velocity,
                speeds.right * Constants.DriveConstants.Meters_Per_Second_to_Counts_per_100_mSec);
    }

    public void setWheelSpeeds(double left, double right) {
        setWheelSpeeds(new DifferentialDrive.WheelSpeeds(left, right));
    }

    public DifferentialDrive.WheelSpeeds getWheelSpeeds() {
        DifferentialDrive.WheelSpeeds result = new DifferentialDrive.WheelSpeeds();
        result.left = getLeftSpeed();
        result.right = getRightSpeed();

        return result;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getLeftDistance(), getRightDistance(), pose);
    }

    private void CreateNetworkTableEntries() {
        NetworkTableInstance.getDefault().getEntry("drive/left_motor_distance").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/right_motor_distance").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rotation").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/leftSpeed").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rightSpeed").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/xSpeed").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rot").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/arcadeDrive").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/leftVolts").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rightVolts").setDouble(0.0);
    }

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonSRX(leftLeader, 0.75, 6800, false);
        PhysicsSim.getInstance().addVictorSPX(leftFollower);
        PhysicsSim.getInstance().addTalonSRX(rightLeader, 0.75, 6800, false);
        PhysicsSim.getInstance().addVictorSPX(rightFollower);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }

        PhysicsSim.getInstance().run();

        gyroSim.setAngle(5.0);
        gyroSim.setRate(1.0);
        NetworkTableInstance.getDefault().getEntry("drive/gyro/getAngle").setDouble(gyro.getAngle());
    }
}
