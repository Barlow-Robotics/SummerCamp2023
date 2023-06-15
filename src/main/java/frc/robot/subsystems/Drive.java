// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
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

    private final DifferentialDriveOdometry odometry;

    private double lastLeftDistance;
    private double lastRightDistance;

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.TrackWidth);

    boolean simulationInitialized = false;

    // private final DifferentialDrive diffDrive = new DifferentialDrive(leftLeader,
    // rightLeader);

    // private final SimpleMotorFeedforward m_feedforward = new
    // SimpleMotorFeedforward(1, 3);

    public Drive() {

        leftLeader = new WPI_TalonSRX(Constants.DriveConstants.LeftLeaderMotorID);
        leftFollower = new WPI_VictorSPX(Constants.DriveConstants.LeftFollowerMotorID);
        rightLeader = new WPI_TalonSRX(Constants.DriveConstants.RightLeaderMotorID);
        rightFollower = new WPI_VictorSPX(Constants.DriveConstants.RightFollowerMotorID);

        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), getLeftDistance(), getRightDistance());

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
        createNetworkTableEntries();
    }

    public void periodic() {
        odometry.update(
                gyro.getRotation2d(),
                getLeftDistance(),
                getRightDistance());

        updateNetworkTableEntries();
    }

    public void setDefaultNeutralMode() {
        leftLeader.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
    }

    private double getLeftSpeed() {
        double s = leftLeader.getSelectedSensorVelocity() * 10.0 * Constants.DriveConstants.MetersPerCount;
        return (s);
    }

    private double getRightSpeed() {
        double s = -rightLeader.getSelectedSensorVelocity() * 10.0 * Constants.DriveConstants.MetersPerCount;
        return (s);
    }

    public double getLeftDistance() {
        double d = (leftLeader.getSelectedSensorPosition() / Constants.DriveConstants.CountsPerRevolution)
                * Constants.DriveConstants.MetersPerRevolution;
        return (d);
    }

    public double getRightDistance() {
        double d = (-rightLeader.getSelectedSensorPosition() / Constants.DriveConstants.CountsPerRevolution)
                * Constants.DriveConstants.MetersPerRevolution;
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
        NetworkTableInstance.getDefault().getEntry("drive/drive/xSpeed").setDouble(xSpeed);
        NetworkTableInstance.getDefault().getEntry("drive/drive/rot").setDouble(rot);

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
                speeds.left * Constants.DriveConstants.MetersPerSecondToCountsPer100MSec);
        rightLeader.set(TalonSRXControlMode.Velocity,
                speeds.right * Constants.DriveConstants.MetersPerSecondToCountsPer100MSec);

        NetworkTableInstance.getDefault().getEntry("drive/setWheelSpeeds/leftSpeed").setDouble(speeds.left);
        NetworkTableInstance.getDefault().getEntry("drive/setWheelSpeeds/rightSpeed").setDouble(speeds.right);

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
    public void setOdometry(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getLeftDistance(), getRightDistance(), pose);
    }

    private void createNetworkTableEntries() {
        NetworkTableInstance.getDefault().getEntry("drive/leftMotorDistance").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/rightMotorDistance").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/odometry/X").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/odometry/Y").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/odometry/heading").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/setWheelSpeeds/leftSpeed").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/setWheelSpeeds/rightSpeed").setDouble(0.0);

        NetworkTableInstance.getDefault().getEntry("drive/drive/xSpeed").setDouble(0.0);
        NetworkTableInstance.getDefault().getEntry("drive/drive/rot").setDouble(0.0);
    }

    private void updateNetworkTableEntries() {
        NetworkTableInstance.getDefault().getEntry("drive/leftMotorDistance").setDouble(getLeftDistance());
        NetworkTableInstance.getDefault().getEntry("drive/rightMotorDistance").setDouble(getRightDistance());

        NetworkTableInstance.getDefault().getEntry("drive/odometry/X").setDouble(odometry.getPoseMeters().getX());
        NetworkTableInstance.getDefault().getEntry("drive/odometry/Y").setDouble(odometry.getPoseMeters().getY());
        NetworkTableInstance.getDefault().getEntry("drive/odometry/heading")
                .setDouble(odometry.getPoseMeters().getRotation().getDegrees());
    }

    private void setMotorConfig(WPI_TalonSRX motor) {
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.DriveConstants.ClosedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.DriveConstants.ManualVoltageRampingConstant);
        motor.config_kF(0, Constants.DriveConstants.kF);
        motor.config_kP(0, Constants.DriveConstants.kP);
        motor.config_kI(0, Constants.DriveConstants.kI);
        motor.config_kD(0, Constants.DriveConstants.kD);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public void simulationInit() {
        final double maxCtsPer100ms = ((5330.0 / 600) * Constants.DriveConstants.EncoderResolution) / Constants.DriveConstants.GearRatio ;

        PhysicsSim.getInstance().addTalonSRX( leftLeader, 0.25, maxCtsPer100ms, true);
        PhysicsSim.getInstance().addVictorSPX(leftFollower);
        PhysicsSim.getInstance().addTalonSRX( rightLeader, 0.25,maxCtsPer100ms, true);
        PhysicsSim.getInstance().addVictorSPX(rightFollower);
    }

    @Override
    public void simulationPeriodic() {
        Twist2d twist = kinematics.toTwist2d(this.getLeftDistance() - lastLeftDistance,
                this.getRightDistance() - lastRightDistance);
        NetworkTableInstance.getDefault().getEntry("drive/twist_angle").setDouble(Units.radiansToDegrees(twist.dtheta));
        gyroSim.setAngle(gyro.getAngle() - Units.radiansToDegrees(twist.dtheta));
        lastLeftDistance = this.getLeftDistance();
        lastRightDistance = this.getRightDistance();

        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Pitch"));
        angle.set(5.0);
    }
}
