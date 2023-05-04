// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
        // All variables are placeholders
        public static final double kMaxSpeed = 3.0; // meters per second
        public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
        public static final double DriveSpeed = 0.5;

        public static final double TrackWidth = 0.381 * 2; // meters
        public static final double WheelRadius = 0.0508; // meters
        public static final int EncoderResolution = 4096;

        public static final double GearRatio = 12.75; // Not 100% sure this is right
        public static final double CountsPerRevolution = EncoderResolution * GearRatio;
        public static final double InchesToMeters = 0.0254;
        public static final double WheelDiameter = 6.0 * InchesToMeters;
        public static final double MetersPerRevolution = WheelDiameter * Math.PI;
        public static final double MetersPerCount = MetersPerRevolution / CountsPerRevolution;
        public static final double CountsPerMeter = CountsPerRevolution / MetersPerRevolution;
        public static final double MetersPerSecondToCountsPer100MSec = CountsPerMeter / 10.0;
        public static final double MetersPerSecondToCountsPerSecond = (1 / MetersPerRevolution)
                * CountsPerRevolution;

        public static final double BaseDiameter = 33 * InchesToMeters;
        public static final double CircumferenceWithBD = BaseDiameter * Math.PI;
        public static final double WheelBase = 22 * InchesToMeters;
        public static final double CircumferenceWithWB = WheelBase * Math.PI;

        public static final double MaxVelocityCounts = kMaxSpeed * MetersPerSecondToCountsPer100MSec;

        public static final double ClosedVoltageRampingConstant = 0.0;
        public static final double ManualVoltageRampingConstant = 0.0;

        public static final double kF = 0.341;
        public static final double kP = 0.001;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kPAutoAlign = 0;
        public static final double kIAutoAlign = 0;
        public static final double kDAutoAlign = 0;
        public static final double CorrectionRotationSpeed = 0;
        
        public static final double DefaultAutoAccel = 4.0;
        public static final double DefaultAutoVelocity = 1.0; //metres per second

        public static int LeftLeaderMotorID = 4;
        public static int LeftFollowerMotorID = 5;
        public static int RightLeaderMotorID = 6;
        public static int RightFollowerMotorID = 7;
    }

    public static final class IndexConstants {
        // All variables are placeholders
        public static final int HopperMotorID = 15;
        public static final int PID_id = 0;

        public static final double ClosedVoltageRampingConstant = 0;
        public static final double ManualVoltageRampingConstant = 0;

        public static final double HopperMotorSpeed = 0.25;
        public static final double HopperKF = 0.048;
        public static final double HopperKP = 0.001;
        public static final double HopperKI = 0.0;
        public static final double HopperKD = 0.0;

        public static final double FeederKF = 0.048;
        public static final double FeederKP = 0.001;
        public static final double FeederKI = 0.0;
        public static final double FeederKD = 0.0;

        public static final double FeederMotorSpeed = 0.9;

    }

    public static final class ShooterConstants {
        // All variables are placeholders

        public static final class Flywheel {
            public static final int FlyWheelMotorID = 11;

            // public static final double kF = 0.048;
            public static final double kF = 0.059;
            // public static final double kP = 0.001;
            public static final double kP = 0.005;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final int PID_ID = 0;

            public static final int UnitsPerMotorRotation = 2048;

            public static final double RPM = 0;

            public static double FlyWheelVelocity;

        }

        public static final double ClosedVoltageRampingConstant = 0.1;
        public static final double ManualVoltageRampingConstant = 0.2;
        public static final int ExtendSolenoidID = 0; // Need to change
        public static final int RetractSolenoidID = 0; // Need to change
        
    }

    public static final class UnderGlowConstants {
    }

    public static final class VisionConstants {
    }

    public static final class AutoConstants {
    }

    public final class Logitech_Dual_Action {

        // Constants for Axes
        public static final int Left_Stick_X = 0;
        public static final int Left_Stick_Y = 1;
        public static final int Right_Stick_X = 2;
        public static final int Right_Stick_Y = 3;

        // Constants for buttons
        public static final int Left_Trigger = 7;
        public static final int Right_Trigger = 8;
        public static final int Button_A = 2;
        public static final int Button_B = 3;
        public static final int Button_X = 1;
        public static final int Button_Y = 4;
        public static final int Left_Bumper = 5;
        public static final int Right_Bumper = 6;
        public static final int Back_Button = 9;
        public static final int Start_Button = 10;
        public static final int Left_Stick = 11;
        public static final int Right_Stick = 12;

        public static final double Forward_Axis_Attenuation = -0.5;
        public static final double Lateral_Axis_Attenuation = 0.5;
        public static final double Yaw_Axis_Attenuation = 0.5;
    }

    public final class Logitech_F310_Controller {
        // Constants for Axes
        public static final int Left_Stick_X = 0;
        public static final int Left_Stick_Y = 1;
        public static final int Left_Trigger = 2;
        public static final int Right_Trigger = 3;
        public static final int Right_Stick_X = 4;
        public static final int Right_Stick_Y = 5;

        // Constants for buttons
        public static final int Button_A = 1;
        public static final int Button_B = 2;
        public static final int Button_X = 3;
        public static final int Button_Y = 4;
        public static final int Left_Bumper = 5;
        public static final int Right_Bumper = 6;
        public static final int Back_Button = 7;
        public static final int Start_Button = 8;
        public static final int Left_Stick = 9;
        public static final int Right_Stick = 10;

        public static final double Forward_Axis_Attenuation = -0.5;
        public static final double Lateral_Axis_Attenuation = 0.5;
        public static final double Yaw_Axis_Attenuation = 0.5;
    }
}
