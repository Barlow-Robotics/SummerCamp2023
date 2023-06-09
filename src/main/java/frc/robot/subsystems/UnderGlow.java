// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class UnderGlow extends SubsystemBase {
    SerialPort port;

    Shooter shooterSub;
    Robot auto;

    boolean autoActivated;
    boolean isShooting;

    int currentMode = 1;

    /** Creates a new UnderGlow. */
    public UnderGlow() {
        try {
            port = new SerialPort(9600, Constants.UnderGlowConstants.Port); //Ask Mr. Kinahan about serial port number
        } catch (Exception ex) {

        }
    }

    @Override
    public void periodic() {
        int desiredMode = Constants.UnderGlowConstants.NeonGreen;
        if (DriverStation.isEnabled()) {
            if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                desiredMode = Constants.UnderGlowConstants.BlueAlliance;
            } else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                desiredMode = Constants.UnderGlowConstants.RedAlliance;
            }

            if (shooterSub.isShooting()) {
                isShooting = true;
                desiredMode = Constants.UnderGlowConstants.IsShooting; // shooting is not a variable in underglow
                                                                       // constants
            }
            if (auto.autoActivated()){
                autoActivated = true;
                desiredMode = Constants.UnderGlowConstants.AutoActivated;
            }
        }

        if (currentMode != desiredMode && port != null) {
            try {
                port.write(new byte[] { (byte) desiredMode }, 1);
            } catch (Exception ex) {
            }
            currentMode = desiredMode;
        }
        // if (DriverStation.isEnabled()) {
        // if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        // desiredMode = Constants.UnderGlowConstants.BlueAlliance;
        // } else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        // desiredMode = Constants.UnderGlowConstants.RedAlliance;
        // }
        // } else

        // if (currentMode != desiredMode && port != null) {
        // try {
        // port.write(new byte[] { (byte) desiredMode }, 1);
        // } catch (Exception ex) {

        // }
        // currentMode = desiredMode;
        // }

        // if (shooterSub.isShooting) {

        // }

    }
    // This method will be called once per scheduler run
    // >>>>>>> 698634a3fffcb5a2a68c5ab6cfff223e07641823
}

