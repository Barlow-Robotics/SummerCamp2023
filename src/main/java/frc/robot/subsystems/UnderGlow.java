// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class UnderGlow extends SubsystemBase {
    SerialPort port;

    Shooter shooterSub;
    Robot robot;

    boolean autoActivated;
    boolean isShooting;

    int currentMode = 1;

    /** Creates a new UnderGlow. */
    public UnderGlow(Shooter s, Robot r) {
        shooterSub = s;
        robot = r;
        try {
            port = new SerialPort(9600, Constants.UnderGlowConstants.Port); 
        } catch (Exception ex) {

        }
    }
    

    @Override
    public void periodic() {
        byte data = 0;

        int desiredMode = Constants.UnderGlowConstants.Enabled;

        if (DriverStation.isEnabled()) {
            data += 8; 
            if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                desiredMode = Constants.UnderGlowConstants.BlueAlliance;
            } else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                data += 2; 
                desiredMode = Constants.UnderGlowConstants.RedAlliance;
            }

            // if (shooterSub.isShooting()) {
            //     data += 1;
            //     isShooting = true;
            //     desiredMode = Constants.UnderGlowConstants.IsShooting;                                                  
            // }

            if (robot.ifAutonomous()) {
                data += 4;
                autoActivated = true;
                desiredMode = Constants.UnderGlowConstants.AutoActivated;
            }
        }

        if (currentMode != desiredMode && port != null) {
            try {
                port.write(new byte[] { (byte) data }, 1);
            } catch (Exception ex) {
                int wpk = 1 ;
            }
            currentMode = desiredMode;
        }

    }
}

