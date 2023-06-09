// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class UnderGlow extends SubsystemBase {

  Shooter shooterSub;
  
  boolean isShooting;

  
  int currentMode = 1;

  /** Creates a new UnderGlow. */
  public UnderGlow() {


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
                desiredMode = Constants.UnderGlowConstants.IsShooting; // shooting is not a variable in underglow constants
          }
          }

      if (currentMode != desiredMode && port != null) {
          try {
              port.write(new byte[] { (byte) desiredMode }, 1);
          } catch (Exception ex) {
          }
          currentMode = desiredMode;
      }
    //   if (DriverStation.isEnabled()) {
    //       if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
    //           desiredMode = Constants.UnderGlowConstants.BlueAlliance;
    //       } else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
    //           desiredMode = Constants.UnderGlowConstants.RedAlliance;
    //       }
    //   } else

    //   if (currentMode != desiredMode && port != null) {
    //       try {
    //           port.write(new byte[] { (byte) desiredMode }, 1);
    //       } catch (Exception ex) {

    //       }
    //       currentMode = desiredMode;
    //     }

    // if (shooterSub.isShooting) {
        
    //   } 
     

  }
    // This method will be called once per scheduler run
// >>>>>>> 698634a3fffcb5a2a68c5ab6cfff223e07641823
  }
}
