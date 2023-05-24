// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  boolean targetIsVisible = false;

  public void LookUpTable(){}
  public ArrayList<Double> lookUpTArrayList;
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public boolean aprilTagIsVisible() {
    return NetworkTableInstance.getDefault().getEntry("vision/april_tag_detected").getBoolean(false);
  }

  public double aprilTagDistanceFromCenter() {
    return NetworkTableInstance.getDefault().getEntry("vision/april_tag_distance_from_center").getDouble(0.0);
  }

  public double bbAprilTagHeight() {
    return NetworkTableInstance.getDefault().getEntry("vision/april_tag_bb_height").getDouble(0.0);
  }

  public double bbAprilTagWidth() {
    return NetworkTableInstance.getDefault().getEntry("vision/april_tag_bb_width").getDouble(0.0);
  }

  public double aprilTagAngle() {
    return NetworkTableInstance.getDefault().getEntry("vision/april_tag_angle").getDouble(0.0);
  }
  
  public double distanceToAprilTag() {
    return NetworkTableInstance.getDefault().getEntry("vision/distance_to_april_tag").getDouble(0.0);
  }
}