// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  DigitalOutput cameraLight;

  boolean aprilTagDetected;
  double aprilTagDistanceFromCenter;
  double aprilTagHeight;
  double aprilTagWidth;
  double aprilTagAngle;
  double distanceToAprilTag;
  String sourceIP = "Nothing Received";

  private DatagramChannel visionChannel = null;
  ByteBuffer buffer = ByteBuffer.allocate(1024);

  public Vision() {
    cameraLight = new DigitalOutput(Constants.VisionConstants.CameraLightID);

    try {
      visionChannel = DatagramChannel.open();
      InetSocketAddress sAddr = new InetSocketAddress(5800);
      visionChannel.bind(sAddr);
      visionChannel.configureBlocking(false);
    } catch (Exception ex) {
      int wpk = 1;
    }
  }

  // public void LookUpTable() {}
  // public ArrayList<Double> lookUpTArrayList;

  @Override
  public void periodic() {
    try {
      boolean done = false;
      String message = "";
      while (!done) {
        InetSocketAddress sender = (InetSocketAddress) visionChannel.receive(buffer);
        buffer.flip();
        int limits = buffer.limit();
        if (limits > 0) {
          byte bytes[] = new byte[limits];
          buffer.get(bytes, 0, limits);
          message = new String(bytes);
          sourceIP = sender.getAddress().toString();
        } else {
          done = true;
        }
        buffer.clear();
      }

      if (message.length() > 0) {
        Map<String, String> myMap = new HashMap<String, String>();

        ObjectMapper objectMapper = new ObjectMapper();
        myMap = objectMapper.readValue(message, new TypeReference<HashMap<String, String>>() {
        });
        this.aprilTagDetected = Boolean.parseBoolean(myMap.get("detected"));
        this.aprilTagDistanceFromCenter = Double.parseDouble(myMap.get("distance_from_center"));
        this.aprilTagHeight = Double.parseDouble(myMap.get("bb_height"));
        this.aprilTagWidth = Double.parseDouble(myMap.get("bb_width"));
        this.aprilTagAngle = Double.parseDouble(myMap.get("angle"));
        this.distanceToAprilTag = Double.parseDouble(myMap.get("distance_to"));

      }
    } catch (Exception ex) {
      System.out.println("Exception reading data");
    }
  }

  public boolean aprilTagDetected() {
    return this.aprilTagDetected;
  }

  public double aprilTagDistanceFromCenter() {
    return this.aprilTagDistanceFromCenter;
  }

  public double aprilTagHeight() {
    return this.aprilTagHeight;
  }

  public double aprilTagWidth() {
    return this.aprilTagWidth;
  }

  public double aprilTagAngle() {
    return this.aprilTagAngle;
  }

  public double distanceToAprilTag() {
    return this.distanceToAprilTag;
  }
}