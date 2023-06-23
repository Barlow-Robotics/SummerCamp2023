// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    /** Creates a new Vision. */

    DigitalOutput cameraLight;

    double aprilTagID;
    boolean aprilTagDetected;
    // double aprilTagX;
    // double aprilTagY;
    // double aprilTagZ;
    // double aprilTagBearing;
    double aprilTagDistToCenter = 700;
    double aprilTagRange;

    String sourceIP = "Nothing Received";

    String DELETE = "{ \"detections\": [ { \"detected\": true, \"distToCenter\": 80, \"id\": 1, \"range\": 1 }, { \"detected\": false }, { \"detected\": false }, { \"detected\": true, \"distToCenter\": -88.25484466552734, \"id\": 4, \"range\": 1.5504128731987945 }]}";

    private DatagramChannel visionChannel = null;
    ByteBuffer buffer = ByteBuffer.allocate(1024);

    public Vision() {
        // cameraLight = new DigitalOutput(Constants.VisionConstants.CameraLightID); //
        // error in simulate code

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

            message = DELETE;

            if (message.length() > 0) {
                // Map<String, String> myMap = new HashMap<String, String>();
                ObjectMapper objectMapper = new ObjectMapper();
                JsonNode rootNode = objectMapper.readTree(message); // .readValue(message, new
                                                                    // TypeReference<HashMap<String,
                                                                    // String>>() {

                ArrayNode detectionsNode = (ArrayNode) rootNode.get("detections");

                for (int i = 0; i < 4; i++) {
                    JsonNode detection = detectionsNode.get(i);
                    if (detection.get("detected").asBoolean()) {
                        if (Math.abs(detection.get("distToCenter").asDouble()) < Math.abs(aprilTagDistToCenter)) {
                            aprilTagDistToCenter = detection.get("distToCenter").asDouble();
                            aprilTagID = i + 1;
                            aprilTagRange = detection.get("range").asDouble();
                            aprilTagDetected = detection.get("detected").asBoolean();
                        }
                    }
                }

                // System.out.println("IDs: " + detectionsNode.findValues("id"));
                // System.out.println("distToCenter: " +
                // detectionsNode.findValues("distToCenter"));
                // System.out.println("Ranges: " + detectionsNode.findValues("range"));
            }
        } catch (Exception ex) {
            this.aprilTagDetected = false;
            System.out.println("Exception reading vison data");
        }

        System.out.println("Deteted :" + aprilTagDetected);
        System.out.println("ID: " + aprilTagID);
        System.out.println("distToCenter: " + aprilTagDistToCenter);
        System.out.println("Range: " + aprilTagRange);

    }

    public double getAprilTagID() {
        return this.aprilTagID;
    }

    public boolean getAprilTagDetected() {
        return this.aprilTagDetected;
    }

    // public double aprilTagX() {
    // return this.aprilTagX;
    // }

    // public double aprilTagY() {
    // return this.aprilTagY;
    // }

    // public double aprilTagZ() {
    // return this.aprilTagZ;
    // }

    // public double aprilTagBearing() {
    // return this.aprilTagBearing;
    // }

    public double getAprilTagRange() {
        return this.aprilTagRange;
    }

    public double getAprilTagDistToCenter() {
        return this.aprilTagDistToCenter;
    }

    public String getSourceIP() {
        return this.sourceIP;
    }

    /******** SHUFFLEBOARD ********/

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.setSmartDashboardType("Vision Subsystem");

        builder.addDoubleProperty("April Tag ID", this::getAprilTagID, null);
        builder.addDoubleProperty("April Tag Distance to Center", this::getAprilTagDistToCenter, null);
        builder.addDoubleProperty("Arpil Tag Range", this::getAprilTagRange, null);
        builder.addBooleanProperty("April Tag Detected", this::getAprilTagDetected, null);
        builder.addStringProperty("Sender ID", this::getSourceIP, null);
    }
}