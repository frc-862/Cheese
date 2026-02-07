// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.shuffleboard.LightningShuffleboard;

public class PhotonVision extends SubsystemBase {
    public record UnpackedData(Pose2d pose, double ambiguity, double timestamp, double counter) {}
    private record VisionInfo(double timestamp, double ambiguity, Pose2d pose) {};

    // The drivetrain to add vision measurments
    Swerve drivetrain;

    // Atomic
    AtomicReference<VisionInfo> pose;

    int previousCounter = -1;
    double macTimeOffset = 0;

    boolean dummyValueSent;

    DatagramSocket socket;

    /** Creates a new PhotonVision.
     * 
     * @param drivetrain The main drivetrain on the robot
     */
    public PhotonVision(Swerve drivetrain) {
        Thread receiveThread;

        try {
            // Bind to the port
            socket = new DatagramSocket(12345); 
            
            // Start a separate thread to receive packets
            receiveThread = new Thread(() -> {
                while (!Thread.currentThread().isInterrupted()) {
                    try {
                        byte[] receiveData = new byte[48];
                        DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
                        socket.receive(receivePacket); // Blocks this thread, not the robot
                        
                        UnpackedData data = parseBinaryPacket(receivePacket);
                        // Store data atomically
                        pose.set(new VisionInfo(data.timestamp(), data.ambiguity(), data.pose()));
                        
                    } catch (Exception e) {
                        log("Thread Error: " + e.getMessage());
                    }
                }
            });

            receiveThread.start();
        } catch (Exception e) {
            log("*** ERROR MAKING DATAGRAM SOCKET ***" + e);
        }
        
        this.drivetrain = drivetrain;
        pose = new AtomicReference<>(null);

        
    }
 
    @Override
    public void periodic() {
        Pose2d localPose = null;
        double ambiguity = 1;
        double timestamp = -1;

        LightningShuffleboard.setDouble("Vision", "robot_time", Utils.getCurrentTimeSeconds());

        if (pose.get() != null && pose.get().pose != null && pose.get().ambiguity < 1 && pose.get().timestamp > 0) {
            VisionInfo updatedPose = pose.getAndSet(null);

            if (macTimeOffset == 0) {
                macTimeOffset = Utils.getCurrentTimeSeconds() - updatedPose.timestamp;
            }

            double bestTagAmbiguity = updatedPose.ambiguity() * 1.5;

            LightningShuffleboard.setPose2d("Vision", "updated pose", updatedPose.pose);
            LightningShuffleboard.setDouble("Vision", "mac time offset", macTimeOffset);
            
            drivetrain.addVisionMeasurement(
                updatedPose.pose(), 
                updatedPose.timestamp + macTimeOffset, 
                VecBuilder.fill(bestTagAmbiguity, bestTagAmbiguity, bestTagAmbiguity));

        }     
    }
    // im lazy
    private void log(String message) {
        System.out.println("[PHOTON VISION] " + message);
    }

    public UnpackedData parseBinaryPacket(DatagramPacket packet) {
        byte[] data = packet.getData();
        
        // Safety check for length
        if (packet.getLength() < 48) {
            throw new IllegalArgumentException("Packet too small");
        }

        log("Unpacking data");

        // Wrap the data in a ByteBuffer
        ByteBuffer buffer = ByteBuffer.wrap(data, 0, 48);

        // Read doubles in the same order they were packed
        double x = buffer.getDouble();
        double y = buffer.getDouble();
        double rotRadians = buffer.getDouble();
        double ambiguity = buffer.getDouble();
        double timestamp = buffer.getDouble();
        double counter = buffer.getDouble();

        Pose2d pose;
        pose = new Pose2d(x, y, new Rotation2d(rotRadians));
        
        return new UnpackedData(pose, ambiguity, timestamp, counter);
    }
}
