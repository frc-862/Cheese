// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.shuffleboard.LightningShuffleboard;

public class PhotonVision extends SubsystemBase implements AutoCloseable {
    private record VisionInfo(double timestamp, double ambiguity, Pose2d pose) {};

    // The drivetrain to add vision measurments
    Swerve drivetrain;

    // Atomic
    AtomicReference<VisionInfo> pose;

    // NT
    NetworkTableInstance nt;
    StructSubscriber<Pose2d> poseSubscriber;
    DoubleSubscriber ambiguitySubscriber;
    DoubleSubscriber timestampSubscriber;
    IntegerSubscriber resultCounterSubscriber;

    // Keep publishers alive to retain topics
    StructPublisher<Pose2d> posePublisher;
    DoublePublisher ambiguityPublisher;
    DoublePublisher timestampPublisher;
    IntegerPublisher resultCounterPublisher;

    int previousCounter = -1;
    double macTimeOffset = 0;

    boolean dummyValueSent;

    /** Creates a new PhotonVision.
     * 
     * @param drivetrain The main drivetrain on the robot
     */
    public PhotonVision(Swerve drivetrain) {
        this.drivetrain = drivetrain;
        pose = new AtomicReference<>(null);

        // Use the default NetworkTables instance (roboRIO is the server)
        nt = NetworkTableInstance.getDefault();

        dummyValueSent = false;

        // Create publishers with "retained" option to ensure topics persist
        // This helps with the race condition where MacMini might not have connected yet
        var macTable = nt.getTable("Mac");

        posePublisher = macTable
            .getStructTopic("estimated_pose", Pose2d.struct)
            .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        posePublisher.set(new Pose2d(-1, 0, new Rotation2d()));

        ambiguityPublisher = macTable
            .getDoubleTopic("pose_ambiguity")
            .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        ambiguityPublisher.set(1);

        timestampPublisher = macTable
            .getDoubleTopic("pose_timestamp")
            .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        timestampPublisher.set(-1);

        resultCounterPublisher = macTable
            .getIntegerTopic("result_counter")
            .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        resultCounterPublisher.set(-1);

        // Now subscribe to the same topics
        // The publishers above ensure topics exist and prevent the race condition
        poseSubscriber = macTable
            .getStructTopic("estimated_pose", Pose2d.struct)
            .subscribe(new Pose2d(-1, 0, new Rotation2d()), PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        ambiguitySubscriber = macTable
            .getDoubleTopic("pose_ambiguity")
            .subscribe(1, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        timestampSubscriber = macTable
            .getDoubleTopic("pose_timestamp")
            .subscribe(-1, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        resultCounterSubscriber = macTable
            .getIntegerTopic("result_counter")
            .subscribe(-1, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    }
 
    @Override
    public void periodic() {
        Pose2d localPose = null;
        double ambiguity = 1;
        double timestamp = -1;

        if (nt.isConnected() && !dummyValueSent) {
            nt.getTable("Foo").getBooleanTopic("dummy").publish().set(true);
            dummyValueSent = true;
        }

        LightningShuffleboard.setDouble("Vision", "robot_time", Utils.getCurrentTimeSeconds());

        // Check if topics are being published (only check once)
        // if (!tablesInitialized) {
        //     boolean poseExists = poseSubscriber.isValid();
        //     boolean ambiguityExists = ambiguitySubscriber.isValid();
        //     boolean timestampExists = timestampSubscriber.isValid();
        //     boolean counterExists = resultCounterSubscriber.isValid();

        //     // log("Checking topic existence - pose: " + poseExists + ", ambiguity: " + ambiguityExists +
        //     //     ", timestamp: " + timestampExists + ", counter: " + counterExists);

        //     tablesInitialized = poseExists && ambiguityExists && timestampExists && counterExists;
        // }

        // Direct read comparison

            int count = (int) resultCounterSubscriber.get();
            // log("Counter - Subscriber: " + count + ", Direct: " + directValue +
            //     ", LastChange: " + counterTs + ", Previous: " + previousCounter);

            // // Only process if we have a valid counter and it's new data
            if (count != -1 && count > previousCounter) {
                previousCounter = count;

                // Read pose
                Pose2d value = poseSubscriber.get();

                if (value.getX() < 0) {
                    pose.set(null);
                    return;
                }
                localPose = value;

                // Read ambiguity
                double ambiguityValue = ambiguitySubscriber.getAsDouble();

                if (ambiguityValue == 1) {
                    pose.set(null);
                    return;
                }
                ambiguity = ambiguityValue;

                // Read timestamp
                double timestampValue = timestampSubscriber.getAsDouble();

                if (timestampValue < 0) {
                    pose.set(null);
                    return;
                }
                timestamp = timestampValue;

                pose.set(new VisionInfo(timestamp, ambiguity, localPose));
            }

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

    @Override
    public void close() throws Exception {
        // Close publishers
        if (posePublisher != null) posePublisher.close();
        if (ambiguityPublisher != null) ambiguityPublisher.close();
        if (timestampPublisher != null) timestampPublisher.close();
        if (resultCounterPublisher != null) resultCounterPublisher.close();

        // Close subscribers
        if (poseSubscriber != null) poseSubscriber.close();
        if (ambiguitySubscriber != null) ambiguitySubscriber.close();
        if (timestampSubscriber != null) timestampSubscriber.close();
        if (resultCounterSubscriber != null) resultCounterSubscriber.close();

        // Note: Don't close the default NetworkTables instance
        // It's shared across the robot program
    }

    // // im lazy
    // private void log(String message) {
    //     System.out.println("[PHOTON VISION] " + message);
    // }
}
