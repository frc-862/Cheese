// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
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

        nt = NetworkTableInstance.create();
        nt.setServer("localhost", 5810);
        nt.startClient4("vision-client");

        nt.close();
        nt.startClient4("vision-client");

        dummyValueSent = false;

        // while(nt.getTable("Mac").getIntegerTopic("result_counter").subscribe(-1).get() < 0) {
        //     if ((System.currentTimeMillis() - startTime) > 3000) {
        //         nt = NetworkTableInstance.create();
        //         nt.setServer("localhost", 5810);
        //         nt.startClient4("akfjasdkljf");
        //     }
        // }

        nt.getTable("Mac").getStructTopic("estimated_pose", Pose2d.struct).publish().set(new Pose2d(-1, 0, new Rotation2d()));
        nt.getTable("Mac").getDoubleTopic("pose_ambiguity").publish().set(1);
        nt.getTable("Mac").getDoubleTopic("pose_timestamp").publish().set(-1);
        nt.getTable("Mac").getIntegerTopic("result_counter").publish().set(-1);

        poseSubscriber = nt.getTable("Mac").getStructTopic("estimated_pose", Pose2d.struct).subscribe(new Pose2d(-1, 0, new Rotation2d()));
        ambiguitySubscriber = nt.getTable("Mac").getDoubleTopic("pose_ambiguity").subscribe(1);
        timestampSubscriber = nt.getTable("Mac").getDoubleTopic("pose_timestamp").subscribe(-1);
        resultCounterSubscriber = nt.getTable("Mac").getIntegerTopic("result_counter").subscribe(-1);

        // tablesInitialized = false;
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
        nt.close();
    }

    // // im lazy
    // private void log(String message) {
    //     System.out.println("[PHOTON VISION] " + message);
    // }
}
