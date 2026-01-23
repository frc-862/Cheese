// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
    private record VisionInfo(double timestamp, double ambiguity, Pose2d pose) {};

    // The drivetrain to add vision measurments
    Swerve drivetrain;

    // Atomic
    AtomicReference<VisionInfo> pose;

    // executor
    ScheduledExecutorService executor1;

    /** Creates a new PhotonVision.
     * 
     * @param drivetrain The main drivetrain on the robot
     */
    public PhotonVision(Swerve drivetrain) {
        this.drivetrain = drivetrain;

        executor1 = Executors.newSingleThreadScheduledExecutor();

        pose = new AtomicReference<>(null);

        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        StructSubscriber<Pose2d> poseSubscriber = nt.getTable("Mac").getStructTopic("estimated_pose", Pose2d.struct).subscribe(new Pose2d());
        DoubleSubscriber ambiguitySubscriber = nt.getTable("Mac").getDoubleTopic("pose_ambiguity").subscribe(1);
        DoubleSubscriber timestampSubscriber = nt.getTable("Mac").getDoubleTopic("pose_ambiguity").subscribe(-1);

        executor1.schedule(() -> {
            while (true) { 
                Pose2d localPose = null;
                double ambiguity = 1;
                double timestamp = -1;

                if (poseSubscriber.exists()) {
                    Pose2d value = poseSubscriber.get();
                    if (value == null) {
                        pose.set(null);
                        continue;
                    }

                    localPose = value;
                }

                if (ambiguitySubscriber.exists()) {
                    double value = ambiguitySubscriber.getAsDouble();
                    
                    if (value == 1) {
                        pose.set(null);
                        continue;
                    }

                    ambiguity = value;
                }

                if (timestampSubscriber.exists()) {
                    double value = timestampSubscriber.getAsDouble();

                    if (value < 0) {
                        pose.set(null);
                        continue;
                    }

                    timestamp = value;
                }

                pose.set(new VisionInfo(timestamp, ambiguity, localPose));
            }
            
        }, 0, TimeUnit.MILLISECONDS);
    }
 
    @Override
    public void periodic() {
        if (pose.get() != null) {
            VisionInfo updatedPose = pose.getAndSet(null);

            double bestTagAmbiguity = updatedPose.ambiguity();
            
            drivetrain.addVisionMeasurement(
                updatedPose.pose(), 
                Utils.fpgaToCurrentTime(updatedPose.timestamp), 
                VecBuilder.fill(bestTagAmbiguity, bestTagAmbiguity, bestTagAmbiguity));
            log("Added vision measurment");
        }
    }

    // im lazy
    private void log(String message) {
        DataLogManager.log("[PHOTON VISION]" + message);
    }
}
