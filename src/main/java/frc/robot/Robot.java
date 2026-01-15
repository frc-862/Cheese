// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.util.LightningRobot;

public class Robot extends LightningRobot {

    private final StringPublisher instructionsPublisher = 
        NetworkTableInstance.getDefault().getTable("Demo").getStringTopic("Instructions").publish();

    public Robot() {
        super(new RobotContainer());
    }

    @Override
    public void robotInit() {
        super.robotInit();

        instructionsPublisher.accept("Copilot: triggers control shooter power. X/B control collector. Y/A control indexer. "
            + "Left bumper for smart collect. Drive: left trigger for robot-centric drive. Start and Back to reset field forward.");
    }

}
